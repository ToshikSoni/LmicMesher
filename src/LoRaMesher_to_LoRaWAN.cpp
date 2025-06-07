#include <Arduino.h>
#include "LoraMesher.h"
#include <lmic.h>
#include <hal/hal.h>
#include <SPI.h>
#include <Wire.h>
#include <queue>

#define BOARD_LED 35
#define LED_ON HIGH
#define LED_OFF LOW
#define CS 8
#define RST 12
#define IRQ 14
#define IO1 13

// LoRaWAN credentials
static const u1_t PROGMEM APPEUI[8] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00}; // in big endian format
void os_getArtEui(u1_t *buf) { memcpy_P(buf, APPEUI, 8); }
static const u1_t PROGMEM DEVEUI[8] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00}; // in little endian format
void os_getDevEui(u1_t *buf) { memcpy_P(buf, DEVEUI, 8); }
static const u1_t PROGMEM APPKEY[16] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00}; // in big endian format
void os_getDevKey(u1_t *buf) { memcpy_P(buf, APPKEY, 16); }

// Forward declarations for functions used before their definition
void setupLoraMesher();
void loraWANTask(void *parameter);

// Radio state management
enum RadioState
{
    RADIO_LORAMESHER, // LoRaMesher is active
    RADIO_LORAWAN,    // LoRaWAN is active
    RADIO_SWITCHING   // Radio is being reconfigured
};
RadioState currentRadioState = RADIO_LORAWAN; // Start with LoRaWAN

// LoRaWAN control flags
bool hasJoined = false;
bool hasFoundDR = false;
bool loraMesherActive = false;
bool waitingForTxComplete = false;
unsigned long lastForwardTime = 0;
TaskHandle_t loRaWANTaskHandle = NULL;
portMUX_TYPE mux = portMUX_INITIALIZER_UNLOCKED;

// Message forwarding queue
struct ForwardMessage
{
    char message[80];
    size_t length;
};
std::queue<ForwardMessage> forwardQueue;

// Function declarations for functions used in event handler
void activateLoRaWAN();
void activateLoRaMesher();
void forwardViaLoRaWAN(const char *message);
void processForwardQueue();

// LoRaMesher setup
LoraMesher &radio = LoraMesher::getInstance();
uint32_t dataCounter = 0;
struct dataPacket
{
    uint32_t messageId;
    uint32_t timestamp;
    uint16_t sourceAddress;
    char message[32];
};
dataPacket *helloPacket = new dataPacket;
TaskHandle_t receiveLoRaMessage_Handle = NULL;

// LoRaWAN task implementation
void loraWANTask(void *parameter)
{
    Serial.println("LoRaWAN task started on core " + String(xPortGetCoreID()));
    os_init();
    LMIC_reset();

    // Send initial join request
    LMIC_startJoining();
    Serial.println("Started joining LoRaWAN network...");

    for (;;)
    {
        // Only run LMIC when in LoRaWAN mode
        if (currentRadioState == RADIO_LORAWAN)
        {
            os_runloop_once();
        }
        vTaskDelay(1 / portTICK_PERIOD_MS);
    }
}

// Function to send data via LoRaWAN
void do_send(uint8_t myData[], size_t size)
{
    if (LMIC.opmode & OP_TXRXPEND)
    {
        Serial.println(F("OP_TXRXPEND, not sending"));

        // Queue the message for later
        ForwardMessage newMsg;
        memcpy(newMsg.message, myData, size);
        newMsg.length = size;
        forwardQueue.push(newMsg);
        Serial.println("Message queued for later transmission");
    }
    else
    {
        Serial.println(size);
        int status = LMIC_setTxData2(5, myData, size, 0);
        Serial.println(status);
        if (status == -1)
        {
            Serial.println("Adjusting TX Data Rate... Data Not Sent");
            hasFoundDR = false;

            // Queue the message for later retry
            ForwardMessage newMsg;
            memcpy(newMsg.message, myData, size);
            newMsg.length = size;
            forwardQueue.push(newMsg);
            Serial.println("Message queued for later retry");
        }
        else if (status == 0)
        {
            Serial.println("Adjusted DR");
            hasFoundDR = true;
            Serial.println(F("Packet queued"));
            waitingForTxComplete = true;
            lastForwardTime = millis();
        }
    }
}

// Radio switching functions - FIXED
void activateLoRaWAN()
{
    Serial.println("Activating LoRaWAN...");
    currentRadioState = RADIO_SWITCHING;

    // If LoRaMesher was active, we need to properly suspend its tasks
    if (loraMesherActive)
    {
        Serial.println("Suspending LoRaMesher tasks...");
        // Use LoRaMesher's standby method to properly suspend all tasks
        radio.standby();

        // Reset the radio hardware
        pinMode(RST, OUTPUT);
        digitalWrite(RST, LOW);
        delay(20);
        digitalWrite(RST, HIGH);
        delay(50);
        loraMesherActive = false;
    }

    // Complete LMIC reset - FIX 1
    os_init();
    LMIC_reset();
    LMIC_setClockError(MAX_CLOCK_ERROR * 5 / 100); // Add clock error tolerance

    // If already joined, restore session settings
    if (hasJoined)
    {
        LMIC_setLinkCheckMode(0);
        LMIC_setAdrMode(0);
    }

    // Wait for LMIC to stabilize - FIX 2
    delay(250);

    currentRadioState = RADIO_LORAWAN;
    Serial.println("LoRaWAN activated");
}

void activateLoRaMesher()
{
    Serial.println("Activating LoRaMesher...");
    currentRadioState = RADIO_SWITCHING;

    // Reset the radio hardware first
    pinMode(RST, OUTPUT);
    digitalWrite(RST, LOW);
    delay(20);
    digitalWrite(RST, HIGH);
    delay(50);

    // If LoRaMesher was already set up but just suspended
    if (loraMesherActive)
    {
        // Just restart the tasks
        Serial.println("Resuming LoRaMesher tasks...");
        radio.start();
    }
    else
    {
        // Initialize or reinitialize LoRaMesher fully
        setupLoraMesher();
        loraMesherActive = true;
    }

    currentRadioState = RADIO_LORAMESHER;
    Serial.println("LoRaMesher activated");
}

// Forward a message via LoRaWAN - FIXED
void forwardViaLoRaWAN(const char *message)
{
    // Switch to LoRaWAN if needed
    if (currentRadioState != RADIO_LORAWAN)
    {
        activateLoRaWAN();
    }

    // Allow LMIC to stabilize - FIX 3
    delay(100);

    // Check if LMIC is busy - FIX 4
    if (LMIC.opmode & OP_TXRXPEND)
    {
        Serial.println("LMIC busy (OP_TXRXPEND), queuing message for later");

        // Queue the message for later
        ForwardMessage newMsg;
        size_t msgLen = strlen(message);
        strncpy(newMsg.message, message, sizeof(newMsg.message));
        newMsg.length = msgLen;
        forwardQueue.push(newMsg);
        return;
    }

    // Send the message
    Serial.println("Forwarding message via LoRaWAN:");
    Serial.println(message);
    do_send((uint8_t *)message, strlen(message));
}

// Process the forward queue
void processForwardQueue()
{
    if (forwardQueue.empty())
    {
        return;
    }

    // Don't process if LMIC is busy
    if (LMIC.opmode & OP_TXRXPEND)
    {
        Serial.println("LMIC busy, will retry queue processing later");
        return;
    }

    ForwardMessage msg = forwardQueue.front();
    forwardQueue.pop();

    Serial.println("Processing queued message:");
    Serial.println(msg.message);

    // Send directly with LMIC to bypass checks - FIX 5
    int status = LMIC_setTxData2(5, (uint8_t *)msg.message, msg.length, 0);
    if (status == 0)
    {
        Serial.println("Queued message sent to LMIC");
        waitingForTxComplete = true;
        lastForwardTime = millis();
    }
    else
    {
        Serial.printf("Failed to send queued message, error: %d\n", status);
        // Put back in queue for retry
        forwardQueue.push(msg);
    }
}

// LED flash helper
void led_Flash(uint16_t flashes, uint16_t delaymS)
{
    uint16_t index;
    for (index = 1; index <= flashes; index++)
    {
        digitalWrite(BOARD_LED, LED_ON);
        delay(delaymS);
        digitalWrite(BOARD_LED, LED_OFF);
        delay(delaymS);
    }
}

// Send a broadcast message via LoRaMesher
void sendBroadcastMessage()
{
    // Check if radio is in the right mode
    if (currentRadioState != RADIO_LORAMESHER)
    {
        Serial.println("Cannot broadcast: Radio not in LoRaMesher mode");
        return;
    }

    led_Flash(1, 100);
    helloPacket->messageId = ++dataCounter;
    helloPacket->timestamp = millis();
    helloPacket->sourceAddress = radio.getLocalAddress();
    snprintf(helloPacket->message, sizeof(helloPacket->message), "Hello #%d", dataCounter);
    Serial.println("Broadcasting discovery message...");
    Serial.printf("Message: %s\n", helloPacket->message);
    radio.createPacketAndSend(51608, helloPacket, sizeof(dataPacket));
    Serial.println("Broadcast sent!");
}

// Process received LoRaMesher packets and forward to LoRaWAN
void processReceivedPackets(void *)
{
    for (;;)
    {
        ulTaskNotifyTake(pdPASS, portMAX_DELAY);
        led_Flash(3, 100); // Quick LED flash to indicate packet arrival

        while (radio.getReceivedQueueSize() > 0)
        {
            Serial.println("ReceivedUserData_TaskHandle notify received");
            Serial.printf("Queue receiveUserData size: %d\n", radio.getReceivedQueueSize());

            // Get the packet from LoRaMesher
            AppPacket<dataPacket> *packet = radio.getNextAppPacket<dataPacket>();
            if (packet != NULL)
            {
                Serial.println("=== RECEIVED LORAMESHER PACKET ===");
                Serial.printf("From: 0x%04X\n", packet->src);
                Serial.printf("Message ID: %d\n", packet->payload->messageId);
                Serial.printf("Message: %s\n", packet->payload->message);

                // Forward to LoRaWAN if we're connected
                bool shouldForward = false;
                portENTER_CRITICAL(&mux);
                shouldForward = hasJoined;
                portEXIT_CRITICAL(&mux);

                if (shouldForward)
                {
                    // Create a formatted message for forwarding
                    char forwardMsg[80];
                    snprintf(forwardMsg, sizeof(forwardMsg),
                             "FWD-%04X: %s", packet->src, packet->payload->message);

                    // Check if we can forward directly or need to queue
                    if (forwardQueue.empty() && !waitingForTxComplete)
                    {
                        Serial.println("=== FORWARDING TO LORAWAN ===");
                        forwardViaLoRaWAN(forwardMsg);
                    }
                    else
                    {
                        // Add to queue
                        ForwardMessage newMsg;
                        strncpy(newMsg.message, forwardMsg, sizeof(newMsg.message));
                        newMsg.length = strlen(forwardMsg);
                        forwardQueue.push(newMsg);
                        Serial.println("=== MESSAGE QUEUED FOR FORWARDING ===");
                    }

                    led_Flash(2, 50); // Indicate forwarding with quick flashes
                }
                else
                {
                    Serial.println("Cannot forward: not joined to LoRaWAN network");
                }

                // Delete the packet when done
                radio.deletePacket(packet);
            }
        }
    }
}

void createReceiveMessages()
{
    int res = xTaskCreate(
        processReceivedPackets,
        "Receive App Task",
        4096,
        (void *)1,
        2,
        &receiveLoRaMessage_Handle);
    if (res != pdPASS)
        Serial.printf("Error: Receive App Task creation gave error: %d\n", res);
    radio.setReceiveAppDataTaskHandle(receiveLoRaMessage_Handle);
}

void createLoRaWANTask()
{
    int res = xTaskCreate(
        loraWANTask,
        "LoRaWAN Task",
        8192,
        (void *)1,
        3,
        &loRaWANTaskHandle);
    if (res != pdPASS)
        Serial.printf("Error: LoRaWAN Task creation gave error: %d\n", res);
}

static uint8_t mydata[] = "Hello World";
static osjob_t sendjob;
const unsigned TX_INTERVAL = 10;

void setupLoraMesher()
{
    Serial.println("Setting up LoRaMesher for India 865MHz SF8 DR5...");
    Serial.println(radio.getLocalAddress());
    LoraMesher::LoraMesherConfig config;
    config.module = LoraMesher::LoraModules::SX1262_MOD;
    config.loraCs = CS;
    config.loraRst = RST;
    config.loraIrq = IRQ;
    config.loraIo1 = IO1;
    config.freq = 865.2;
    config.bw = 125.0;
    config.sf = 8;
    config.cr = 5;
    config.power = 14;
    config.preambleLength = 8;
    config.syncWord = 0x12;
    SPI.begin(9, 11, 10, CS);
    config.spi = &SPI;
    Serial.println("=== LoRa Configuration ===");
    Serial.printf("Frequency: %.1f MHz\n", config.freq);
    Serial.printf("Bandwidth: %.1f kHz\n", config.bw);
    Serial.printf("Spreading Factor: SF%d\n", config.sf);
    Serial.printf("Coding Rate: 4/%d\n", config.cr);
    Serial.printf("Output Power: %d dBm\n", config.power);
    Serial.printf("Data Rate: DR5 (SF8BW125)\n");
    Serial.println("Region: India 865MHz band");

    // Initialize radio with configuration
    radio.begin(config);

    // Create receive task
    createReceiveMessages();

    // Mark LoRaMesher as active
    loraMesherActive = true;

    // Start LoRaMesher - this activates all tasks
    radio.start();

    Serial.println("LoRaMesher initialized for Indian region!");
}

class cHalConfiguration_t : public Arduino_LMIC::HalConfiguration_t
{
public:
    virtual u1_t queryBusyPin(void) override { return 13; };
    virtual bool queryUsingDcdc(void) override { return true; };
    virtual bool queryUsingDIO2AsRfSwitch(void) override { return true; };
    virtual bool queryUsingDIO3AsTCXOSwitch(void) override { return true; };
};

cHalConfiguration_t myConfig;

const lmic_pinmap lmic_pins = {
    .nss = 8,
    .rxtx = LMIC_UNUSED_PIN,
    .rst = 12,
    .dio = {14, LMIC_UNUSED_PIN, LMIC_UNUSED_PIN},
    .pConfig = &myConfig,
};

void printHex2(unsigned v)
{
    v &= 0xff;
    if (v < 16)
        Serial.print('0');
    Serial.print(v, HEX);
}

// LoRaWAN event handler with thread-safe flag update
void onEvent(ev_t ev)
{
    Serial.print(os_getTime());
    Serial.print(": ");
    switch (ev)
    {
    case EV_SCAN_TIMEOUT:
        Serial.println(F("EV_SCAN_TIMEOUT"));
        break;
    case EV_BEACON_FOUND:
        Serial.println(F("EV_BEACON_FOUND"));
        break;
    case EV_BEACON_MISSED:
        Serial.println(F("EV_BEACON_MISSED"));
        break;
    case EV_BEACON_TRACKED:
        Serial.println(F("EV_BEACON_TRACKED"));
        break;
    case EV_JOINING:
        Serial.println(F("EV_JOINING"));
        break;
    case EV_JOINED:
        Serial.println(F("EV_JOINED"));
        {
            u4_t netid = 0;
            devaddr_t devaddr = 0;
            u1_t nwkKey[16];
            u1_t artKey[16];
            LMIC_getSessionKeys(&netid, &devaddr, nwkKey, artKey);
            Serial.print("netid: ");
            Serial.println(netid, DEC);
            Serial.print("devaddr: ");
            Serial.println(devaddr, HEX);
            Serial.print("AppSKey: ");
            for (size_t i = 0; i < sizeof(artKey); ++i)
            {
                if (i != 0)
                    Serial.print("-");
                printHex2(artKey[i]);
            }
            Serial.println("");
            Serial.print("NwkSKey: ");
            for (size_t i = 0; i < sizeof(nwkKey); ++i)
            {
                if (i != 0)
                    Serial.print("-");
                printHex2(nwkKey[i]);
            }
            Serial.println();
        }
        LMIC_setLinkCheckMode(0);

        // Thread-safe update of joined status
        portENTER_CRITICAL(&mux);
        hasJoined = true;
        portEXIT_CRITICAL(&mux);

        Serial.println("*** LORAWAN CONNECTED - SWITCHING TO LORAMESHER ***");

        // Now that we've joined LoRaWAN, switch to LoRaMesher mode
        activateLoRaMesher();
        break;

    case EV_JOIN_FAILED:
        Serial.println(F("EV_JOIN_FAILED"));
        // Try to still use LoRaMesher mode even if LoRaWAN join failed
        activateLoRaMesher();
        break;
    case EV_REJOIN_FAILED:
        Serial.println(F("EV_REJOIN_FAILED"));
        break;
    case EV_TXCOMPLETE:
        Serial.println(F("EV_TXCOMPLETE (includes waiting for RX windows)"));
        if (LMIC.txrxFlags & TXRX_ACK)
            Serial.println(F("Received ack"));
        if (LMIC.dataLen)
        {
            Serial.print(F("Received "));
            Serial.print(LMIC.dataLen);
            Serial.println(F(" bytes of payload"));
            Serial.print("RETURN:");
            for (int i = 0; i < LMIC.dataLen; i++)
            {
                Serial.print((char)(LMIC.frame[LMIC.dataBeg + i]));
                Serial.print(" ");
            }
            Serial.println();
        }

        // Reset the waiting flag
        waitingForTxComplete = false;

        // Check if there are more messages to send
        if (!forwardQueue.empty())
        {
            // Process the next message in queue
            Serial.println("Processing next message in queue");
            processForwardQueue();
        }
        else
        {
            // No more messages, switch back to LoRaMesher mode
            Serial.println("No more messages to forward, switching back to LoRaMesher");
            activateLoRaMesher();
        }
        break;
    case EV_LOST_TSYNC:
        Serial.println(F("EV_LOST_TSYNC"));
        break;
    case EV_RESET:
        Serial.println(F("EV_RESET"));
        break;
    case EV_RXCOMPLETE:
        Serial.println(F("EV_RXCOMPLETE"));
        break;
    case EV_LINK_DEAD:
        Serial.println(F("EV_LINK_DEAD"));
        portENTER_CRITICAL(&mux);
        hasJoined = false;
        portEXIT_CRITICAL(&mux);
        Serial.println("*** LORAWAN DISCONNECTED - FORWARDING DISABLED ***");
        break;
    case EV_LINK_ALIVE:
        Serial.println(F("EV_LINK_ALIVE"));
        break;
    case EV_TXSTART:
        Serial.println(F("EV_TXSTART"));
        break;
    case EV_TXCANCELED:
        Serial.println(F("EV_TXCANCELED"));
        break;
    case EV_RXSTART:
        break;
    case EV_JOIN_TXCOMPLETE:
        Serial.println(F("EV_JOIN_TXCOMPLETE: no JoinAccept"));
        break;
    default:
        Serial.print(F("Unknown event: "));
        Serial.println((unsigned)ev);
        break;
    }
}

void setup()
{
    Serial.begin(115200);
    delay(2000);
    pinMode(BOARD_LED, OUTPUT);
    Serial.println(F("Starting"));
    Serial.println(F("LORAMESHER TO LORAWAN BRIDGE"));
    led_Flash(8, 100);

    pinMode(Vext, OUTPUT);
    digitalWrite(Vext, LOW);

    // Initialize hardware - explicit radio reset
    pinMode(RST, OUTPUT);
    digitalWrite(RST, LOW);
    delay(20);
    digitalWrite(RST, HIGH);
    delay(50);

    // Disable watchdog timers to prevent crashes
    disableCore0WDT();
    disableCore1WDT();

    // LMIC configuration improvements - FIX 6
    LMIC_setClockError(MAX_CLOCK_ERROR * 5 / 100);

    // First create and start LoRaWAN task
    Serial.println("Starting in LoRaWAN mode for initial network join");
    createLoRaWANTask();

    // LoraMesher will be activated after LoRaWAN join is complete

    Serial.println("Setup complete - waiting for LoRaWAN join before activating LoRaMesher");
}

void printByteArrayHex(uint8_t *x, size_t len)
{
    for (size_t i = 0; i < len; i++)
    {
        if (x[i] < 16)
            Serial.print("0");
        Serial.print(x[i], HEX);
        Serial.print(" ");
    }
    Serial.println();
}

void loop()
{
    // Handle serial input and commands
    if (Serial.available())
    {
        String msg = Serial.readStringUntil('\n');

        // Command handling
        if (msg.equals("lorawan"))
        {
            Serial.println("Manual switch to LoRaWAN mode");
            activateLoRaWAN();
        }
        else if (msg.equals("loramesher"))
        {
            Serial.println("Manual switch to LoRaMesher mode");
            activateLoRaMesher();
        }
        else if (msg.equals("status"))
        {
            Serial.println("\n=== BRIDGE STATUS ===");
            Serial.printf("LoRaWAN joined: %s\n", hasJoined ? "Yes" : "No");
            Serial.printf("Current mode: %s\n",
                          (currentRadioState == RADIO_LORAMESHER) ? "LoRaMesher" : (currentRadioState == RADIO_LORAWAN) ? "LoRaWAN"
                                                                                                                        : "Switching");
            Serial.printf("Messages in queue: %d\n", forwardQueue.size());
            if (loraMesherActive)
            {
                Serial.printf("LoRaMesher address: 0x%04X\n", radio.getLocalAddress());
                Serial.printf("Routing table size: %d\n", radio.routingTableSize());
            }
            else
            {
                Serial.println("LoRaMesher not yet active");
            }
        }
        else
        {
            // Regular message handling
            uint8_t buffer[51];
            size_t len = msg.length();
            if (len > 0 && len <= 50)
            {
                msg.getBytes(buffer, len + 1);
                Serial.print("Message to send: ");
                for (size_t i = 0; i < len; i++)
                    Serial.print((char)buffer[i]);
                Serial.println();

                // Send via LoRaWAN if joined
                portENTER_CRITICAL(&mux);
                bool canSend = hasJoined;
                portEXIT_CRITICAL(&mux);

                if (canSend)
                {
                    // If we're not in LoRaWAN mode, switch modes
                    if (currentRadioState != RADIO_LORAWAN)
                    {
                        activateLoRaWAN();
                    }
                    do_send(buffer, len);
                }
                else
                {
                    Serial.println("Cannot send: not joined to LoRaWAN network");
                }
            }
        }
    }

    // Periodic broadcast to help discover this bridge node - only in LoRaMesher mode
    static unsigned long lastBroadcast = 0;
    if (currentRadioState == RADIO_LORAMESHER && millis() - lastBroadcast > 15000)
    { // Every 15 seconds
        Serial.printf("\n=== SENDING PACKET #%d ===\n", dataCounter + 1);
        sendBroadcastMessage();
        lastBroadcast = millis();
    }

    // Safety check - if LoRaWAN forwarding is taking too long, switch back
    if (waitingForTxComplete && (millis() - lastForwardTime > 30000))
    {
        Serial.println("WARNING: LoRaWAN TX timeout, switching back to LoRaMesher");
        waitingForTxComplete = false;

        // If there are remaining messages in the queue
        if (!forwardQueue.empty())
        {
            Serial.println("Moving messages back to queue for later retry");
            // Don't switch to LoRaMesher yet as we still have messages to send
        }
        else
        {
            // No messages to send, safe to switch to LoRaMesher
            activateLoRaMesher();
        }
    }

    // Retry mechanism for failed transmissions - FIX 7
    static unsigned long lastRetry = 0;
    if (currentRadioState == RADIO_LORAWAN && !forwardQueue.empty() &&
        !waitingForTxComplete && millis() - lastRetry > 5000)
    {
        Serial.println("Attempting to retry queued message transmission");
        processForwardQueue();
        lastRetry = millis();
    }

    delay(10); // Small delay for task scheduling
}