#include <Arduino.h>
#include "LoraMesher.h"
#include <lmic.h>
#include <hal/hal.h>
#include <SPI.h>
#include <Wire.h>
#define BOARD_LED 35
#define LED_ON HIGH
#define LED_OFF LOW
#define CS 8
#define RST 12
#define IRQ 14
#define IO1 13
bool hasJoined = false;
bool hasFoundDR = false;
void do_send(uint8_t myData[], size_t size)
{
    if (LMIC.opmode & OP_TXRXPEND)
        Serial.println(F("OP_TXRXPEND, not sending"));
    else
    {
        Serial.println(size);
        int status = LMIC_setTxData2(5, myData, size, 0);
        Serial.println(status);
        if (status == -1)
        {
            Serial.println("Adjusting TX Data Rate... Data Not Sent");
            hasFoundDR = false;
        }
        else if (status == 0)
        {
            Serial.println("Adjusted DR");
            hasFoundDR = true;
        }
        Serial.println(F("Packet queued"));
    }
}
void loraWANTask(void *parameter)
{
    Serial.println("LoRaWAN task started on core " + String(xPortGetCoreID()));
    os_init();
    LMIC_reset();
    uint8_t initData[] = "Hello";
    do_send(initData, sizeof(initData));
    for (;;)
    {
        os_runloop_once();
        vTaskDelay(1 / portTICK_PERIOD_MS);
    }
}
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
void sendBroadcastMessage()
{
    led_Flash(1, 100);
    helloPacket->messageId = ++dataCounter;
    helloPacket->timestamp = millis();
    helloPacket->sourceAddress = radio.getLocalAddress();
    snprintf(helloPacket->message, sizeof(helloPacket->message), "Hello #%d", dataCounter);
    Serial.println("Broadcasting discovery message...");
    Serial.printf("Message: %s\n", helloPacket->message);
    radio.createPacketAndSend(44992, helloPacket, 1);
    Serial.println("Broadcast sent!");
}
void processReceivedPackets(void *)
{
    for (;;)
    {
        ulTaskNotifyTake(pdPASS, portMAX_DELAY);
        led_Flash(3, 100);
        while (radio.getReceivedQueueSize() > 0)
        {
            Serial.println("ReceivedUserData_TaskHandle notify received");
            Serial.printf("Queue receiveUserData size: %d\n", radio.getReceivedQueueSize());
            AppPacket<dataPacket> *packet = radio.getNextAppPacket<dataPacket>();
            radio.deletePacket(packet);
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
        NULL);
}
static const u1_t PROGMEM APPEUI[8] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
void os_getArtEui(u1_t *buf) { memcpy_P(buf, APPEUI, 8); }
static const u1_t PROGMEM DEVEUI[8] = {0x52, 0x5e, 0x26, 0x00, 0x47, 0x7c, 0x8d, 0xb2};
void os_getDevEui(u1_t *buf) { memcpy_P(buf, DEVEUI, 8); }
static const u1_t PROGMEM APPKEY[16] = {0xa5, 0xc4, 0x27, 0x65, 0xd3, 0x8a, 0x31, 0x2a, 0x66, 0xe3, 0x77, 0xfd, 0xf5, 0x48, 0x37, 0x84};
void os_getDevKey(u1_t *buf) { memcpy_P(buf, APPKEY, 16); }
static uint8_t mydata[] = "Hello World";
static osjob_t sendjob;
const unsigned TX_INTERVAL = 10;
void setupLoraMesher()
{
    Serial.println("Setting up LoRaMesher for India 865MHz SF8 DR5...");
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
    radio.begin(config);
    createReceiveMessages();
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
        hasJoined = true;
        break;
    case EV_JOIN_FAILED:
        Serial.println(F("EV_JOIN_FAILED"));
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
                Serial.print((LMIC.frame[LMIC.dataBeg + i]));
                Serial.print(" ");
            }
            Serial.println();
        }
        if (!hasFoundDR)
        {
            uint8_t data[] = "hello";
            do_send(data, sizeof(data));
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
    led_Flash(8, 100);
    pinMode(Vext, OUTPUT);
    digitalWrite(Vext, LOW);
    pinMode(lmic_pins.rst, OUTPUT);
    digitalWrite(lmic_pins.rst, LOW);
    delay(10);
    digitalWrite(lmic_pins.rst, HIGH);
    delay(10);
    disableCore0WDT();
    disableCore1WDT();
    setupLoraMesher();
    createLoRaWANTask();
    uint16_t myAddress = radio.getLocalAddress();
    Serial.println("\n=== DEVICE INFORMATION ===");
    Serial.printf("My Device Address: 0x%04X (%d)\n", myAddress, myAddress);

    Serial.println("Setup complete - now running from main loop");
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
    if (Serial.available())
    {
        String msg = Serial.readStringUntil('\n');
        uint8_t buffer[51];
        size_t len = msg.length();
        if (len > 0 && len <= 50)
        {
            msg.getBytes(buffer, len + 1);
            Serial.print("Message to send: ");
            for (size_t i = 0; i < len; i++)
                Serial.print((char)buffer[i]);
            Serial.println();
            do_send(buffer, len);
        }
    }
    Serial.printf("\n=== SENDING PACKET #%d ===\n", dataCounter + 1);
    sendBroadcastMessage();
    Serial.println("Waiting 15 seconds for next transmission...\n");
    delay(15000);
    delay(10);
}