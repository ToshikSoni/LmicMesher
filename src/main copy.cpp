#if 0
#include <Arduino.h>
#include "LoraMesher.h"

#ifdef SHOULD_USE_LMIC

#include "lmic.h"
#include <hal/hal.h>

#endif

//Using LILYGO TTGO T-BEAM v1.1 
#define BOARD_LED   4
#define LED_ON      LOW
#define LED_OFF     HIGH

bool hasJoined = false;
bool isUsingLmic = true;

bool hasFoundDR = false;

LoraMesher& radio = LoraMesher::getInstance();

uint32_t dataCounter = 0;
struct dataPacket {
    int type = 0; // 0 for counter, 1 for wan packet
    union {
        uint32_t counter = 0;
        uint8_t data[4];
    } data;
};

dataPacket* helloPacket = new dataPacket;

dataPacket* returnPacket = new dataPacket;

#ifdef SHOULD_USE_LMIC

void do_send(uint8_t mydata[], size_t size) {
    // Send the payload as a byte array
   
    // Check if there is not a current TX/RX job running
    if (LMIC.opmode & OP_TXRXPEND) {
        Serial.println(F("OP_TXRXPEND, not sending"));
    } else {
        // Prepare upstream data transmission at the next possible time.
        int status = LMIC_setTxData2(5, mydata, size - 1, 0);

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
        // LMIC_setTxData(5, mydata, size - 1, 0);
        // LMIC_setTxData2(1, jsonString, sizeof(mydata)-1, 0);
        Serial.println(F("Packet queued"));
    }
    // Next TX is scheduled after TX_COMPLETE event.
 }

// LMIC values
// This EUI must be in little-endian format, so least-significant-byte
// first. When copying an EUI from ttnctl output, this means to reverse
// the bytes. For TTN issued EUIs the last bytes should be 0xD5, 0xB3,
// 0x70.
static const u1_t PROGMEM APPEUI[8]={0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
void os_getArtEui (u1_t* buf) { memcpy_P(buf, APPEUI, 8);}


// This should also be in little endian format, see above.
static const u1_t PROGMEM DEVEUI[8]={0xac, 0x94, 0xb1, 0xb8, 0xb4, 0x0f, 0x68, 0xf5}; //f5680fb4b8b194ac
void os_getDevEui (u1_t* buf) { memcpy_P(buf, DEVEUI, 8);}


// This key should be in big endian format (or, since it is not really a
// number but a block of memory, endianness does not really apply). In
// practice, a key taken from ttnctl can be copied as-is.
static const u1_t PROGMEM APPKEY[16] = {0x96,0x44,0x4b,0xef,0xd2,0xbf,0x4e,0x30,0xdf,0x44,0xa1,0xbf,0x9c,0x84,0xa2,0x1b};//96444befd2bf4e30df44a1bf9c84a21b
void os_getDevKey (u1_t* buf) {  memcpy_P(buf, APPKEY, 16);}

static uint8_t mydata[] = "Hello World";
static osjob_t sendjob;


class cHalConfiguration_t: public Arduino_LMIC::HalConfiguration_t
{
  public:
    virtual u1_t queryBusyPin(void) override {return 13;};
    virtual bool queryUsingDcdc(void) override {return true;};
    virtual bool queryUsingDIO2AsRfSwitch(void) override {return true;};
    virtual bool queryUsingDIO3AsTCXOSwitch(void) override {return true;};
};

cHalConfiguration_t myConfig;


// Pin mapping
const lmic_pinmap lmic_pins = {
   .nss = CS,
   .rxtx = LMIC_UNUSED_PIN,
   .rst = RST,
   .dio = {IRQ, LMIC_UNUSED_PIN, LMIC_UNUSED_PIN},
   .pConfig = &myConfig,
};

void printHex2(unsigned v) {
    v &= 0xff;
    if (v < 16)
        Serial.print('0');
    Serial.print(v, HEX);
 }



 void onEvent (ev_t ev) {
    Serial.print(os_getTime());
    Serial.print(": ");
    switch(ev) {
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
            hasJoined = true;
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
              for (size_t i=0; i<sizeof(artKey); ++i) {
                if (i != 0)
                  Serial.print("-");
                printHex2(artKey[i]);
              }
              Serial.println("");
              Serial.print("NwkSKey: ");
              for (size_t i=0; i<sizeof(nwkKey); ++i) {
                      if (i != 0)
                              Serial.print("-");
                      printHex2(nwkKey[i]);
              }
              Serial.println();
            }
            // Disable link check validation (automatically enabled
            // during join, but because slow data rates change max TX
      // size, we don't use it in this example.
            LMIC_setLinkCheckMode(0);
            break;
        /*
        || This event is defined but not used in the code. No
        || point in wasting codespace on it.
        ||
        || case EV_RFU1:
        ||     Serial.println(F("EV_RFU1"));
        ||     break;
        */
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
            if (LMIC.dataLen) {
              Serial.print(F("Received "));
              Serial.print(LMIC.dataLen);
              Serial.println(F(" bytes of payload"));
              returnPacket->type = 1;
              for (int i = 0; i < LMIC.dataLen - 2; i++)
              {
                  returnPacket->data.data[i] = (LMIC.frame[LMIC.dataBeg + i]);
              }
              uint16_t dst = LMIC.frame[LMIC.dataBeg + LMIC.dataLen - 1] + 256 * LMIC.frame[LMIC.dataBeg + LMIC.dataLen - 2];
              isUsingLmic = false;
              radio.start();
              radio.createPacketAndSend(dst, returnPacket, 1); 
              break;
            }
            else {
                isUsingLmic = false;
            }

            
            if (hasFoundDR)
            {
                Serial.println("Restarting Mesher");
                radio.start(); //Start the LoRaMesher to send the next packet
            }
            else{
                uint8_t data[] = "hello";
                do_send(data, sizeof(data));
                isUsingLmic = true;
            }
            // os_setTimedCallback(&sendjob, os_getTime()+sec2osticks(TX_INTERVAL), do_send);
            break;
        case EV_LOST_TSYNC:
            Serial.println(F("EV_LOST_TSYNC"));
            break;
        case EV_RESET:
            Serial.println(F("EV_RESET"));
            break;
        case EV_RXCOMPLETE:
            // data received in ping slot
            Serial.println(F("EV_RXCOMPLETE"));
            break;
        case EV_LINK_DEAD:
            Serial.println(F("EV_LINK_DEAD"));
            break;
        case EV_LINK_ALIVE:
            Serial.println(F("EV_LINK_ALIVE"));
            break;
        /*
        || This event is defined but not used in the code. No
        || point in wasting codespace on it.
        ||
        || case EV_SCAN_FOUND:
        ||    Serial.println(F("EV_SCAN_FOUND"));
        ||    break;
        */
        case EV_TXSTART:
            Serial.println(F("EV_TXSTART"));
            break;
        case EV_TXCANCELED:
            Serial.println(F("EV_TXCANCELED"));
            break;
        case EV_RXSTART:
            /* do not print anything -- it wrecks timing */
            break;
        case EV_JOIN_TXCOMPLETE:
            Serial.println(F("EV_JOIN_TXCOMPLETE: no JoinAccept"));
            break;
 
 
        default:
            Serial.print(F("Unknown event: "));
            Serial.println((unsigned) ev);
            break;
    }
 }

#endif //SHOULD_USE_LMIC





//Led flash
void led_Flash(uint16_t flashes, uint16_t delaymS) {
    uint16_t index;
    for (index = 1; index <= flashes; index++) {
        digitalWrite(BOARD_LED, LED_ON);
        delay(delaymS);
        digitalWrite(BOARD_LED, LED_OFF);
        delay(delaymS);
    }
}

/**
 * @brief Print the counter of the packet
 *
 * @param data
 */
void printPacket(dataPacket data, uint16_t src) {
    if (data.type == 0)
    {
        Serial.printf("DATA:%d %x\n", data.data.counter, src);
        // Serial.printf("Hello Counte r received nº %d\n", data.data.counter);
    }
    else{
        for(int i = 0; i < 4; i++)
        {
            Serial.println(data.data.data[i]);
        }
    }
}

/**
 * @brief Iterate through the payload of the packet and print the counter of the packet
 *
 * @param packet
 */
void printDataPacket(AppPacket<dataPacket>* packet) {
    Serial.printf("Packet arrived from %X with size %d\n", packet->src, packet->payloadSize);

    //Get the payload to iterate through it
    dataPacket* dPacket = packet->payload;
    size_t payloadLength = packet->getPayloadLength();
    uint16_t src = packet->src;

    for (size_t i = 0; i < payloadLength; i++) {
        //Print the packet
        printPacket(dPacket[i], src);
    }
}

/**
 * @brief Function that process the received packets
 *
 */
void processReceivedPackets(void*) {
    for (;;) {
        /* Wait for the notification of processReceivedPackets and enter blocking */
        ulTaskNotifyTake(pdPASS, portMAX_DELAY);
        led_Flash(1, 100); //one quick LED flashes to indicate a packet has arrived

        //Iterate through all the packets inside the Received User Packets Queue
        while (radio.getReceivedQueueSize() > 0) {
            Serial.println("ReceivedUserData_TaskHandle notify received");
            Serial.printf("Queue receiveUserData size: %d\n", radio.getReceivedQueueSize());

            //Get the first element inside the Received User Packets Queue
            AppPacket<dataPacket>* packet = radio.getNextAppPacket<dataPacket>();

            //Print the data packet
            printDataPacket(packet);

            #ifdef SHOULD_USE_LMIC
                Serial.println("Sending packet to LMIC");
                radio.standby(); //Standby the LoraMesher

                isUsingLmic = true;

                
                dataPacket* dPacket = packet->payload;
                
                uint32_t x = dPacket->data.counter;

                uint8_t bytes[8];

                bytes[0] = (x >> 24) & 0xFF;
                bytes[1] = (x >> 16) & 0xFF;
                bytes[2] = (x >> 8) & 0xFF;
                bytes[3] = (x) & 0xFF;
                bytes[4] = '\0'; 
                uint16_t src = packet->src;
                bytes[5] = (src >> 8) & 0xFF;
                bytes[6] = (src) & 0xFF;
                bytes[7] = '\0';

                Serial.println(bytes[0]);
                Serial.println(bytes[1]);
                Serial.println(bytes[2]);
                Serial.println(bytes[3]);
                
                do_send(bytes, sizeof(bytes));
                
                delay(1000);

            
            #endif //SHOULD_USE_LMIC

            //Delete the packet when used. It is very important to call this function to release the memory of the packet.
            radio.deletePacket(packet);
        }
    }
}

TaskHandle_t receiveLoRaMessage_Handle = NULL;

/**
 * @brief Create a Receive Messages Task and add it to the LoRaMesher
 *
 */
void createReceiveMessages() {
    int res = xTaskCreate(
        processReceivedPackets,
        "Receive App Task",
        4096,
        (void*) 1,
        2,
        &receiveLoRaMessage_Handle);
    if (res != pdPASS) {
        Serial.printf("Error: Receive App Task creation gave error: %d\n", res);
    }

    radio.setReceiveAppDataTaskHandle(receiveLoRaMessage_Handle);
}


/**
 * @brief Initialize LoRaMesher
 *
 */
void setupLoraMesher() {
    // Example on how to change the module. See LoraMesherConfig to see all the configurable parameters.
    LoraMesher::LoraMesherConfig config;
    #ifdef HELTEC
        config.module = LoraMesher::LoraModules::SX1262_MOD;
    #endif

    #ifndef HELTEC
        config.module = LoraMesher::LoraModules::SX1276_MOD;
    #endif

    config.loraCs = CS;
    config.loraRst = RST;
    config.loraIrq = IRQ;
    config.loraIo1 = IO1;

    #ifdef HELTEC
        SPI.begin(9, 11, 10, CS); //Initialize SPI with the correct pins
        config.spi = &SPI;
    #endif

    //Init the loramesher with a processReceivedPackets function
    radio.begin(config);

    //Create the receive task and add it to the LoRaMesher
    createReceiveMessages();

    //Start LoRaMesher
    #ifndef SHOULD_USE_LMIC
        radio.start();
    #endif //SHOULD_USE_LMIC

    #ifdef SHOULD_USE_LMIC
        radio.addGatewayRole();
    #endif

    Serial.println("Lora initialized");
}




void setup() {
    Serial.begin(115200);

    Serial.println("initBoard");
    pinMode(BOARD_LED, OUTPUT); //setup pin as output for indicator LED
    led_Flash(2, 125);          //two quick LED flashes to indicate program start
    setupLoraMesher();
    static uint8_t initData[] = "Hello";
    #ifdef SHOULD_USE_LMIC
        SPI.begin(9, 11, 10, CS); //Initialize SPI with the correct pins
        os_init(); //Initialize the LMIC
        LMIC_reset(); //Reset the LMIC
        do_send(initData, sizeof(initData)); //Send the packet to LMIC
    #endif //SHOULD_USE_LMIC
}


void loop() {
    #ifdef SHOULD_USE_LMIC
        if (!hasJoined || isUsingLmic || !hasFoundDR)
        {
            os_runloop_once();
            return;
        }
    #endif
    #ifndef SHOULD_USE_LMIC
        for (;;) {
            Serial.printf("Send packet %d\n", dataCounter);

            helloPacket->data.counter = dataCounter++;
            helloPacket->type = 0;
            
            RouteNode* dst = radio.getClosestGateway(); //Get the closest gateway to send the packet
            uint16_t addr;
            if (dst != nullptr)
                addr = dst->networkNode.address; //Get the address of the destination node
            else 
                addr = BROADCAST_ADDR;
            Serial.printf("Sending packet to %X\n", addr);

            //Create packet and send it.
            // radio.createPacketAndSend(BROADCAST_ADDR, helloPacket, 1);
            radio.createPacketAndSend(addr, helloPacket, 1);

            //Wait 20 seconds to send the next packet
            vTaskDelay(20000 / portTICK_PERIOD_MS);
        }
    #endif
}
#endif // 0