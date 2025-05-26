/*
 * LMIC Mesher Sequential for Heltec LoRa32 V3.2 HTIT-WB32LAF
 * User: ToshikSoni
 * Date: 2025-05-26 09:46:51 UTC
 * 
 * Sequential operation: LoRaMesher for mesh, then LMIC for LoRaWAN
 * Avoids SPI conflicts by using time-division approach
 */

#include <Arduino.h>
#include <lmic.h>
#include <hal/hal.h>
#include <SPI.h>
#include "LoraMesher.h"

// Remove conflicting LED_BUILTIN definition
#ifdef LED_BUILTIN
#undef LED_BUILTIN
#endif
#define BOARD_LED 35

// LoRaWAN Keys for ChirpStack OTAA
static const u1_t PROGMEM APPEUI[8]  = { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };
static const u1_t PROGMEM DEVEUI[8]  = { 0xd8, 0x5a, 0x10, 0x66, 0xd1, 0xc3, 0x0d, 0x3f }; 
static const u1_t PROGMEM APPKEY[16] = { 0x7a, 0x39, 0xb4, 0xa1, 0x5d, 0xd0, 0x7f, 0xe9, 0x54, 0x3c, 0x03, 0x57, 0xc3, 0xb8, 0xe9, 0x5e };

void os_getArtEui(u1_t* buf) { memcpy_P(buf, APPEUI, 8); }
void os_getDevEui(u1_t* buf) { memcpy_P(buf, DEVEUI, 8); }
void os_getDevKey(u1_t* buf) { memcpy_P(buf, APPKEY, 16); }

// Heltec LoRa32 V3.2 correct pin mapping
#define VCC_ENABLE 37
#define LORA_CS 8
#define LORA_RST 12
#define LORA_IRQ 14
#define LORA_BUSY 13

// LMIC Pin mapping for Heltec V3.2
const lmic_pinmap lmic_pins = {
    .nss = LORA_CS,
    .rxtx = LMIC_UNUSED_PIN,
    .rst = LORA_RST,
    .dio = {LORA_IRQ, LMIC_UNUSED_PIN, LMIC_UNUSED_PIN},
    .pConfig = nullptr,
};

// Operation modes
enum OperationMode {
    MODE_MESH,
    MODE_LORAWAN,
    MODE_SWITCHING
};

OperationMode currentMode = MODE_MESH;

// LoRaMesher instance
LoraMesher& meshRadio = LoraMesher::getInstance();

// Data structures
uint32_t dataCounter = 0;
struct meshPacket {
    uint32_t counter = 0;
    uint16_t nodeId = 0;
    int8_t rssi = 0;
    uint8_t type = 0;
};

struct lorawanPacket {
    uint32_t meshCounter = 0;
    uint16_t sourceNode = 0;
    int8_t meshRssi = 0;
    uint8_t meshNodes = 0;
    char status[16] = "MESH_OK";
};

meshPacket* helloPacket = new meshPacket;
lorawanPacket gatewayData;

// Timing variables
static osjob_t sendjob;
const unsigned MESH_DURATION = 60000;  // 60 seconds for mesh
const unsigned LORAWAN_DURATION = 10000; // 10 seconds for LoRaWAN
unsigned long modeStartTime = 0;
unsigned long lastMeshSend = 0;
const unsigned MESH_INTERVAL = 10000;
bool hasJoined = false;
bool meshStarted = false;
bool lorawanInitialized = false;

// Task handles
TaskHandle_t receiveLoRaMessage_Handle = NULL;

// LED control
void led_Flash(uint16_t flashes, uint16_t delaymS) {
    for (uint16_t i = 0; i < flashes; i++) {
        digitalWrite(BOARD_LED, HIGH);
        delay(delaymS);
        digitalWrite(BOARD_LED, LOW);
        delay(delaymS);
    }
}

// Print helper functions
void printHex2(unsigned v) {
    v &= 0xff;
    if (v < 16) Serial.print('0');
    Serial.print(v, HEX);
}

void printMeshPacket(meshPacket data, uint16_t src) {
    Serial.printf("MESH DATA: Counter=%d from Node=%04X (RSSI=%d)\n", 
                  data.counter, src, data.rssi);
}

void printMeshDataPacket(AppPacket<meshPacket>* packet) {
    Serial.printf("Mesh packet from %04X, size %d bytes\n", 
                  packet->src, packet->payloadSize);
    
    meshPacket* dPacket = packet->payload;
    size_t payloadLength = packet->getPayloadLength();
    
    for (size_t i = 0; i < payloadLength; i++) {
        printMeshPacket(dPacket[i], packet->src);
        
        // Update gateway data with latest mesh info
        gatewayData.meshCounter = dPacket[i].counter;
        gatewayData.sourceNode = packet->src;
        gatewayData.meshRssi = dPacket[i].rssi;
        gatewayData.meshNodes = meshRadio.routingTableSize() + 1;
        strncpy(gatewayData.status, "MESH_RX", sizeof(gatewayData.status));
    }
}

// Mesh packet processing task
void processReceivedPackets(void*) {
    for (;;) {
        ulTaskNotifyTake(pdPASS, portMAX_DELAY);
        led_Flash(1, 50);
        
        while (meshRadio.getReceivedQueueSize() > 0) {
            Serial.println("Mesh packet received");
            
            AppPacket<meshPacket>* packet = meshRadio.getNextAppPacket<meshPacket>();
            printMeshDataPacket(packet);
            meshRadio.deletePacket(packet);
        }
    }
}

void createReceiveMessages() {
    int res = xTaskCreate(
        processReceivedPackets,
        "Mesh Receive Task",
        4096,
        (void*) 1,
        2,
        &receiveLoRaMessage_Handle);
    
    if (res != pdPASS) {
        Serial.printf("Error: Mesh receive task creation failed: %d\n", res);
    }
}

// Stop LoRaMesher safely
void stopMesh() {
    if (meshStarted) {
        Serial.println("Stopping LoRaMesher...");
        meshRadio.standby();
        vTaskDelete(receiveLoRaMessage_Handle);
        receiveLoRaMessage_Handle = NULL;
        meshStarted = false;
        
        // Reset SPI
        SPI.end();
        delay(100);
        
        Serial.println("LoRaMesher stopped");
    }
}

// Start LoRaMesher
void startMesh() {
    if (!meshStarted) {
        Serial.println("Starting LoRaMesher...");
        
        // Initialize SPI for mesh
        SPI.begin(9, 11, 10, LORA_CS);
        delay(100);
        
        LoraMesher::LoraMesherConfig config = LoraMesher::LoraMesherConfig();
        
        config.loraCs = LORA_CS;
        config.loraRst = LORA_RST;
        config.loraIrq = LORA_IRQ;
        config.loraIo1 = LMIC_UNUSED_PIN;
        
        config.module = LoraMesher::LoraModules::SX1262_MOD;
        config.spi = &SPI;
        
        config.freq = 915.0;
        config.sf = 7;
        config.bw = 125.0;
        config.cr = 5;
        config.power = 14;
        config.syncWord = 0x12;
        config.preambleLength = 8;
        
        meshRadio.begin(config);
        createReceiveMessages();
        meshRadio.setReceiveAppDataTaskHandle(receiveLoRaMessage_Handle);
        meshRadio.start();
        
        meshStarted = true;
        Serial.println("LoRaMesher started");
    }
}

// LoRaWAN functions
void do_send(osjob_t* j) {
    if (LMIC.opmode & OP_TXRXPEND) {
        Serial.println(F("OP_TXRXPEND, not sending"));
    } else {
        gatewayData.meshCounter = dataCounter;
        gatewayData.sourceNode = 0xAFC0; // Use fixed address when mesh is off
        gatewayData.meshNodes = 1;
        
        Serial.printf("Sending to gateway: Counter=%d, Nodes=%d, Status=%s\n",
                     gatewayData.meshCounter, gatewayData.meshNodes, gatewayData.status);
        
        LMIC_setAdrMode(0);
        LMIC_setDrTxpow(DR_SF8, 14);
        
        LMIC_setTxData2(1, (uint8_t*)&gatewayData, sizeof(gatewayData), 0);
        Serial.println(F("LoRaWAN packet queued"));
        
        led_Flash(2, 100);
    }
}

void onEvent(ev_t ev) {
    Serial.print(os_getTime());
    Serial.print(": ");
    
    switch(ev) {
        case EV_JOINING:
            Serial.println(F("EV_JOINING"));
            strncpy(gatewayData.status, "JOINING", sizeof(gatewayData.status));
            break;
            
        case EV_JOINED:
            Serial.println(F("EV_JOINED"));
            LMIC_setLinkCheckMode(0);
            hasJoined = true;
            strncpy(gatewayData.status, "JOINED", sizeof(gatewayData.status));
            led_Flash(3, 200);
            
            // Send first packet
            do_send(&sendjob);
            break;
            
        case EV_JOIN_FAILED:
            Serial.println(F("EV_JOIN_FAILED"));
            strncpy(gatewayData.status, "JOIN_FAIL", sizeof(gatewayData.status));
            break;
            
        case EV_TXCOMPLETE:
            Serial.println(F("EV_TXCOMPLETE"));
            if (LMIC.txrxFlags & TXRX_ACK)
                Serial.println(F("Received ack"));
            if (LMIC.dataLen) {
                Serial.printf("Received %d bytes\n", LMIC.dataLen);
            }
            break;
            
        default:
            Serial.printf("Event: %u\n", (unsigned) ev);
            break;
    }
}

void startLoRaWAN() {
    if (!lorawanInitialized) {
        Serial.println("Starting LoRaWAN...");
        
        // Reset SPI for LoRaWAN
        SPI.end();
        delay(100);
        SPI.begin();
        delay(100);
        
        os_init();
        LMIC_reset();
        
        LMIC_selectSubBand(1);
        LMIC_setAdrMode(0);
        LMIC_setDrTxpow(DR_SF8, 14);
        
        if (!hasJoined) {
            LMIC_startJoining();
        } else {
            do_send(&sendjob);
        }
        
        lorawanInitialized = true;
        Serial.println("LoRaWAN started");
    }
}

void stopLoRaWAN() {
    if (lorawanInitialized) {
        Serial.println("Stopping LoRaWAN...");
        LMIC_reset();
        SPI.end();
        delay(100);
        lorawanInitialized = false;
        Serial.println("LoRaWAN stopped");
    }
}

void switchMode() {
    currentMode = MODE_SWITCHING;
    Serial.printf("=== Mode Switch at %lu ms ===\n", millis());
    
    if (meshStarted) {
        stopMesh();
        delay(500);
        startLoRaWAN();
        currentMode = MODE_LORAWAN;
        Serial.println("Switched to LoRaWAN mode");
    } else if (lorawanInitialized) {
        stopLoRaWAN();
        delay(500);
        startMesh();
        currentMode = MODE_MESH;
        Serial.println("Switched to Mesh mode");
    }
    
    modeStartTime = millis();
}

void setup() {
    Serial.begin(115200);
    delay(2000);
    
    Serial.println("=== LMIC Mesher Sequential for Heltec LoRa32 V3.2 ===");
    Serial.println("User: ToshikSoni");
    Serial.println("Date: 2025-05-26 09:46:51 UTC");
    Serial.println("Sequential Mode: Mesh -> LoRaWAN -> Mesh");
    
    #ifdef VCC_ENABLE
    pinMode(VCC_ENABLE, OUTPUT);
    digitalWrite(VCC_ENABLE, HIGH);
    delay(100);
    Serial.println("VCC enabled");
    #endif
    
    pinMode(BOARD_LED, OUTPUT);
    digitalWrite(BOARD_LED, LOW);
    led_Flash(2, 125);
    
    // Start with mesh mode
    Serial.println("=== Starting in Mesh Mode ===");
    startMesh();
    modeStartTime = millis();
}

void loop() {
    unsigned long currentTime = millis();
    
    // Handle mode switching
    if (currentMode == MODE_MESH && (currentTime - modeStartTime >= MESH_DURATION)) {
        switchMode(); // Switch to LoRaWAN
    } else if (currentMode == MODE_LORAWAN && (currentTime - modeStartTime >= LORAWAN_DURATION)) {
        switchMode(); // Switch to Mesh
    }
    
    // Handle current mode operations
    if (currentMode == MODE_MESH && meshStarted) {
        // Mesh operations
        if (currentTime - lastMeshSend >= MESH_INTERVAL) {
            lastMeshSend = currentTime;
            
            helloPacket->counter = dataCounter++;
            helloPacket->nodeId = meshRadio.getLocalAddress();
            helloPacket->rssi = 0;
            helloPacket->type = 0;
            
            Serial.printf("Mesh TX: packet %d from %04X\n", 
                         helloPacket->counter, helloPacket->nodeId);
            
            meshRadio.createPacketAndSend(BROADCAST_ADDR, helloPacket, 1);
            strncpy(gatewayData.status, "MESH_TX", sizeof(gatewayData.status));
            led_Flash(1, 100);
        }
    } else if (currentMode == MODE_LORAWAN && lorawanInitialized) {
        // LoRaWAN operations
        os_runloop_once();
    }
    
    delay(10);
}