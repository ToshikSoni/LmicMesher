#include <Arduino.h>
#include "LoraMesher.h"
#include <Wire.h>
#define BOARD_LED 35 // Built-in LED pin for Heltec v3.2
#define LED_ON HIGH
#define LED_OFF LOW
#define CS 8   // LoRa CS pin
#define RST 12 // LoRa Reset pin
#define IRQ 14 // LoRa DIO0/IRQ pin
#define IO1 13 // LoRa DIO1 pin
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
    snprintf(helloPacket->message, sizeof(helloPacket->message),
             "Hello #%d", dataCounter);
    Serial.println("Broadcasting discovery message...");
    Serial.printf("Message: %s\n", helloPacket->message);
    radio.createPacketAndSend(46064, helloPacket, sizeof(dataPacket));
    Serial.println("Broadcast sent!");
}
void sendPacketsToDiscoveredDevices()
{
    if (radio.routingTableSize() == 0)
    {
        Serial.println("No devices to send to - broadcasting hello");
        sendBroadcastMessage();
        return;
    }
    NetworkNode *nodes = RoutingTableService::getAllNetworkNodes();
    if (nodes != nullptr)
    {
        for (size_t i = 0; i < radio.routingTableSize(); i++)
        {
            uint16_t targetAddress = nodes[i].address;
            Serial.printf("\n=== SENDING TO DEVICE 0x%04X ===\n", targetAddress);
            helloPacket->messageId = ++dataCounter;
            helloPacket->timestamp = millis();
            helloPacket->sourceAddress = radio.getLocalAddress();
            snprintf(helloPacket->message, sizeof(helloPacket->message),
                     "SF12-DR5 from 0x%04X #%d", radio.getLocalAddress(), dataCounter);
            Serial.printf("Message: %s\n", helloPacket->message);
            Serial.printf("Hops required: %d\n", nodes[i].metric);
            led_Flash(2, 100); // Flash LED to indicate sending
            radio.createPacketAndSend(46064, helloPacket, 1);
            Serial.println("Packet sent via mesh!");
            delay(2000); // Small delay between sends to respect duty cycle
        }
        delete[] nodes;
    }
}

void printDataPacket(AppPacket<dataPacket> *packet)
{
    Serial.printf("=== PACKET RECEIVED ===\n");
    Serial.printf("From: 0x%04X\n", packet->src);
    Serial.printf("Size: %d bytes\n", packet->payloadSize);
    dataPacket *dPacket = packet->payload;
    size_t payloadLength = packet->getPayloadLength();
    for (size_t i = 0; i < payloadLength; i++)
    {
        Serial.printf("Message ID: %d\n", dPacket[i].messageId);
        Serial.printf("Source: 0x%04X\n", dPacket[i].sourceAddress);
        Serial.printf("Message: %s\n", dPacket[i].message);
        uint32_t transitTime = millis() - dPacket[i].timestamp;
        Serial.printf("Transit time: %d ms\n", transitTime);
        Serial.printf("Expected SF12 air time: ~1.8 seconds\n");
    }
}
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
            AppPacket<dataPacket> *packet = radio.getNextAppPacket<dataPacket>();
            printDataPacket(packet);
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
void setupLoraMesher()
{
    Serial.println("Setting up LoRaMesher for India 865MHz SF12 DR5...");
    LoraMesher::LoraMesherConfig config;
    config.module = LoraMesher::LoraModules::SX1262_MOD;
    config.loraCs = CS;
    config.loraRst = RST;
    config.loraIrq = IRQ;
    config.loraIo1 = IO1;
    config.freq = 865.2;       // 865.2 MHz - Indian LoRa frequency
    config.bw = 125.0;         // 125 kHz bandwidth (standard for DR5)
    config.sf = 8;             // SF12 (Spreading Factor 7 for DR5)
    config.cr = 5;             // Coding Rate 4/5 (denominator 5)
    config.power = 14;         // 14 dBm output power (adjust per Indian regulations)
    config.preambleLength = 8; // 8 preamble symbols
    config.syncWord = 0x12;    // Sync word for mesh network identification
    SPI.begin(9, 11, 10, CS);
    config.spi = &SPI;
    Serial.println("=== LoRa Configuration ===");
    Serial.printf("Frequency: %.1f MHz\n", config.freq);
    Serial.printf("Bandwidth: %.1f kHz\n", config.bw);
    Serial.printf("Spreading Factor: SF%d\n", config.sf);
    Serial.printf("Coding Rate: 4/%d\n", config.cr);
    Serial.printf("Output Power: %d dBm\n", config.power);
    Serial.printf("Data Rate: DR5 (SF12BW125)\n");
    Serial.println("Region: India 865MHz band");
    radio.begin(config);
    createReceiveMessages();
    radio.start();
    Serial.println("LoRaMesher initialized for Indian region!");
}
void setup()
{
    Serial.begin(115200);
    delay(2000);
    pinMode(BOARD_LED, OUTPUT);
    led_Flash(8, 100); // 8 quick LED flashes to indicate program start
    setupLoraMesher();
    pinMode(Vext, OUTPUT);
    digitalWrite(Vext, LOW);
    delay(100);
    uint16_t myAddress = radio.getLocalAddress();
    Serial.println("\n=== DEVICE INFORMATION ===");
    Serial.printf("My Device Address: 0x%04X (%d)\n", myAddress, myAddress);
    delay(2000);
}
void loop()
{
    Serial.printf("\n=== SENDING PACKET #%d ===\n", dataCounter + 1);
    sendBroadcastMessage();
    sendPacketsToDiscoveredDevices();
    Serial.println("Waiting 15 seconds for next transmission...\n");
    delay(15000);
}