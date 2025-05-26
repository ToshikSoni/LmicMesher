// project-specific definitions for LMIC library
#pragma once

// Select the India region
#define CFG_in866 1

// Select the radio type for Heltec LoRa32 v3 (SX1262)
#define CFG_sx1262_radio 1

// Board definition
#define ARDUINO_heltec_wifi_lora_32_V3 1

// Disable features we don't need
#define DISABLE_PING 1
#define DISABLE_BEACONS 1

// Enable interrupts for better performance
#define LMIC_USE_INTERRUPTS 1

// Don't define any other CFG_* region flags
// Only CFG_in866 should be defined