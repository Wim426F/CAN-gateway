#include <Arduino.h>
#include <FlexCAN_T4.h>
#include <math.h>
#include <Snooze.h>

// CAN settings
FlexCAN_T4<CAN1, RX_SIZE_256, TX_SIZE_16> Can1; // BMW-E90 side
FlexCAN_T4<CAN2, RX_SIZE_256, TX_SIZE_16> Can2; // VCU side
CAN_message_t rxMsg, txMsg;

// Stored latest 0x1A0 message from Can1
CAN_message_t last1A0Msg;

// Timing variables
const unsigned long LED_INTERVAL = 100;
unsigned long lastLedTime = 0;

const unsigned long X1A0_SEND_INTERVAL = 5; //ms
unsigned long last1a0SendTime = 0;

SnoozeDigital digital; // Driver for digital pin wake
SnoozeBlock config(digital); // Install driver

unsigned long lastCan1Time = 0;
const unsigned long SLEEP_TIMEOUT = 1200000; // 20 min; test with shorter like 60000 (1 min) initially

const int CAN1_RX_PIN = 23; // Default for Teensy 4.1 CAN1 RX (adjust if different)

void initCanFilters() {
  // Can1 filters (BMW side)
  Can1.setMBFilter(MB0, 0x130); // E90 CAS
  Can1.setMBFilter(MB1, 0x2FC); // E90 Enclosure status
  Can1.setMBFilter(MB2, 0x480); // Network Management
  Can1.setMBFilter(MB3, 0x1A0); // Speed
  Can1.setMBFilter(MB4, 0x194); // Cruise control
  Can1.setMBFilter(MB5, 0x19E); // Brake DTC status
  Can1.setMBFilter(MB6, 0x0C4); // Steering wheel angle
  Can1.setMBFilter(MB7, 0x0CE); // Wheel speeds
  Can1.setMBFilter(MB8, 0x0C8); // Steering wheel angle slow
  Can1.setMBFilter(MB9, 0x0B6); // Dynamic cruise controle torque demand
  Can1.setMBFilter(MB10, 0x5A9); // Services DSC
  Can1.setMBFilter(MB11, 0x1B4); // Vehicle speed
  Can1.setMBFilter(MB12, 0x1A1); // Vehicle speed
  Can1.setMBFilter(MB13, 0x1A6); // Vehicle speed for instrument cluster
  Can1.setMBFilter(MB14, 0x1D6); // Steering wheel buttons
  Can1.setMBFilter(MB15, 0x26E); // Ignition status
  Can1.setMBFilter(MB16, 0x2CA); // Outside Temperature
  Can1.setMBFilter(MB17, 0x2D6); // AC compressor on/off request 
  Can1.setMBFilter(MB18, 0x2E6); // Climate control settings
  Can1.setMBFilter(MB19, 0x2EA); // Climate control settings passenger
  Can1.setMBFilter(MB20, 0x242); // Climate front status
  Can1.setMBFilter(MB21, 0x200); // Cruise control status
  Can1.setMBFilter(MB22, 0x1A0); // Vehicle speed for EPS

  // Can2 filters (VCU side)
  Can2.setMBFilter(MB0, 0x0AA); // ABS/DSC info
  Can2.setMBFilter(MB1, 0x0A8); // Brake pedal status
  Can2.setMBFilter(MB2, 0X0A9); // Counter
  Can2.setMBFilter(MB3, 0X0BA); // Gear
  Can2.setMBFilter(MB4, 0X1D0); // Dashboard Engine info
  Can2.setMBFilter(MB5, 0x332); // Instrument cluster init
  Can2.setMBFilter(MB6, 0x1D2); // Shift position
  Can2.setMBFilter(MB7, 0x592); // Error lights
}

void setup() {
  // Initialize CAN buses
  Can1.begin(); // BMW-E90 side  
  Can1.setBaudRate(500000); 
  Can2.begin(); // VCU side
  Can2.setBaudRate(500000);
  initCanFilters(); // Set all filters

  pinMode(LED_BUILTIN, OUTPUT);

  // Initialize last1A0Msg with static default values
  last1A0Msg.id = 0x1A0;
  last1A0Msg.len = 8;
  memset(last1A0Msg.buf, 0, 8);
  last1A0Msg.buf[0] = 0x6F;

  // Configure wake on CAN RX pin
  digital.pinMode(CAN1_RX_PIN, INPUT_PULLUP, FALLING); // pin, mode, type
}

void loop() {
  if (Can1.read(rxMsg)) {
    lastCan1Time = millis(); // Any message resets
    if (rxMsg.id == 0x1A0) {
      last1A0Msg = rxMsg;
    } else {
      Can2.write(rxMsg);
    }
  }
  
  if (Can2.read(rxMsg)) {
    Can1.write(rxMsg);
  }

  if (millis() - lastLedTime >= LED_INTERVAL) {
    lastLedTime = millis();
    digitalToggle(LED_BUILTIN);
  }

  if (millis() - last1a0SendTime >= X1A0_SEND_INTERVAL) {
    last1a0SendTime = millis();
    txMsg = last1A0Msg;

    // Extract raw VehicleSpeed (12-bit, little-endian, unsigned)
    uint16_t raw = ((uint16_t)(txMsg.buf[1] & 0x0F) << 8) | txMsg.buf[0];

    // Calculate physical speed
    float speed_kph = raw * 0.103f;

    // Apply minimum limit
    if (speed_kph < 10.0f) {
      speed_kph = 10.0f;
    }

    // Calculate new raw value
    uint16_t new_raw = (uint16_t)roundf(speed_kph / 0.103f);

    // Pack back into buffer, modifying only the VehicleSpeed bits
    txMsg.buf[0] = (uint8_t)(new_raw & 0xFF);
    txMsg.buf[1] = (txMsg.buf[1] & 0xF0) | (uint8_t)((new_raw >> 8) & 0x0F);

    Can2.write(txMsg);
  }

  if (millis() - lastCan1Time > SLEEP_TIMEOUT) {
    enterLowPower();
  }
}

void enterLowPower() {
  // Prep: Stop LED
  digitalWrite(LED_BUILTIN, LOW); // Off

  // Sleep: Wake on CAN1 RX
  Snooze.deepSleep(config);

  // Post-wake: Re-init CAN buses and filters
  Can1.begin(); Can1.setBaudRate(500000); 
  Can2.begin(); Can2.setBaudRate(500000);
  initCanFilters(); // Re-set all filters
  lastCan1Time = millis(); // Reset
}