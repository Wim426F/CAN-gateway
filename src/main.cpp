#include <Arduino.h>
#include <FlexCAN_T4.h>

// CAN settings
FlexCAN_T4<CAN1, RX_SIZE_256, TX_SIZE_16> Can1; // BMW-E90 side
FlexCAN_T4<CAN2, RX_SIZE_256, TX_SIZE_16> Can2; // VCU side
CAN_message_t rxMsg, txMsg;

// Timing variables
const unsigned long CAN_SEND_INTERVAL = 100;
unsigned long lastCanSendTime = 0;

void setup() {
  // Initialize CAN buses
  Can1.begin(); // BMW-E90 side  
  Can1.setBaudRate(500000); 
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

  Can2.begin(); // VCU side
  Can2.setBaudRate(500000);
  Can2.setMBFilter(MB0, 0x0AA); // ABS/DSC info
  Can2.setMBFilter(MB1, 0x0A8); // Brake pedal status
  Can2.setMBFilter(MB2, 0X0A9); // Counter
  Can2.setMBFilter(MB3, 0X0BA); // Gear
  Can2.setMBFilter(MB4, 0X1D0); // Dashboard Engine info
  Can2.setMBFilter(MB5, 0x332); // Instrument cluster init
  Can2.setMBFilter(MB6, 0x1D2); // Shift position
  Can2.setMBFilter(MB7, 0x592); // Error lights

  pinMode(LED_BUILTIN, OUTPUT);
}

void loop() {
  if (Can1.read(rxMsg)) {
    Can2.write(rxMsg);
  }
  
  if (Can2.read(rxMsg)) {
    Can1.write(rxMsg);
  }

  if (millis() - lastCanSendTime >= CAN_SEND_INTERVAL) {
    lastCanSendTime = millis();
    digitalToggle(LED_BUILTIN);
  }
}