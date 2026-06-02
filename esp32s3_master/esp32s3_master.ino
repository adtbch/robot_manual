#include "robot_config.h"

// Paket terakhir untuk debugging/manual processing.
static EspNowControlPacket gLastRxPacket = {};

void setup() {
  Serial.begin(115200);
  SetupMotors();
  setupServos();
  setupEncoders();
  setupLimits();
  
  // Step 1: Homing process
  Serial.println("Starting homing...");
  while (!setHoming()) {
    Serial.println("Homing in progress...");
    delay(100);
  }
  Serial.println("Homing complete!");
  
  // Step 2: Reset encoder counts setelah homing
  resetEncoderCount(0);  // Reset encoder motor X
  resetEncoderCount(1);  // Reset encoder motor Z
  Serial.println("Encoder counts reset to 0");
  
  // Step 3: Move to center position
  Serial.println("Moving to center position...");
  while (!moveToCenter()) {
    long posX = getEncoderCount(0);
    long posZ = getEncoderCount(1);
    Serial.printf("Moving... X: %ld/%ld, Z: %ld/%ld\n", 
                  posX, CENTER_POSITION_X, posZ, CENTER_POSITION_Z);  
  }
  Serial.println("Arm at center position!");
  
  motorStopAll(); // Pastikan motor berhenti
  
  // Step 4: Initialize ESP-NOW
  bool espNowReady = espNowControlInit();
  Serial.printf("ESP-NOW control: %s\n", espNowReady ? "READY" : "ERROR");
  
  // Step 5: Initialize Serial Command Handler
  setupSerialCommand();
  
  Serial.println("Robot ready!");
}

void loop() {
  // 1) Serial command handler (motor & servo control via USB)
  serialCommandTick();

  // 2) ESP-NOW control receiver service
  espNowControlTick();

  // 3) Example: consume ESP-NOW packet (optional, for debugging)
  if (espNowControlReadPacket(gLastRxPacket)) {
    Serial.printf("RX seq=%u x=%d y=%d w=%d connected=%u\n",
                  gLastRxPacket.seq,
                  gLastRxPacket.x,
                  gLastRxPacket.y,
                  gLastRxPacket.w,
                  gLastRxPacket.connected);
  }
}