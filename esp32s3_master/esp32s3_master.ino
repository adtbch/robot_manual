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
    delay(100);
  }
  Serial.println("Arm at center position!");
  
  motorStopAll(); // Pastikan motor berhenti
  
  // Step 4: Initialize ESP-NOW
  bool espNowReady = espNowControlInit();
  Serial.printf("ESP-NOW control: %s\n", espNowReady ? "READY" : "ERROR");
  Serial.println("Robot ready!");
}

void loop() {
  // setServoAngle(1, 90); // Contoh: set servo 1 ke posisi 90 derajat

   // 4) Mode normal: jalankan service receiver ESP-NOW.
  espNowControlTick();

  // 5) Contoh konsumsi paket mentah ESP-NOW.
  //    Bagian ini aman untuk diganti dengan logika aplikasi kamu nanti.
  if (espNowControlReadPacket(gLastRxPacket)) {
    Serial.printf("RX seq=%u x=%d y=%d w=%d connected=%u\n",
                  gLastRxPacket.seq,
                  gLastRxPacket.x,
                  gLastRxPacket.y,
                  gLastRxPacket.w,
                  gLastRxPacket.connected);
  }
}