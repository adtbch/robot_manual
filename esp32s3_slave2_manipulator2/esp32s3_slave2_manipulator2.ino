/*
 * File: esp32s3_slave2_manipulator2.ino (MASTER FILE)
 * Deskripsi: Sistem kontrol arm box - manipulator 2
 * Hardware: ESP32-S3
 *
 * Author: Robot Manual Team
 * Date: 3 Juni 2026
 */

#include "armbox_config.h"

void setup() {
  Serial.begin(115200);

  Serial.println("\n========================================");
  Serial.println("  SISTEM ARM BOX - MANIPULATOR 2");
  Serial.println("  Versi: 5.0 (Master Pattern)");
  Serial.println("========================================\n");

  // Step 1: Setup all hardware
  Serial.println("Memulai inisialisasi...");
  SetupMotors();
  setupServos();
  setupEncoders();
  setupRelays();
  setupLimits();
  initUART();
  setupSerialCommand();

  // Step 2: Homing process
  Serial.println("\nStarting homing...");
  while (!setHoming()) {
    setServoAngle(0, servoHomeAngle);
    Serial.println("Homing in progress...");
  }
  Serial.println("Homing complete!");
  
  // Step 3: Reset encoder counts setelah homing
  for (size_t i = 0; i < encoders.size(); i++) {
    resetEncoderCount(i);
  }
  Serial.println("Encoder counts reset to 0");

  motorStopAll();
  Serial.println("\n========================================");
  Serial.println("Robot ready!");
  Serial.println("========================================\n");
}

// Flag untuk continuous encoder monitor (toggle via serial: monitor)
bool encoderMonitor = false;

void loop() {
  // 1) Serial command handler (USB Serial)
  serialCommandTick();

  // 2) UART command handler (from master ESP32)
  readUART();
  processUARTCommand();

  // 3) Continuous encoder monitor (periodik 500ms)
  static uint32_t lastEncPrint = 0;
  if (encoderMonitor && millis() - lastEncPrint >= 500) {
    lastEncPrint = millis();
    printAllEncoders();
  }
}
