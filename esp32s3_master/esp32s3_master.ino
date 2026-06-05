#include <WiFi.h>
#include "robot_config.h"

// Paket terakhir untuk debugging/manual processing.
static ControlPacket gLastRxPacket = {};

// Edge detection untuk tombol Options (mode toggle)
static bool optionsPrev = false;

// =====================================================================
//  SHARED: consume packet + print throttled (tiap 20 paket)
// =====================================================================

static void consumePacket(const char *source, ControlPacket &pkt) {
  static uint32_t rxPrintCounter = 0;
  rxPrintCounter++;
  if (rxPrintCounter % 50 == 1) {
    Serial.printf("[%s] seq=%u btn=0x%08lX conn=%d\n",
                  source, pkt.seq, (unsigned long)pkt.buttons, pkt.connected);
  }

  // Deteksi edge tombol Options → toggle mode
  bool optionsNow = (pkt.buttons & BTN_OPTIONS) != 0;
  if (optionsNow && !optionsPrev) {
    mode_toggle();
  }
  optionsPrev = optionsNow;
}

void setup() {
  Serial.begin(115200);
  Serial.print("ESP32 MAC Address for ESP-NOW: ");
  Serial.println(WiFi.macAddress());

  SetupMotors();
  setupServos();
  setupEncoders();
  setupLimits();
  
  Serial.println("Starting homing...");
  while (!setHoming()) {
    Serial.println("Homing in progress...");
    delay(100);
  }
  Serial.println("Homing complete!");
  
  resetEncoderCount(0);
  resetEncoderCount(1);
  Serial.println("Encoder counts reset to 0");
  
  Serial.println("Moving to center position...");
  while (!moveToCenter()) {
    long posX = getEncoderCount(0);
    long posZ = getEncoderCount(1);
    Serial.printf("Moving... X: %ld/%ld, Z: %ld/%ld\n", 
                  posX, CENTER_POSITION_X, posZ, CENTER_POSITION_Z);  
  }
  Serial.println("Arm at center position!");
  
  motorStopAll();
  
  bool espNowReady = espNowControlInit();
  Serial.printf("ESP-NOW control: %s\n", espNowReady ? "READY" : "ERROR");
  
  motion_serial_init();
  setupSerialCommand();
  mode_init();
  
  Serial.println("Robot ready!");
}

void loop() {
  serialCommandTick();
  espNowControlTick();
  motion_serial_tick();
  motion_serial_checkLinkTimeout();

  if (espNowControlReadPacket(gLastRxPacket)) {
    consumePacket("ESPNOW-RX", gLastRxPacket);
    mecanum_control_tick(gLastRxPacket);
    gripper_tick(gLastRxPacket);
    gripper_motor_tick(gLastRxPacket);
  }

  if (motion_serialReadPacket(gLastRxPacket)) {
    consumePacket("MOTION-RX", gLastRxPacket);
    mecanum_control_tick(gLastRxPacket);
    gripper_tick(gLastRxPacket);
    gripper_motor_tick(gLastRxPacket);
  }

  if (digitalRead(limitSwitchAxisX) == LOW){
        resetEncoderCount(0); // Reset encoder X saat mulai gerak horizontal
  } else if (digitalRead(limitSwitchAxisY) == LOW){
        resetEncoderCount(1); // Reset encoder Z saat mulai gerak vertikal
  }

  motion_serialPrintStats();
}