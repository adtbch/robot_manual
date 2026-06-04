#include <WiFi.h>
#include "robot_config.h"

// Paket terakhir untuk debugging/manual processing.
static ControlPacket gLastRxPacket = {};

// =====================================================================
//  SHARED: consume packet + print throttled (tiap 20 paket)
// =====================================================================

static void consumePacket(const char *source, ControlPacket &pkt) {
  static uint32_t rxPrintCounter = 0;
  rxPrintCounter++;
  if (rxPrintCounter % 20 == 1) {
    Serial.printf("[%s] seq=%u x=%d y=%d w=%d lx=%d ly=%d l2=%u r2=%u btn=%lu conn=%d\n",
                  source,
                  pkt.seq, pkt.x, pkt.y, pkt.w,
                  pkt.lx, pkt.ly, pkt.l2Value, pkt.r2Value,
                  (unsigned long)pkt.buttons, pkt.connected);
  }
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
  
  Serial.println("Robot ready!");
}

void loop() {
  serialCommandTick();
  espNowControlTick();
  motion_serial_tick();
  if (espNowControlReadPacket(gLastRxPacket)) {
    consumePacket("ESPNOW-RX", gLastRxPacket);
    mecanum_control_tick(gLastRxPacket);
  }

  if (motion_serialReadPacket(gLastRxPacket)) {
    consumePacket("MOTION-RX", gLastRxPacket);
    mecanum_control_tick(gLastRxPacket);
  }

  motion_serialPrintStats();
}