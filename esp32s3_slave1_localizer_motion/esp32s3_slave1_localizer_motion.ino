#include "robot_config.h"

// ============================================================
// State BOOT button untuk trigger auto-tuner
// ============================================================
static uint32_t gBootPressStartMs = 0;
static bool gBootWasPressed = false;

// Paket terakhir untuk debugging/manual processing.
static EspNowControlPacket gLastRxPacket = {};

void setup() {
  Serial.begin(115200);
  pinMode(BOOT_BUTTON_PIN, INPUT_PULLUP);
  SetupMotors();
  pidControllerInit();
  setupEncoders();
  printSerialUsage();
  
  Serial.println("=== Robot Manual Control ===");
  Serial.println("Press BOOT pin for 3 seconds to start auto-tuner");
}

void loop() {
  convertEncoderToRPM();
  autoTunerTick(digitalRead(BOOT_BUTTON_PIN) == LOW);

  // Process serial commands from USB (non-blocking)
  processSerialCommands();
  serialCommandsTick();
  serialContinuousTick();
}