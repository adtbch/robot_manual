#include "robot_config.h"
#include <Wire.h>

// ============================================================
// State BOOT button untuk trigger auto-tuner
// ============================================================
static uint32_t gBootPressStartMs = 0;
static bool gBootWasPressed = false;

// Paket terakhir untuk debugging/manual processing.
static EspNowControlPacket gLastRxPacket = {};

void setup() {
  Serial.begin(115200);

  // Inisialisasi Serial1 untuk komunikasi dengan Master
  // Parameter: baud rate, protocol, RX pin, TX pin
  Serial1.begin(921600, SERIAL_8N1, serial_1_rxPin, serial_1_txPin);

  Wire.begin(sdaPin,sclPin);
  Wire.setClock(400000);
  pinMode(BOOT_BUTTON_PIN, INPUT_PULLUP);
  SetupMotors();
  pidControllerInit();
  setupEncoders();
  printSerialUsage();
  if (!setupMPU()) {
    Serial.println("MPU: not ready, yaw will stay 0");
  }
  setupOLED();
  Serial.println("=== Robot Manual Control ===");
  Serial.println("Press BOOT pin for 3 seconds to start auto-tuner");
}

void loop() {
  convertEncoderToRPM();
  updateYaw();
  autoTunerTick(digitalRead(BOOT_BUTTON_PIN) == LOW);

  // Process serial commands from USB (non-blocking)
  processSerialCommands();
  serialCommandsTick();
  serialContinuousTick();

  // Update OLED display setiap 200ms
  const char* status = autoTunerIsActive() ? "AUTOTUNE" : "READY";
  displayYaw(getYaw(), status);
}