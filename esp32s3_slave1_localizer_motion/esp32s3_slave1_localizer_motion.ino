#include "robot_config.h"
#include <Wire.h>

// ============================================================
// State BOOT button untuk trigger auto-tuner
// ============================================================
static uint32_t gBootPressStartMs = 0;
static bool gBootWasPressed = false;

void setup() {
  Serial.begin(115200);

  // Inisialisasi WSN-31 + relay ke Master
  wsn_serial_init();

  Wire.begin(sdaPin,sclPin);
  Wire.setClock(400000);
  pinMode(BOOT_BUTTON_PIN, INPUT_PULLUP);
  setupOLED();
  SetupMotors();
  pidControllerInit();
  setupEncoders();
  printSerialUsage();
  if (!setupMPU()) {
    Serial.println("MPU: not ready, yaw will stay 0");
  }
  // Reinit I2C bus — MPU9250 library may leave bus in bad state
  Serial.println("=== Robot Slave — WSN-31 Relay ===");
}

void loop() {
  // Relay: teruskan semua byte dari WSN-31 ke Master (tanpa parsing)
  wsn_serial_tick();

  // convertEncoderToRPM dengan interval 40ms — konsisten dengan autoTuner tick
  static uint32_t lastEncoderMs = 0;
  if (millis() - lastEncoderMs >= 40) {
    lastEncoderMs = millis();
    convertEncoderToRPM();
  }

  updateYaw();
  autoTunerTick(digitalRead(BOOT_BUTTON_PIN) == LOW);

  // Process serial commands from USB (non-blocking)
  processSerialCommands();
  serialCommandsTick();
  serialContinuousTick();

  // Update OLED — rate limit 200ms supaya tidak starve I2C bus MPU9250
  static uint32_t lastOledMs = 0;
  if (millis() - lastOledMs >= 200) {
    lastOledMs = millis();
    const char* status = autoTunerIsActive() ? "AUTOTUNE" : "READY";
    displayYaw(getYaw(), status);
  }
}