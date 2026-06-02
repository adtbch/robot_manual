#include "robot_config.h"
#include <Wire.h>

// ============================================================
// State BOOT button untuk trigger auto-tuner
// ============================================================
static uint32_t gBootPressStartMs = 0;
static bool gBootWasPressed = false;

// Paket terakhir untuk debugging/manual processing.
static ControlPacket gLastRxPacket = {};

void setup() {
  Serial.begin(115200);

  // Inisialisasi Serial1 untuk komunikasi dengan Master
  // Parameter: baud rate, protocol, RX pin, TX pin
  Serial1.begin(921600, SERIAL_8N1, serial_1_rxPin, serial_1_txPin);

  // Inisialisasi WSN-31 (UART2) untuk terima paket dari ESP32 Controller
  wsn_serial_init();

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
  // Baca semua byte dari WSN-31 (UART2) dan parse frame binary
  wsn_serial_tick();

  // Cek apakah ada paket baru dari ESP32 Controller via WSN-31
  ControlPacket wsnPacket;
  if (wsn_serial_readPacket(wsnPacket)) {
    // Simpan paket terakhir untuk keperluan lain
    gLastRxPacket = wsnPacket;

    // Cetak hasil parse ke Serial Monitor (throttle: tiap 20 paket / ~500ms)
    static uint32_t rxPrintCounter = 0;
    rxPrintCounter++;
    if (rxPrintCounter % 20 == 1) {
      Serial.printf("[WSN-RX] seq=%u x=%d y=%d w=%d lx=%d ly=%d rx=%d ry=%d btn=%lu conn=%d\n",
        wsnPacket.seq, wsnPacket.x, wsnPacket.y, wsnPacket.w,
        wsnPacket.lx, wsnPacket.ly, wsnPacket.rx, wsnPacket.ry,
        (unsigned long)wsnPacket.buttons, wsnPacket.connected);
    }
  }

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