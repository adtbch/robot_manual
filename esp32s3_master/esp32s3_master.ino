#include "robot_config.h"

// Paket terakhir untuk debugging/manual processing.
static EspNowControlPacket gLastRxPacket = {};

void setup() {
  Serial.begin(115200);
  SetupMotors();
  setupServos();
  pidControllerInit();
  setupEncoders();

  bool espNowReady = espNowControlInit();
  
  Serial.printf("ESP-NOW control: %s\n", espNowReady ? "READY" : "ERROR");
}

void loop() {
  // 1) Cek BOOT button selama 3 detik untuk mulai auto-tuner.
  bool bootPressed = (digitalRead(BOOT_BUTTON_PIN) == LOW);
  
  if (bootPressed && !gBootWasPressed) {
    gBootPressStartMs = millis();
    gBootWasPressed = true;
  } else if (!bootPressed && gBootWasPressed) {
    gBootWasPressed = false;
  }
  
  // 3 detik hold -> trigger auto-tuner.
  if (bootPressed && !autoTunerIsActive() && (millis() - gBootPressStartMs >= 3000)) {
    autoTunerStart();
    Serial.println("Auto-tuner started!");
  }
  
  // 2) Update pembacaan encoder.
  convertEncoderToRPM();
  
  // 3) Saat auto-tuner aktif, mode normal dihentikan sementara.
  if (autoTunerIsActive()) {
    autoTunerTick(bootPressed);
    return;
  }
  
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