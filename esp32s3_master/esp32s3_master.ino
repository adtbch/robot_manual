#include "robot_config.h"

// Paket terakhir untuk debugging/manual processing.
static EspNowControlPacket gLastRxPacket = {};

void setup() {
  Serial.begin(115200);
  SetupMotors();
  setupServos();
  setupEncoders();
  setHoming();
  bool espNowReady = espNowControlInit();
  
  Serial.printf("ESP-NOW control: %s\n", espNowReady ? "READY" : "ERROR");
}

void loop() {
  setServoAngle(1, 90); // Contoh: set servo 1 ke posisi 90 derajat
  
 
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