/*
 * =====================================================================
 * FILE    : manipulator_serial.ino
 * PERAN   : Komunikasi UART ke Slave2 (Manipulator) via Serial2.
 *           Mengirim data dummy "halo" secara periodik untuk testing.
 *
 * WIRING:
 *   ESP32-S3 Master TX (manipulator_serial_txPin=5) → Slave2 RX
 *   ESP32-S3 Master RX (manipulator_serial_rxPin=4) ← Slave2 TX
 * =====================================================================
 */

#include "robot_config.h"

// =====================================================================
//  INISIALISASI
// =====================================================================

void manipulator_serial_init() {
  manipulator_serial.begin(921600, SERIAL_8N1, manipulator_serial_rxPin, manipulator_serial_txPin);
  Serial.printf("[MANIP-SERIAL] Init — RX=%d TX=%d @ 921600\n",
                manipulator_serial_rxPin, manipulator_serial_txPin);
}

// =====================================================================
//  TICK — kirim "halo" secara periodik
// =====================================================================

void manipulator_serial_tick() {
  // Baca data dari Slave2 (jika ada)
  while (manipulator_serial.available()) {
    char c = manipulator_serial.read();
    Serial.write(c);
  }
}
