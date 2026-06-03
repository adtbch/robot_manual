/*
 * =====================================================================
 * FILE    : wsn_serial.ino
 * PERAN   : Menerima data mentah dari modul radio WSN-31 via UART
 *           dan meneruskan langsung ke ESP32-S3 Master via Serial1.
 *           TIDAK ADA parsing — Slave hanya relay (transparent bridge).
 *
 * ALIRAN DATA:
 *   ESP32 Controller → WSN-31 → [ESP32-S3 Slave] → Serial1 → [ESP32-S3 Master]
 *
 * WIRING:
 *   WSN-31 TX → ESP32-S3 Slave RX (wsn_serial_rxPin=12)
 *   WSN-31 RX ← ESP32-S3 Slave TX (wsn_serial_txPin=11)
 *   ESP32-S3 Slave TX (master_serial_txPin=10) → ESP32-S3 Master RX
 * =====================================================================
 */

#include "robot_config.h"

#define wsn_serial  Serial2
#define master_serial Serial1

// =====================================================================
//  FUNGSI: INISIALISASI UART
// =====================================================================

void wsn_serial_init() {
    // Pin SET HIGH = mode normal (transparent)
    pinMode(kWsnSetPin, OUTPUT);
    digitalWrite(kWsnSetPin, HIGH);

    // UART ke WSN-31 (receiver)
    wsn_serial.begin(9600, SERIAL_8N1, wsn_serial_rxPin, wsn_serial_txPin);
    // UART ke Master (forwarder)
    master_serial.begin(921600, SERIAL_8N1, master_serial_rxPin, master_serial_txPin);

    Serial.printf("[WSN-31] Relay init — RX=%d TX=%d → Master TX=%d RX=%d\n",
                  wsn_serial_rxPin, wsn_serial_txPin, master_serial_txPin, master_serial_rxPin);
}

// =====================================================================
//  FUNGSI: RELAY — forwarding tanpa parsing
// =====================================================================

/**
 * Baca semua byte dari WSN-31 dan teruskan langsung ke Master.
 * Dipanggil setiap iterasi loop().
 */
void wsn_serial_tick() {
    // Forward: WSN-31 → Master
    while (wsn_serial.available()) {
        uint8_t b = wsn_serial.read();
        master_serial.write(b);
    }
}


