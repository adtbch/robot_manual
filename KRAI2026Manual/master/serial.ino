/*
 * =====================================================================
 * FILE    : serial.ino
 * PERAN   : Inisialisasi UART1 dan UART2.
 *
 * UART1   : Serial1 (RX=45, TX=48) → slave1
 * UART2   : Serial2 (RX=47, TX=21) → slave2
 *
 * BOARD   : ESP32-S3 (Master)
 * =====================================================================
 */

#include "serial.h"

// =====================================================================
//  SETUP
// =====================================================================

void setupSerial() {
    // UART1 — ke slave1
    Serial1.begin(SERIAL1_BAUD, SERIAL_8N1, SERIAL1_RX, SERIAL1_TX);

    // UART2 — ke slave2
    Serial2.begin(SERIAL2_BAUD, SERIAL_8N1, SERIAL2_RX, SERIAL2_TX);
}
