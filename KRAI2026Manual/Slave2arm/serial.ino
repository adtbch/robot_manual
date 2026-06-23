/*
 * =====================================================================
 * FILE    : serial.ino
 * PERAN   : Inisialisasi UART1.
 *
 * UART1   : Serial1 (RX=36, TX=35) → master
 *
 * BOARD   : ESP32-S3 (Slave2 Arm)
 * =====================================================================
 */

#include "serial.h"

// =====================================================================
//  SETUP
// =====================================================================

void setupSerial() {
    Serial1.begin(SERIAL1_BAUD, SERIAL_8N1, SERIAL1_RX, SERIAL1_TX);
}
