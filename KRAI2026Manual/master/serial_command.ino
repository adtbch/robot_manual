/*
 * =====================================================================
 * FILE    : serial_command.ino
 * PERAN   : Helper functions untuk kirim serial command ke slave boards.
 *
 * SLAVE1 (UART1): motion control (mecanum)
 * SLAVE2 (UART2): arm manipulator
 *
 * BOARD   : ESP32-S3 (Master)
 * =====================================================================
 */

#include "serial.h"

// =====================================================================
//  SLAVE1 — Motion (Mecanum)
// =====================================================================

void sendRpmCommand(int16_t fr, int16_t fl, int16_t br, int16_t bl) {
    slave1Serial.printf("rpm %d %d %d %d\n", fr, fl, br, bl);
}

// =====================================================================
//  SLAVE2 — Arm Manipulator
// =====================================================================

// Tambahkan command ke slave2 di sini nanti
// contoh:
// void sendArmCommand(...) {
//     slave2Serial.printf("...");
// }
