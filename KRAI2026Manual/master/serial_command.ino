/*
 * =====================================================================
 * FILE    : serial_command.ino
 * PERAN   : Helper functions untuk kirim serial command ke slave boards.
 *
 * SLAVE1 (UART1): motion control — kn vx vy yaw (field-cent + yaw PID)
 * SLAVE2 (UART2): arm manipulator
 *
 * BOARD   : ESP32-S3 (Master)
 * =====================================================================
 */

#include "serial.h"

// =====================================================================
//  SLAVE1 — Motion (Mecanum)
// =====================================================================

void sendKnCommand(int16_t vx, int16_t vy, int16_t yawTarget) {
    slave1Serial.printf("kn %d %d %d\n", vx, vy, yawTarget);
}

void sendSlave1Stop() {
    slave1Serial.println("stop");
}

// =====================================================================
//  SLAVE2 — Arm Manipulator
// =====================================================================

// Tambahkan command ke slave2 di sini nanti
// contoh:
// void sendArmCommand(...) {
//     slave2Serial.printf("...");
// }
