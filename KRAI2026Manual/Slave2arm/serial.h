/*
 * =====================================================================
 * FILE    : serial.h
 * PERAN   : Konfigurasi modul serial (UART1).
 *
 * BOARD   : ESP32-S3 (Slave2 Arm)
 * =====================================================================
 */

#ifndef SERIAL_H
#define SERIAL_H

#include "config.h"

// =====================================================================
//  PIN SERIAL — berdasarkan schematic KRAI 2026
// =====================================================================

// Serial1 (UART1) — komunikasi ke master
constexpr uint8_t SERIAL1_RX = 36;
constexpr uint8_t SERIAL1_TX = 35;

// =====================================================================
//  BAUD RATE
// =====================================================================
constexpr uint32_t SERIAL1_BAUD = 115200;

// =====================================================================
//  SHARED FUNCTION DECLARATIONS
// =====================================================================
void setupSerial();

#endif // SERIAL_H
