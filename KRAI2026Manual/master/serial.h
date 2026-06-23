/*
 * =====================================================================
 * FILE    : serial.h
 * PERAN   : Konfigurasi modul serial (UART1, UART2).
 *
 * BOARD   : ESP32-S3 (Master)
 * =====================================================================
 */

#ifndef SERIAL_H
#define SERIAL_H

#include "config.h"

// =====================================================================
//  PIN SERIAL — berdasarkan schematic KRAI 2026
// =====================================================================

// Serial1 (UART1) — komunikasi ke slave1
constexpr uint8_t SERIAL1_RX = 45;
constexpr uint8_t SERIAL1_TX = 48;

// Serial2 (UART2) — komunikasi ke slave2
constexpr uint8_t SERIAL2_RX = 47;
constexpr uint8_t SERIAL2_TX = 21;

// =====================================================================
//  BAUD RATE
// =====================================================================
constexpr uint32_t SERIAL1_BAUD = 115200;
constexpr uint32_t SERIAL2_BAUD = 115200;

// =====================================================================
//  SHARED FUNCTION DECLARATIONS
// =====================================================================
void setupSerial();

#endif // SERIAL_H
