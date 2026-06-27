/*
 * =====================================================================
 * FILE    : serial.h
 * PERAN   : Konfigurasi modul serial (UART1 → slave1, UART2 → slave2).
 *           Unified command handler: PC (USB Serial) & slave1 & slave2.
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

// UART1 — komunikasi ke slave1
constexpr uint8_t SLAVE1_RX = 47;
constexpr uint8_t SLAVE1_TX = 21;

// UART2 — komunikasi ke slave2
constexpr uint8_t SLAVE2_RX = 45;
constexpr uint8_t SLAVE2_TX = 48;

// =====================================================================
//  BAUD RATE
// =====================================================================
constexpr uint32_t SLAVE1_BAUD = 921600;
constexpr uint32_t SLAVE2_BAUD = 921600;

// =====================================================================
//  UART INSTANCES — global, bisa diakses modul lain
// =====================================================================
extern HardwareSerial slave1Serial;
extern HardwareSerial slave2Serial;

// =====================================================================
//  SHARED FUNCTION DECLARATIONS
// =====================================================================
void setupSerial();
void setupSerialCommand();
void serialCommandTick();

#endif // SERIAL_H
