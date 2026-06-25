/*
 * =====================================================================
 * FILE    : serial.h
 * PERAN   : Konfigurasi modul serial (UART1 → master).
 *           Unified command handler: PC (USB Serial) & master sama.
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

// UART1 — komunikasi ke master
constexpr uint8_t MASTER_RX = 36;
constexpr uint8_t MASTER_TX = 35;

// =====================================================================
//  BAUD RATE
// =====================================================================
constexpr uint32_t MASTER_BAUD = 921600;

// =====================================================================
//  SHARED FUNCTION DECLARATIONS
// =====================================================================
void setupSerial();
void setupSerialCommand();
void serialCommandTick();

#endif // SERIAL_H
