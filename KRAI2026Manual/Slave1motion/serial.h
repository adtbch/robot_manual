/*
 * =====================================================================
 * FILE    : serial.h
 * PERAN   : Konfigurasi modul serial (UART1 ke master, UART2 WSN-31).
 *
 * BOARD   : ESP32-S3 (Slave1 Motion)
 * =====================================================================
 */

#ifndef SERIAL_H
#define SERIAL_H

#include "config.h"

// =====================================================================
//  PIN SERIAL — berdasarkan schematic KRAI 2026
// =====================================================================

// Serial1 (UART1) — ke master
constexpr uint8_t SERIAL_MASTER_RX = 21;
constexpr uint8_t SERIAL_MASTER_TX = 20;

// Serial2 (UART2) — ke WSN-31
constexpr uint8_t SERIAL_WSN_RX = 12;
constexpr uint8_t SERIAL_WSN_TX = 11;

// WSN-31 SET pin
constexpr uint8_t WSN_SET_PIN = 19;

// =====================================================================
//  BAUD RATE
// =====================================================================
constexpr uint32_t SERIAL_MASTER_BAUD = 921600;
constexpr uint32_t SERIAL_WSN_BAUD = 9600;

// =====================================================================
//  SHARED FUNCTION DECLARATIONS
// =====================================================================
void setupSerial();
void serialRelayTick();
void parseSerialCommand();  // Parser USB Serial untuk tuning PID

#endif // SERIAL_H
