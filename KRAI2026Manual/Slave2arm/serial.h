/*
 * =====================================================================
 * FILE    : serial.h
 * PERAN   : Konfigurasi modul serial (UART1 → master).
 *
 * UART1   : masterSerial (RX=36, TX=35) → master
 *
 * BOARD   : ESP32-S3 (Slave2 Arm)
 * =====================================================================
 */

#ifndef SERIAL_H
#define SERIAL_H

#include "config.h"

extern HardwareSerial masterSerial;

// =====================================================================
//  PIN SERIAL — berdasarkan schematic KRAI 2026
// =====================================================================

constexpr uint8_t MASTER_RX = 36;
constexpr uint8_t MASTER_TX = 35;

// =====================================================================
//  BAUD RATE
// =====================================================================
constexpr uint32_t MASTER_BAUD = 921600;

// =====================================================================
//  SHARED FUNCTION DECLARATIONS
// =====================================================================

// serial.ino — parsing perintah masuk
void setupSerial();
void setupSerialCommand();
void serialCommandTick();

// ponytail: UART TX ke master hanya dari loop() core 1 — web task core 0 queue di sini
void masterUartProxyTick();
void masterUartSendLine(const char* line);

// serial_command.ino — kirim data ke master
void sendProximityStatus(char side, bool detected);
void sendLimitStatus(const char* name, bool triggered);
void sendEncoderStatus(char id, long count);
void sendPneumaticStatus(char side, bool state);
void sendFullStatus();

#endif // SERIAL_H
