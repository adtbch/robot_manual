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

// UART2 — komunikasi ke slave2arm
constexpr uint8_t SLAVE2_RX = 45;
constexpr uint8_t SLAVE2_TX = 48;

// =====================================================================
//  BAUD RATE
// =====================================================================
constexpr uint32_t SLAVE_BAUD = 921600;

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
void sendShowOdomCommand();

// serial_command.ino — status dari slave1
bool parseSlave1Status(char* line);

// serial_command.ino — sensor data dari slave2
bool parseSlave2Sensor(char* cmd);
bool slave2ProxR();
bool slave2ProxL();
bool slave2LimitDepan();
bool slave2LimitBelakang();
bool slave2LimitTurun();
long slave2EncX();
long slave2EncY();
long slave2EncK();
bool slave2PneR();
bool slave2PneL();

// serial_command.ino — kirim command ke slave
void sendKnCommand(int16_t vx, int16_t vy, int16_t yawTarget);
void sendSlave2Command(const char* fmt, ...);

#endif // SERIAL_H
