/*
 * =====================================================================
 * FILE    : pneumatic.h
 * PERAN   : Konfigurasi modul pneumatic valve (pin, state).
 *
 * BOARD   : ESP32-S3 (Slave2 Arm)
 * =====================================================================
 */

#ifndef PNEUMATIC_H
#define PNEUMATIC_H

#include "config.h"

// =====================================================================
//  PIN PNEUMATIC — berdasarkan schematic KRAI 2026
// =====================================================================
constexpr uint8_t PNEUMATIC_R_PIN = 46;
constexpr uint8_t PNEUMATIC_RK_PIN = 9;
constexpr uint8_t PNEUMATIC_L_PIN = 11;
constexpr uint8_t PNEUMATIC_LK_PIN = 10;

// =====================================================================
//  JUMLAH PNEUMATIC
// =====================================================================
constexpr size_t PNEUMATIC_COUNT = 4;

// =====================================================================
//  SHARED FUNCTION DECLARATIONS
// =====================================================================
void setupPneumatic();
void pneumaticOn(const char* index);    // "r", "l", "lk", "rk"
void pneumaticOff(const char* index);
void pneumaticToggle(const char* index);
bool pneumaticState(const char* index);
void pneumaticAllOff();

#endif // PNEUMATIC_H
