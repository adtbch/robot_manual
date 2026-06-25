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
constexpr uint8_t PNEUMATIC_1_PIN = 49;
constexpr uint8_t PNEUMATIC_2_PIN = 9;
constexpr uint8_t PNEUMATIC_3_PIN = 10;
constexpr uint8_t PNEUMATIC_4_PIN = 11;

// =====================================================================
//  JUMLAH PNEUMATIC
// =====================================================================
constexpr size_t PNEUMATIC_COUNT = 4;

// =====================================================================
//  SHARED FUNCTION DECLARATIONS
// =====================================================================
void setupPneumatic();
void pneumaticOn(char index);
void pneumaticOff(char index);
void pneumaticToggle(char index);
bool pneumaticState(char index);
void pneumaticAllOff();

#endif // PNEUMATIC_H
