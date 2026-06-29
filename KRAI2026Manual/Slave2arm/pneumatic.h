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
constexpr uint8_t PNEUMATIC_R_PIN = 49;
constexpr uint8_t PNEUMATIC_L_PIN = 9;
// constexpr uint8_t PNEUMATIC_UNUSED_1 = 10;  // ponytail: deadcode
// constexpr uint8_t PNEUMATIC_UNUSED_2 = 11;  // ponytail: deadcode

// =====================================================================
//  JUMLAH PNEUMATIC
// =====================================================================
constexpr size_t PNEUMATIC_COUNT = 2;  // ponytail: 2 valves deadcode dulu

// =====================================================================
//  SHARED FUNCTION DECLARATIONS
// =====================================================================
void setupPneumatic();
void pneumaticOn(char index);    // 'r' atau 'l'
void pneumaticOff(char index);
void pneumaticToggle(char index);
bool pneumaticState(char index);
void pneumaticAllOff();

#endif // PNEUMATIC_H
