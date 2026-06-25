/*
 * =====================================================================
 * FILE    : limit_switch.h
 * PERAN   : Konfigurasi modul limit switch (pin).
 *
 * BOARD   : ESP32-S3 (Slave2 Arm)
 * =====================================================================
 */

#ifndef LIMIT_SWITCH_H
#define LIMIT_SWITCH_H

#include "config.h"

// =====================================================================
//  PIN LIMIT SWITCH — berdasarkan schematic KRAI 2026
// =====================================================================
constexpr uint8_t LIMIT_SWITCH_1 = 40;
constexpr uint8_t LIMIT_SWITCH_2 = 39;
constexpr uint8_t LIMIT_SWITCH_3 = 38;
constexpr uint8_t LIMIT_SWITCH_4 = 37;

// =====================================================================
//  JUMLAH LIMIT SWITCH
// =====================================================================
constexpr size_t LIMIT_COUNT = 4;

// =====================================================================
//  SHARED FUNCTION DECLARATIONS
// =====================================================================
void setupLimits();
bool readLimitSwitch(char index);

#endif // LIMIT_SWITCH_H
