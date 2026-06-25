/*
 * =====================================================================
 * FILE    : limit_switch.h
 * PERAN   : Konfigurasi modul limit switch (pin).
 *
 * BOARD   : ESP32-S3 (Master)
 * =====================================================================
 */

#ifndef LIMIT_SWITCH_H
#define LIMIT_SWITCH_H

#include "config.h"

// =====================================================================
//  PIN LIMIT SWITCH — berdasarkan schematic KRAI 2026
// =====================================================================
constexpr uint8_t LIMIT_SWITCH_1 = 6;
constexpr uint8_t LIMIT_SWITCH_2 = 7;
constexpr uint8_t LIMIT_SWITCH_3 = 15;
constexpr uint8_t LIMIT_SWITCH_4 = 16;

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
