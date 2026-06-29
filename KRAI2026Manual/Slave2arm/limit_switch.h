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
//  LIMIT SWITCH ID — index ke array limitPins[]
// =====================================================================
constexpr uint8_t LIMIT_ARMBOX_DEPAN    = 0;
constexpr uint8_t LIMIT_ARMBOX_BELAKANG = 1;
constexpr uint8_t LIMIT_ARMBOX_TURUN    = 2;
// constexpr uint8_t LIMIT_UNUSED      = 3;  // ponytail: deadcode, belum dipakai

constexpr size_t LIMIT_COUNT = 3;  // ponytail: 4th switch deadcode dulu

// =====================================================================
//  PIN LIMIT SWITCH — berdasarkan schematic KRAI 2026
// =====================================================================
constexpr uint8_t LIMIT_PIN_ARMBOX_DEPAN    = 40;
constexpr uint8_t LIMIT_PIN_ARMBOX_BELAKANG = 39;
constexpr uint8_t LIMIT_PIN_ARMBOX_TURUN    = 38;
constexpr uint8_t LIMIT_PIN_UNUSED          = 37;  // ponytail: deadcode

// =====================================================================
//  SHARED FUNCTION DECLARATIONS
// =====================================================================
void setupLimits();
bool readLimitSwitch(char index);

#endif // LIMIT_SWITCH_H
