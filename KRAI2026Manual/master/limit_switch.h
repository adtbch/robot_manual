/*
 * =====================================================================
 * FILE    : limit_switch.h
 * PERAN   : Konfigurasi modul limit switch (pin).
 *
 * ARM CAPIT SENJATA (motor X/Y di master):
 *   LIMIT_Y_BAWAH  — Y turun mentok (homing + jog)
 *   LIMIT_X_MUNDUR — X mundur mentok (homing + jog)
 *
 * ARM BOX (motor di slave2, limit dibaca di master):
 *   LIMIT_ARMBOX_DEPAN    — pin 15
 *   LIMIT_ARMBOX_BELAKANG — pin 16
 *
 * BOARD   : ESP32-S3 (Master)
 * =====================================================================
 */

#ifndef LIMIT_SWITCH_H
#define LIMIT_SWITCH_H

#include "config.h"

// =====================================================================
//  LIMIT SWITCH ID — index ke array limitPins[]
// =====================================================================
constexpr uint8_t LIMIT_Y_BAWAH         = 0;
constexpr uint8_t LIMIT_X_MUNDUR        = 1;
constexpr uint8_t LIMIT_ARMBOX_DEPAN    = 2;
constexpr uint8_t LIMIT_ARMBOX_BELAKANG = 3;

constexpr size_t LIMIT_COUNT = 4;

// =====================================================================
//  PIN LIMIT SWITCH — berdasarkan schematic KRAI 2026
// =====================================================================
constexpr uint8_t LIMIT_PIN_Y_BAWAH         = 46;
constexpr uint8_t LIMIT_PIN_X_MUNDUR        = 7;
constexpr uint8_t LIMIT_PIN_ARMBOX_DEPAN    = 15;
constexpr uint8_t LIMIT_PIN_ARMBOX_BELAKANG = 16;

// =====================================================================
//  SHARED FUNCTION DECLARATIONS
// =====================================================================
void setupLimits();
bool readLimitSwitch(uint8_t limitId);

#endif // LIMIT_SWITCH_H
