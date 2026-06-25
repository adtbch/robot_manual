/*
 * =====================================================================
 * FILE    : proximity.h
 * PERAN   : Konfigurasi modul proximity sensor (pin).
 *
 * BOARD   : ESP32-S3 (Slave2 Arm)
 * =====================================================================
 */

#ifndef PROXIMITY_H
#define PROXIMITY_H

#include "config.h"

// =====================================================================
//  PIN PROXIMITY — berdasarkan schematic KRAI 2026
// =====================================================================
constexpr uint8_t PROXIMITY_1_PIN = 15;
constexpr uint8_t PROXIMITY_2_PIN = 16;

// =====================================================================
//  JUMLAH PROXIMITY
// =====================================================================
constexpr size_t PROXIMITY_COUNT = 2;

// =====================================================================
//  SHARED FUNCTION DECLARATIONS
// =====================================================================
void setupProximity();
bool readProximity(char index);

#endif // PROXIMITY_H
