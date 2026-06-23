/*
 * =====================================================================
 * FILE    : proximity.h
 * PERAN   : Konfigurasi modul proximity sensor (pin).
 *
 * BOARD   : ESP32-S3 (Master)
 * =====================================================================
 */

#ifndef PROXIMITY_H
#define PROXIMITY_H

#include "config.h"

// =====================================================================
//  PIN PROXIMITY — berdasarkan schematic KRAI 2026
// =====================================================================
constexpr uint8_t PROXIMITY_1_PIN = 14;

// =====================================================================
//  SHARED FUNCTION DECLARATIONS
// =====================================================================
void setupProximity();
bool readProximity();

#endif // PROXIMITY_H
