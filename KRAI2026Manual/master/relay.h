/*
 * =====================================================================
 * FILE    : relay.h
 * PERAN   : Konfigurasi modul relay (pin, state).
 *
 * BOARD   : ESP32-S3 (Master)
 * =====================================================================
 */

#ifndef RELAY_H
#define RELAY_H

#include "config.h"

// =====================================================================
//  PIN RELAY — berdasarkan schematic KRAI 2026
// =====================================================================
constexpr uint8_t RELAY_1_PIN = 5;

// =====================================================================
//  SHARED FUNCTION DECLARATIONS
// =====================================================================
void setupRelay();
void relayOn();
void relayOff();
void relayToggle();
bool relayState();

#endif // RELAY_H
