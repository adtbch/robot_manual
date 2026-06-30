/*
 * =====================================================================
 * FILE    : relay.h
 * PERAN   : Konfigurasi flash lamp (pin, state).
 *           HIGH sebentar → LOW — sebagai flash indicator.
 *
 * BOARD   : ESP32-S3 (Master)
 * =====================================================================
 */

#ifndef RELAY_H
#define RELAY_H

#include "config.h"

// =====================================================================
//  PIN FLASH — pin sama dengan relay lama (pin 5)
// =====================================================================
constexpr uint8_t FLASH_PIN = 5;

// =====================================================================
//  SHARED FUNCTION DECLARATIONS
// =====================================================================
void setupRelay();
void flashFire();
void flashTick();

#endif // RELAY_H
