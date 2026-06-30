/*
 * =====================================================================
 * FILE    : relay.ino
 * PERAN   : Flash lamp — HIGH sebentar, LOW lagi.
 *           Triggered via flashFire(), auto-off via flashTick().
 *
 * BOARD   : ESP32-S3 (Master)
 * =====================================================================
 */

#include "relay.h"

// =====================================================================
//  CONFIG
// =====================================================================

constexpr uint32_t FLASH_DURATION_MS = 50;

// =====================================================================
//  STATE
// =====================================================================

namespace {
bool     gFlashOn = false;
uint32_t gFlashStartMs = 0;
} // anonymous namespace

// =====================================================================
//  SETUP
// =====================================================================

void setupRelay() {
    pinMode(FLASH_PIN, OUTPUT);
    digitalWrite(FLASH_PIN, LOW);
}

// =====================================================================
//  CONTROL
// =====================================================================

void flashFire() {
    if (gFlashOn) return;  // sudah nyala, skip
    digitalWrite(FLASH_PIN, HIGH);
    gFlashOn = true;
    gFlashStartMs = millis();
}

void flashTick() {
    if (!gFlashOn) return;
    if (millis() - gFlashStartMs >= FLASH_DURATION_MS) {
        digitalWrite(FLASH_PIN, LOW);
        gFlashOn = false;
    }
}
