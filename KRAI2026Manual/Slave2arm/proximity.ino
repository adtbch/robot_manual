/*
 * =====================================================================
 * FILE    : proximity.ino
 * PERAN   : Baca proximity sensor (digital).
 *
 * BOARD   : ESP32-S3 (Slave2 Arm)
 * =====================================================================
 */

#include "proximity.h"

// =====================================================================
//  STATE
// =====================================================================

namespace {

struct ProximityConfig {
    char id;
    uint8_t pin;
};

constexpr ProximityConfig proximitySensors[PROXIMITY_COUNT] = {
    {'r', PROXIMITY_R_PIN},
    {'l', PROXIMITY_L_PIN},
};

} // anonymous namespace

// =====================================================================
//  SETUP
// =====================================================================

void setupProximity() {
    for (size_t i = 0; i < PROXIMITY_COUNT; i++) {
        pinMode(proximitySensors[i].pin, INPUT_PULLUP);
    }
}

// =====================================================================
//  READ
// =====================================================================

bool readProximity(char index) {
    for (size_t i = 0; i < PROXIMITY_COUNT; i++) {
        if (proximitySensors[i].id == index) {
            return (digitalRead(proximitySensors[i].pin) == LOW);  // active LOW
        }
    }
    return false;
}
