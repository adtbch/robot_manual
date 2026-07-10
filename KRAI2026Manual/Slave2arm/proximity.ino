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

const int PROX_THRESHOLD = 102;
} // anonymous namespace

// =====================================================================
//  SETUP
// =====================================================================

void setupProximity() {
    for (size_t i = 0; i < PROXIMITY_COUNT; i++) { // full range 0-3.3V// full range 0-3.3V
        pinMode(proximitySensors[i].pin, INPUT);
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
