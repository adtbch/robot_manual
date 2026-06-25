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

constexpr uint8_t proximityPins[PROXIMITY_COUNT] = {
    PROXIMITY_1_PIN,
    PROXIMITY_2_PIN
};

} // anonymous namespace

// =====================================================================
//  SETUP
// =====================================================================

void setupProximity() {
    for (size_t i = 0; i < PROXIMITY_COUNT; i++) {
        pinMode(proximityPins[i], INPUT_PULLUP);
    }
}

// =====================================================================
//  READ
// =====================================================================

bool readProximity(char index) {
    if (index >= PROXIMITY_COUNT) {
        return false;
    }
    return (digitalRead(proximityPins[index]) == LOW);  // active LOW
}
