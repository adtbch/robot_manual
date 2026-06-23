/*
 * =====================================================================
 * FILE    : pneumatic.ino
 * PERAN   : Control pneumatic valve via digital output.
 *
 * BOARD   : ESP32-S3 (Slave2 Arm)
 * =====================================================================
 */

#include "pneumatic.h"

// =====================================================================
//  STATE
// =====================================================================

namespace {

constexpr uint8_t pneumaticPins[PNEUMATIC_COUNT] = {
    PNEUMATIC_1_PIN,
    PNEUMATIC_2_PIN,
    PNEUMATIC_3_PIN,
    PNEUMATIC_4_PIN
};

bool pneumaticActive[PNEUMATIC_COUNT] = {};

} // anonymous namespace

// =====================================================================
//  SETUP
// =====================================================================

void setupPneumatic() {
    for (size_t i = 0; i < PNEUMATIC_COUNT; i++) {
        pinMode(pneumaticPins[i], OUTPUT);
        digitalWrite(pneumaticPins[i], LOW);  // default OFF
    }
}

// =====================================================================
//  CONTROL
// =====================================================================

void pneumaticOn(uint8_t index) {
    if (index >= PNEUMATIC_COUNT) return;
    digitalWrite(pneumaticPins[index], HIGH);
    pneumaticActive[index] = true;
}

void pneumaticOff(uint8_t index) {
    if (index >= PNEUMATIC_COUNT) return;
    digitalWrite(pneumaticPins[index], LOW);
    pneumaticActive[index] = false;
}

void pneumaticToggle(uint8_t index) {
    if (index >= PNEUMATIC_COUNT) return;
    if (pneumaticActive[index]) {
        pneumaticOff(index);
    } else {
        pneumaticOn(index);
    }
}

bool pneumaticState(uint8_t index) {
    if (index >= PNEUMATIC_COUNT) return false;
    return pneumaticActive[index];
}

void pneumaticAllOff() {
    for (size_t i = 0; i < PNEUMATIC_COUNT; i++) {
        pneumaticOff(i);
    }
}
