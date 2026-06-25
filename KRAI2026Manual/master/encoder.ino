/*
 * =====================================================================
 * FILE    : encoder.ino
 * PERAN   : Baca posisi encoder via library ESP32Encoder.
 *           2 encoder: Encoder1 (axis X), Encoder2 (axis Y).
 *
 * LIBRARY : ESP32Encoder (lucky68t)
 *           attachHalfQuad() — 4x resolution, half-quad wiring
 *
 * BOARD   : ESP32-S3 (Master)
 * =====================================================================
 */

#include "encoder.h"
#include <ESP32Encoder.h>

// =====================================================================
//  STATE
// =====================================================================

namespace {

ESP32Encoder encoders[ENCODER_COUNT];

} // anonymous namespace

// =====================================================================
//  SETUP
// =====================================================================

void setupEncoders() {
    // Encoder1 — half quad, pullup internal
    encoders[0].attachHalfQuad(ENCODER1_PIN_A, ENCODER1_PIN_B);
    encoders[0].clearCount();

    // Encoder2 — half quad, pullup internal
    encoders[1].attachHalfQuad(ENCODER2_PIN_A, ENCODER2_PIN_B);
    encoders[1].clearCount();
}

// =====================================================================
//  READ
// =====================================================================

long getEncoderCount(char encoderIndex) {
    if (encoderIndex >= ENCODER_COUNT) {
        return 0;
    }
    return (long)encoders[encoderIndex].getCount();
}

// =====================================================================
//  RESET
// =====================================================================

void resetEncoderCount(char encoderIndex) {
    if (encoderIndex >= ENCODER_COUNT) {
        return;
    }
    encoders[encoderIndex].clearCount();
}