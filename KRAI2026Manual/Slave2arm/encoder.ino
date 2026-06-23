/*
 * =====================================================================
 * FILE    : encoder.ino
 * PERAN   : Baca posisi encoder via library ESP32Encoder.
 *           2 encoder: Encoder1, Encoder2.
 *
 * LIBRARY : ESP32Encoder (lucky68t)
 *           attachHalfQuad() — 4x resolution, half-quad wiring
 *
 * BOARD   : ESP32-S3 (Slave2 Arm)
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
    encoders[0].attachHalfQuad(ENCODER1_PIN_A, ENCODER1_PIN_B);
    encoders[0].clearCount();

    encoders[1].attachHalfQuad(ENCODER2_PIN_A, ENCODER2_PIN_B);
    encoders[1].clearCount();
}

// =====================================================================
//  READ
// =====================================================================

long getEncoderCount(uint8_t encoderIndex) {
    if (encoderIndex >= ENCODER_COUNT) {
        return 0;
    }
    return (long)encoders[encoderIndex].getCount();
}

// =====================================================================
//  RESET
// =====================================================================

void resetEncoderCount(uint8_t encoderIndex) {
    if (encoderIndex >= ENCODER_COUNT) {
        return;
    }
    encoders[encoderIndex].clearCount();
}
