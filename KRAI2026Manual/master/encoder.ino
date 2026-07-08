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

// Mapping: 'x' = encoder axis X, 'y' = encoder axis Y
int findEncoderIndex(char id) {
    if (id == 'x') return 0;
    if (id == 'y') return 1;
    return -1;
}

} // anonymous namespace

// =====================================================================
//  SETUP
// =====================================================================

void setupEncoders() {
    // Encoder1 — half quad, pullup internal
    encoders[0].attachHalfQuad(ENCODER1_PIN_A, ENCODER1_PIN_B);
    encoders[0].setFilter(1023); //extEncoders[i].setFilter(1023); 
    encoders[0].clearCount();

    // Encoder2 — half quad, pullup internal
    encoders[1].attachHalfQuad(ENCODER2_PIN_A, ENCODER2_PIN_B);
    encoders[1].setFilter(1023); //extEncoders[i].setFilter(1023); 
    encoders[1].clearCount();
}

// =====================================================================
//  READ
// =====================================================================

long getEncoderCount(char id) {
    int idx = findEncoderIndex(id);
    if (idx < 0) return 0;
    return (long)encoders[idx].getCount();
}

// =====================================================================
//  RESET
// =====================================================================

void resetEncoderCount(char id) {
    int idx = findEncoderIndex(id);
    if (idx < 0) return;
    encoders[idx].clearCount();
}