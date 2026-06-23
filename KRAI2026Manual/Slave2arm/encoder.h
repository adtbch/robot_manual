/*
 * =====================================================================
 * FILE    : encoder.h
 * PERAN   : Konfigurasi modul encoder (pin, ESP32Encoder library).
 *
 * BOARD   : ESP32-S3 (Slave2 Arm)
 * =====================================================================
 */

#ifndef ENCODER_H
#define ENCODER_H

#include "config.h"

// =====================================================================
//  PIN ENCODER — berdasarkan schematic KRAI 2026
// =====================================================================
// Encoder1
constexpr uint8_t ENCODER1_PIN_A = 41;
constexpr uint8_t ENCODER1_PIN_B = 42;

// Encoder2
constexpr uint8_t ENCODER2_PIN_A = 1;
constexpr uint8_t ENCODER2_PIN_B = 2;

// =====================================================================
//  JUMLAH ENCODER
// =====================================================================
constexpr size_t ENCODER_COUNT = 2;

// =====================================================================
//  SHARED FUNCTION DECLARATIONS
// =====================================================================
void setupEncoders();
long getEncoderCount(uint8_t encoderIndex);
void resetEncoderCount(uint8_t encoderIndex);

#endif // ENCODER_H
