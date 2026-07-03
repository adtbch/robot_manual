/*
 * =====================================================================
 * FILE    : encoder.h
 * PERAN   : Konfigurasi modul encoder (pin, library ESP32Encoder).
 *
 * BOARD   : ESP32-S3 (Master)
 * =====================================================================
 */

#ifndef ENCODER_H
#define ENCODER_H

#include "config.h"

// =====================================================================
//  PIN ENCODER — berdasarkan schematic KRAI 2026
// =====================================================================
// Encoder 'x' (axis X / motor1)
constexpr uint8_t ENCODER1_PIN_A = 39;
constexpr uint8_t ENCODER1_PIN_B = 40;

// Encoder 'y' (axis Y / motor2)
constexpr uint8_t ENCODER2_PIN_A = 38;
constexpr uint8_t ENCODER2_PIN_B = 37;

// =====================================================================
//  JUMLAH ENCODER
// =====================================================================
constexpr size_t ENCODER_COUNT = 2;

// =====================================================================
//  SHARED FUNCTION DECLARATIONS
// =====================================================================
void setupEncoders();
long getEncoderCount(char encoderIndex);
void resetEncoderCount(char encoderIndex);

#endif // ENCODER_H
