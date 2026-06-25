/*
 * =====================================================================
 * FILE    : encoder.h
 * PERAN   : Konfigurasi modul encoder (pin, ESP32Encoder library).
 *           4 encoder untuk mecanum drive.
 *
 * BOARD   : ESP32-S3 (Slave1 Motion)
 * =====================================================================
 */

#ifndef ENCODER_H
#define ENCODER_H

#include "config.h"

// =====================================================================
//  PIN ENCODER — berdasarkan schematic KRAI 2026
// =====================================================================

// Encoder Depan Kanan (Front Right)
constexpr uint8_t ENCODER_FR_A = 40;
constexpr uint8_t ENCODER_FR_B = 39;

// Encoder Depan Kiri (Front Left)
constexpr uint8_t ENCODER_FL_A = 4;
constexpr uint8_t ENCODER_FL_B = 5;

// Encoder Belakang Kanan (Back Right)
constexpr uint8_t ENCODER_BR_A = 1;
constexpr uint8_t ENCODER_BR_B = 2;

// Encoder Belakang Kiri (Back Left)
constexpr uint8_t ENCODER_BL_A = 41;
constexpr uint8_t ENCODER_BL_B = 42;

// =====================================================================
//  JUMLAH ENCODER
// =====================================================================
constexpr size_t ENCODER_COUNT = 4;

// =====================================================================
//  RPM CALCULATION
// =====================================================================
constexpr uint32_t RPM_INTERVAL_MS = 40;

// =====================================================================
//  SHARED FUNCTION DECLARATIONS
// =====================================================================
void setupEncoders();
void convertEncoderToRPM();
float getEncoderVelocityRpm(int motorIdx);
float getEncoderVelocityRadS(int motorIdx);
float getEncoderYawRateRads();
float getEncoderConfidence();

#endif // ENCODER_H
