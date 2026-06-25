/*
 * =====================================================================
 * FILE    : encoder.h
 * PERAN   : Konfigurasi 2 jenis encoder:
 *           - Internal (motor): ISR interrupt → RPM buat PID
 *           - External (wheel): PCNT hardware → odometry
 *
 * BOARD   : ESP32-S3 (Slave1 Motion)
 * =====================================================================
 */

#ifndef ENCODER_H
#define ENCODER_H

#include "config.h"

// =====================================================================
//  PIN — INTERNAL MOTOR ENCODER (ISR based)
//  Baca via interrupt GPIO. RPM feedback untuk PID.
// =====================================================================

constexpr uint8_t INT_ENC_FR_A = 42;
constexpr uint8_t INT_ENC_FR_B = 41;
constexpr uint8_t INT_ENC_FL_A = 4;
constexpr uint8_t INT_ENC_FL_B = 5;
constexpr uint8_t INT_ENC_BR_A = 1;
constexpr uint8_t INT_ENC_BR_B = 2;
constexpr uint8_t INT_ENC_BL_A = 40;
constexpr uint8_t INT_ENC_BL_B = 39;

// =====================================================================
//  PIN — EXTERNAL WHEEL ENCODER (PCNT / ESP32Encoder)
//  Baca via PCNT hardware counter. Akurat untuk odometry.
// =====================================================================

constexpr uint8_t EXT_ENC_FR_A = 36;
constexpr uint8_t EXT_ENC_FR_B = 35;
constexpr uint8_t EXT_ENC_FL_A = 38;
constexpr uint8_t EXT_ENC_FL_B = 37;
constexpr uint8_t EXT_ENC_BR_A = 46;
constexpr uint8_t EXT_ENC_BR_B = 47;
constexpr uint8_t EXT_ENC_BL_A = 9;
constexpr uint8_t EXT_ENC_BL_B = 10;

// =====================================================================
//  KONSTANTA
// =====================================================================

constexpr size_t INT_ENCODER_COUNT = 4;
constexpr size_t EXT_ENCODER_COUNT = 4;
constexpr uint32_t RPM_INTERVAL_MS = 40;

// =====================================================================
//  SHARED FUNCTION DECLARATIONS
// =====================================================================

void setupEncoders();                       // Init internal + external

// Internal (ISR) — RPM buat PID
void convertEncoderToRPM();                 // Hitung RPM dari delta ISR count
float getEncoderVelocityRpm(int idx);       // Dapatkan RPM motor
float getEncoderVelocityRadS(int idx);      // Dapatkan kecepatan motor (rad/s)

// External (PCNT) — raw count buat odometry
int64_t getExtEncoderCount(int idx);
void resetExtEncoderCount(int idx);

// Yaw rate & confidence (dari internal encoder RPM)
float getEncoderYawRateRads();
float getEncoderConfidence();

#endif // ENCODER_H
