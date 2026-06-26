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

constexpr uint8_t INT_ENC_FR_A = 41;
constexpr uint8_t INT_ENC_FR_B = 42;
constexpr uint8_t INT_ENC_FL_A = 5;
constexpr uint8_t INT_ENC_FL_B = 4;
constexpr uint8_t INT_ENC_BR_A = 2;
constexpr uint8_t INT_ENC_BR_B = 1;
constexpr uint8_t INT_ENC_BL_A = 40;
constexpr uint8_t INT_ENC_BL_B = 39;

// =====================================================================
//  PIN — EXTERNAL WHEEL ENCODER (PCNT / ESP32Encoder)
//  Baca via PCNT hardware counter. Akurat untuk odometry.
// =====================================================================

constexpr uint8_t EXT_ENC_FR_A = 35;
constexpr uint8_t EXT_ENC_FR_B = 36;
constexpr uint8_t EXT_ENC_FL_A = 38;
constexpr uint8_t EXT_ENC_FL_B = 37;
constexpr uint8_t EXT_ENC_BR_A = 47;
constexpr uint8_t EXT_ENC_BR_B = 48;
constexpr uint8_t EXT_ENC_BL_A = 9;
constexpr uint8_t EXT_ENC_BL_B = 10;

// =====================================================================
//  KONSTANTA
// =====================================================================

constexpr size_t INT_ENCODER_COUNT = 4;
constexpr size_t EXT_ENCODER_COUNT = 4;
constexpr uint32_t RPM_INTERVAL_MS = 40;

constexpr float ROBOT_LX = 0.4225f;           // setengah lebar (m) -> 84.5cm / 2
constexpr float ROBOT_LY = 0.4175f;           // setengah panjang (m) -> 83.5cm / 2
constexpr float WHEEL_RADIUS_M = 0.03f;        // meter (3 cm)

// =====================================================================
//  ODOMETRY CONFIG (X-config omni wheels)
// =====================================================================
// External encoder PPR — sesuaikan dengan encoder yang dipasang
constexpr int EXT_ENCODER_PPR = 400;
// Internal motor encoder PPR
constexpr int ENCODER_PPR = 270;              // pulses per revolution
// Jarak pusat robot ke sumbu roda (m)
constexpr float ODOM_L = ROBOT_LX + ROBOT_LY;

// =====================================================================
//  SHARED FUNCTION DECLARATIONS
// =====================================================================

void setupEncoders();                       // Init internal + external

// Internal (ISR) — RPM buat PID
void convertEncoderToRPM();                 // Hitung RPM dari delta ISR count
float getEncoderVelocityRpm(int idx);       // Dapatkan RPM motor

// External (PCNT) — odometry
int64_t getExtEncoderCount(int idx);
void resetExtEncoderCount(int idx);
void updateOdometry();                      // Update pose dari external encoder
extern float odomX;                          // Posisi x robot (meter)
extern float odomY;                          // Posisi y robot (meter)
extern float odomTheta;                      // Heading robot (derajat)
void resetOdometry();                        // Reset pose ke (0, 0, 0)
void setOdomTheta(float theta);             // Set heading manual (dari MPU)

#endif // ENCODER_H
