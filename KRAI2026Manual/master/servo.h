/*
 * =====================================================================
 * FILE    : servo.h
 * PERAN   : Konfigurasi modul servo (pin, frequency, resolution).
 *
 * BOARD   : ESP32-S3 (Master)
 * =====================================================================
 */

#ifndef SERVO_H
#define SERVO_H

#include "config.h"

// =====================================================================
//  STRUCT SERVO CONFIG
// =====================================================================
struct ServoConfig {
    char id;            // karakter unik: 'd'=depan, 't'=tengah, 'b'=belakang, dst
    uint8_t pin;
};

// =====================================================================
//  PIN SERVO — berdasarkan schematic KRAI 2026
// =====================================================================
constexpr uint8_t SERVO_1_PIN = 18;
constexpr uint8_t SERVO_2_PIN = 8;
constexpr uint8_t SERVO_3_PIN = 17;

// =====================================================================
//  JUMLAH SERVO
// =====================================================================
constexpr size_t SERVO_COUNT = 3;

// =====================================================================
//  PWM CONFIGURATION (servo standard 50Hz)
// =====================================================================
constexpr int SERVO_FREQUENCY  = 50;     // Hz (standard servo)
constexpr int SERVO_RESOLUTION = 14;     // bit (2^14 = 16384)
constexpr int SERVO_DUTY_MIN   = 500;    // us (0 derajat)
constexpr int SERVO_DUTY_MAX   = 2400;   // us (180 derajat)

// =====================================================================
//  SHARED FUNCTION DECLARATIONS
// =====================================================================
void setupServos();
void setServoAngle(char servoId, int angle);

#endif // SERVO_H
