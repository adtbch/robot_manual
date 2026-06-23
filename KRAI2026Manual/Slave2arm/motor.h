/*
 * =====================================================================
 * FILE    : motor.h
 * PERAN   : Konfigurasi modul motor (pin, PWM, struct).
 *           4 motor untuk arm manipulator.
 *
 * BOARD   : ESP32-S3 (Slave2 Arm)
 * =====================================================================
 */

#ifndef MOTOR_H
#define MOTOR_H

#include "config.h"

// =====================================================================
//  STRUCT MOTOR CONFIG
// =====================================================================
struct MotorConfig {
    uint8_t pin1;         // pin 1 (direction)
    uint8_t pin2;         // pin 2 (direction)
    uint8_t ledc_channel; // LEDC channel
};

// =====================================================================
//  PIN MOTOR — berdasarkan schematic KRAI 2026
// =====================================================================
// Motor1
constexpr uint8_t MOTOR1_PIN_1 = 4;
constexpr uint8_t MOTOR1_PIN_2 = 7;

// Motor2
constexpr uint8_t MOTOR2_PIN_1 = 6;
constexpr uint8_t MOTOR2_PIN_2 = 5;

// Motor3
constexpr uint8_t MOTOR3_PIN_1 = 17;
constexpr uint8_t MOTOR3_PIN_2 = 18;

// Motor4
constexpr uint8_t MOTOR4_PIN_1 = 8;
constexpr uint8_t MOTOR4_PIN_2 = 3;

// =====================================================================
//  JUMLAH MOTOR
// =====================================================================
constexpr size_t MOTOR_COUNT = 4;

// =====================================================================
//  PWM CONFIGURATION
// =====================================================================
constexpr int PWM_MAX        = 1023;
constexpr int PWM_MIN        = -1023;
constexpr int PWM_FREQUENCY  = 20000;   // Hz
constexpr int PWM_RESOLUTION = 10;      // bit (2^10 = 1024)

// =====================================================================
//  SHARED FUNCTION DECLARATIONS
// =====================================================================
void SetupMotors();
void pwmMotor(int idMotor, int pwmValue);
void motorStopAll();

#endif // MOTOR_H
