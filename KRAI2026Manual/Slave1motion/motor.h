/*
 * =====================================================================
 * FILE    : motor.h
 * PERAN   : Konfigurasi modul motor (pin, PWM, struct).
 *           4 motor mecanum drive.
 *
 * BOARD   : ESP32-S3 (Slave1 Motion)
 * =====================================================================
 */

#ifndef MOTOR_H
#define MOTOR_H

#include "config.h"

// =====================================================================
//  PIN MOTOR — berdasarkan schematic KRAI 2026
// =====================================================================

// Motor Depan Kanan (Front Right)
constexpr uint8_t MOTOR_FR_DIR = 6;
constexpr uint8_t MOTOR_FR_PWM = 7;

// Motor Depan Kiri (Front Left)
constexpr uint8_t MOTOR_FL_DIR = 3;
constexpr uint8_t MOTOR_FL_PWM = 8;

// Motor Belakang Kanan (Back Right)
constexpr uint8_t MOTOR_BR_DIR = 15;
constexpr uint8_t MOTOR_BR_PWM = 16;

// Motor Belakang Kiri (Back Left)
constexpr uint8_t MOTOR_BL_DIR = 18;
constexpr uint8_t MOTOR_BL_PWM = 17;

// =====================================================================
//  JUMLAH MOTOR
// =====================================================================
constexpr size_t MOTOR_COUNT = 4;

// =====================================================================
//  RPM LIMITS
// =====================================================================
constexpr float RPM_MIN = -500.0f;
constexpr float RPM_MAX = 500.0f;

// =====================================================================
//  SHARED FUNCTION DECLARATIONS
// =====================================================================
void SetupMotors();
void pwmMotor(int idMotor, int pwmValue);
void motorStopAll();

#endif // MOTOR_H
