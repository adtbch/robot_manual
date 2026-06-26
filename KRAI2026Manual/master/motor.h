/*
 * =====================================================================
 * FILE    : motor.h
 * PERAN   : Konfigurasi modul motor (pin, PWM, struct).
 *
 * BOARD   : ESP32-S3 (Master)
 * =====================================================================
 */

#ifndef MOTOR_H
#define MOTOR_H

#include "config.h"

// =====================================================================
//  STRUCT MOTOR CONFIG
// =====================================================================
struct MotorConfig {
    char id;            // karakter unik: 'x'=axis X, 'y'=axis Y, dst
    uint8_t pin_dir;    // direction pin (OUTPUT)
    uint8_t pin_pwm;    // speed pin (LEDC PWM)
};

// =====================================================================
//  PIN MOTOR — berdasarkan schematic KRAI 2026
// =====================================================================
// Motor1 (axis X / arm)
constexpr uint8_t MOTOR1_PIN_DIR  = 1;   // direction pin
constexpr uint8_t MOTOR1_PIN_PWM  = 2;   // speed pin (LEDC PWM)

// Motor2 (axis Y / arm)
constexpr uint8_t MOTOR2_PIN_DIR  = 42;   // direction pin
constexpr uint8_t MOTOR2_PIN_PWM  = 41;   // speed pin (LEDC PWM)

// =====================================================================
//  JUMLAH MOTOR
// =====================================================================
constexpr size_t MOTOR_COUNT = 2;

// =====================================================================
//  PWM CONFIGURATION
// =====================================================================
constexpr int PWM_MAX        = 1023;
constexpr int PWM_MIN        = -1023;
constexpr int PWM_FREQUENCY  = 20000;   // Hz
constexpr int PWM_RESOLUTION = 10;      // bit (2^10 = 1024)

// Encoder travel limits — sesuaikan setelah kalibrasi hardware
constexpr long MOTOR_X_ENC_MIN = -2500;
constexpr long MOTOR_X_ENC_MAX =  2500;
constexpr long MOTOR_Y_ENC_MIN = -2500;
constexpr long MOTOR_Y_ENC_MAX =  2500;

// =====================================================================
//  SHARED FUNCTION DECLARATIONS
// =====================================================================
void SetupMotors();
void pwmMotor(char motorId, int pwmValue);
void motorStopAll();

// =====================================================================
//  MOTOR X — encoder positioning
// =====================================================================
void motorXSetTarget(long targetEncoder);
void motorXAdjustTarget(long deltaPulse);
long motorXGetTarget();
void motorXStop();
bool motorXIsActive();
void motorXPositionTick();

// =====================================================================
//  MOTOR Y — encoder positioning + hold anti-gravitasi di target
// =====================================================================
void motorYSetTarget(long targetEncoder);
void motorYAdjustTarget(long deltaPulse);
long motorYGetTarget();
void motorYStop();
bool motorYIsActive();
void motorYPositionTick();

#endif // MOTOR_H
