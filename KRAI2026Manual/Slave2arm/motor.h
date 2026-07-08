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
    char id;            // karakter unik: '1'-'4' atau bebas
    uint8_t pin1;       // pin 1 (direction)
    uint8_t pin2;       // pin 2 (direction)
};

// =====================================================================
//  PIN MOTOR — berdasarkan schematic KRAI 2026
// =====================================================================
// MotorXArmKanan (axis X arm kanan)
constexpr uint8_t MOTOR_XARM_KANAN_PIN1 = 6;
constexpr uint8_t MOTOR_XARM_KANAN_PIN2 = 5;

// MotorYArmKanan (axis Y arm kanan)
constexpr uint8_t MOTOR_YARM_KANAN_PIN1 = 4;
constexpr uint8_t MOTOR_YARM_KANAN_PIN2 = 7;

// MotorXArmKiri (axis X arm kiri)
constexpr uint8_t MOTOR_XARM_KIRI_PIN1 = 3;
constexpr uint8_t MOTOR_XARM_KIRI_PIN2 = 8;

// Motor4 — ponytail: deadcode, belum dipakai
// constexpr uint8_t MOTOR4_UNUSED_PIN1 = 8;
// constexpr uint8_t MOTOR4_UNUSED_.PIN2 = 3;

// =====================================================================
//  JUMLAH MOTOR
// =====================================================================
constexpr size_t MOTOR_COUNT = 3;  // ponytail: 4th motor deadcode dulu

// =====================================================================
//  PWM CONFIGURATION
// =====================================================================
constexpr int PWM_MAX        = 1023;
constexpr int PWM_MIN        = -1023;
constexpr int PWM_FREQUENCY  = 20000;   // Hz
constexpr int PWM_RESOLUTION = 10;      // bit (2^10 = 1024)

// =====================================================================
//  MOTOR Y — encoder position (bang-bang, tanpa PID)
// =====================================================================
constexpr int  MOTOR_Y_MOVE_PWM = 700;
constexpr long MOTOR_Y_ENC_MIN  = 0;
constexpr long MOTOR_Y_ENC_MAX  = 4058;
constexpr long MOTOR_Y_TOLERANCE = 5;

// =====================================================================
//  MOTOR X/K — continuous run with limit switch stop
// =====================================================================
constexpr int MOTOR_XK_MOVE_PWM = 400;

// =====================================================================
//  SHARED FUNCTION DECLARATIONS
// =====================================================================
void SetupMotors();
void pwmMotor(char motorId, int pwmValue);
void motorStopAll();

// Motor Y — encoder positioning
void motorYSetTarget(long targetEncoder);
void motorYAdjustTarget(long deltaPulse);
long motorYGetTarget();
void motorYStop();
bool motorYIsActive();
void motorYPositionTick();
void motorYLimitTick();

// Motor X — continuous run with limit switch
void motorRunStart(char id, int pwm);
void motorRunStop(char id);
void motorRunStopAll();
bool motorRunIsActive(char id);
void motorRunTick();

// Shared motor command — serial + web API
bool executeMotorCommand(char motorId, int pwm);
int motorRunGetPwm(char id);
int motorYGetLastPwm();

// Homing — blocking, dipanggil dari setup()
void motorYHoming();
void motorXHoming();

#endif // MOTOR_H
