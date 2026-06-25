/*
 * =====================================================================
 * FILE    : motor.ino
 * PERAN   : Setup & control 4 motor mecanum drive.
 *
 * BOARD   : ESP32-S3 (Slave1 Motion)
 * =====================================================================
 */

#include "motor.h"

// =====================================================================
//  STATE
// =====================================================================

namespace {

MotorConfig motors[MOTOR_COUNT] = {
    {MOTOR_FR_DIR, MOTOR_FR_PWM},   // 0: Front Right
    {MOTOR_FL_DIR, MOTOR_FL_PWM},   // 1: Front Left
    {MOTOR_BR_DIR, MOTOR_BR_PWM},   // 2: Back Right
    {MOTOR_BL_DIR, MOTOR_BL_PWM},   // 3: Back Left
};

} // anonymous namespace

// =====================================================================
//  SETUP
// =====================================================================

void SetupMotors() {
    for (size_t i = 0; i < MOTOR_COUNT; i++) {
        pinMode(motors[i].pin_dir, OUTPUT);
        digitalWrite(motors[i].pin_dir, LOW);

        ledcAttach(motors[i].pin_pwm, PWM_FREQUENCY, PWM_RESOLUTION);
        ledcWrite(motors[i].pin_pwm, 0);
    }
}

// =====================================================================
//  CONTROL
// =====================================================================

void pwmMotor(int idMotor, int pwmValue) {
    if (idMotor < 0 || (size_t)idMotor >= MOTOR_COUNT) {
        return;
    }

    pwmValue = constrain(pwmValue, PWM_MIN, PWM_MAX);

    if (pwmValue > 0) {
        digitalWrite(motors[idMotor].pin_dir, LOW);
        ledcWrite(motors[idMotor].pin_pwm, pwmValue);
    } else if (pwmValue < 0) {
        digitalWrite(motors[idMotor].pin_dir, HIGH);
        ledcWrite(motors[idMotor].pin_pwm, PWM_MAX + pwmValue);
    } else {
        ledcWrite(motors[idMotor].pin_pwm, 0);
        digitalWrite(motors[idMotor].pin_dir, LOW);
    }
}

void motorStopAll() {
    for (size_t i = 0; i < MOTOR_COUNT; i++) {
        pwmMotor(i, 0);
    }
}
