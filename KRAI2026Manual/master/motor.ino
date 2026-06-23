/*
 * =====================================================================
 * FILE    : motor.ino
 * PERAN   : Setup & control motor via H-bridge (L298N/drv8833).
 *           2 motor: Motor1 (axis X), Motor2 (axis Y).
 *
 * BOARD   : ESP32-S3 (Master)
 * =====================================================================
 */

#include "motor.h"

// =====================================================================
//  STATE
// =====================================================================

namespace {
    MotorConfig motors[MOTOR_COUNT] = {
        {MOTOR1_PIN_DIR, MOTOR1_PIN_PWM, 0},
        {MOTOR2_PIN_DIR, MOTOR2_PIN_PWM, 1},
    };
}
// =====================================================================
//  SETUP
// =====================================================================

void SetupMotors() {
    for (size_t i = 0; i < MOTOR_COUNT; i++) {
        pinMode(motors[i].pin_dir, OUTPUT);
        digitalWrite(motors[i].pin_dir, LOW);

        ledcSetup(motors[i].ledc_channel, PWM_FREQUENCY, PWM_RESOLUTION);
        ledcAttachPin(motors[i].pin_pwm, motors[i].ledc_channel);
        ledcWrite(motors[i].ledc_channel, 0);
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
        ledcWrite(motors[idMotor].ledc_channel, pwmValue);
        digitalWrite(motors[idMotor].pin_dir, LOW);
    } else if (pwmValue < 0) {
        ledcWrite(motors[idMotor].ledc_channel, PWM_MAX + pwmValue);
        digitalWrite(motors[idMotor].pin_dir, HIGH);
    } else {
        ledcWrite(motors[idMotor].ledc_channel, 0);
        digitalWrite(motors[idMotor].pin_dir, LOW);
    }
}

void motorStopAll() {
    for (size_t i = 0; i < MOTOR_COUNT; i++) {
        pwmMotor(i, 0);
    }
}
