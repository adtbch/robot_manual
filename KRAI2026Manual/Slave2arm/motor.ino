/*
 * =====================================================================
 * FILE    : motor.ino
 * PERAN   : Setup & control 4 motor arm via H-bridge.
 *
 * BOARD   : ESP32-S3 (Slave2 Arm)
 * =====================================================================
 */

#include "motor.h"

// =====================================================================
//  STATE
// =====================================================================

namespace {

MotorConfig motors[MOTOR_COUNT] = {
    {MOTOR1_PIN_1, MOTOR1_PIN_2, 0},
    {MOTOR2_PIN_1, MOTOR2_PIN_2, 1},
    {MOTOR3_PIN_1, MOTOR3_PIN_2, 2},
    {MOTOR4_PIN_1, MOTOR4_PIN_2, 3},
};

} // anonymous namespace

// =====================================================================
//  SETUP
// =====================================================================

void SetupMotors() {
    for (size_t i = 0; i < MOTOR_COUNT; i++) {
        pinMode(motors[i].pin1, OUTPUT);
        pinMode(motors[i].pin2, OUTPUT);
        digitalWrite(motors[i].pin1, LOW);
        digitalWrite(motors[i].pin2, LOW);

        ledcSetup(motors[i].ledc_channel, PWM_FREQUENCY, PWM_RESOLUTION);
        ledcAttachPin(motors[i].pin1, motors[i].ledc_channel);
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
        digitalWrite(motors[idMotor].pin2, LOW);
        ledcWrite(motors[idMotor].ledc_channel, pwmValue);
    } else if (pwmValue < 0) {
        digitalWrite(motors[idMotor].pin2, HIGH);
        ledcWrite(motors[idMotor].ledc_channel, PWM_MAX + pwmValue);
    } else {
        ledcWrite(motors[idMotor].ledc_channel, 0);
        digitalWrite(motors[idMotor].pin2, LOW);
    }
}

void motorStopAll() {
    for (size_t i = 0; i < MOTOR_COUNT; i++) {
        pwmMotor(i, 0);
    }
}
