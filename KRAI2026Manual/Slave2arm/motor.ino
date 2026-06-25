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
    {'1', MOTOR1_PIN_1, MOTOR1_PIN_2},
    {'2', MOTOR2_PIN_1, MOTOR2_PIN_2},
    {'3', MOTOR3_PIN_1, MOTOR3_PIN_2},
    {'4', MOTOR4_PIN_1, MOTOR4_PIN_2},
};

// Cari index berdasarkan id. Return -1 jika tidak ditemukan.
int findMotorIndex(char id) {
    for (size_t i = 0; i < MOTOR_COUNT; i++) {
        if (motors[i].id == id) return (int)i;
    }
    return -1;
}

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

        ledcAttach(motors[i].pin1, PWM_FREQUENCY, PWM_RESOLUTION);
        ledcWrite(motors[i].pin1, 0);
    }
}

// =====================================================================
//  CONTROL
// =====================================================================

void pwmMotor(char motorId, int pwmValue) {
    int idx = findMotorIndex(motorId);
    if (idx < 0) return;

    pwmValue = constrain(pwmValue, PWM_MIN, PWM_MAX);

    if (pwmValue > 0) {
        digitalWrite(motors[idx].pin2, LOW);
        ledcWrite(motors[idx].pin1, pwmValue);
    } else if (pwmValue < 0) {
        digitalWrite(motors[idx].pin2, HIGH);
        ledcWrite(motors[idx].pin1, PWM_MAX + pwmValue);
    } else {
        ledcWrite(motors[idx].pin1, 0);
        digitalWrite(motors[idx].pin2, LOW);
    }
}

void motorStopAll() {
    for (size_t i = 0; i < MOTOR_COUNT; i++) {
        pwmMotor(motors[i].id, 0);
    }
}
