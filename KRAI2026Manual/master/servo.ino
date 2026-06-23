/*
 * =====================================================================
 * FILE    : servo.ino
 * PERAN   : Control servo via LEDC PWM.
 *           3 servo: Servo1, Servo2, Servo3.
 *
 * LIBRARY : Arduino ESP32 Core (LEDC)
 *
 * BOARD   : ESP32-S3 (Master)
 * =====================================================================
 */

#include "servo.h"

// =====================================================================
//  STATE
// =====================================================================

namespace {

    ServoConfig servos[SERVO_COUNT] = {
        {SERVO_1_PIN, 2},   // ch 0-1 dipakai motor
        {SERVO_2_PIN, 3},
        {SERVO_3_PIN, 4},
    };

} // anonymous namespace

// =====================================================================
//  SETUP
// =====================================================================

void setupServos() {
    for (size_t i = 0; i < SERVO_COUNT; i++) {
        ledcSetup(servos[i].ledc_channel, SERVO_FREQUENCY, SERVO_RESOLUTION);
        ledcAttachPin(servos[i].pin, servos[i].ledc_channel);
    }
}

// =====================================================================
//  CONTROL — set sudut servo (0-180 derajat)
// =====================================================================

void setServoAngle(uint8_t servoIndex, int angle) {
    if (servoIndex >= SERVO_COUNT) {
        return;
    }

    angle = constrain(angle, 0, 180);

    // Konversi sudut ke pulse width (500us - 2400us)
    long pulseWidth = map(angle, 0, 180, SERVO_DUTY_MIN, SERVO_DUTY_MAX);

    // Konversi pulse width ke duty cycle (14-bit: 0 - 16383)
    // Rumus: (pulseWidth / period_20000us) × max_duty
    long duty = (pulseWidth * ((1L << SERVO_RESOLUTION) - 1)) / 20000;

    ledcWrite(servos[servoIndex].ledc_channel, duty);
}
