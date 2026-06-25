/*
 * =====================================================================
 * FILE    : servo.ino
 * PERAN   : Control servo via LEDC PWM.
 *           3 servo: 'd'=depan, 't'=tengah, 'b'=belakang.
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
        {'d', SERVO_1_PIN},   // depan
        {'t', SERVO_2_PIN},   // tengah
        {'b', SERVO_3_PIN},   // belakang
    };

    // Cari index berdasarkan id. Return -1 jika tidak ditemukan.
    int findServoIndex(char id) {
        for (size_t i = 0; i < SERVO_COUNT; i++) {
            if (servos[i].id == id) return (int)i;
        }
        return -1;
    }

} // anonymous namespace

// =====================================================================
//  SETUP
// =====================================================================

void setupServos() {
    for (size_t i = 0; i < SERVO_COUNT; i++) {
        ledcAttach(servos[i].pin, SERVO_FREQUENCY, SERVO_RESOLUTION);
    }
}

// =====================================================================
//  CONTROL — set sudut servo (0-180 derajat)
// =====================================================================

void setServoAngle(char servoId, int angle) {
    int idx = findServoIndex(servoId);
    if (idx < 0) return;

    angle = constrain(angle, 0, 180);

    // Konversi sudut ke pulse width (500us - 2400us)
    long pulseWidth = map(angle, 0, 180, SERVO_DUTY_MIN, SERVO_DUTY_MAX);

    // Konversi pulse width ke duty cycle (14-bit: 0 - 16383)
    // Rumus: (pulseWidth / period_20000us) × max_duty
    long duty = (pulseWidth * ((1L << SERVO_RESOLUTION) - 1)) / 20000;

    ledcWrite(servos[idx].pin, duty);
}
