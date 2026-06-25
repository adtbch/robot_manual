/*
 * =====================================================================
 * FILE    : config.h
 * PERAN   : Pusat konfigurasi KRAI 2026 Slave1 Motion Board.
 *           Shared types yang dipakai oleh SEMUA modul.
 *
 * BOARD   : ESP32-S3 (Slave1 Motion)
 * =====================================================================
 */

#ifndef CONFIG_H
#define CONFIG_H

#include <Arduino.h>

// =====================================================================
//  UNIT CONVERSION HELPERS
// =====================================================================
constexpr float RAD_PER_SEC_TO_RPM = 9.54929659f;   // 30 / π
constexpr float RPM_TO_RAD_PER_SEC = 0.10471976f;   // π / 30

// =====================================================================
//  PID & NVS CONFIGURATION
// =====================================================================
static constexpr const char* PID_NVS_NAMESPACE = "pid_tuning";
static constexpr float KP_MIN = 0.1f;
static constexpr float KP_MAX = 500.0f;
static constexpr float KI_MIN = 0.0f;
static constexpr float KI_MAX = 100.0f;
static constexpr float KD_MIN = 0.0f;
static constexpr float KD_MAX = 10.0f;

// =====================================================================
//  BOOT BUTTON
// =====================================================================
constexpr uint8_t BOOT_BUTTON_PIN = 0;

// =====================================================================
//  ROBOT PARAMETERS
// =====================================================================
constexpr float WHEEL_RADIUS_M = 0.0635f;     // meter
constexpr int ENCODER_PPR = 270;              // pulses per revolution

// Robot geometry (wheelbase)
constexpr float ROBOT_LX = 0.1325f;           // setengah lebar (m)
constexpr float ROBOT_LY = 0.0925f;           // setengah panjang (m)

// =====================================================================
//  PWM CONFIGURATION
// =====================================================================
constexpr int PWM_MAX = 1023;
constexpr int PWM_MIN = -1023;
constexpr int PWM_ZERO = 0;
constexpr int PWM_FREQUENCY = 20000;          // Hz
constexpr int PWM_RESOLUTION = 10;            // bit

// =====================================================================
//  STRUCTS
// =====================================================================

struct Jeda {
    uint32_t lastMs = 0;

    bool check(uint32_t intervalMs) {
        const uint32_t nowMs = millis();
        if (nowMs - lastMs < intervalMs) {
            return false;
        }
        lastMs = nowMs;
        return true;
    }

    void reset() {
        lastMs = millis();
    }
};

struct __attribute__((packed)) ControlPacket {
    uint16_t magic;
    int16_t x;
    int16_t y;
    int16_t w;
    int8_t lx;
    int8_t ly;
    int8_t rx;
    int8_t ry;
    uint8_t l2Value;
    uint8_t r2Value;
    int16_t gyrX;
    int16_t gyrY;
    int16_t gyrZ;
    uint32_t buttons;
    uint16_t seq;
    uint8_t connected;
    uint8_t command;
};

// =====================================================================
//  MOTOR STRUCT
// =====================================================================
struct MotorConfig {
    uint8_t pin_dir;
    uint8_t pin_pwm;
};

// =====================================================================
//  ENCODER STRUCT
// =====================================================================
struct EncoderConfig {
    uint8_t pin_a;
    uint8_t pin_b;
};

// =====================================================================
//  PID STATE
// =====================================================================
struct PIDState {
    float kp = 0.0f;
    float ki = 0.0f;
    float kd = 0.0f;
    float integral = 0.0f;
    float lastError = 0.0f;
    float lastTime = 0.0f;
    float lastTarget = 0.0f;

    void reset() {
        integral = 0.0f;
        lastError = 0.0f;
        lastTime = 0.0f;
        lastTarget = 0.0f;
    }
};

#endif // CONFIG_H
