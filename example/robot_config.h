#pragma once

#include <vector> // Tambahkan header untuk std::vector
#include <Preferences.h>

// ============================================================
// PID & NVS Configuration
// ============================================================
#define PID_NVS_NAMESPACE  "pid_tuning"
const float kpMin = 0.1f;
const float kpMax = 500.0f;
const float kiMin = 0.0f;
const float kiMax = 100.0f;
const float kdMin = 0.0f;
const float kdMax = 10.0f;

// ============================================================
// Boot Button Pin (for auto-tuner trigger)
// ============================================================
#define BOOT_BUTTON_PIN  0  // ESP32 default BOOT pin

// ============================================================
// ESP-NOW Manual Control Configuration
// ============================================================
#define ESPNOW_PACKET_MAGIC  0xA5B4
const bool espNowEnableMacWhitelist = true;
const uint8_t espNowAllowedTransmitterStaMac[6] = {0x58, 0xBF, 0x25, 0x8B, 0xDB, 0x18};
const uint8_t espNowAllowedTransmitterApMac[6] = {0x58, 0xBF, 0x25, 0x8B, 0xDB, 0x19};
const uint8_t espNowChannel = 1;
const unsigned long espNowLinkAliveMs = 180;
const unsigned long espNowStatsIntervalMs = 1000;

// ============================================================
// Unit conversion helpers (rad/s ↔ RPM)
// ============================================================
constexpr float kRadPerSecToRpm = 9.54929659f;  // 30 / π

constexpr float kRpmToRadPerSec = 0.10471976f; // π / 30

// ============================================================
// Shared types and extern globals
// ============================================================

typedef struct {
	uint8_t pin_direction;    // direction pin (OUTPUT, HIGH=maju LOW=mundur)
	uint8_t pin_pwm;          // speed pin (LEDC PWM)
	uint8_t ledc_channel;     // LEDC channel
} MotorConfig;

typedef struct {
	uint8_t encoderPinA;
	uint8_t encoderPinB;
	volatile long count;
} EncoderConfig;

typedef struct {
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
} EspNowControlPacket;

// Forward declarations for globals (defined in motor.ino / encoder.ino)
extern std::vector<MotorConfig> motors;
extern std::vector<EncoderConfig> encoders;

// ============================================================
// PIN MOTOR - 2 pin per motor (H-bridge IN1/IN2, misal L298N)
// ============================================================

// Motor Depan (Front)
#define motorDepanKanan_A  15
#define motorDepanKanan_B  16
#define encoderMotorDepanKanan_A  1
#define encoderMotorDepanKanan_B  2

#define motorDepanKiri_A   4
#define motorDepanKiri_B   5
#define encoderMotorDepanKiri_A   39
#define encoderMotorDepanKiri_B   40

// Motor Belakang (Back)
#define motorBelakangKanan_A  8
#define motorBelakangKanan_B  3
#define encoderMotorBelakangKanan_A  38
#define encoderMotorBelakangKanan_B  37

#define motorBelakangKiri_A   17
#define motorBelakangKiri_B   18
#define encoderMotorBelakangKiri_A   47
#define encoderMotorBelakangKiri_B   48

// ============================================================
// PARAMETER ROBOT — sesuaikan dengan hardware
// ============================================================
const float radiusRoda = 0.0635;   // radius roda dalam meter
const int encoderMotorPpr = 270;    // pulses per revolution encoder

const int maxPwm = 1023;
const int minPwm = -1023;
const int pwmFrequency = 20000;   // frekuensi PWM untuk motor (Hz)
const int pwmResolution = 10;   // resolusi PWM (bit)


#define AUTOTUNE_TARGET_VEL   3.0f   // target velocity saat tuning (rad/s) — edit sesuai kebutuhan
#define AUTOTUNE_RUN_MS       10000  // durasi test per siklus (ms)
#define AUTOTUNE_COOLDOWN_MS  3000   // cooldown antar siklus (ms)
#define AUTOTUNE_MAX_CYCLES   12     // jumlah siklus per motor
#define AUTOTUNE_SHOW_MS      3000   // tampil hasil per motor sebelum lanjut (ms)

// ============================================================
// Shared function declarations
// ============================================================
void pwmMotor(int idMotor, int pwmValue);
void pidControllerInit();
void rpmMotorControl(int targetRPM0, int targetRPM1, int targetRPM2, int targetRPM3);
void rpmMotorControlTargets(const std::vector<float> &targetRpm);
void motorStopAll();
void convertEncoderToRPM();
bool autoTunerIsActive();
void autoTunerStart();
void autoTunerTick(bool bootPressed);

bool espNowControlInit();
void espNowControlTick();
bool espNowControlReadPacket(EspNowControlPacket &outPacket);
bool espNowControlIsLinkAlive();
