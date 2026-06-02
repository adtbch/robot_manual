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
// Shared types and extern globals
// ============================================================

typedef struct {
	uint8_t pin_direction;    // direction pin (OUTPUT, HIGH=maju LOW=mundur)
	uint8_t pin_pwm;          // speed pin (LEDC PWM)
	uint8_t ledc_channel;     // LEDC channel              // apakah motor sedang dalam proses homing
} MotorConfig;

struct MotorX{
	bool homing;
};

struct MotorZ{
	bool homing;
};


typedef struct {
	uint8_t encoderPinA;
	uint8_t encoderPinB;
	volatile long count;
} EncoderConfig;

typedef struct {
	uint8_t servoPin;// speed pin (LEDC PWM)
	uint8_t ledc_channel; 
} ServoConfig;

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
#define motorAxisX_A  21
#define motorAxisX_B  35
#define encoderMotorAxisX_A 18
#define encoderMotorAxisX_B 8
#define limitSwitchAxisX  3

#define motorAxisY_A   37
#define motorAxisY_B   36
#define encoderMotorAxisY_A   10
#define encoderMotorAxisY_B   11
#define limitSwitchAxisY   9

// Servo 
#define servoRotation  2
#define servoGrib  42

#define baudrate 921600
#define serial_1_rxPin 7
#define serial_1_txPin 6

#define serial_2_rxPin 4
#define serial_2_txPin 5

const int maxPwm = 1023;
const int minPwm = -1023;
const int pwmFrequency = 20000;   // frekuensi PWM untuk motor (Hz)
const int pwmResolution = 10;   // resolusi PWM (bit)
const int servoFrequency = 50;
const int servoResolution = 14;

// ============================================================
// Shared function declarations
// ============================================================
void pwmMotor(int idMotor, int pwmValue);
void motorStopAll();
bool espNowControlInit();
void espNowControlTick();
bool espNowControlReadPacket(EspNowControlPacket &outPacket);
bool espNowControlIsLinkAlive();
  void setServoAngle(int idServo, int angle);
  void SetupMotors();
  void setupServos();
  void setupEncoders();
