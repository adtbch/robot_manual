#pragma once

#include <vector> // Tambahkan header untuk std::vector
#include <Preferences.h>

// ============================================================
// PIN MOTOR - 2 pin per motor (H-bridge IN1/IN2, misal L298N)
// ============================================================

// Motor Depan (Front)
#define motorAxisX_A  35
#define motorAxisX_B  21
#define encoderMotorAxisX_A 8
#define encoderMotorAxisX_B 18
#define limitSwitchAxisX  3

#define motorAxisY_A   37
#define motorAxisY_B   36
#define encoderMotorAxisY_A   11
#define encoderMotorAxisY_B   10
#define limitSwitchAxisY   9

// Servo 
#define servoRotation  2
#define servoGrib  42

#define baudrate 921600
#define motion_serial_rxPin 7
#define motion_serial_txPin 6

#define serial_2_rxPin 4
#define serial_2_txPin 5

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
const uint8_t espNowAllowedTransmitterStaMac[6] = {0xD4, 0xE9, 0xF4, 0x8A, 0xBC, 0x3C};
const uint8_t espNowAllowedTransmitterApMac[6] = {0xD4, 0xE9, 0xF4, 0x8A, 0xBC, 0x3C};
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

struct MotorState {
	bool xhoming;
	bool zhoming;
	bool xAtCenter;
	bool zAtCenter;
};

// Deklarasi objek motor state
extern MotorState motorArm;

typedef struct {
	uint8_t encoderPinA;
	uint8_t encoderPinB;
	volatile long count;
} EncoderConfig;

typedef struct {
	uint8_t servoPin;// speed pin (LEDC PWM)
	uint8_t ledc_channel; 
} ServoConfig;

struct __attribute__((packed)) ControlPacket {
	uint16_t magic;      // Harus = ESPNOW_PACKET_MAGIC (0xA5B4)
	int16_t x;           // Gerakan lateral    (+= kanan, -= kiri)
	int16_t y;           // Gerakan maju/mundur (+= maju, -= mundur)
	int16_t w;           // Rotasi             (+= CCW,  -= CW)
	int8_t lx;           // Analog kiri  X
	int8_t ly;           // Analog kiri  Y
	int8_t rx;           // Analog kanan X
	int8_t ry;           // Analog kanan Y
	uint8_t l2Value;     // Trigger L2
	uint8_t r2Value;     // Trigger R2
	int16_t gyrX;        // Gyro sumbu X
	int16_t gyrY;        // Gyro sumbu Y
	int16_t gyrZ;        // Gyro sumbu Z
	uint32_t buttons;    // Bitmask semua tombol
	uint16_t seq;        // Nomor urut paket
	uint8_t  connected;  // 1 = PS4 terhubung, 0 = disconnect
};

extern std::vector<MotorConfig> motors;
extern std::vector<EncoderConfig> encoders;
// Forward declarations for globals (defined in motor.ino / encoder.ino)

const int maxPwm = 1023;
const int minPwm = -1023;
const int pwmFrequency = 20000;   // frekuensi PWM untuk motor (Hz)
const int pwmResolution = 10;   // resolusi PWM (bit)
const int servoFrequency = 50;
const int servoResolution = 14;

// ============================================================
// ARM CENTER POSITION CONFIGURATION (encoder counts)
// ============================================================
// Target encoder counts untuk posisi tengah setelah homing
// Sesuaikan nilai ini dengan range pergerakan fisik arm Anda
const long CENTER_POSITION_X = 2500;  // Target encoder count untuk axis X di tengah
const long CENTER_POSITION_Z = 500;  // Target encoder count untuk axis Z di tengah  // Toleransi error positioning (counts)
const int MOVE_SPEED = 300;  // Kecepatan PWM untuk move to center

// ============================================================
// ARM HOMING & POSITIONING LIMITS
// ============================================================
const int HOMING_SPEED = 500;  // Kecepatan PWM untuk homing ke limit switch
const long MAX_ENCODER_POSITION_X = 2000;  // Batas maksimum encoder X
const long MAX_ENCODER_POSITION_Z = 2000;  // Batas maksimum encoder Z

// ============================================================
// Shared function declarations
// ============================================================
void pwmMotor(int idMotor, int pwmValue);
void motorStopAll();
bool espNowControlInit();
void espNowControlTick();
bool espNowControlReadPacket(ControlPacket &outPacket);
bool espNowControlIsLinkAlive();
void motion_serial_init();
void motion_serial_tick();
bool motion_serialReadPacket(ControlPacket &outPacket);
void motion_serialPrintStats();
void mecanum_control_tick(const ControlPacket &pkt);
void gripper_init();
void gripper_tick(const ControlPacket &pkt);
void gripper_motor_tick(const ControlPacket &pkt);
void setServoAngle(int idServo, int angle);
void SetupMotors();
void setupServos();
void setupEncoders();
void setupLimits();

// Encoder functions
void resetEncoderCount(uint8_t motorIndex);
long getEncoderCount(uint8_t motorIndex);

// Arm positioning functions
bool setHoming();
bool moveToCenter();
bool moveTargetPosition(uint8_t motorIndex, long targetPosition);
void setMotorTarget(uint8_t motorIndex, long targetPosition);
void stopMotorTarget(uint8_t motorIndex);
void stopAllMotorTargets();
void updateMotorPositioning();
// Serial command functions
void setupSerialCommand();
void serialCommandTick();
