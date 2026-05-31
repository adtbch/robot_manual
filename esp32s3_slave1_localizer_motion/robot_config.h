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
	long prev_count; // Tambahkan prev_count untuk kalkulasi delta
	unsigned long last_time; // Tambahkan last_time untuk dt dinamis
} EncoderConfig;

typedef struct {
	float kp;
	float ki;
	float kd;
	float integral;
	float lastError;
	float lastTime;

	void reset() {
		integral = 0.0f;
		lastError = 0.0f;
		lastTime = 0.0f;
	}
} PIDState;

typedef struct {
	double Kp;
	double Ki;
	double Kd;
	double error;
	double integral;
	double derivative;
	double previousError;
	unsigned long lastTime;
} PIDData;

typedef struct {
	float kp;
	float ki;
	float kd;
	float integral;
	float prev_error;
} PID;

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
extern std::vector<PIDData> pidData;

// ============================================================
// PIN MOTOR - 2 pin per motor (H-bridge IN1/IN2
// ============================================================

// Motor Depan (Front)
#define motorDepanKanan_A  6
#define motorDepanKanan_B  7
#define encoderMotorDepanKanan_A  40
#define encoderMotorDepanKanan_B  39

#define motorDepanKiri_A   3
#define motorDepanKiri_B   8
#define encoderMotorDepanKiri_A   4
#define encoderMotorDepanKiri_B   5

// Motor Belakang (Back)
#define motorBelakangKanan_A  15
#define motorBelakangKanan_B  16
#define encoderMotorBelakangKanan_A  1
#define encoderMotorBelakangKanan_B  2

#define motorBelakangKiri_A   18
#define motorBelakangKiri_B   17
#define encoderMotorBelakangKiri_A   41
#define encoderMotorBelakangKiri_B   42


// ============================================================
// PARAMETER ROBOT — sesuaikan dengan hardware
// ============================================================
const float radiusRoda = 0.0635;   // radius roda dalam meter
const int encoderMotorPpr = 270;    // pulses per revolution encoder

const int maxPwm = 1023;
const int minPwm = -1023;
const int zeroPwm = 0;
const int pwmFrequency = 20000;   // frekuensi PWM untuk motor (Hz)
const int pwmResolution = 10;   // resolusi PWM (bit)
const float minrpm = -500.0f;
const float maxrpm = 500.0f;

// ============================================================
// PIN sda/scl untuk I2C (jika diperlukan, misal untuk IMU)
// ============================================================
#define sdaPin 13
#define sclPin  14

// ============================================================
// PIN Serial 1 untuk komunikasi dengan ESP32 Master 
// ============================================================
#define serial_1_rxPin 9
#define serial_1_txPin 10

// ============================================================
// PIN Serial 2 untuk komunikasi dengan Modul Radio
// ============================================================
#define serial_2_rxPin 11
#define serial_2_txPin 12
#define setRadionPin 19

#define AUTOTUNE_TARGET_RPM   100.0f // target velocity saat tuning dalam RPM — edit sesuai kebutuhan
#define AUTOTUNE_RUN_MS       10000  // durasi test per siklus (ms)
#define AUTOTUNE_COOLDOWN_MS  3000   // cooldown antar siklus (ms)
#define AUTOTUNE_MAX_CYCLES   12     // jumlah siklus per motor
#define AUTOTUNE_SHOW_MS      3000   // tampil hasil per motor sebelum lanjut (ms)


// ============================================================
// deklarasi variabel global untuk menyimpan RPM motor saat ini (update di encoder.ino)
// ============================================================
extern uint16_t rpmMotorDepanKanan;
extern uint16_t rpmMotorDepanKiri;
extern uint16_t rpmMotorBelakangKanan;
extern uint16_t rpmMotorBelakangKiri;

// ============================================================
// Shared function declarations
// ============================================================
void pwmMotor(int idMotor, int pwmValue);
void rpmMotor(int rpm1, int rpm2, int rpm3, int rpm4);
void pidControllerInit();
double computePID(int index, double setpoint, double input, double Kp, double Ki, double Kd, double Minintegral, double Maxintegral);
void rpmMotorControl(int targetRPM0, int targetRPM1, int targetRPM2, int targetRPM3);
void rpmMotorControlTargets(const std::vector<float> &targetRpm);
void motorStopAll();
void convertEncoderToRPM();
float getEncoderVelocityRpm(int motorIdx);
float getEncoderVelocityRadS(int motorIdx);
bool autoTunerIsActive();
void autoTunerStart();
void autoTunerTick(bool bootPressed);
void autoTunerStartSingle(int motorIdx, float initKp = -1.0f, float initKi = -1.0f, float initKd = -1.0f);
void autoTunerAbort();

// Serial commands
void printSerialUsage();
void processSerialCommands();

// Kinematik functions
void driveRobotCentric(int vx, int vy, int vtheta);
void driveFieldCentric(int vx, int vy, int vtheta);
void skalaKecepatan(int motor1, int motor2, int motor3, int motor4);

bool espNowControlInit();
void espNowControlTick();
bool espNowControlReadPacket(EspNowControlPacket &outPacket);
bool espNowControlIsLinkAlive();
