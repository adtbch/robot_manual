#pragma once

#include <vector> // Tambahkan header untuk std::vector
#include <Preferences.h>

#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps612.h"

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
	long last_delta;  // delta tick terakhir (untuk confidence filter encoder)
} EncoderConfig;

typedef struct {
	float kp;
	float ki;
	float kd;
	float integral;
	float lastError;
	float lastTime;
	float lastTarget;  // Track previous target untuk deteksi perubahan

	void reset() {
		integral = 0.0f;
		lastError = 0.0f;
		lastTime = 0.0f;
		lastTarget = 0.0f;
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
	double previousInput;
	unsigned long lastTime;
} PIDData;

typedef struct {
	float kp;
	float ki;
	float kd;
	float integral;
	float prev_error;
} PID;

// Magic number validasi paket (harus sama dengan esp32controller)
constexpr uint16_t kPacketMagic = 0xA5B4;

struct __attribute__((packed)) ControlPacket {
	uint16_t magic;      // Harus = kPacketMagic (0xA5B4)
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
	uint8_t  command;    // Perintah aksi dari Controller
};

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
const float radiusRoda = 0.0635f;   // radius roda dalam meter
const int encoderMotorPpr = 270;    // pulses per revolution encoder

// Robot geometry (wheelbase) — setengah jarak antar roda
constexpr float ROBOT_Lx = 0.1325f; // (0.265/2) setengah lebar kiri-kanan dalam meter
constexpr float ROBOT_Ly = 0.0925f; // (0.185/2) setengah panjang depan-belakang dalam meter

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
const int sdaPin = 13;
const int sclPin = 14;

// ============================================================
// PIN Serial 1 untuk komunikasi dengan ESP32 Master 
// ============================================================
#define master_serial_rxPin 21
#define master_serial_txPin 20

// ============================================================
// PIN WSN-31 untuk komunikasi dengan Modul Radio
// ============================================================
#define wsn_serial_rxPin 12
#define wsn_serial_txPin 11
#define kWsnSetPin 19

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
int pidCompute(PIDState &pid, float target, float current, float dt);
int pidCompute(int motorIdx, float targetRPM, float dt);
double computePID(int index, double setpoint, double input, double Kp, double Ki, double Kd, double Minintegral, double Maxintegral);
void rpmMotorControl(int targetRPM0, int targetRPM1, int targetRPM2, int targetRPM3);
void rpmMotorControlTargets(const std::vector<float> &targetRpm);
void motorStopAll();
void convertEncoderToRPM();
float getEncoderVelocityRpm(int motorIdx);
float getEncoderVelocityRadS(int motorIdx);
float getEncoderYawRateRads(); // yaw rate dari 4 encoder (rad/s)
float getEncoderConfidence();  // confidence [0-1] dari max delta tick
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
void driveFieldCentricWithYawCorrection(int vx, int vy, int yawTarget);
int pidComputeYaw(PIDState &pid, float target, float current, float dt);
void initYawPid();
void saveYawPid();
void showYawPid();
extern PIDState pidKinematicYaw;
void skalaKecepatan(int motor1, int motor2, int motor3, int motor4);

// ============================================================
// IMU / MPU9250 Declarations
// ============================================================
bool setupMPUWithMagnetic();  // fused gyro+mag (yaw relatif dari heading awal)
bool setupMPUGyro();          // gyro only, manual integration (drift accumulate)
bool setupMPU();              // alias ke WithMagnetic (compatibilitas)
void calibrateGyro();         // kalibrasi accel+gyro+mag (skip mag jika gyro-only)
void calibrateGyroHot();      // hot recalibration via serial
void updateYaw();             // panggil setiap loop
float getYaw();              // return yaw dalam derajat, rentang -180..180
float getFilteredGyroZ();     // return filtered gyro Z (untuk debugging/diagnostic)
void resetYaw();             // reset reference ke heading saat ini (yaw=0)

// Tuning filter gyro-only (sesuaikan dengan noise motor)
// Nilai: 0.05 (sangat smooth) - 0.5 (responsive)
// default 0.2 di mpu.ino (GYRO_FILTER_ALPHA)
void setGyroFilterAlpha(float alpha);

// ============================================================
// Gyro + Encoder Complementary Filter (gyro-only mode)
// ============================================================
float getEncoderYawRateRads();  // yaw rate dari 4 encoder (rad/s)
float getEncoderConfidence();   // confidence [0-1] dari max delta tick

bool espNowControlInit();
void espNowControlTick();
bool espNowControlReadPacket(ControlPacket &outPacket);
bool espNowControlIsLinkAlive();

// ============================================================
// Serial Master Communication Declarations
// ============================================================

namespace SerialMaster {
  // Function declarations
  bool serialMasterInit();
  void serialMasterTick();
  bool getMotorCommand(int16_t &vx, int16_t &vy, int16_t &vtheta);
  bool isStopRequested();
  bool isStatusRequested();
  bool sendStatusReply(uint16_t rpm1, uint16_t rpm2, uint16_t rpm3, uint16_t rpm4, uint8_t status);
  bool sendOdometryData(float posX, float posY, float heading);
  bool isLinkAlive();
  void printStats();
}

#define INTERRUPT_PIN 46