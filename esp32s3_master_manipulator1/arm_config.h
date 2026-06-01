#pragma once

#include <Arduino.h>

// ============================================================
// ARM MOTOR CONFIGURATION - Sumbu X & Z Lengan 1
// ============================================================
// Motor menggunakan driver H-Bridge (misal BTS7960 atau L298N)
// dengan 2 pin PWM (L dan R) untuk kontrol arah dan kecepatan
// ============================================================

// ============================================================
// PIN MOTOR SUMBU Z (Vertical)
// ============================================================
#define MOTOR_Z_PWM_L  32   // PWM Left (mundur/turun)
#define MOTOR_Z_PWM_R  36   // PWM Right (maju/naik)
#define MOTOR_Z_ENC_A  10   // Encoder A
#define MOTOR_Z_ENC_B  11   // Encoder B
#define MOTOR_Z_LIMIT  9    // Limit switch (LOW = hit)

// ============================================================
// PIN MOTOR SUMBU X (Horizontal)
// ============================================================
#define MOTOR_X_PWM_L  21   // PWM Left (mundur/kiri)
#define MOTOR_X_PWM_R  35   // PWM Right (maju/kanan)
#define MOTOR_X_ENC_A  18   // Encoder A
#define MOTOR_X_ENC_B  8    // Encoder B
#define MOTOR_X_LIMIT  3    // Limit switch (LOW = hit)

// ============================================================
// PARAMETER MOTOR
// ============================================================
#define ARM_PWM_FREQUENCY  20000   // 20kHz PWM frequency
#define ARM_PWM_RESOLUTION 10      // 10-bit resolution (0-1023)
#define ARM_PWM_MAX        1023    // Maximum PWM value
#define ARM_PWM_MIN        0       // Minimum PWM value

// LEDC Channels untuk PWM
#define MOTOR_Z_LEDC_CH_L  0
#define MOTOR_Z_LEDC_CH_R  1
#define MOTOR_X_LEDC_CH_L  2
#define MOTOR_X_LEDC_CH_R  3

// ============================================================
// PIN SERVO - Capit & Rotasi
// ============================================================
#define SERVO_ROTATE_PIN   2    // Servo rotasi capit
#define SERVO_GRIPPER_PIN  42   // Servo buka/tutup capit

// LEDC Channels untuk Servo (50Hz PWM)
#define SERVO_ROTATE_LEDC_CH   4
#define SERVO_GRIPPER_LEDC_CH  5

// ============================================================
// PARAMETER SERVO
// ============================================================
#define SERVO_FREQUENCY    50      // 50Hz untuk servo standar
#define SERVO_RESOLUTION   16      // 16-bit resolution untuk presisi tinggi

// Pulse width dalam microseconds (sesuaikan dengan servo Anda)
#define SERVO_MIN_US       500     // Minimum pulse width (0 derajat)
#define SERVO_MAX_US       2500    // Maximum pulse width (180 derajat)
#define SERVO_CENTER_US    1500    // Center pulse width (90 derajat)

// Posisi servo dalam derajat (0-180)
#define GRIPPER_OPEN_ANGLE    10   // Derajat untuk capit terbuka
#define GRIPPER_CLOSE_ANGLE   90   // Derajat untuk capit tertutup
#define ROTATE_MIN_ANGLE      0    // Rotasi minimum
#define ROTATE_MAX_ANGLE      180  // Rotasi maksimum
#define ROTATE_CENTER_ANGLE   90   // Rotasi tengah

// ============================================================
// PARAMETER ENCODER
// ============================================================
#define ARM_ENCODER_PPR    270     // Pulses per revolution (sesuaikan dengan encoder Anda)

// ============================================================
// PARAMETER HOMING
// ============================================================
#define HOMING_SPEED_PWM   300     // PWM untuk homing (lambat untuk safety)
#define HOMING_TIMEOUT_MS  10000   // Timeout homing 10 detik
#define DEBOUNCE_DELAY_MS  50      // Debounce limit switch

// Posisi tengah (dalam pulsa encoder dari limit switch)
// SESUAIKAN dengan panjang travel sumbu Anda
#define CENTER_POSITION_X  1000    // Pulsa dari limit ke tengah sumbu X
#define CENTER_POSITION_Z  800     // Pulsa dari limit ke tengah sumbu Z

// ============================================================
// STRUKTUR DATA
// ============================================================

// Status motor
typedef enum {
  MOTOR_IDLE = 0,
  MOTOR_MOVING,
  MOTOR_HOMING,
  MOTOR_ERROR
} MotorState;

// Data encoder
typedef struct {
  volatile long count;       // Encoder count (volatile karena diubah di ISR)
  long targetPosition;       // Target position dalam pulsa
  long homePosition;         // Posisi home (0)
  long centerPosition;       // Posisi tengah
  bool isHomed;              // Sudah homing atau belum
} EncoderData;

// Data motor
typedef struct {
  uint8_t pwmPinL;           // Pin PWM Left
  uint8_t pwmPinR;           // Pin PWM Right
  uint8_t ledcChannelL;      // LEDC channel Left
  uint8_t ledcChannelR;      // LEDC channel Right
  uint8_t encPinA;           // Encoder pin A
  uint8_t encPinB;           // Encoder pin B
  uint8_t limitPin;          // Limit switch pin
  MotorState state;          // Status motor
  EncoderData encoder;       // Data encoder
} ArmMotor;

// ============================================================
// GLOBAL VARIABLES (extern - definisi di arm_motor.ino)
// ============================================================
extern ArmMotor motorX;
extern ArmMotor motorZ;

// ============================================================
// FUNCTION DECLARATIONS
// ============================================================

// Inisialisasi
bool armMotorInit();
bool armEncoderInit();

// Kontrol motor
void armMotorSetPWM(ArmMotor &motor, int pwmValue);  // -1023 to 1023
void armMotorStop(ArmMotor &motor);
void armMotorStopAll();

// Encoder
long armEncoderGetCount(ArmMotor &motor);
void armEncoderReset(ArmMotor &motor);

// Limit switch
bool armLimitSwitchPressed(ArmMotor &motor);

// Homing
bool armHomingStart();
void armHomingTick();
bool armHomingIsComplete();
bool armHomingIsRunning();

// Movement
bool armMoveToCenter();
bool armMoveToPosition(ArmMotor &motor, long targetPosition);
void armMovementTick();

// ============================================================
// SERVO FUNCTION DECLARATIONS
// ============================================================

// Inisialisasi servo
bool armServoInit();

// Kontrol servo rotasi (0-180 derajat)
void armServoRotateSetAngle(int angle);
int armServoRotateGetAngle();

// Kontrol servo capit (open/close)
void armServoGripperOpen();
void armServoGripperClose();
void armServoGripperSetAngle(int angle);
int armServoGripperGetAngle();
bool armServoGripperIsOpen();
