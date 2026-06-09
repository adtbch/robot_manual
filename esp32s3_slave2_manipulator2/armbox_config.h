#pragma once

#include <vector>

// ============================================================
// PIN MOTOR - 2 pin per motor (H-bridge RPWM/LPWM)
// ============================================================

// Sumbu W (Motor Putar / Rotasi)
#define motorAxisW_A        16
#define motorAxisW_B        15
#define encoderMotorAxisW_A   39
#define encoderMotorAxisW_B   40
#define limitSwitchAxisW       10

// Sumbu Z (Motor Naik Turun / Vertikal)
#define motorAxisZ_A        7
#define motorAxisZ_B        6
#define encoderMotorAxisZ_A   41
#define encoderMotorAxisZ_B   42
#define limitSwitchAxisZ       11

// Sumbu Y (Motor Maju Mundur / Horizontal)
#define motorAxisY_A        5
#define motorAxisY_B        4
#define encoderMotorAxisY_A   1
#define encoderMotorAxisY_B   2
#define limitSwitchAxisY       3

// Motor Indices
#define MOTOR_W       0
#define MOTOR_Z       1
#define MOTOR_Y       2
#define MOTOR_COUNT   3

// ============================================================
// PIN SERVO
// ============================================================
#define SERVO_PIN      14
#define SERVO_CHANNEL  6

// ============================================================
// PIN RELAY
// ============================================================
#define RELAY_1_PIN    12
#define RELAY_2_PIN    13

// ============================================================
// PIN UART (Master Serial)
// ============================================================
#define master_serial_rxPin    21
#define master_serial_txPin    38

// ============================================================
// PWM CONSTANTS
// ============================================================
const int maxPwm = 1023;
const int minPwm = -1023;
const int pwmFrequency = 20000;
const int pwmResolution = 10;

// ============================================================
// SERVO CONSTANTS
// ============================================================
const int servoFrequency = 50;
const int servoResolution = 14;
const int servoMinPulseUs = 500;
const int servoMaxPulseUs = 2500;
const int servoMinAngle = 0;
const int servoMaxAngle = 180;
const int servoHomeAngle = 180;

// ============================================================
// ENCODER CONSTANTS
// ============================================================
const int ENCODER_PPR = 360;
const float LEAD_SCREW_PITCH = 8.0;

// ============================================================
// MOTOR LIMITS
// ============================================================
const int PWM_MIN_MOVE = 150;
const int PWM_SLOW = 350;
const int PWM_MEDIUM = 450;
const int PWM_FAST = 600;

// ============================================================
// HOMING CONSTANTS
// ============================================================
const int HOMING_SPEED = 200;
const unsigned long HOMING_TIMEOUT = 30000;

// ============================================================
// POSITIONING CONSTANTS
// ============================================================
const int MOTOR_POSITION_TOLERANCE = 2;
const long MAX_ENCODER_POSITION = 2000; // fallback, dipakai per-axis di bawah
const int MOVE_SPEED = 400;

// Safety limits per-axis (max encoder count, hardware limit)
const long MAX_POS_W =  4000;  // Sumbu W (Rotasi) — range terbatas
const long MAX_POS_Y = 2900;  // Sumbu Y (Maju Mundur)
const long MAX_POS_Z = 4000; 

long encoderMotorW, encoderMotorZ, encoderMotorY; // Sumbu Z (Naik Turun) — travel terpanjang

// ============================================================
// Shared types
// ============================================================

typedef struct {
    uint8_t pwmPin;
    uint8_t pinDirection;
    uint8_t ledc_channel;
    uint8_t encA;
    uint8_t encB;
    uint8_t limitPin;
    int8_t limitDir;  // +1=check on positive, -1=on negative
} MotorConfig;

typedef struct {
    uint8_t servoPin;
    uint8_t ledc_channel;
} ServoConfig;

typedef struct {
    uint8_t pin;
    bool state;
} RelayConfig;

typedef struct {
    uint8_t encoderPinA;
    uint8_t encoderPinB;
    volatile long count;
} EncoderConfig;

struct MotorState {
    bool homed[MOTOR_COUNT];
};

extern MotorState motorArm;

// ============================================================
// Shared function declarations
// ============================================================

// Vector externs
extern std::vector<MotorConfig> motors;
extern std::vector<ServoConfig> servos;
extern std::vector<EncoderConfig> encoders;
extern std::vector<RelayConfig> relays;

// Motor
void SetupMotors();
void pwmMotor(int idMotor, int pwmValue);
void motorStopAll();
int getLastPwmValue(uint8_t motorIndex);

// Servo
void setupServos();
void setServoAngle(int idServo, int angle);

// Encoder
void setupEncoders();
void resetEncoderCount(uint8_t motorIndex);
void updateEncoderCounts();
void checkLimitSwitches();
void printAllEncoders();

// Relay
void setupRelays();
void relay(int idRelay, int value); // value: 0=ON, 1=OFF

// Limit Switch
void setupLimits();

// Arm positioning
void setMotorTarget(uint8_t motorIndex, long targetPosition);
void stopMotorTarget(uint8_t motorIndex);
void stopAllMotorTargets();
void updateMotorPositioning();
bool isPositionSafe(uint8_t motorIndex, long pos);
void moveToPosition(long posW, long posZ, long posY);
bool moveTargetPosition(uint8_t motorIndex, long targetPosition);

// PID NVS
typedef struct {
    float kp;
    float ki;
    float kd;
    float integral;
    float lastError;
    float lastTarget;
} PIDState;

extern PIDState pidW;

void initPidW();
void setPidW(float p, float i, float d);
void showPidW();
int pidCompute(PIDState &pid, float target, float current, float dt);

// Homing
bool setHoming();
bool homingMotor(uint8_t id);
bool homingAllMotors();
bool isAllMotorHomed();
void printHomingStatus();

// Serial
void setupSerialCommand();
void serialCommandTick();

// UART
void initUART();
void readUART();
void processUARTCommand();
