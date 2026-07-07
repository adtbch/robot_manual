/*
 * =====================================================================
 * FILE    : motor.ino
 * PERAN   : Setup & control 4 motor arm via H-bridge.
 *
 * BOARD   : ESP32-S3 (Slave2 Arm)
 * =====================================================================
 */

#include "motor.h"
#include "encoder.h"
#include "limit_switch.h"

// =====================================================================
//  STATE
// =====================================================================

namespace {

MotorConfig motors[MOTOR_COUNT] = {
    {'x', MOTOR_XARM_KANAN_PIN1, MOTOR_XARM_KANAN_PIN2},  // arm kanan X
    {'y', MOTOR_YARM_KANAN_PIN1, MOTOR_YARM_KANAN_PIN2},  // arm kanan Y
    {'k', MOTOR_XARM_KIRI_PIN1,  MOTOR_XARM_KIRI_PIN2},   // arm kiri X
    // ponytail: 4th motor deadcode dulu
};

struct MotorTarget {
    bool active = false;
    long target = 0;
};

MotorTarget gMotorY;

// Motor X/K — continuous run
struct MotorRun {
    bool active = false;
    int  pwm    = 0;
};

MotorRun gMotorXRun;
MotorRun gMotorKRun;
int gMotorYLastPwm = 0;

int findMotorIndex(char id) {
    for (size_t i = 0; i < MOTOR_COUNT; i++) {
        if (motors[i].id == id) return (int)i;
    }
    return -1;
}

} // anonymous namespace

// =====================================================================
//  SETUP
// =====================================================================

void SetupMotors() {
    for (size_t i = 0; i < MOTOR_COUNT; i++) {
        pinMode(motors[i].pin1, OUTPUT);
        pinMode(motors[i].pin2, OUTPUT);
        digitalWrite(motors[i].pin1, LOW);
        digitalWrite(motors[i].pin2, LOW);

        ledcAttach(motors[i].pin1, PWM_FREQUENCY, PWM_RESOLUTION);
        ledcWrite(motors[i].pin1, 0);
    }
}

// =====================================================================
//  CONTROL
// =====================================================================

void pwmMotor(char motorId, int pwmValue) {
    int idx = findMotorIndex(motorId);
    if (idx < 0) return;

    pwmValue = constrain(pwmValue, PWM_MIN, PWM_MAX);
    if (motorId == 'y') gMotorYLastPwm = pwmValue;

    if (pwmValue > 0) {
        digitalWrite(motors[idx].pin2, LOW);
        ledcWrite(motors[idx].pin1, pwmValue);
    } else if (pwmValue < 0) {
        digitalWrite(motors[idx].pin2, HIGH);
        ledcWrite(motors[idx].pin1, PWM_MAX + pwmValue);
    } else {
        ledcWrite(motors[idx].pin1, 0);
        digitalWrite(motors[idx].pin2, LOW);
    }
}

void motorStopAll() {
    for (size_t i = 0; i < MOTOR_COUNT; i++) {
        pwmMotor(motors[i].id, 0);
    }
    gMotorY.active = false;
    gMotorXRun.active = false;
    gMotorKRun.active = false;
}

// =====================================================================
//  MOTOR Y — encoder positioning (bang-bang)
// =====================================================================

constexpr int MOTOR_Y_HOLD_PWM = 0;

namespace {

void motorYHoldAtTarget(long error) {
    if (error > 0) {
        pwmMotor('y', MOTOR_Y_HOLD_PWM);
    } else if (error < 0) {
        pwmMotor('y', -MOTOR_Y_HOLD_PWM);
    } else {
        pwmMotor('y', MOTOR_Y_HOLD_PWM);
    }
}

} // anonymous namespace

void motorYSetTarget(long targetEncoder) {
    gMotorY.target = constrain(targetEncoder, MOTOR_Y_ENC_MIN, MOTOR_Y_ENC_MAX);
    gMotorY.active = true;
}

void motorYAdjustTarget(long deltaPulse) {
    if (!gMotorY.active) {
        gMotorY.target = getEncoderCount('y');
        gMotorY.active = true;
    }
    gMotorY.target = constrain(gMotorY.target + deltaPulse, MOTOR_Y_ENC_MIN, MOTOR_Y_ENC_MAX);
}

long motorYGetTarget() {
    return gMotorY.target;
}

void motorYStop() {
    gMotorY.active = false;
    pwmMotor('y', 0);
}

bool motorYIsActive() {
    return gMotorY.active;
}

void motorYPositionTick() {
    if (!gMotorY.active) return;

    const long current = getEncoderCount('y');
    const long error = gMotorY.target - current;

    if (abs(error) <= MOTOR_Y_TOLERANCE) {
        motorYHoldAtTarget(error);
        return;
    }

    if (error > 0) {
        pwmMotor('y', MOTOR_Y_MOVE_PWM);
    } else {
        pwmMotor('y', -MOTOR_Y_MOVE_PWM);
    }
}

void motorYLimitTick() {
    if (!readLimitSwitch(LIMIT_ARMBOX_TURUN)) return;

    const bool movingUp = gMotorYLastPwm > 0 || gMotorY.target > 0;
    if (!movingUp) {
        motorYStop();
        gMotorY.target = 0;
        resetEncoderCount('y');
    }
}

// =====================================================================
//  MOTOR X — continuous run with limit switch
// =====================================================================

void motorRunStart(char id, int pwm) {
    if (id == 'x') {
        gMotorXRun.active = true;
        gMotorXRun.pwm = constrain(pwm, PWM_MIN, PWM_MAX);
    } else if (id == 'k') {
        gMotorKRun.active = true;
        gMotorKRun.pwm = constrain(pwm, PWM_MIN, PWM_MAX);
    }
}

void motorRunStop(char id) {
    if (id == 'x') {
        gMotorXRun.active = false;
        pwmMotor('x', 0);
    } else if (id == 'k') {
        gMotorKRun.active = false;
        pwmMotor('k', 0);
    }
}

void motorRunStopAll() {
    motorRunStop('x');
    motorRunStop('k');
}

bool motorRunIsActive(char id) {
    if (id == 'x') return gMotorXRun.active;
    if (id == 'k') return gMotorKRun.active;
    return false;
}

bool executeMotorCommand(char motorId, int pwm) {
    if (findMotorIndex(motorId) < 0) return false;
    pwm = constrain(pwm, PWM_MIN, PWM_MAX);
    if (motorId == 'x' || motorId == 'k') {
        if (pwm == 0) motorRunStop(motorId);
        else motorRunStart(motorId, pwm);
        return true;
    }
    pwmMotor(motorId, pwm);
    return true;
}

int motorRunGetPwm(char id) {
    if (id == 'x' && gMotorXRun.active) return gMotorXRun.pwm;
    if (id == 'k' && gMotorKRun.active) return gMotorKRun.pwm;
    return 0;
}

int motorYGetLastPwm() {
    return gMotorYLastPwm;
}

// =====================================================================
//  HOMING — blocking, dipanggil dari setup()
// =====================================================================

constexpr int HOMING_PWM = 400;

void motorYHoming() {
    motorYStop();
    pwmMotor('y', -HOMING_PWM);
    while (!readLimitSwitch(LIMIT_ARMBOX_TURUN)) {
        delay(2);
    }
    pwmMotor('y', 0);
    resetEncoderCount('y');
    gMotorY.target = 0;
}

void motorXHoming() {
    motorRunStop('x');
    pwmMotor('x', -HOMING_PWM);
    while (!readLimitSwitch(LIMIT_ARMBOX_BELAKANG)) {
        delay(2);
    }
    pwmMotor('x', 0);
}

void motorRunTick() {
    // Motor X — limit switch lokal (cek sesuai arah gerak)
    if (gMotorXRun.active) {
        if (gMotorXRun.pwm > 0 && readLimitSwitch(LIMIT_ARMBOX_DEPAN)) {
            motorRunStop('x');
        } else if (gMotorXRun.pwm < 0 && readLimitSwitch(LIMIT_ARMBOX_BELAKANG)) {
            motorRunStop('x');
        } else {
            pwmMotor('x', gMotorXRun.pwm);
        }
    }

    // Motor K — jalan terus (master kirim motor k 0 utk stop)
    if (gMotorKRun.active) {
        pwmMotor('k', gMotorKRun.pwm);
    }
}