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
        pwmMotor('y', 0);
        return;
    }

    if (error > 0) {
        pwmMotor('y', MOTOR_Y_MOVE_PWM);
    } else {
        pwmMotor('y', -MOTOR_Y_MOVE_PWM);
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