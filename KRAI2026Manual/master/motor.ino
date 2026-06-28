/*
 * =====================================================================
 * FILE    : motor.ino
 * PERAN   : Setup & control motor via H-bridge (L298N/drv8833).
 *           2 motor: 'x'=axis X, 'y'=axis Y.
 *           Motor X/Y → target encoder + bang-bang saat bergerak.
 *           Motor Y → hold PWM anti-gravitasi saat sudah di target.
 *
 * BOARD   : ESP32-S3 (Master)
 * =====================================================================
 */

#include "motor.h"
#include "encoder.h"

// =====================================================================
//  ENCODER POSITION — Motor X & Y
// =====================================================================

constexpr int  MOTOR_X_MOVE_PWM = 400;
constexpr int  MOTOR_Y_MOVE_PWM = 400;
constexpr long MOTOR_X_POSITION_TOLERANCE = 5;
constexpr long MOTOR_Y_POSITION_TOLERANCE = 5;

// Hold anti-gravitasi — hanya motor Y saat sudah di target
constexpr int  MOTOR_Y_HOLD_PWM = 50;

// =====================================================================
//  STATE
// =====================================================================

namespace {

    MotorConfig motors[MOTOR_COUNT] = {
        {'x', MOTOR1_PIN_DIR, MOTOR1_PIN_PWM},
        {'y', MOTOR2_PIN_DIR, MOTOR2_PIN_PWM},
    };

    struct MotorTarget {
        bool active = false;
        long target = 0;
    };

    MotorTarget gMotorX;
    MotorTarget gMotorY;

    int findMotorIndex(char id) {
        for (size_t i = 0; i < MOTOR_COUNT; i++) {
            if (motors[i].id == id) return (int)i;
        }
        return -1;
    }

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

// =====================================================================
//  SETUP
// =====================================================================

void SetupMotors() {
    for (size_t i = 0; i < MOTOR_COUNT; i++) {
        pinMode(motors[i].pin_dir, OUTPUT);
        digitalWrite(motors[i].pin_dir, LOW);

        ledcAttach(motors[i].pin_pwm, PWM_FREQUENCY, PWM_RESOLUTION);
        ledcWrite(motors[i].pin_pwm, 0);
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
        ledcWrite(motors[idx].pin_pwm, pwmValue);
        digitalWrite(motors[idx].pin_dir, LOW);
    } else if (pwmValue < 0) {
        ledcWrite(motors[idx].pin_pwm, PWM_MAX + pwmValue);
        digitalWrite(motors[idx].pin_dir, HIGH);
    } else {
        ledcWrite(motors[idx].pin_pwm, 0);
        digitalWrite(motors[idx].pin_dir, LOW);
    }
}

void motorStopAll() {
    for (size_t i = 0; i < MOTOR_COUNT; i++) {
        pwmMotor(motors[i].id, 0);
    }
    gMotorX.active = false;
    gMotorY.active = false;
}

// =====================================================================
//  ENCODER POSITION — Motor X
// =====================================================================

void motorXSetTarget(long targetEncoder) {
    gMotorX.target = constrain(targetEncoder, MOTOR_X_ENC_MIN, MOTOR_X_ENC_MAX);
    gMotorX.active = true;
}

void motorXAdjustTarget(long deltaPulse) {
    if (!gMotorX.active) {
        gMotorX.target = getEncoderCount('x');
        gMotorX.active = true;
    }
    gMotorX.target = constrain(gMotorX.target + deltaPulse, MOTOR_X_ENC_MIN, MOTOR_X_ENC_MAX);
}

long motorXGetTarget() {
    return gMotorX.target;
}

void motorXStop() {
    gMotorX.active = false;
    pwmMotor('x', 0);
}

bool motorXIsActive() {
    return gMotorX.active;
}

void motorXPositionTick() {
    if (!gMotorX.active) return;

    const long current = getEncoderCount('x');
    const long error = gMotorX.target - current;

    if (abs(error) <= MOTOR_X_POSITION_TOLERANCE) {
        pwmMotor('x', 0);
        return;
    }

    if (error > 0) {
        pwmMotor('x', MOTOR_X_MOVE_PWM);
    } else {
        pwmMotor('x', -MOTOR_X_MOVE_PWM);
    }
}

// =====================================================================
//  ENCODER POSITION — Motor Y (+ hold anti-gravitasi di target)
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

    if (abs(error) <= MOTOR_Y_POSITION_TOLERANCE) {
        motorYHoldAtTarget(error);
        return;
    }

    if (error > 0) {
        pwmMotor('y', MOTOR_Y_MOVE_PWM);
    } else {
        pwmMotor('y', -MOTOR_Y_MOVE_PWM);
    }
}

bool motorYAtLevel(uint8_t level) {
    if (level > MOTOR_Y_LEVEL_MAX) return false;
    const long current = getEncoderCount('y');
    const long target = MOTOR_Y_LEVEL_ENC[level];
    return abs(current - target) <= MOTOR_Y_POSITION_TOLERANCE;
}
