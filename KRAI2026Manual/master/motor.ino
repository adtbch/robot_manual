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
#include "limit_switch.h"
#include "serial.h"
#include <Preferences.h>

// =====================================================================
//  ENCODER POSITION — Motor X & Y
// =====================================================================

constexpr int  MOTOR_X_MOVE_PWM = 800;
constexpr int  MOTOR_Y_MOVE_PWM = 800;
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
    int gMotorYLastPwm = 0;
    int gMotorXLastPwm = 0;

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
//  MOTOR Y LEVELS — runtime + Preferences (master only)
// =====================================================================

long gMotorYLevelEnc[6] = {
    MOTOR_Y_LEVEL_DEFAULT[0],
    MOTOR_Y_LEVEL_DEFAULT[1],
    MOTOR_Y_LEVEL_DEFAULT[2],
    MOTOR_Y_LEVEL_DEFAULT[3],
    MOTOR_Y_LEVEL_DEFAULT[4],
    MOTOR_Y_LEVEL_DEFAULT[5],
};

namespace {

constexpr const char* MOTOR_Y_LEVEL_NVS_NS = "motor_y_level";

} // anonymous namespace

void initMotorYLevels() {
    Preferences prefs;
    prefs.begin(MOTOR_Y_LEVEL_NVS_NS, true);
    for (uint8_t i = 0; i <= MOTOR_Y_LEVEL_MAX; i++) {
        char key[3] = {'l', static_cast<char>('0' + i), '\0'};
        gMotorYLevelEnc[i] = prefs.getLong(key, MOTOR_Y_LEVEL_DEFAULT[i]);
    }
    prefs.end();
    Serial.printf("[MotorY] levels: %ld %ld %ld %ld %ld %ld\n",
                  gMotorYLevelEnc[0], gMotorYLevelEnc[1], gMotorYLevelEnc[2],
                  gMotorYLevelEnc[3], gMotorYLevelEnc[4], gMotorYLevelEnc[5]);
}

bool motorYLevelSave(const long levels[6]) {
    for (uint8_t i = 0; i <= MOTOR_Y_LEVEL_MAX; i++) {
        gMotorYLevelEnc[i] = constrain(levels[i], MOTOR_Y_ENC_MIN, MOTOR_Y_ENC_MAX);
    }
    Preferences prefs;
    prefs.begin(MOTOR_Y_LEVEL_NVS_NS, false);
    for (uint8_t i = 0; i <= MOTOR_Y_LEVEL_MAX; i++) {
        char key[3] = {'l', static_cast<char>('0' + i), '\0'};
        prefs.putLong(key, gMotorYLevelEnc[i]);
    }
    prefs.end();
    return true;
}

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
    if (motorId == 'y') gMotorYLastPwm = pwmValue;

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
    const long target = gMotorYLevelEnc[level];
    return abs(current - target) <= MOTOR_Y_POSITION_TOLERANCE;
}

void motorYLimitTick() {
    if (!readLimitSwitch(LIMIT_Y_BAWAH)) return;

    const bool movingUp = gMotorYLastPwm > 0 || gMotorY.target > 0;
    if (!movingUp) {
        motorYStop();
        gMotorY.target = 0;
        resetEncoderCount('y');
    }
}

void motorXLimitTick() {
    if (!readLimitSwitch(LIMIT_X_MUNDUR)) return;

    const bool movingFront = gMotorXLastPwm > 0 || gMotorX.target > 0;
    if (!movingFront) {
        motorXStop();
        gMotorX.target = 0;
        resetEncoderCount('x');
    }
}

// =====================================================================
//  MOTOR K LIMIT — safety: stop via slave2 kalau kena limit switch
// =====================================================================

namespace {
int gMotorKDirection = 0;  // +1=maju, -1=mundur, 0=stop
} // anonymous namespace

void motorKSetDirection(int direction) {
    gMotorKDirection = direction;
}

void motorKLimitTick() {
    if (gMotorKDirection < 0 && readLimitSwitch(LIMIT_ARMBOX_DEPAN)) {
        sendSlave2Command("motor k 0");
        gMotorKDirection = 0;
    } else if (gMotorKDirection > 0 && readLimitSwitch(LIMIT_ARMBOX_BELAKANG)) {
        sendSlave2Command("motor k 0");
        gMotorKDirection = 0;
    }
}
