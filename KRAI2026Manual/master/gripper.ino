/*
 * =====================================================================
 * FILE    : gripper.ino
 * PERAN   : Auto gripper & motor homing — non-blocking.
 *
 * AUTO GRIPPER:
 *   Proximity detect → tutup gripper (servo d) → luruskan arm (servo b)
 *
 * MOTOR HOMING (sequential):
 *   1. Motor Y turun mentok → kena LIMIT_Y_BAWAH → reset encoder Y = 0
 *   2. Motor X mundur mentok → kena LIMIT_X_MUNDUR → reset encoder X = 0
 *   Origin encoder (0,0) = posisi bawah + mundur.
 *
 * BOARD   : ESP32-S3 (Master)
 * =====================================================================
 */

#include "servo.h"
#include "proximity.h"
#include "motor.h"
#include "encoder.h"
#include "limit_switch.h"

// =====================================================================
//  CONFIG
// =====================================================================

constexpr int HOMING_PWM = 400;

// =====================================================================
//  STATE
// =====================================================================

GripperState gGripperState = IDLE;

namespace {

Jeda gJeda;

enum HomingState { HOMING_IDLE, HOMING_Y_DOWN, HOMING_X_MUNDUR };
HomingState gHomingState = HOMING_IDLE;

} // anonymous namespace

void setServoHoming() {
    setServoAngle('d', 0);
    setServoAngle('b', 70);
    gGripperState = IDLE;
}

void setMotorHoming() {
    motorXStop();
    motorYStop();
    gHomingState = HOMING_Y_DOWN;
    pwmMotor('y', HOMING_PWM);  // Y turun mentok
}

void setHomingAll() {
    setMotorHoming();
    setServoHoming();
}

void gripperHomingCancel() {
    if (gHomingState == HOMING_IDLE) return;
    pwmMotor('x', 0);
    pwmMotor('y', 0);
    gHomingState = HOMING_IDLE;
}

// =====================================================================
//  MOTOR HOMING TICK — panggil di loop(), non-blocking
// =====================================================================

void motorHomingTick() {
    switch (gHomingState) {

        case HOMING_IDLE:
            break;

        // ── Step 1: Y turun sampai limit bawah ──────────────────
        case HOMING_Y_DOWN:
            if (readLimitSwitch(LIMIT_Y_BAWAH)) {
                pwmMotor('y', 0);
                resetEncoderCount('y');
                gHomingState = HOMING_X_MUNDUR;
                pwmMotor('x', HOMING_PWM);  // X mundur mentok
            }
            break;

        // ── Step 2: X mundur sampai limit depan ─────────────────
        case HOMING_X_MUNDUR:
            if (readLimitSwitch(LIMIT_X_MUNDUR)) {
                pwmMotor('x', 0);
                resetEncoderCount('x');
                gHomingState = HOMING_IDLE;
                gripperMotorYResetLevel();
                motorXSetTarget(0);
            }
            break;
    }
}

bool motorHomingIsActive() {
    return gHomingState != HOMING_IDLE;
}

// =====================================================================
//  GRIPPER TICK — panggil di loop(), non-blocking
// =====================================================================

void gripperZone1() {
    switch (gGripperState) {

        case IDLE:
            if (readProximity()) {
                setServoAngle('d', 90);
                gJeda.reset();
                gGripperState = CLOSING;
            }
            break;

        case CLOSING:
            if (gJeda.check(1000)) {
                setServoAngle('b', 100);
                gGripperState = STRAIGHTEN;
            }
            break;

        case STRAIGHTEN: break;
    }
}

void gripperReadytoStab() {
    if (gGripperState == STRAIGHTEN) {
        setServoAngle('b', 0);
        gGripperState = READY_TO_STAB;
    } else return;
}

// =====================================================================
//  RESET
// =====================================================================

void gripperReset() {
    gGripperState = IDLE;
}
