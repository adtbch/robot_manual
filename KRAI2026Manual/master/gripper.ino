/*
 * =====================================================================
 * FILE    : gripper.ino
 * PERAN   : Auto gripper & motor homing — non-blocking.
 *
 * AUTO GRIPPER:
 *   Proximity detect → tutup gripper (servo d) → luruskan arm (servo b)
 *
 * MOTOR HOMING (sequential):
 *   1. Motor Y naik sampai kena limit → reset encoder Y
 *   2. Motor X ke kanan sampai kena limit → reset encoder X
 *   3. Kedua motor ke posisi tengah (encoder = 0)
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
constexpr long HOME_X_CENTER = 0;   // posisi tengah X (encoder pulse)
constexpr long HOME_Y_CENTER = 0;   // posisi tengah Y (encoder pulse)
constexpr long HOME_THRESHOLD = 5;  // toleransi posisi tengah

// =====================================================================
//  STATE
// =====================================================================

// =====================================================================
//  STATE — gGripperState didefinisikan di sini (extern di config.h)
// =====================================================================

GripperState gGripperState = IDLE;

namespace {

Jeda gJeda;

// Motor homing — sequential
enum HomingState { HOMING_IDLE, HOMING_Y, HOMING_X, HOMING_CENTER };
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
    gHomingState = HOMING_Y;
    pwmMotor('y', -HOMING_PWM);  // Y naik terus
}

void setHomingAll() {
    setMotorHoming();
    setServoHoming();
}

// =====================================================================
//  MOTOR HOMING TICK — panggil di loop(), non-blocking
// =====================================================================

void motorHomingTick() {
    switch (gHomingState) {

        case HOMING_IDLE:
            break;

        // ── Step 1: Y naik sampai limit ─────────────────────────
        case HOMING_Y:
            if (readLimitSwitch(LIMIT_SWITCH_X1)) {
                pwmMotor('y', 0);
                resetEncoderCount('y');
                gHomingState = HOMING_X;
                pwmMotor('x', -HOMING_PWM);  // X ke kanan
            }
            break;

        // ── Step 2: X ke kanan sampai limit ────────────────────
        case HOMING_X:
            if (readLimitSwitch(LIMIT_SWITCH_X2)) {
                pwmMotor('x', 0);
                resetEncoderCount('x');
                gHomingState = HOMING_CENTER;
                // Gerak ke arah tengah (berlawanan dari limit)
                pwmMotor('x', HOMING_PWM);
                pwmMotor('y', HOMING_PWM);
            }
            break;

        // ── Step 3: Gerak ke tengah (encoder = 0) ──────────────
        case HOMING_CENTER: {
            long encX = getEncoderCount('x');
            long encY = getEncoderCount('y');
            bool xDone = (abs(encX - HOME_X_CENTER) <= HOME_THRESHOLD);
            bool yDone = (abs(encY - HOME_Y_CENTER) <= HOME_THRESHOLD);

            if (xDone) pwmMotor('x', 0);
            if (yDone) pwmMotor('y', 0);

            if (xDone && yDone) {
                gHomingState = HOMING_IDLE;
            }
            break;
        }
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
