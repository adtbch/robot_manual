/*
 * =====================================================================
 * FILE    : gripper.ino
 * PERAN   : Auto gripper (non-blocking loop) + motor homing (blocking, setup saja).
 *
 * AUTO GRIPPER:
 *   Proximity → tutup (servo d) → CLOSING → servo b lurus
 *   → UP: motor Y ke level 1 → STRAIGHTEN (tunggu Segitiga)
 *
 * SETUP ZONE1 (blocking, dipanggil sekali dari setup()):
 *   Waypoint limit Y bawah → reset enc Y
 *   Waypoint limit X mundur → reset enc X
 *   Motor Y level 0, motor X target encoder 0
 *
 * setHomingAll() — servo + setMotorHoming() (serial / manual re-homing)
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

} // anonymous namespace

void setServoHoming() {
    setServoAngle('d', 0);
    setServoAngle('b', 70);
    gGripperState = IDLE;
}

void setMotorHoming() {
    motorXStop();
    motorYStop();

    pwmMotor('y', HOMING_PWM);
    while (!readLimitSwitch(LIMIT_Y_BAWAH)) {
        delay(2);
    }
    pwmMotor('y', 0);
    resetEncoderCount('y');
    
    pwmMotor('x', HOMING_PWM);
    while (!readLimitSwitch(LIMIT_X_MUNDUR)) {
        delay(2);
    }
    pwmMotor('x', 0);
    resetEncoderCount('x');
}

void setHomingAll() {
    setMotorHoming();
    setServoHoming();
}

void setupZone1() {
    setServoHoming();
    // WAYPOINT: ready to setup zone1
    gripperMotorYSetLevel(0);
    motorXSetTarget(200);
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
            if (gJeda.check(300)) {
                setServoAngle('b', 100);
                gripperMotorYSetLevel(1);
                gGripperState = UP;
            }
            break;

        case UP:
            if (motorYAtLevel(1)) {
                // WAYPOINT: ready to setup zone1
                gGripperState = STRAIGHTEN;
            };
            break;

        case STRAIGHTEN:
            // Cek WAYPOINT
            gripperReadytoStab();
            gYawTarget = 90;
            // WAYPOINT menuju ke arah stab
            break;
        
        case READY_TO_STAB:
            break;
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
