/*
 * =====================================================================
 * FILE    : otomat.ino
 * PERAN   : Semua logic otomatis robot — satu file untuk gampang edit.
 *
 * ISI:
 *   1. setupZone1()         — blocking setup saat masuk zone 1
 *   2. gripperZone1()       — auto gripper state machine (proximity → stab)
 *   3. gripperReadytoStab() — trigger straighten → ready
 *   4. gripperReset()       — reset gripper ke IDLE
 *   5. armBoxTick()         — auto arm box state machine (proximity → grab)
 *   6. armBoxDone()         — trigger arm box GRAB → DONE → BACK → IDLE
 *   7. armBoxReset()        — reset arm box ke IDLE
 *
 * BOARD   : ESP32-S3 (Master)
 * =====================================================================
 */

#include "servo.h"
#include "proximity.h"
#include "motor.h"
#include "encoder.h"
#include "limit_switch.h"
#include "serial.h"

// =====================================================================
//  STATE — GRIPPER
// =====================================================================

GripperState gGripperState = READY_TO_STAB;

namespace {

Jeda gJedaGripper;

} // anonymous namespace

// =====================================================================
//  STATE — ARM BOX
// =====================================================================

enum ArmBoxState { ARMBOX_IDLE, ARMBOX_WAIT, ARMBOX_GRAB, ARMBOX_DONE, ARMBOX_BACK };

namespace {

ArmBoxState gArmBoxR = ARMBOX_IDLE;
ArmBoxState gArmBoxL = ARMBOX_IDLE;
Jeda jedaArmBoxR;
Jeda jedaArmBoxL;
Jeda jeda;
Jeda jedaOpening;
} // anonymous namespace

void armBoxStartGrab(char side) {
    if (side == 'r') {
        sendSlave2Command("pne r off");
        gArmBoxR = ARMBOX_WAIT;
        jedaArmBoxR.reset();
    } else if (side == 'l') {
        sendSlave2Command("pne l off");
        gArmBoxL = ARMBOX_WAIT;
        jedaArmBoxL.reset();
    }
}

void setupZone1() {
    setServoAngle('d', 90);
    setServoAngle('t', 0);
    setServoAngle('b', 0);
    if (gControllerMode == 1) {
        odomGoto(1);
    }
    gripperMotorYSetLevel(0);
    motorXSetTarget(2000);
    gGripperState = IDLE;
    jeda.reset();
}

// =====================================================================
//  GRIPPER TICK — panggil di loop(), non-blocking
// =====================================================================

void gripperZone1() {
    switch (gGripperState) {

        case IDLE:
            setServoAngle('t', 82);
            if (jeda.check(500)) {
                setServoAngle('d', 0);
                gGripperState = OPENING;
                jedaOpening.reset();
            }
            break;
        case OPENING:
            if (jedaOpening.check(300)) {
                if (readProximity()) {
                setServoAngle('d', 80);
                gGripperState = CLOSING;
                gJedaGripper.reset();
                }
            }
            break;

        case CLOSING:
            if (gJedaGripper.check(300)) {
                gripperMotorYSetLevel(1);
                gGripperState = UP;
            }
            break;

        case UP:
            if (motorYAtLevel(1)) {
                gripperReadytoStab();
                if (gControllerMode == 1) {
                    odomGoto(2);
                }
            };
            break;

        case STRAIGHTEN:
            motorXSetTarget(0);
            if (!gMotionWaypointMode && gControllerMode == 1) {
                odomGoto(3);
                gGripperState = READY_TO_STAB;
            }
            if (gControllerMode == 0) {
                gGripperState = READY_TO_STAB;
            }
            break;
        case READY_TO_STAB:
            break;
    }
}

void gripperReadytoStab() {
    if (gGripperState == UP) {
        setServoAngle('t', 0);
        gGripperState = STRAIGHTEN;
    }
}

void gripperReset() {
    gGripperState = IDLE;
}

// =====================================================================
//  ARM BOX TICK — non-blocking state machine
// =====================================================================

constexpr long MOTOR_SPEED = 255;

void armBoxTick() {
    // ── Sisi R ─────────────────────────────────────────────────
    switch (gArmBoxR) {
        case ARMBOX_IDLE:
            if (slave2ProxR() && gControllerMode == 1) {
            sendSlave2Command("pne r off");
                gArmBoxR = ARMBOX_WAIT;
                jedaArmBoxR.reset();
            }
            break;
            
        case ARMBOX_WAIT:
            if (!jedaArmBoxR.check(300)) break;
            sendSlave2Command("motortarget %ld", gMotorYLevelEnc[4]);
            armBoxFBbySpeed('r', -255);
            gArmBoxR = ARMBOX_GRAB;
            break;

        case ARMBOX_GRAB:
            break;

        default:
            break;
    }

    // ── Sisi L ─────────────────────────────────────────────────
    switch (gArmBoxL) {
        case ARMBOX_IDLE:
            if (slave2ProxL() && gControllerMode == 1) {
                sendSlave2Command("pne l off");
                gArmBoxL = ARMBOX_WAIT;
                jedaArmBoxL.reset();
            }
            break;
            
        case ARMBOX_WAIT:
            if (!jedaArmBoxL.check(300)) break;
            motorYSetTarget(gMotorYLevelEnc[4]);
            armBoxFBbySpeed('l', -255);
            gArmBoxL = ARMBOX_GRAB;
            break;

        case ARMBOX_GRAB:
            break;
        
        default:
            break;
    }
}

// =====================================================================
//  ARM BOX DONE — trigger dari luar (tombol / serial)
// =====================================================================

void armBoxDone(char side) {
    if (side == 'r') {
        if (gArmBoxR == ARMBOX_GRAB) {
            sendSlave2Command("motortarget %ld", gMotorYLevelEnc[5]);
            gArmBoxR = ARMBOX_DONE;
            return;
        } else if (gArmBoxR == ARMBOX_DONE) {
            sendSlave2Command("pne r on");
            gArmBoxR = ARMBOX_BACK;
            return;
        } else if (gArmBoxR == ARMBOX_BACK) {
            gArmBoxR = ARMBOX_IDLE;
        }
    } else if (side == 'l') {
        if (gArmBoxL == ARMBOX_GRAB) {
            motorYSetTarget(gMotorYLevelEnc[5]);
            gArmBoxL = ARMBOX_DONE;
            return;
        } else if (gArmBoxL == ARMBOX_DONE) {
            sendSlave2Command("pne l on");
            gArmBoxL = ARMBOX_BACK;
            return;
        } else if (gArmBoxL == ARMBOX_BACK) {
            gArmBoxL = ARMBOX_IDLE;
        }
    }
}

// =====================================================================
//  ARM BOX RESET
// =====================================================================

void armBoxReset() {
    sendSlave2Command("pneall");
    gArmBoxR = ARMBOX_IDLE;
    gArmBoxL = ARMBOX_IDLE;
}
