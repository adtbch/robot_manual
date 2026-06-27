/*
 * =====================================================================
 * FILE    : gripper_control.ino
 * PERAN   : Mapping tombol controller → aksi gripper + motor X/Y.
 *
 * BUTTON MAPPING:
 *   Segitiga      → masuk mode siap stab (READY_TO_STAB)
 *   Segitiga + L2 → homing (servo + motor)
 *
 * MOTOR X/Y (tanpa Segitiga):
 *   Input berlawanan dengan motion control:
 *     MODE_DPAD  → analog kiri → motor X/Y
 *     MODE_ANALOG → DPAD → motor X/Y
 *   R1 hold → fast, L1 hold → slow
 *   Motor X/Y → target encoder + bang-bang
 *   Motor Y hold PWM anti-gravitasi saat sudah di target
 *
 * MANUAL ARM (saat READY_TO_STAB + tahan Segitiga):
 *   Input berlawanan dengan motion control:
 *     MODE_DPAD  → analog kiri → servo b
 *     MODE_ANALOG → DPAD → servo b
 *
 * BOARD   : ESP32-S3 (Master)
 * =====================================================================
 */

#include "config.h"
#include "servo.h"
#include "motor.h"
#include "encoder.h"
#include "limit_switch.h"

// =====================================================================
//  CONFIG
// =====================================================================

constexpr long MOTOR_Y_STEP_SLOW   = 5;
constexpr long MOTOR_Y_STEP_NORMAL = 15;
constexpr long MOTOR_Y_STEP_FAST   = 40;
constexpr uint32_t MOTOR_Y_STEP_INTERVAL_MS = 50;

constexpr long MOTOR_X_STEP_SLOW   = 5;
constexpr long MOTOR_X_STEP_NORMAL = 15;
constexpr long MOTOR_X_STEP_FAST   = 40;
constexpr uint32_t MOTOR_X_STEP_INTERVAL_MS = 50;

constexpr int ARM_STEP_SLOW  = 5;
constexpr int ARM_STEP_NORMAL = 10;
constexpr int ARM_STEP_FAST  = 20;

// =====================================================================
//  STATE
// =====================================================================

namespace {

uint32_t gPrevButtons = 0;
int gServoBAngle = 70;
Jeda gJedaMotorYStep;
Jeda gJedaMotorXStep;

bool isPressed(uint32_t buttons, uint32_t mask) {
    return (buttons & mask) && !(gPrevButtons & mask);
}

// +1 = encoder target naik, -1 = encoder target turun
int getMotorYJogDir(int8_t ly) {
    if (gInputMode == MODE_DPAD) {
        // Analog kiri: stick atas = ly negatif → encoder tambah
        return (ly < 0) ? +1 : -1;
    }
    // DPAD: UP = ly positif → encoder tambah
    return (ly > 0) ? +1 : -1;
}

void driveMotorY(int8_t ly, uint32_t buttons) {
    if (ly == 0) {
        if (!motorYIsActive()) {
            motorYSetTarget(getEncoderCount('y'));
        }
        return;
    }

    if (!gJedaMotorYStep.check(MOTOR_Y_STEP_INTERVAL_MS)) return;

    long step = MOTOR_Y_STEP_NORMAL;
    if (buttons & BTN_R1) step = MOTOR_Y_STEP_FAST;
    else if (buttons & BTN_L1) step = MOTOR_Y_STEP_SLOW;

    const int dir = getMotorYJogDir(ly);
    if (dir > 0 && readLimitSwitch(LIMIT_SWITCH_X1)) return;

    motorYAdjustTarget(dir * step);
}

// +1 = encoder target tambah, -1 = encoder target kurang
int getMotorXJogDir(int8_t lx) {
    // Analog kanan = lx positif, DPAD RIGHT = lx positif → encoder tambah
    return (lx > 0) ? +1 : -1;
}

void driveMotorX(int8_t lx, uint32_t buttons) {
    if (lx == 0) {
        if (!motorXIsActive()) {
            motorXSetTarget(getEncoderCount('x'));
        }
        return;
    }

    if (!gJedaMotorXStep.check(MOTOR_X_STEP_INTERVAL_MS)) return;

    long step = MOTOR_X_STEP_NORMAL;
    if (buttons & BTN_R1) step = MOTOR_X_STEP_FAST;
    else if (buttons & BTN_L1) step = MOTOR_X_STEP_SLOW;

    const int dir = getMotorXJogDir(lx);
    if (dir < 0 && readLimitSwitch(LIMIT_SWITCH_X2)) return;

    motorXAdjustTarget(dir * step);
}

} // anonymous namespace

// =====================================================================
//  SERVO B
// =====================================================================

int getServoBAngle() {
    return gServoBAngle;
}

void setServoBAngle(int angle) {
    gServoBAngle = constrain(angle, 0, 180);
    setServoAngle('b', gServoBAngle);
}

// =====================================================================
//  TICK
// =====================================================================

void gripperControlTick(const ControlPacket &pkt) {

    // ── Combo: Segitiga + L2 → homing ──────────────────────────
    if ((pkt.buttons & BTN_TRIANGLE) && (pkt.buttons & BTN_L2)) {
        if (!((gPrevButtons & BTN_TRIANGLE) && (gPrevButtons & BTN_L2))) {
            setHomingAll();
        }
    }
    // ── Single: Segitiga → masuk READY_TO_STAB ─────────────────
    else if (isPressed(pkt.buttons, BTN_TRIANGLE)) {
        gripperReadytoStab();
    }

    // ── Manual arm (READY_TO_STAB + tahan Segitiga) ────────────
    if (gGripperState == READY_TO_STAB && (pkt.buttons & BTN_TRIANGLE)) {
        int step = ARM_STEP_NORMAL;
        if (pkt.buttons & BTN_R1) step = ARM_STEP_FAST;
        else if (pkt.buttons & BTN_L1) step = ARM_STEP_SLOW;

        int moveDir = 0;
        if (gInputMode == MODE_DPAD) {
            if (pkt.ly > 30)       moveDir = +step;
            else if (pkt.ly < -30) moveDir = -step;
        } else {
            if (pkt.buttons & BTN_DOWN) moveDir = +step;
            if (pkt.buttons & BTN_UP)   moveDir = -step;
        }

        if (moveDir != 0) {
            setServoBAngle(gServoBAngle + moveDir);
        }
    }
    // ── Motor X/Y (tanpa Segitiga, skip saat homing) ───────────
    else if (!motorHomingIsActive()) {
        int8_t lx = 0, ly = 0;

        // Input berlawanan dengan motion control
        if (gInputMode == MODE_DPAD) {
            lx = pkt.lx;
            ly = pkt.ly;
        } else {
            if (pkt.buttons & BTN_UP)    ly =  127;
            if (pkt.buttons & BTN_DOWN)  ly = -127;
            if (pkt.buttons & BTN_LEFT)  lx = -127;
            if (pkt.buttons & BTN_RIGHT) lx =  127;
        }

        if (abs(lx) < 30) lx = 0;
        if (abs(ly) < 30) ly = 0;

        driveMotorY(ly, pkt.buttons);
        driveMotorX(lx, pkt.buttons);
    }

    gPrevButtons = pkt.buttons;
}
