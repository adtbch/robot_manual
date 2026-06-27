/*
 * =====================================================================
 * FILE    : gripper_control.ino
 * PERAN   : Mapping tombol controller → aksi gripper + motor X/Y.
 *
 * BUTTON MAPPING:
 *   Segitiga      → masuk mode siap stab (READY_TO_STAB)
 *   Segitiga + L2 → setServoHoming + motor Y ke level 0
 *
 * INPUT (berlawanan motion control — lihat motion_control.ino):
 *   Motor Y (tanpa Segitiga)     → level 0–5
 *   Motor X (tahan Segitiga)     → axis horizontal
 *   Servo B (READY_TO_STAB)      → axis vertikal
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

constexpr int8_t  AXIS_DEADZONE = 30;
constexpr int8_t  AXIS_MAX      = 127;

constexpr uint32_t MOTOR_Y_LEVEL_INTERVAL_MS  = 300;
constexpr uint32_t MOTOR_X_STEP_INTERVAL_MS     = 50;

constexpr long MOTOR_X_STEP_SLOW   = 5;
constexpr long MOTOR_X_STEP_NORMAL = 15;
constexpr long MOTOR_X_STEP_FAST   = 40;

constexpr int ARM_STEP_SLOW   = 5;
constexpr int ARM_STEP_NORMAL = 10;
constexpr int ARM_STEP_FAST   = 20;

// =====================================================================
//  STATE
// =====================================================================

static uint8_t gMotorYLevel = 0;

namespace {

uint32_t gPrevButtons = 0;
int gServoBAngle = 70;
Jeda gJedaMotorYLevel;
Jeda gJedaMotorXStep;

// ── Input helpers (inverted vs motion_control.ino) ─────────────────

int8_t readInvertedAxisY(const ControlPacket &pkt) {
    if (gInputMode == MODE_DPAD) return pkt.ly;
    if (pkt.buttons & BTN_UP)    return AXIS_MAX;
    if (pkt.buttons & BTN_DOWN)  return -AXIS_MAX;
    return 0;
}

int8_t readInvertedAxisX(const ControlPacket &pkt) {
    if (gInputMode == MODE_DPAD) return pkt.lx;
    if (pkt.buttons & BTN_RIGHT) return AXIS_MAX;
    if (pkt.buttons & BTN_LEFT)  return -AXIS_MAX;
    return 0;
}

int8_t applyDeadzone(int8_t axis) {
    return (abs(axis) < AXIS_DEADZONE) ? 0 : axis;
}

// ── Button helpers ─────────────────────────────────────────────────

bool isPressed(uint32_t buttons, uint32_t mask) {
    return (buttons & mask) && !(gPrevButtons & mask);
}

bool isTriangleHeld(uint32_t buttons) {
    return (buttons & BTN_TRIANGLE) && !(buttons & BTN_L2);
}

bool isComboEdge(uint32_t now, uint32_t prev, uint32_t a, uint32_t b) {
    return (now & a) && (now & b) && !((prev & a) && (prev & b));
}

template <typename T>
T pickSpeedStep(uint32_t buttons, T normal, T slow, T fast) {
    if (buttons & BTN_R1) return fast;
    if (buttons & BTN_L1) return slow;
    return normal;
}

// ── Motor Y ────────────────────────────────────────────────────────

int motorYLevelDir(int8_t ly) {
    if (gInputMode == MODE_DPAD) {
        if (ly < -AXIS_DEADZONE) return +1;
        if (ly >  AXIS_DEADZONE) return -1;
        return 0;
    }
    if (ly > 0) return +1;
    if (ly < 0) return -1;
    return 0;
}

void driveMotorY(int8_t ly) {
    const int dir = motorYLevelDir(ly);
    if (dir == 0) return;
    if (!gJedaMotorYLevel.check(MOTOR_Y_LEVEL_INTERVAL_MS)) return;

    const int newLevel = constrain((int)gMotorYLevel + dir, 0, (int)MOTOR_Y_LEVEL_MAX);
    if (newLevel == gMotorYLevel) return;
    if (newLevel < gMotorYLevel && readLimitSwitch(LIMIT_Y_BAWAH)) return;

    gMotorYLevel = (uint8_t)newLevel;
    motorYSetTarget(MOTOR_Y_LEVEL_ENC[gMotorYLevel]);
}

// ── Motor X ────────────────────────────────────────────────────────

void driveMotorX(int8_t lx, uint32_t buttons) {
    if (lx == 0) {
        if (!motorXIsActive()) {
            motorXSetTarget(getEncoderCount('x'));
        }
        return;
    }
    if (!gJedaMotorXStep.check(MOTOR_X_STEP_INTERVAL_MS)) return;

    const long step = pickSpeedStep(buttons,
        MOTOR_X_STEP_NORMAL, MOTOR_X_STEP_SLOW, MOTOR_X_STEP_FAST);
    if (lx < 0 && readLimitSwitch(LIMIT_X_MUNDUR)) return;

    motorXAdjustTarget((lx > 0) ? step : -step);
}

// ── Servo B (axis vertikal, mapping berbeda dari motor Y) ─────────

int servoBMoveDir(const ControlPacket &pkt) {
    if (gInputMode == MODE_DPAD) {
        if (pkt.ly >  AXIS_DEADZONE) return +1;
        if (pkt.ly < -AXIS_DEADZONE) return -1;
        return 0;
    }
    if (pkt.buttons & BTN_DOWN) return +1;
    if (pkt.buttons & BTN_UP)   return -1;
    return 0;
}

// ── Tick sections ──────────────────────────────────────────────────

void handleGripperButtons(const ControlPacket &pkt) {
    if (isComboEdge(pkt.buttons, gPrevButtons, BTN_TRIANGLE, BTN_L2)) {
        gripperHomingCancel();
        setServoHoming();
        gripperMotorYResetLevel();
        return;
    }
    if (isPressed(pkt.buttons, BTN_TRIANGLE)) {
        gripperReadytoStab();
    }
}

void handleManualServoB(const ControlPacket &pkt) {
    if (gGripperState != READY_TO_STAB || !isTriangleHeld(pkt.buttons)) return;

    const int dir = servoBMoveDir(pkt);
    if (dir == 0) return;

    const int step = pickSpeedStep(pkt.buttons, ARM_STEP_NORMAL, ARM_STEP_SLOW, ARM_STEP_FAST);
    setServoBAngle(gServoBAngle + dir * step);
}

void handleGripperMotors(const ControlPacket &pkt) {
    if (motorHomingIsActive()) return;

    driveMotorY(applyDeadzone(readInvertedAxisY(pkt)));

    if (isTriangleHeld(pkt.buttons)) {
        driveMotorX(applyDeadzone(readInvertedAxisX(pkt)), pkt.buttons);
    }
}

} // anonymous namespace

void gripperMotorYResetLevel() {
    gMotorYLevel = 0;
    motorYSetTarget(MOTOR_Y_LEVEL_ENC[0]);
}

int getServoBAngle() {
    return gServoBAngle;
}

void setServoBAngle(int angle) {
    gServoBAngle = constrain(angle, 0, 100);
    setServoAngle('b', gServoBAngle);
}

void gripperControlTick(const ControlPacket &pkt) {
    handleGripperButtons(pkt);
    handleManualServoB(pkt);
    handleGripperMotors(pkt);
    gPrevButtons = pkt.buttons;
}
