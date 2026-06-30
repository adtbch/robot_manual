/*
 * =====================================================================
 * FILE    : gripper_control.ino
 * PERAN   : Mapping tombol controller → aksi gripper + motor X/Y.
 *
 * BUTTON MAPPING:
 *   R2 hold       → masuk mode gripper jog (motor X/Y manual)
 *   Segitiga hold (READY_TO_STAB) → servo B manual
 *   Segitiga + L2 → setServoHoming + motor Y ke level 0
 *
 * INPUT (berlawanan motion control — lihat motion_control.ino):
 *   Motor Y (tanpa Segitiga)     → jog encoder step (manual)
 *   Motor X (tahan R2)          → jog encoder step
 *   Level Y (0–5)                → gripperMotorYSetLevel() / auto gripper
 *   Servo B (READY_TO_STAB + Segitiga hold) → axis vertikal
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

constexpr uint32_t MOTOR_Y_STEP_INTERVAL_MS = 50;
constexpr uint32_t MOTOR_X_STEP_INTERVAL_MS = 50;

// ponytail: level Y — dead until tombol dipetakan (pakai driveMotorYByLevel nanti)
constexpr uint32_t MOTOR_Y_LEVEL_INTERVAL_MS = 150;

constexpr long MOTOR_Y_STEP_SLOW   = 5;
constexpr long MOTOR_Y_STEP_NORMAL = 15;
constexpr long MOTOR_Y_STEP_FAST   = 40;

constexpr long MOTOR_X_STEP_SLOW   = 5;
constexpr long MOTOR_X_STEP_NORMAL = 15;
constexpr long MOTOR_X_STEP_FAST   = 40;

constexpr int ARM_STEP_SLOW   = 5;
constexpr int ARM_STEP_NORMAL = 10;
constexpr int ARM_STEP_FAST   = 20;

constexpr uint32_t ARMBOX_Y_ENC_INTERVAL_MS = 100;
constexpr long ARMBOX_Y_ENC_MIN = 0;
constexpr long ARMBOX_Y_ENC_MAX = 5000;

// =====================================================================
//  STATE
// =====================================================================

static uint8_t gMotorYLevel = 0;

namespace {

uint32_t gPrevButtons = 0;
int gServoBAngle = 70;
Jeda gJedaMotorYStep;
Jeda gJedaMotorYLevel;
Jeda gJedaMotorXStep;
Jeda gJedaArmBoxYEnc;

// ── Input helpers (inverted vs motion_control.ino) ─────────────────
// ly: int16_t + negasi — aman untuk ly=-128 (-(-128) = +128)
// lx: int8_t cukup (tidak di-negate)

int16_t readGripperStickLy(const ControlPacket &pkt) {
    return -(int16_t)pkt.ly;
}

int16_t readGripperStickRy(const ControlPacket &pkt) {
    return -(int16_t)pkt.ry;
}

int16_t readInvertedAxisY(const ControlPacket &pkt) {
    if (gInputMode == MODE_DPAD) return readGripperStickLy(pkt);
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

int16_t applyDeadzoneY(int16_t axis) {
    return (abs(axis) < 120) ? 0 : axis;
}

int8_t applyDeadzoneX(int8_t axis) {
    return (abs(axis) < 120) ? 0 : axis;
}

// ── Button helpers ─────────────────────────────────────────────────

bool isPressed(uint32_t buttons, uint32_t mask) {
    return (buttons & mask) && !(gPrevButtons & mask);
}

bool isR2Held(uint32_t buttons) {
    return (buttons & BTN_R2) && !(buttons & BTN_L2);
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

// ── Motor Y — jog encoder (level → gripperMotorYSetLevel) ──────────

void driveMotorY(int16_t ly, uint32_t buttons) {
    if (ly == 0) {
        if (!motorYIsActive()) {
            motorYSetTarget(getEncoderCount('y'));
        }
        return;
    }
    if (!gJedaMotorYStep.check(MOTOR_Y_STEP_INTERVAL_MS)) return;

    const long step = pickSpeedStep(buttons,
        MOTOR_Y_STEP_NORMAL, MOTOR_Y_STEP_SLOW, MOTOR_Y_STEP_FAST);
    if (ly < 0 && readLimitSwitch(LIMIT_Y_BAWAH)) return;

    motorYAdjustTarget((ly > 0) ? step : -step);
}

// ponytail: dead — naik/turun level preset; wiring tombol belum ditentukan
int motorYLevelDir(int16_t ly) {
    if (gInputMode == MODE_DPAD) {
        if (ly <= -120) return +1;
        if (ly >= 120) return -1;
        return 0;
    }
    if (ly > 0) return +1;
    if (ly < 0) return -1;
    return 0;
}

void driveMotorYByLevel(int16_t ly) {
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
    if (gModeInvert) lx = (int8_t)(-lx);

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
        const int16_t ly = readGripperStickLy(pkt);
        if (ly >  AXIS_DEADZONE) return +1;
        if (ly < -AXIS_DEADZONE) return -1;
        return 0;
    }
    if (pkt.buttons & BTN_DOWN) return +1;
    if (pkt.buttons & BTN_UP)   return -1;
    return 0;
}

// ── Tick sections ──────────────────────────────────────────────────

void handleGripperButtons(const ControlPacket &pkt) {
    if (isPressed(pkt.buttons, BTN_OPTIONS)) {
        if (zoneState == 1) {
            setupZone1();
        } else if (zoneState == 2) {
            modeKinematics = !modeKinematics;
        }
        return;
    }
}

void handleManualServoB(const ControlPacket &pkt) {
    if (gGripperState != READY_TO_STAB || !(pkt.buttons & BTN_TRIANGLE)) return;

    const int dir = servoBMoveDir(pkt);
    if (dir == 0) return;

    const int step = pickSpeedStep(pkt.buttons, ARM_STEP_NORMAL, ARM_STEP_SLOW, ARM_STEP_FAST);
    setServoBAngle(gServoBAngle + dir * step);
}


void handleGripperMotors(const ControlPacket &pkt) { 
    if (isR2Held(pkt.buttons)) {
        driveMotorY(applyDeadzoneY(readGripperStickRy(pkt)), pkt.buttons);
        driveMotorX(applyDeadzoneX(pkt.rx), pkt.buttons);
    }
}

// ── R2 hold + analog kanan → armbox motor Y jog manual (slave2) ──

void handleArmBoxYEnc(const ControlPacket &pkt) {
    if (!(pkt.buttons & BTN_R2)) return;

    int16_t ly = 0;
    if (gInputMode == MODE_DPAD) {
        ly = -(int16_t)pkt.ly;
    } else {
        if (pkt.buttons & BTN_UP)   ly =  AXIS_MAX;
        if (pkt.buttons & BTN_DOWN) ly = -AXIS_MAX;
    }
    if (abs(ly) <= AXIS_DEADZONE) return;
    if (!gJedaArmBoxYEnc.check(ARMBOX_Y_ENC_INTERVAL_MS)) return;

    const int dir = (ly > 0) ? +1 : -1;  // stick atas = naik
    const long step = pickSpeedStep(pkt.buttons,
        MOTOR_Y_STEP_NORMAL, MOTOR_Y_STEP_SLOW, MOTOR_Y_STEP_FAST);
    long newTarget = slave2EncY() + dir * step;

    // limit switch protection — slave2arm gak punya
    if (newTarget <= MOTOR_Y_ENC_MIN && slave2LimitTurun()) return;
    if (newTarget >= MOTOR_Y_ENC_MAX) return;

    newTarget = constrain(newTarget, MOTOR_Y_ENC_MIN, MOTOR_Y_ENC_MAX);
    sendSlave2Command("motortarget %ld", newTarget);
}

// ── R2 + L2 analog max-5 → flash lamp ────────────────────────────

constexpr uint8_t TRIGGER_MAX_THRESHOLD = 250;  // 255 - 5

void handleFlashTrigger(const ControlPacket &pkt) {
    if (pkt.l2Value >= TRIGGER_MAX_THRESHOLD && pkt.r2Value >= TRIGGER_MAX_THRESHOLD) {
        flashFire();
    }
}

} // anonymous namespace

void gripperMotorYSetLevel(uint8_t level) {
    level = constrain(level, 0, MOTOR_Y_LEVEL_MAX);
    gMotorYLevel = level;
    motorYSetTarget(MOTOR_Y_LEVEL_ENC[level]);
}

void gripperMotorYResetLevel() {
    gripperMotorYSetLevel(0);
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
    handleArmBoxYEnc(pkt);
    handleFlashTrigger(pkt);
    gPrevButtons = pkt.buttons;
}
