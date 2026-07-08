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
#include "odom.h"

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
constexpr int ARM_STEP_NORMAL = 15;
constexpr int ARM_STEP_FAST   = 40;

constexpr uint32_t ARMBOX_Y_ENC_INTERVAL_MS = 50;
constexpr uint32_t SERVO_B_STEP_INTERVAL_MS = 100;
constexpr long ARMBOX_Y_ENC_MIN = 0;
constexpr long ARMBOX_Y_ENC_MAX = 4058;

// =====================================================================
//  STATE
// =====================================================================

static uint8_t gMotorYLevel = 0;

namespace {

uint32_t gPrevButtons = 0;
int gServoTAngle = 80;
Jeda gJedaMotorYStep;
Jeda gJedaMotorYLevel;
Jeda gJedaMotorXStep;
Jeda gJedaArmBoxYEnc;
Jeda gJedaServoBStep;

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
    motorYSetTarget(gMotorYLevelEnc[gMotorYLevel]);
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
        setupZone1();
        return;
    }
    if (isPressed(pkt.buttons, BTN_CROSS) && !(pkt.buttons & BTN_L2)) {
        static bool servoBState = false;
        servoBState = !servoBState;
        setServoAngle('b', servoBState ? 90 : 0);
    }
}

void handleManualServoB(const ControlPacket &pkt) {
    if (!(pkt.buttons & BTN_TRIANGLE)) return;

    const int dir = servoBMoveDir(pkt);
    if (dir == 0) return;
    if (!gJedaServoBStep.check(SERVO_B_STEP_INTERVAL_MS)) return;

    const int step = pickSpeedStep(pkt.buttons, ARM_STEP_NORMAL, ARM_STEP_SLOW, ARM_STEP_FAST);
    setServoTAngle(gServoTAngle + dir * step);
}

void handleGripperMotors(const ControlPacket &pkt) { 
    if (isR2Held(pkt.buttons)) {
        driveMotorY(applyDeadzoneY(readInvertedAxisY(pkt)), pkt.buttons);
        driveMotorX(applyDeadzoneX(readInvertedAxisX(pkt)), pkt.buttons);
    }
}

// ── R2 hold + analog kanan → armbox motor Y jog manual (slave2) ──

void handleArmBoxYEnc(const ControlPacket &pkt) {
    if (!(pkt.buttons & BTN_R2)) return;

    const int16_t ry = pkt.ry;
    if (abs(ry) <= AXIS_DEADZONE) return;
    if (!gJedaArmBoxYEnc.check(ARMBOX_Y_ENC_INTERVAL_MS)) return;

    const char dir = (ry < 0) ? 'u' : 'd';
    const long step = pickSpeedStep(pkt.buttons,
        MOTOR_Y_STEP_NORMAL, MOTOR_Y_STEP_SLOW, MOTOR_Y_STEP_FAST);

    sendSlave2Command("motory %c %ld", dir, step);
}

// ── R2 + L2 analog max-5 → flash lamp ────────────────────────────

constexpr uint8_t TRIGGER_MAX_THRESHOLD = 250;  // 255 - 5

void handleFlashTrigger(const ControlPacket &pkt) {
    if (pkt.l2Value >= TRIGGER_MAX_THRESHOLD && pkt.r2Value >= TRIGGER_MAX_THRESHOLD) {
        flashFire();
    }
}

// ── L2 + Triangle (mode manual) → user-defined action ────────────

void handleL2TriangleManual(const ControlPacket &pkt) {
    if (gControllerMode != 0) return;
    if (!isComboEdge(pkt.buttons, gPrevButtons, BTN_L2, BTN_TRIANGLE)) return;

    if (gGripperState == OPENING) {
        setServoAngle('d', 75);
        gGripperState = CLOSING;
    } else if (gGripperState == UP) {
        gripperReadytoStab();
    }
}

} // anonymous namespace

void gripperMotorYSetLevel(uint8_t level) {
    level = constrain(level, 0, MOTOR_Y_LEVEL_MAX);
    gMotorYLevel = level;
    motorYSetTarget(gMotorYLevelEnc[level]);
}

void gripperMotorYResetLevel() {
    gripperMotorYSetLevel(0);
}

int getServoTAngle() {
    return gServoTAngle;
}

void setServoTAngle(int angle) {
    gServoTAngle = constrain(angle, 0, 100);
    setServoAngle('t', gServoTAngle);
}

void gripperControlTick(const ControlPacket &pkt) {
    if (odomIsModeSave()) return;
    handleGripperButtons(pkt);
    handleManualServoB(pkt);
    handleGripperMotors(pkt);
    handleArmBoxYEnc(pkt);
    handleFlashTrigger(pkt);
    handleL2TriangleManual(pkt);
    gPrevButtons = pkt.buttons;
}