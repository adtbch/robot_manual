/*
 * =====================================================================
 * FILE    : motion_control.ino
 * PERAN   : Mapping joystick/dpad → field-centric motion ke slave1motion
 *           via UART1.
 *
 * DUA MODE KONTROL:
 *   GOTO (modeKinematics=false):
 *     stick → step-based increment gTargetX/Y_cm → sendGotoCommand
 *     L1/R1 → atur step interval (seberapa sering target ++):
 *       normal 20ms, L1 slow 100ms, R1 fast 5ms
 *     Setiap step: ±GOTO_STEP_CM per axis, sendGotoCommand tiap tick
 *
 *   KINEMATICS (modeKinematics=true):
 *     stick → vx/vy langsung → sendKnCommand
 *     L1/R1 → atur RPM (kecepatan robot):
 *       normal 75, L1 slow 25, R1 fast 150
 *
 * INPUT MODE (toggle dengan SHARE) — hanya untuk vx/vy:
 *   ANALOG → analog kiri joystick
 *   DPAD   → tombol panah (digital, full RPM)
 *
 * YAW (terpisah):
 *   Analog kanan (rx) → ±yaw manual
 *   rx ±1° per step — normal 50ms, L1 slow 150ms, R1 fast 25ms
 *   L2 + analog kanan → snap kardinal
 *
 * INVERT INPUT (toggle L1+R1+L2+R2): gModeInvert
 *
 * BOARD   : ESP32-S3 (Master)
 * =====================================================================
 */

#include "config.h"
#include "serial.h"
#include "espnow.h"
#include "odom.h"

// =====================================================================
//  CONFIG
// =====================================================================

constexpr int16_t JOYSTICK_DEADZONE = 20;
constexpr int16_t JOYSTICK_MAX      = 127;

// KINEMATICS mode: RPM — kecepatan langsung vx/vy
constexpr int16_t SPEED_RPM_NORMAL = 50;
constexpr int16_t SPEED_RPM_SLOW = 15;
constexpr int16_t SPEED_RPM_FAST = 150;

constexpr int16_t YAW_STICK_THRESHOLD = 30;
constexpr uint32_t YAW_STEP_INTERVAL_NORMAL_MS = 50;
constexpr uint32_t YAW_STEP_INTERVAL_SLOW_MS   = 150;
constexpr uint32_t YAW_STEP_INTERVAL_FAST_MS   = 25;
constexpr uint32_t kSendIntervalMs = 20;  // kirim ke slave1motion tiap 20ms

// =====================================================================
//  STATE — shared (extern di config.h)
// =====================================================================

InputMode gInputMode = MODE_DPAD;
int16_t gYawTarget = 0;
bool gModeInvert = false;
float gTargetX_cm = 0.0f;
float gTargetY_cm = 0.0f;
int16_t gTargetSpeedRpm = SPEED_RPM_NORMAL;
bool gMotionWaypointMode = false;
bool modeKinematics = true;  // KN-only — GOTO dihapus

namespace {

uint32_t gMotionPrevButtons = 0;
Jeda gJedaYawStep;
Jeda gJedaSend;

constexpr uint32_t BTN_INVERT_COMBO = BTN_L1 | BTN_R1 | BTN_L2 | BTN_R2;

bool allInvertComboHeld(uint32_t buttons) {
    return (buttons & BTN_INVERT_COMBO) == BTN_INVERT_COMBO;
}

int16_t mapJoystickToRpm(int16_t joyVal, int16_t gTargetSpeedRpm) {
    if (abs(joyVal) < JOYSTICK_DEADZONE) return 0;
    return (int16_t)((int32_t)joyVal * gTargetSpeedRpm / JOYSTICK_MAX);
}

int16_t applyStickDeadzone(int16_t val) {
    if (abs(val) < JOYSTICK_DEADZONE) return 0;
    if (val > 0) return map(val, JOYSTICK_DEADZONE, JOYSTICK_MAX, 0, JOYSTICK_MAX);
    return -map(-val, JOYSTICK_DEADZONE, JOYSTICK_MAX, 0, JOYSTICK_MAX);
}

int16_t wrapYawTarget(int16_t angle) {
    while (angle > 180) angle -= 360;
    while (angle < -179) angle += 360;
    return angle;
}

void scaleFieldVelocity(int16_t &vx, int16_t &vy, int16_t limit) {
    const int16_t peak = max(abs(vx), abs(vy));
    if (peak <= limit) return;
    vx = (int16_t)((int32_t)vx * limit / peak);
    vy = (int16_t)((int32_t)vy * limit / peak);
}

void updateYawTargetFromStick(int16_t rawRx, uint32_t stepIntervalMs) {
    if (abs(rawRx) <= YAW_STICK_THRESHOLD) return;
    if (!gJedaYawStep.check(stepIntervalMs)) return;

    const int dir = (rawRx > 0) ? 1 : -1;
    gYawTarget = wrapYawTarget(gYawTarget + dir);
}

void updateYawTargetFromCardinalStick(int16_t rawRx, int16_t rawRy) {
    if (abs(rawRx) <= YAW_STICK_THRESHOLD && abs(rawRy) <= YAW_STICK_THRESHOLD) return;

    int16_t target;
    if (abs(rawRx) >= abs(rawRy)) {
        target = (rawRx > 0) ? 90 : -90;
    } else {
        target = (rawRy > 0) ? 0 : 180;
    }
    if (gModeInvert) {
        if (target == 0)   target = 180;
        else if (target == 180) target = 0;
        else if (target == 90)  target = -90;
        else if (target == -90) target = 90;
    }
    gYawTarget = wrapYawTarget(target);
}

} // anonymous namespace

// =====================================================================
//  TICK — panggil di loop() dengan ControlPacket terbaru
// =====================================================================

void motionControlTick(const ControlPacket &pkt) {
    int16_t vx = 0;
    int16_t vy = 0;
    uint32_t yawStepMs = YAW_STEP_INTERVAL_NORMAL_MS;
    
    const bool linkUp = pkt.connected && espNowControlIsLinkAlive();
    
    if (linkUp && (!gMotionWaypointMode)) {
        gTargetSpeedRpm = SPEED_RPM_NORMAL;
        bool shareNow = (pkt.buttons & BTN_SHARE) != 0;
        const bool odomCombo = (pkt.buttons & (BTN_SHARE | BTN_TOUCHPAD)) == (BTN_SHARE | BTN_TOUCHPAD);
        // Jangan toggle mode saat SHARE+TOUCHPAD (odom record) atau mode_save aktif
        if (shareNow && !(gMotionPrevButtons & BTN_SHARE) && !(pkt.buttons & BTN_OPTIONS)
            && !odomCombo && !odomIsModeSave()) {
            gInputMode = (gInputMode == MODE_ANALOG) ? MODE_DPAD : MODE_ANALOG;
        }
        if (allInvertComboHeld(pkt.buttons) && !allInvertComboHeld(gMotionPrevButtons)) {
            gModeInvert = !gModeInvert;
        }
        gMotionPrevButtons = pkt.buttons;

        int16_t lx = 0, ly = 0;

        if (gInputMode == MODE_ANALOG) {
            lx = pkt.lx;
            ly = -(int16_t)pkt.ly;
        } else {
            if (pkt.buttons & BTN_UP)    ly =  127;
            if (pkt.buttons & BTN_DOWN)  ly = -127;
            if (pkt.buttons & BTN_LEFT)  lx = -127;
            if (pkt.buttons & BTN_RIGHT) lx =  127;
        }

        if (gModeInvert) {
            lx = -lx;
            ly = -ly;
        }

        if (pkt.buttons & BTN_R1) {
            yawStepMs = YAW_STEP_INTERVAL_FAST_MS;
            gTargetSpeedRpm = SPEED_RPM_FAST;
        } else if (pkt.buttons & BTN_L1) {
            yawStepMs = YAW_STEP_INTERVAL_SLOW_MS;
            gTargetSpeedRpm = SPEED_RPM_SLOW;
        }

        vx = mapJoystickToRpm(ly, gTargetSpeedRpm);
        vy = mapJoystickToRpm(lx, gTargetSpeedRpm);
        scaleFieldVelocity(vx, vy, gTargetSpeedRpm);

        const int16_t rawRx = applyStickDeadzone(pkt.rx);
        const int16_t rawRy = applyStickDeadzone(-(int16_t)pkt.ry);
        // ponytail: pakai l2Value threshold bukan BTN_L2 — trigger PS4 butuh penuh untuk set bit digital
        const bool l2Active = (pkt.l2Value > 64) && !(pkt.buttons & BTN_R2);
        const bool r2Active = (pkt.r2Value > 64);
        if (l2Active) {
            updateYawTargetFromCardinalStick(rawRx, rawRy);
        } else if (!r2Active) {
            updateYawTargetFromStick(rawRx, yawStepMs);
        }
        if (pkt.buttons & BTN_SQUARE) {
            vx = 0;
            vy = 0;
        }
    } else {
        vx = 0;
        vy = 0;
    }
    if(!gMotionWaypointMode){
        if (gJedaSend.check(kSendIntervalMs)) {
            sendKnCommand(vx, vy, gYawTarget);
        }
    }
}