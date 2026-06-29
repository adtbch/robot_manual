/*
 * =====================================================================
 * FILE    : motion_control.ino
 * PERAN   : Mapping joystick/dpad → field-centric motion ke slave1motion
 *           via UART1 (kn vx vy yawTarget).
 *
 * INPUT MODE (toggle dengan SHARE) — hanya untuk vx/vy:
 *   ANALOG → analog kiri joystick
 *   DPAD   → tombol panah (digital, full RPM)
 *
 * YAW (terpisah dari input mode di atas):
 *   Analog kanan (rx) → ±yaw manual, selalu aktif (kecuali R2 / L2)
 *   rx ±1° per step — normal 50ms, L1 slow 150ms, R1 fast 25ms
 *   L2 + analog kanan → snap kardinal: atas 0, kanan 90, bawah 180, kiri -90
 *   gModeInvert → atas 180, bawah 0, kiri 90, kanan -90
 *   Nilai disimpan di gYawTarget master → dikirim sebagai argumen ke-3 pada kn
 *
 * SPEED MODE: vx/vy RPM max — normal 75, L1 slow 25, R1 fast 150
 *
 * INVERT INPUT (toggle L1+R1+L2+R2): gModeInvert — atas↔bawah, kiri↔kanan
 *
 * STREAM KE SLAVE1 (selalu, link hidup/mati):
 *   kirim kn tiap KN_SEND_INTERVAL_MS
 *   idle / link mati → kn 0 0 <yawTarget>  (cegah motor jalan sendiri)
 *
 * BOARD   : ESP32-S3 (Master)
 * =====================================================================
 */

#include "config.h"
#include "serial.h"
#include "espnow.h"

// =====================================================================
//  CONFIG
// =====================================================================

constexpr int16_t JOYSTICK_DEADZONE = 20;
constexpr int16_t JOYSTICK_MAX      = 127;

constexpr int16_t SPEED_RPM_NORMAL = 75;
constexpr int16_t SPEED_RPM_SLOW   = 25;
constexpr int16_t SPEED_RPM_FAST   = 150;

constexpr int16_t YAW_STICK_THRESHOLD = 30;
constexpr uint32_t YAW_STEP_INTERVAL_NORMAL_MS = 50;
constexpr uint32_t YAW_STEP_INTERVAL_SLOW_MS   = 150;
constexpr uint32_t YAW_STEP_INTERVAL_FAST_MS   = 25;
constexpr uint32_t KN_SEND_INTERVAL_MS = 20;

// =====================================================================
//  STATE — gInputMode, gYawTarget, gModeInvert (extern di config.h)
// =====================================================================

InputMode gInputMode = MODE_DPAD;
int16_t gYawTarget = 0;
bool gModeInvert = false;

namespace {

uint32_t gMotionPrevButtons = 0;
Jeda gJedaYawStep;
Jeda gJedaKnSend;

constexpr uint32_t BTN_INVERT_COMBO = BTN_L1 | BTN_R1 | BTN_L2 | BTN_R2;

bool allInvertComboHeld(uint32_t buttons) {
    return (buttons & BTN_INVERT_COMBO) == BTN_INVERT_COMBO;
}

int16_t mapJoystickToRpm(int16_t joyVal, int16_t rpmMax) {
    if (abs(joyVal) < JOYSTICK_DEADZONE) return 0;
    return (int16_t)((int32_t)joyVal * rpmMax / JOYSTICK_MAX);
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

    const bool linkUp = pkt.connected && espNowControlIsLinkAlive();

    if (linkUp) {
        bool shareNow = (pkt.buttons & BTN_SHARE) != 0;
        if (shareNow && !(gMotionPrevButtons & BTN_SHARE)) {
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

        int16_t rpmMax = SPEED_RPM_NORMAL;
        uint32_t yawStepMs = YAW_STEP_INTERVAL_NORMAL_MS;
        if (pkt.buttons & BTN_R1) {
            rpmMax = SPEED_RPM_FAST;
            yawStepMs = YAW_STEP_INTERVAL_FAST_MS;
        } else if (pkt.buttons & BTN_L1) {
            rpmMax = SPEED_RPM_SLOW;
            yawStepMs = YAW_STEP_INTERVAL_SLOW_MS;
        }

        vx = mapJoystickToRpm(ly, rpmMax);
        vy = mapJoystickToRpm(lx, rpmMax);
        scaleFieldVelocity(vx, vy, rpmMax);

        const int16_t rawRx = applyStickDeadzone(pkt.rx);
        const int16_t rawRy = applyStickDeadzone(-(int16_t)pkt.ry);
        if (pkt.buttons & BTN_L2) {
            updateYawTargetFromCardinalStick(rawRx, rawRy);
        } else if (!(pkt.buttons & BTN_R2)) {
            updateYawTargetFromStick(rawRx, yawStepMs);
        }
    } else {
        vx = 0;
        vy = 0;
    }
    sendKnCommand(vx, vy, gYawTarget);
}
