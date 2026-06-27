/*
 * =====================================================================
 * FILE    : motion_control.ino
 * PERAN   : Mapping joystick/dpad → field-centric motion ke slave1motion
 *           via UART1 (kn vx vy yaw).
 *
 * INPUT MODE (toggle dengan SHARE):
 *   ANALOG → analog kiri joystick (default)
 *   DPAD   → tombol panah (digital, full RPM)
 *
 * SPEED MODE:
 *   R1 hold → fast (1.5x)
 *   L1 hold → slow (0.5x)
 *   default → normal (1.0x)
 *
 * YAW TARGET:
 *   Analog kanan (rx) → geser target heading ±1..5° per tick
 *
 * SERIAL COMMAND KE SLAVE1:
 *   kn <vx> <vy> <yawDeg>  — field-cent RPM + yaw correction (slave1)
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

constexpr int16_t JOYSTICK_DEADZONE = 20;     // noise filter
constexpr int16_t JOYSTICK_MAX      = 127;    // max joystick value
constexpr int16_t RPM_MAX           = 100;    // max RPM ke slave1

// Speed multiplier (hold R1/L1)
constexpr float SPEED_FAST   = 1.5f;
constexpr float SPEED_NORMAL = 1.0f;
constexpr float SPEED_SLOW   = 0.5f;

// =====================================================================
//  STATE — gInputMode didefinisikan di sini (extern di config.h)
// =====================================================================

InputMode gInputMode = MODE_DPAD;

namespace {

uint32_t gMotionPrevButtons = 0;  // edge detection untuk SHARE
int16_t gYawTarget = 0;
int16_t gPrevVx = 0;
int16_t gPrevVy = 0;
int16_t gPrevYaw = 0;
bool gWasActive = false;

int16_t mapJoystickToRpm(int16_t joyVal, int16_t rpmMax) {
    if (abs(joyVal) < JOYSTICK_DEADZONE) return 0;
    return (int16_t)((int32_t)joyVal * rpmMax / JOYSTICK_MAX);
}

void scaleFieldVelocity(int16_t &vx, int16_t &vy, int16_t limit) {
    const int16_t peak = max(abs(vx), abs(vy));
    if (peak <= limit) return;
    vx = (int16_t)((int32_t)vx * limit / peak);
    vy = (int16_t)((int32_t)vy * limit / peak);
}

void adjustYawTargetFromStick(int16_t rx) {
    if (abs(rx) < JOYSTICK_DEADZONE) return;

    const int inc = map(abs(rx), JOYSTICK_DEADZONE, JOYSTICK_MAX, 1, 5);
    if (rx > 0) {
        gYawTarget += inc;
        if (gYawTarget > 180) gYawTarget = -179;
    } else {
        gYawTarget -= inc;
        if (gYawTarget < -179) gYawTarget = 180;
    }
}

} // anonymous namespace

// =====================================================================
//  TICK — panggil di loop() dengan ControlPacket terbaru
// =====================================================================

void motionControlTick(const ControlPacket &pkt) {
    //  SAFETY: PS4 disconnected ATAU ESP-NOW putus → STOP
    if (!pkt.connected || !espNowControlIsLinkAlive()) {
        if (gWasActive) {
            sendSlave1Stop();
            gWasActive = false;
            gPrevVx = gPrevVy = 0;
        }
        return;
    }

    // ============================================================
    //  TOGGLE INPUT MODE — SHARE (edge detection)
    // ============================================================
    bool shareNow = (pkt.buttons & BTN_SHARE) != 0;
    if (shareNow && !(gMotionPrevButtons & BTN_SHARE)) {
        gInputMode = (gInputMode == MODE_ANALOG) ? MODE_DPAD : MODE_ANALOG;
    }
    gMotionPrevButtons = pkt.buttons;

    // ============================================================
    //  BACA INPUT — analog atau dpad
    // ============================================================
    int16_t lx = 0, ly = 0;

    if (gInputMode == MODE_ANALOG) {
        lx = pkt.lx;
        ly = -(int16_t)pkt.ly;  // int16_t: -(-128) = +128, tidak overflow int8_t
    } else {
        if (pkt.buttons & BTN_UP)    ly =  127;
        if (pkt.buttons & BTN_DOWN)  ly = -127;
        if (pkt.buttons & BTN_LEFT)  lx = -127;
        if (pkt.buttons & BTN_RIGHT) lx =  127;
    }

    // Speed mode → skala rpmMax (bukan multiply setelah map, supaya R1/L1 kelihatan)
    float speedMul = SPEED_NORMAL;
    if (pkt.buttons & BTN_R1) speedMul = SPEED_FAST;
    else if (pkt.buttons & BTN_L1) speedMul = SPEED_SLOW;
    const int16_t rpmMax = (int16_t)(RPM_MAX * speedMul);

    int16_t vx = mapJoystickToRpm(ly, rpmMax);
    int16_t vy = mapJoystickToRpm(lx, rpmMax);
    scaleFieldVelocity(vx, vy, rpmMax);

    adjustYawTargetFromStick(pkt.rx);

    const bool translating = (vx != 0 || vy != 0);
    const bool changed = (vx != gPrevVx || vy != gPrevVy || gYawTarget != gPrevYaw);

    if (translating || changed || gWasActive) {
        sendKnCommand(vx, vy, gYawTarget);
    }

    gPrevVx = vx;
    gPrevVy = vy;
    gPrevYaw = gYawTarget;
    gWasActive = translating;
}
