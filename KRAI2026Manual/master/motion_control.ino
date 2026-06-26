/*
 * =====================================================================
 * FILE    : motion_control.ino
 * PERAN   : Mapping joystick/dpad → perintah motor mecanum
 *           ke slave1motion via UART1.
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
 * SERIAL COMMAND KE SLAVE1:
 *   rpm <fr> <fl> <br> <bl>
 *
 * KINEMATIK MECANUM:
 *   FR = LY + LX
 *   FL = LY - LX
 *   BR = LY - LX
 *   BL = LY + LX
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
constexpr int16_t RPM_MAX           = 1000;   // max RPM ke slave1

// Speed multiplier (hold R1/L1)
constexpr float SPEED_FAST  = 1.5f;
constexpr float SPEED_NORMAL = 1.0f;
constexpr float SPEED_SLOW  = 0.5f;

// =====================================================================
//  STATE
// =====================================================================

// =====================================================================
//  STATE — gInputMode didefinisikan di sini (extern di config.h)
// =====================================================================

InputMode gInputMode = MODE_DPAD;

namespace {

uint32_t gMotionPrevButtons = 0;  // edge detection untuk SHARE

int16_t gPrevFr = 0, gPrevFl = 0, gPrevBr = 0, gPrevBl = 0;
bool gWasMoving = false;

int16_t mapJoystickToRpm(int8_t joyVal) {
    // Deadzone
    if (abs(joyVal) < JOYSTICK_DEADZONE) return 0;
    // Map -127..127 → -RPM_MAX..RPM_MAX
    return (int16_t)((int32_t)joyVal * RPM_MAX / JOYSTICK_MAX);
}

} // anonymous namespace

// =====================================================================
//  TICK — panggil di loop() dengan ControlPacket terbaru
// =====================================================================

void motionControlTick(const ControlPacket &pkt) {
    //  SAFETY: PS4 disconnected ATAU ESP-NOW putus → STOP
    if (!(pkt.connected != 0) || !espNowControlIsLinkAlive()) {
        if (gWasMoving) {
            sendRpmCommand(0, 0, 0, 0);
            gWasMoving = false;
            gPrevFr = gPrevFl = gPrevBr = gPrevBl = 0;
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
    int8_t lx = 0, ly = 0;

    if (gInputMode == MODE_ANALOG) {
        // Analog kiri joystick
        lx = pkt.lx;
        ly = pkt.ly;
    } else {
        // D-pad → digital (on/off = ±127 → mapJoystickToRpm)
        if (pkt.buttons & BTN_UP)    ly =  127;
        if (pkt.buttons & BTN_DOWN)  ly = -127;
        if (pkt.buttons & BTN_LEFT)  lx = -127;
        if (pkt.buttons & BTN_RIGHT) lx =  127;
    }

    // Speed mode (hold button)
    float speedMul = SPEED_NORMAL;
    if (pkt.buttons & BTN_R1) speedMul = SPEED_FAST;
    else if (pkt.buttons & BTN_L1) speedMul = SPEED_SLOW;

    // Kinematik mecanum
    int16_t fr = mapJoystickToRpm(ly + lx);
    int16_t fl = mapJoystickToRpm(ly - lx);
    int16_t br = mapJoystickToRpm(ly - lx);
    int16_t bl = mapJoystickToRpm(ly + lx);

    // Apply speed multiplier
    fr = (int16_t)(fr * speedMul);
    fl = (int16_t)(fl * speedMul);
    br = (int16_t)(br * speedMul);
    bl = (int16_t)(bl * speedMul);

    // Constrain ke ±RPM_MAX
    fr = constrain(fr, -RPM_MAX, RPM_MAX);
    fl = constrain(fl, -RPM_MAX, RPM_MAX);
    br = constrain(br, -RPM_MAX, RPM_MAX);
    bl = constrain(bl, -RPM_MAX, RPM_MAX);

    // Cek apakah ada perubahan
    bool isMoving = (fr != 0 || fl != 0 || br != 0 || bl != 0);
    bool changed = (fr != gPrevFr || fl != gPrevFl || br != gPrevBr || bl != gPrevBl);

    // Kirim jika:
    // 1. Baru mulai gerak (stop → move)
    // 2. Masih gerak tapi ada perubahan RPM
    // 3. Baru berhenti (move → stop)
    if (isMoving || gWasMoving) {
        if (changed || isMoving != gWasMoving) {
            sendRpmCommand(fr, fl, br, bl);
        }
    }

    // Update state
    gPrevFr = fr;
    gPrevFl = fl;
    gPrevBr = br;
    gPrevBl = bl;
    gWasMoving = isMoving;
}
