/*
 * =====================================================================
 * FILE    : armBox_control.ino
 * PERAN   : Manual arm box — Circle hold + arah input kiri.
 *
 * BUTTON MAPPING (tahan Circle + arah, edge saat arah berubah):
 *   Kanan → armBoxFBToggle('l')  motor k — arah dari limit master (depan/belakang)
 *   Kiri  → armBoxFBToggle('r')  motor x — arah dari slave2LimitDepan/Belakang
 *   Atas  → armBoxDone('l')
 *   Bawah → armBoxDone('r')
 *
 * INPUT (terbalik motion_control — sama pola gripper_control, bukan gModeInvert):
 *   MODE_DPAD   → analog kiri (ly negasi int16)
 *   MODE_ANALOG → tombol panah
 *
 * BOARD   : ESP32-S3 (Master)
 * =====================================================================
 */

#include "config.h"
#include "serial.h"
#include "odom.h"

// armBox.ino
void armBoxDone(char side);
void armBoxFBToggle(char side);
void armBoxStartGrab(char side);

namespace {

constexpr int16_t ARMBOX_AXIS_DEADZONE = 100;

enum ArmBoxInputDir : int8_t { ARMBOX_DIR_NONE = -1, ARMBOX_DIR_UP, ARMBOX_DIR_DOWN, ARMBOX_DIR_LEFT, ARMBOX_DIR_RIGHT };

ArmBoxInputDir gArmBoxPrevDir = ARMBOX_DIR_NONE;

int16_t readArmBoxAxisLx(const ControlPacket &pkt) {
    if (gInputMode == MODE_DPAD) return pkt.lx;
    if (pkt.buttons & BTN_RIGHT) return 127;
    if (pkt.buttons & BTN_LEFT)  return -127;
    return 0;
}

int16_t readArmBoxAxisLy(const ControlPacket &pkt) {
    if (gInputMode == MODE_DPAD) return -(int16_t)pkt.ly;
    if (pkt.buttons & BTN_UP)    return 127;
    if (pkt.buttons & BTN_DOWN)  return -127;
    return 0;
}

ArmBoxInputDir readArmBoxInputDir(const ControlPacket &pkt) {
    const int16_t lx = readArmBoxAxisLx(pkt);
    const int16_t ly = readArmBoxAxisLy(pkt);
    if (abs(lx) <= ARMBOX_AXIS_DEADZONE && abs(ly) <= ARMBOX_AXIS_DEADZONE) {
        return ARMBOX_DIR_NONE;
    }
    if (abs(lx) >= abs(ly)) {
        return (lx > 0) ? ARMBOX_DIR_RIGHT : ARMBOX_DIR_LEFT;
    }
    return (ly > 0) ? ARMBOX_DIR_UP : ARMBOX_DIR_DOWN;
}

void runArmBoxInputAction(ArmBoxInputDir dir) {
    switch (dir) {
        case ARMBOX_DIR_RIGHT:
            armBoxFBToggle('r');
            break;
        case ARMBOX_DIR_LEFT:
            armBoxFBToggle('l');
            break;
        case ARMBOX_DIR_UP:
            armBoxDone('l');
            break;
        case ARMBOX_DIR_DOWN:
            armBoxDone('r');
            break;
        default:
            break;
    }
}

} // anonymous namespace

// ── L2 + Square (mode manual) → user-defined action ──────────────

namespace {

uint32_t gArmBoxPrevButtons = 0;

} // anonymous namespace

void handleL2SquareManual(const ControlPacket &pkt) {
    if (gControllerMode != 0) return;
    if (!isComboEdge(pkt.buttons, gArmBoxPrevButtons, BTN_L2, BTN_SQUARE)) return;

    gripperMotorYSetLevel(4);
    armBoxFBbySpeed('l', -255);
    sendSlave2Command("motortarget %ld", gMotorYLevelEnc[4]);
    armBoxFBbySpeed('r', -255);
    sendSlave2Command("pne l on");
    sendSlave2Command("pne lk on");
    sendSlave2Command("pne r on");
    sendSlave2Command("pne rk on");
}

// ── L2 + Cross (mode manual) → user-defined action ──────────────

void handleL2CrossManual(const ControlPacket &pkt) {
    if (gControllerMode != 0) return;
    if (!isComboEdge(pkt.buttons, gArmBoxPrevButtons, BTN_L2, BTN_CROSS)) return;

    // TODO: isi action di sini
}

// ── L2 + Circle + analog kiri (mode manual) → pne R/L ────────────

constexpr int8_t LX_DIRECTION_THRESHOLD = 100;

void handleL2CircleLxManual(const ControlPacket &pkt) {
    if (gControllerMode != 0) return;
    if (!(pkt.buttons & BTN_L2)) return;
    if (!(pkt.buttons & BTN_CIRCLE)) return;

    const int8_t lx = pkt.lx;
    if (abs(lx) <= LX_DIRECTION_THRESHOLD) return;

    if (lx > 0) {
        armBoxStartGrab('r');
    } else {
        armBoxStartGrab('l');
    }
}

void armBoxControlTick(const ControlPacket &pkt) {
    if (odomIsModeSave()) return;

    handleL2SquareManual(pkt);
    handleL2CrossManual(pkt);
    handleL2CircleLxManual(pkt);

    // Skip Circle+arah jika L2 hold (sudah ditangani handleL2CircleLxManual)
    if (pkt.buttons & BTN_L2) {
        gArmBoxPrevButtons = pkt.buttons;
        return;
    }

    if (!(pkt.buttons & BTN_CIRCLE)) {
        gArmBoxPrevDir = ARMBOX_DIR_NONE;
        gArmBoxPrevButtons = pkt.buttons;
        return;
    }

    const ArmBoxInputDir dir = readArmBoxInputDir(pkt);
    if (dir == ARMBOX_DIR_NONE) {
        gArmBoxPrevDir = ARMBOX_DIR_NONE;
        gArmBoxPrevButtons = pkt.buttons;
        return;
    }
    if (dir == gArmBoxPrevDir) {
        gArmBoxPrevButtons = pkt.buttons;
        return;
    }

    gArmBoxPrevDir = dir;
    gArmBoxPrevButtons = pkt.buttons;
    runArmBoxInputAction(dir);
}
