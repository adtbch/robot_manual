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

namespace {

constexpr int16_t ARMBOX_AXIS_DEADZONE = 30;

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

void armBoxControlTick(const ControlPacket &pkt) {
    if (odomIsModeSave()) return;
    if (!(pkt.buttons & BTN_CIRCLE)) {
        gArmBoxPrevDir = ARMBOX_DIR_NONE;
        return;
    }

    const ArmBoxInputDir dir = readArmBoxInputDir(pkt);
    if (dir == ARMBOX_DIR_NONE) {
        gArmBoxPrevDir = ARMBOX_DIR_NONE;
        return;
    }
    if (dir == gArmBoxPrevDir) return;

    gArmBoxPrevDir = dir;
    runArmBoxInputAction(dir);
}
