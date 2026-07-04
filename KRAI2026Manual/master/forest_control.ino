/*
 * =====================================================================
 * FILE    : forest_control.ino
 * PERAN   : Square + DPAD → forest goto slot 1/2, exit.
 * BOARD   : ESP32-S3 (Master)
 * =====================================================================
 */

#include "forest.h"
#include "config.h"
#include "espnow.h"
#include "odom.h"

void forestControlTick(const ControlPacket& pkt) {
    if (gLastRxPacket.mode != 1) return;
    if (odomIsModeSave()) return;
    static uint32_t prevButtons = 0;

    if (!pkt.connected || !espNowControlIsLinkAlive()) {
        prevButtons = pkt.buttons;
        return;
    }

    if (!(pkt.buttons & BTN_SQUARE)) {
        prevButtons = pkt.buttons;
        return;
    }

    constexpr uint32_t kDpad = BTN_UP | BTN_DOWN | BTN_LEFT | BTN_RIGHT;
    const uint32_t prevDpad = prevButtons & kDpad;
    const uint32_t nowDpad  = pkt.buttons & kDpad;

    if ((nowDpad & BTN_UP) && !(prevDpad & BTN_UP)) {
        gripperMotorYSetLevel(4);
        armBoxFBbySpeed('l', -255);
        sendSlave2Command("motortarget %ld", gMotorYLevelEnc[4]);
        armBoxFBbySpeed('r', -255);
        forestGotoSlot(1);

    } else if ((nowDpad & BTN_LEFT) && !(prevDpad & BTN_LEFT)) {
        forestGotoSlot(2);
    } else if ((nowDpad & BTN_DOWN) && !(prevDpad & BTN_DOWN)) {
        forestTriggerExit();
        zoneState = 2;
    }

    prevButtons = pkt.buttons;
}
