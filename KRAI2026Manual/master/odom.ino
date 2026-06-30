/*
 * =====================================================================
 * FILE    : odom.ino
 * PERAN   : Record odom slave1 per tekan tombol (edge, bukan loop).
 *
 * TRIGGER : TOUCHPAD + SHARE bersamaan
 *   → sendShowOdomCommand()
 *   → tunggu odomToMaster dari UART1
 *   → push {x, y, yaw} ke buffer tetap
 *
 * BOARD   : ESP32-S3 (Master)
 * =====================================================================
 */

#include "odom.h"
#include "serial.h"

namespace {

constexpr uint32_t BTN_RECORD_COMBO = BTN_TOUCHPAD | BTN_SHARE;
constexpr size_t   ODOM_RECORD_MAX  = 64;

uint32_t    gOdomRecordPrevButtons = 0;
bool        gOdomRecordPending     = false;
size_t      gOdomRecordCount       = 0;
OdomSample  gOdomRecord[ODOM_RECORD_MAX];

bool comboHeld(uint32_t buttons, uint32_t combo) {
    return (buttons & combo) == combo;
}

bool comboPressed(uint32_t now, uint32_t prev, uint32_t combo) {
    return comboHeld(now, combo) && !comboHeld(prev, combo);
}

} // namespace

void odomRecordTick(const ControlPacket& pkt) {
    if (!pkt.connected) return;

    if (comboPressed(pkt.buttons, gOdomRecordPrevButtons, BTN_RECORD_COMBO)) {
        gOdomRecordPending = true;
        sendShowOdomCommand();
        Serial.println("[OdomRecord] poll → tunggu odomToMaster");
    }

    gOdomRecordPrevButtons = pkt.buttons;
}

void odomOnSampleReceived(float x_m, float y_m, float yaw_deg) {
    if (!gOdomRecordPending) return;

    gOdomRecordPending = false;

    if (gOdomRecordCount >= ODOM_RECORD_MAX) {
        Serial.println("[OdomRecord] buffer penuh");
        return;
    }

    gOdomRecord[gOdomRecordCount] = {x_m, y_m, yaw_deg};
    gOdomRecordCount++;
    Serial.printf("[OdomRecord] #%u saved x=%.3fm y=%.3fm yaw=%.1fdeg\n",
                  (unsigned)gOdomRecordCount, x_m, y_m, yaw_deg);
}

void odomRecordClear() {
    gOdomRecordCount   = 0;
    gOdomRecordPending = false;
    Serial.println("[OdomRecord] cleared");
}

size_t odomRecordCount() {
    return gOdomRecordCount;
}

void odomRecordPrint(Print& out) {
    out.printf("OdomRecord: %u titik\n", (unsigned)gOdomRecordCount);
    for (size_t i = 0; i < gOdomRecordCount; i++) {
        out.printf("  %u: x=%.3fm y=%.3fm yaw=%.1fdeg\n",
                   (unsigned)i,
                   gOdomRecord[i].x_m,
                   gOdomRecord[i].y_m,
                   gOdomRecord[i].yaw_deg);
    }
}
