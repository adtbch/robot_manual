/*
 * =====================================================================
 * FILE    : odom.ino
 * PERAN   : Record 4 waypoint odom slave1.
 *
 * SHARE + TOUCHPAD tahan 5 detik → reset 4 titik + mode_save ON
 * SHARE + TOUCHPAD tap (<5s)      → record #1..#4 (hanya di mode_save)
 * R3 + L3 edge                    → keluar mode_save
 *
 * Slave1 kirim meter → disimpan sebagai cm di master.
 *
 * BOARD   : ESP32-S3 (Master)
 * =====================================================================
 */

#include "odom.h"
#include "serial.h"

OdomWaypoint gOdomWaypoints[ODOM_WP_COUNT] = {};
uint8_t      gOdomWpFilled  = 0;
bool         gOdomModeSave    = false;

namespace {

constexpr uint32_t BTN_RECORD_COMBO = BTN_TOUCHPAD | BTN_SHARE;
constexpr uint32_t BTN_EXIT_COMBO   = BTN_R3 | BTN_L3;

uint32_t gOdomPrevButtons  = 0;
bool     gOdomRecordPending = false;
bool     gOdomComboHeld     = false;
bool     gOdomHoldFired     = false;
uint32_t gOdomHoldStartMs   = 0;

bool comboHeld(uint32_t buttons, uint32_t combo) {
    return (buttons & combo) == combo;
}

bool comboPressed(uint32_t now, uint32_t prev, uint32_t combo) {
    return comboHeld(now, combo) && !comboHeld(prev, combo);
}

bool comboReleased(uint32_t now, uint32_t prev, uint32_t combo) {
    return !comboHeld(now, combo) && comboHeld(prev, combo);
}

void odomRequestRecord() {
    if (gOdomWpFilled >= ODOM_WP_COUNT) {
        Serial.println("[OdomRecord] 4 titik penuh");
        return;
    }
    gOdomRecordPending = true;
    sendShowOdomCommand();
    Serial.printf("[OdomRecord] poll → tunggu titik #%u\n", gOdomWpFilled + 1);
}

void odomEnterModeSave() {
    odomRecordClear();
    gOdomModeSave = true;
    Serial.println("[OdomRecord] mode_save ON — tap SHARE+TOUCHPAD untuk #1");
}

} // anonymous namespace

void odomRecordTick(const ControlPacket& pkt) {
    if (!pkt.connected) return;

    const uint32_t btn = pkt.buttons;

    if (comboPressed(btn, gOdomPrevButtons, BTN_EXIT_COMBO) && gOdomModeSave) {
        gOdomModeSave = false;
        Serial.println("[OdomRecord] mode_save OFF");
    }

    if (comboPressed(btn, gOdomPrevButtons, BTN_RECORD_COMBO)) {
        gOdomComboHeld   = true;
        gOdomHoldFired   = false;
        gOdomHoldStartMs = millis();
    }

    if (gOdomComboHeld && comboHeld(btn, BTN_RECORD_COMBO)) {
        if (!gOdomHoldFired && (millis() - gOdomHoldStartMs) >= ODOM_MODE_HOLD_MS) {
            gOdomHoldFired = true;
            odomEnterModeSave();
        }
    }

    if (comboReleased(btn, gOdomPrevButtons, BTN_RECORD_COMBO)) {
        if (gOdomModeSave && !gOdomHoldFired) {
            odomRequestRecord();
        }
        gOdomComboHeld = false;
        gOdomHoldFired  = false;
    }

    gOdomPrevButtons = btn;
}

void odomOnSampleReceived(float x_m, float y_m, float yaw_deg) {
    if (!gOdomRecordPending) return;

    gOdomRecordPending = false;

    if (!gOdomModeSave) return;

    if (gOdomWpFilled >= ODOM_WP_COUNT) {
        Serial.println("[OdomRecord] 4 titik penuh");
        return;
    }

    OdomWaypoint& wp = gOdomWaypoints[gOdomWpFilled];
    wp.x_cm          = x_m * 100.0f;
    wp.y_cm          = y_m * 100.0f;
    wp.yaw_deg       = yaw_deg;
    wp.maxspeed_rpm  = ODOM_WP_DEFAULT_SPEED_RPM;
    gOdomWpFilled++;

    Serial.printf("[OdomRecord] #%u saved x=%.1fcm y=%.1fcm yaw=%.1fdeg speed=%drpm\n",
                  gOdomWpFilled, wp.x_cm, wp.y_cm, wp.yaw_deg, wp.maxspeed_rpm);
}

void odomRecordClear() {
    for (size_t i = 0; i < ODOM_WP_COUNT; i++) {
        gOdomWaypoints[i] = {};
    }
    gOdomWpFilled      = 0;
    gOdomRecordPending = false;
}

bool odomIsModeSave() {
    return gOdomModeSave;
}

void odomRecordPrint(Print& out) {
    out.printf("OdomRecord: %u/4 titik (mode_save=%s)\n",
               gOdomWpFilled, gOdomModeSave ? "ON" : "OFF");
    for (uint8_t i = 0; i < gOdomWpFilled; i++) {
        const OdomWaypoint& wp = gOdomWaypoints[i];
        out.printf("  #%u: x=%.1fcm y=%.1fcm yaw=%.1fdeg speed=%drpm\n",
                   i + 1, wp.x_cm, wp.y_cm, wp.yaw_deg, wp.maxspeed_rpm);
    }
}

bool odomGoto(uint8_t slot) {
    if (slot < 1 || slot > ODOM_WP_COUNT || slot > gOdomWpFilled) {
        return false;
    }

    const OdomWaypoint& wp = gOdomWaypoints[slot - 1];
    gTargetX_cm           = wp.x_cm;
    gTargetY_cm           = wp.y_cm;
    gYawTarget            = (int16_t)lroundf(wp.yaw_deg);
    gTargetSpeedRpm       = wp.maxspeed_rpm;
    gMotionWaypointMode   = true;

    Serial.printf("[OdomGoto] #%u → x=%.1f y=%.1f yaw=%d speed=%d\n",
                  slot, gTargetX_cm, gTargetY_cm, gYawTarget, gTargetSpeedRpm);
    return true;
}
