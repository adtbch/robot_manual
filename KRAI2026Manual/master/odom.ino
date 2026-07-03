/*
 * =====================================================================
 * FILE    : odom.ino
 * PERAN   : Record waypoint odom slave1 — zone1 RED/BLUE, approach, forest.
 *
 * BOARD   : ESP32-S3 (Master)
 * =====================================================================
 */

#include "odom.h"
#include "forest.h"
#include "serial.h"
#include <Preferences.h>

OdomWaypoint gOdomZone1[2][ODOM_WP_COUNT] = {};
OdomWaypoint gOdomWaypoints[ODOM_WP_COUNT] = {};
bool         gOdomModeSave = false;

namespace {

constexpr uint32_t BTN_ENTER_COMBO = BTN_TOUCHPAD | BTN_SHARE;
constexpr uint32_t BTN_EXIT_COMBO  = BTN_R3 | BTN_L3;
constexpr const char* ODOM_NVS_NS    = "odom_rec";

struct RecCombo {
    uint32_t      mask;
    OdomRecTarget target;
};

constexpr RecCombo REC_COMBOS[] = {
    { BTN_R1 | BTN_TOUCHPAD,       OdomRecTarget::ZONE1_0 },
    { BTN_L1 | BTN_TOUCHPAD,       OdomRecTarget::ZONE1_1 },
    { BTN_R2 | BTN_TOUCHPAD,       OdomRecTarget::ZONE1_2 },
    { BTN_L2 | BTN_TOUCHPAD,       OdomRecTarget::ZONE1_3 },
    { BTN_R1 | BTN_TRIANGLE,       OdomRecTarget::APPROACH_0 },
    { BTN_L1 | BTN_SQUARE,         OdomRecTarget::APPROACH_1 },
    { BTN_R2 | BTN_TRIANGLE,       OdomRecTarget::APPROACH_2 },
    { BTN_L2 | BTN_SQUARE,         OdomRecTarget::APPROACH_3 },
    { BTN_CROSS | BTN_TOUCHPAD,    OdomRecTarget::FOREST_2 },
    { BTN_SQUARE | BTN_TOUCHPAD,   OdomRecTarget::FOREST_6 },
    { BTN_CIRCLE | BTN_TOUCHPAD,   OdomRecTarget::FOREST_7 },
    { BTN_TRIANGLE | BTN_TOUCHPAD, OdomRecTarget::FOREST_11 },
};

uint32_t      gOdomPrevButtons   = 0;
bool          gOdomRecordPending = false;
OdomRecTarget gOdomPendingTarget = OdomRecTarget::NONE;
bool          gOdomComboHeld     = false;
bool          gOdomHoldFired     = false;
uint32_t      gOdomHoldStartMs   = 0;

char allianceNvsChar(AllianceColor c) {
    return (c == AllianceColor::BLUE) ? 'b' : 'r';
}

bool comboHeld(uint32_t buttons, uint32_t combo) {
    return (buttons & combo) == combo;
}

bool comboPressed(uint32_t now, uint32_t prev, uint32_t combo) {
    return comboHeld(now, combo) && !comboHeld(prev, combo);
}

void odomCopyZoneToActive(AllianceColor c) {
    const uint8_t ai = allianceIdx(c);
    for (size_t i = 0; i < ODOM_WP_COUNT; i++) {
        gOdomWaypoints[i] = gOdomZone1[ai][i];
    }
}

void odomSaveZoneSlot(AllianceColor c, uint8_t idx) {
    if (idx >= ODOM_WP_COUNT) return;
    const OdomWaypoint& wp = gOdomZone1[allianceIdx(c)][idx];
    char key[12];
    Preferences prefs;
    prefs.begin(ODOM_NVS_NS, false);
    snprintf(key, sizeof(key), "z1_%c_%u_x", allianceNvsChar(c), idx);
    prefs.putFloat(key, wp.x_cm);
    snprintf(key, sizeof(key), "z1_%c_%u_y", allianceNvsChar(c), idx);
    prefs.putFloat(key, wp.y_cm);
    snprintf(key, sizeof(key), "z1_%c_%u_w", allianceNvsChar(c), idx);
    prefs.putFloat(key, wp.yaw_deg);
    snprintf(key, sizeof(key), "z1_%c_%u_v", allianceNvsChar(c), idx);
    prefs.putShort(key, wp.maxspeed_rpm);
    snprintf(key, sizeof(key), "z1_%c_%u_ok", allianceNvsChar(c), idx);
    prefs.putUChar(key, 1);
    prefs.end();
}

void odomLoadZoneSlot(AllianceColor c, uint8_t idx) {
    if (idx >= ODOM_WP_COUNT) return;

    char key[12];
    Preferences prefs;
    prefs.begin(ODOM_NVS_NS, true);

    snprintf(key, sizeof(key), "z1_%c_%u_ok", allianceNvsChar(c), idx);
    uint8_t ok = prefs.getUChar(key, 0);
    bool legacy = false;
    if (ok == 0 && c == AllianceColor::RED) {
        snprintf(key, sizeof(key), "z1_%u_ok", idx);
        ok = prefs.getUChar(key, 0);
        legacy = (ok != 0);
    }
    if (ok == 0) {
        prefs.end();
        return;
    }

    OdomWaypoint& wp = gOdomZone1[allianceIdx(c)][idx];
    if (legacy) {
        snprintf(key, sizeof(key), "z1_%u_x", idx);
        wp.x_cm = prefs.getFloat(key, 0.0f);
        snprintf(key, sizeof(key), "z1_%u_y", idx);
        wp.y_cm = prefs.getFloat(key, 0.0f);
        snprintf(key, sizeof(key), "z1_%u_w", idx);
        wp.yaw_deg = prefs.getFloat(key, 0.0f);
        snprintf(key, sizeof(key), "z1_%u_v", idx);
        wp.maxspeed_rpm = prefs.getShort(key, ODOM_WP_DEFAULT_SPEED_RPM);
    } else {
        snprintf(key, sizeof(key), "z1_%c_%u_x", allianceNvsChar(c), idx);
        wp.x_cm = prefs.getFloat(key, 0.0f);
        snprintf(key, sizeof(key), "z1_%c_%u_y", allianceNvsChar(c), idx);
        wp.y_cm = prefs.getFloat(key, 0.0f);
        snprintf(key, sizeof(key), "z1_%c_%u_w", allianceNvsChar(c), idx);
        wp.yaw_deg = prefs.getFloat(key, 0.0f);
        snprintf(key, sizeof(key), "z1_%c_%u_v", allianceNvsChar(c), idx);
        wp.maxspeed_rpm = prefs.getShort(key, ODOM_WP_DEFAULT_SPEED_RPM);
    }
    wp.valid = true;
    prefs.end();
}

void odomClearZoneSlotNvs(AllianceColor c, uint8_t idx) {
    char key[12];
    Preferences prefs;
    prefs.begin(ODOM_NVS_NS, false);
    snprintf(key, sizeof(key), "z1_%c_%u_ok", allianceNvsChar(c), idx);
    prefs.remove(key);
    if (c == AllianceColor::RED) {
        snprintf(key, sizeof(key), "z1_%u_ok", idx);
        prefs.remove(key);
    }
    prefs.end();
}

void odomRequestRecord(OdomRecTarget target) {
    if (gOdomRecordPending || target == OdomRecTarget::NONE) return;
    gOdomPendingTarget = target;
    gOdomRecordPending = true;
    sendShowOdomCommand();
    Serial.printf("[OdomRecord] poll → target %u (%s)\n",
                  static_cast<uint8_t>(target), allianceLabel(gAllianceColor));
}

void odomEnterModeSave() {
    if (gOdomModeSave) return;
    gOdomModeSave = true;
    Serial.printf("[OdomRecord] mode ON (%s) — R3+L3 keluar\n", allianceLabel(gAllianceColor));
}

void odomApplySample(OdomRecTarget target, float x_cm, float y_cm, float yaw_deg) {
    const uint8_t ai = allianceIdx();
    switch (target) {
        case OdomRecTarget::ZONE1_0:
        case OdomRecTarget::ZONE1_1:
        case OdomRecTarget::ZONE1_2:
        case OdomRecTarget::ZONE1_3: {
            const uint8_t idx = static_cast<uint8_t>(target) - static_cast<uint8_t>(OdomRecTarget::ZONE1_0);
            OdomWaypoint& wp  = gOdomZone1[ai][idx];
            wp.x_cm           = x_cm;
            wp.y_cm           = y_cm;
            wp.yaw_deg        = yaw_deg;
            wp.maxspeed_rpm   = ODOM_WP_DEFAULT_SPEED_RPM;
            wp.valid          = true;
            gOdomWaypoints[idx] = wp;
            odomSaveZoneSlot(gAllianceColor, idx);
            Serial.printf("[OdomRecord] %s zone1 #%u x=%.1f y=%.1f yaw=%.1f\n",
                          allianceLabel(gAllianceColor), idx + 1, wp.x_cm, wp.y_cm, wp.yaw_deg);
            break;
        }
        case OdomRecTarget::APPROACH_0:
        case OdomRecTarget::APPROACH_1:
        case OdomRecTarget::APPROACH_2:
        case OdomRecTarget::APPROACH_3: {
            const uint8_t idx = static_cast<uint8_t>(target) - static_cast<uint8_t>(OdomRecTarget::APPROACH_0);
            forestRecordApproach(idx, x_cm, y_cm, yaw_deg);
            break;
        }
        case OdomRecTarget::FOREST_2:
            forestRecordWp(2, x_cm, y_cm);
            break;
        case OdomRecTarget::FOREST_6:
            forestRecordWp(6, x_cm, y_cm);
            break;
        case OdomRecTarget::FOREST_7:
            forestRecordWp(7, x_cm, y_cm);
            break;
        case OdomRecTarget::FOREST_11:
            forestRecordWp(11, x_cm, y_cm);
            break;
        default:
            break;
    }
}

} // anonymous namespace

void odomApplyAlliance(AllianceColor c) {
    odomCopyZoneToActive(c);
}

void odomApplyAlliance() {
    odomApplyAlliance(gAllianceColor);
}

void initOdomRec() {
    for (uint8_t ai = 0; ai < 2; ai++) {
        for (size_t i = 0; i < ODOM_WP_COUNT; i++) {
            odomLoadZoneSlot(static_cast<AllianceColor>(ai), static_cast<uint8_t>(i));
        }
    }
    odomApplyAlliance();
    Serial.printf("[OdomRecord] NVS loaded RED=%u/4 BLUE=%u/4 active=%s\n",
                  odomZone1ValidCount(AllianceColor::RED),
                  odomZone1ValidCount(AllianceColor::BLUE),
                  allianceLabel(gAllianceColor));
}

void odomRecordTick(const ControlPacket& pkt) {
    if (!pkt.connected) return;

    const uint32_t btn = pkt.buttons;

    if (comboPressed(btn, gOdomPrevButtons, BTN_EXIT_COMBO) && gOdomModeSave) {
        gOdomModeSave = false;
        Serial.println("[OdomRecord] mode OFF");
    }

    if (!gOdomModeSave && comboPressed(btn, gOdomPrevButtons, BTN_ENTER_COMBO)) {
        gOdomComboHeld   = true;
        gOdomHoldFired   = false;
        gOdomHoldStartMs = millis();
    }

    if (gOdomComboHeld && comboHeld(btn, BTN_ENTER_COMBO)) {
        if (!gOdomHoldFired && (millis() - gOdomHoldStartMs) >= ODOM_MODE_HOLD_MS) {
            gOdomHoldFired = true;
            odomEnterModeSave();
        }
    }

    if (!comboHeld(btn, BTN_ENTER_COMBO) && gOdomComboHeld) {
        gOdomComboHeld = false;
        gOdomHoldFired = false;
    }

    if (gOdomModeSave && !gOdomRecordPending) {
        for (const RecCombo& rc : REC_COMBOS) {
            if (comboPressed(btn, gOdomPrevButtons, rc.mask)) {
                odomRequestRecord(rc.target);
                break;
            }
        }
    }

    gOdomPrevButtons = btn;
}

void odomOnSampleReceived(float x_m, float y_m, float yaw_deg) {
    if (!gOdomRecordPending) return;

    const OdomRecTarget target = gOdomPendingTarget;
    gOdomRecordPending         = false;
    gOdomPendingTarget         = OdomRecTarget::NONE;

    if (!gOdomModeSave || target == OdomRecTarget::NONE) return;

    odomApplySample(target, x_m * 100.0f, y_m * 100.0f, yaw_deg);
}

void odomRecordClear(AllianceColor c) {
    const uint8_t ai = allianceIdx(c);
    for (size_t i = 0; i < ODOM_WP_COUNT; i++) {
        gOdomZone1[ai][i] = {};
        odomClearZoneSlotNvs(c, static_cast<uint8_t>(i));
    }
    if (c == gAllianceColor) {
        odomApplyAlliance(c);
    }
    gOdomRecordPending = false;
    gOdomPendingTarget = OdomRecTarget::NONE;
    Serial.printf("[OdomRecord] %s zone1 cleared\n", allianceLabel(c));
}

void odomRecordClear() {
    odomRecordClear(gAllianceColor);
}

bool odomIsModeSave() {
    return gOdomModeSave;
}

uint8_t odomZone1ValidCount(AllianceColor c) {
    uint8_t n = 0;
    const uint8_t ai = allianceIdx(c);
    for (size_t i = 0; i < ODOM_WP_COUNT; i++) {
        if (gOdomZone1[ai][i].valid) n++;
    }
    return n;
}

uint8_t odomZone1ValidCount() {
    return odomZone1ValidCount(gAllianceColor);
}

namespace {

void odomPrintAlliance(Print& out, AllianceColor c) {
    out.printf("  %s: %u/4 valid\n", allianceLabel(c), odomZone1ValidCount(c));
    const uint8_t ai = allianceIdx(c);
    for (uint8_t i = 0; i < ODOM_WP_COUNT; i++) {
        const OdomWaypoint& wp = gOdomZone1[ai][i];
        if (wp.valid) {
            out.printf("    #%u: x=%.1fcm y=%.1fcm yaw=%.1fdeg speed=%drpm\n",
                       i + 1, wp.x_cm, wp.y_cm, wp.yaw_deg, wp.maxspeed_rpm);
        } else {
            out.printf("    #%u: (belum terekam)\n", i + 1);
        }
    }
}

} // anonymous namespace

void odomRecordPrint(Print& out) {
    out.printf("OdomRecord zone1 (active=%s, mode=%s)\n",
               allianceLabel(gAllianceColor), gOdomModeSave ? "ON" : "OFF");
    odomPrintAlliance(out, AllianceColor::RED);
    odomPrintAlliance(out, AllianceColor::BLUE);
}

bool odomGoto(uint8_t slot) {
    if (slot < 1 || slot > ODOM_WP_COUNT || !gOdomWaypoints[slot - 1].valid) {
        return false;
    }

    const OdomWaypoint& wp = gOdomWaypoints[slot - 1];
    gTargetX_cm         = wp.x_cm;
    gTargetY_cm         = wp.y_cm;
    gYawTarget          = (int16_t)lroundf(wp.yaw_deg);
    gTargetSpeedRpm     = wp.maxspeed_rpm;
    gMotionWaypointMode = true;

    sendGotoCommand(gTargetX_cm, gTargetY_cm, gYawTarget, gTargetSpeedRpm);
    return true;
}
