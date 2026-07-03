/*
 * =====================================================================
 * FILE    : forest.ino
 * PERAN   : Forest waypoint lookup — 12 posisi forest, 2 hilang (5, 8).
 *
 * BOARD   : ESP32-S3 (Master)
 * =====================================================================
 */

#include "forest.h"
#include "motor.h"
#include "serial.h"
#include <Preferences.h>

uint8_t gForestDest1     = 0;
uint8_t gForestDest2     = 0;
bool    gForestDest1Done = false;

ForestWaypoint    gForestWp[13];
ForestColApproach gForestApproach[4];
ForestAllianceRec gForestRec[2] = {};

static uint8_t sActiveSlotMission = 0;

static void forestNotifySlave2(const char* evt) {
    slave2Serial.printf("forest evt %s\n", evt);
}

static void forestResetDest1Done() {
    gForestDest1Done = false;
    forestNotifySlave2("dest1_reset");
}

namespace {

constexpr float   CELL_CM           = 120.0f;
constexpr float   ORIGIN_X_CM       = 0.0f;
constexpr float   ORIGIN_Y_CM       = 0.0f;
constexpr int16_t DEFAULT_SPEED_RPM = 100;
constexpr float   GRID_CENTER_X_CM  = ORIGIN_X_CM + 1.5f * CELL_CM;

constexpr uint8_t FOREST10A_IDX = 10;
constexpr uint8_t FOREST10B_IDX = 0;
constexpr int8_t  FOREST10A_AP  = 0;
constexpr int8_t  FOREST10B_AP  = 3;

} // namespace (constants only — reopened below)

constexpr uint8_t FOREST_REC_IDS[] = {2, 6, 7, 11};
constexpr const char* FOREST_WP_NVS_NS = "forest_wp";
constexpr float       FOREST_CELL_CM   = 120.0f;  // jarak grid; samakan dengan CELL_CM di namespace

// x/y non-record dihitung di forestDeriveWpFromRecorded() dari anchor 2,6,7,11
const ForestWaypoint FOREST_WP_DEFAULT[13] = {
    {0.0f, 0.0f, 2, DEFAULT_SPEED_RPM, true},  // [0]  10b — derived dari F7
    {0.0f, 0.0f, 3, DEFAULT_SPEED_RPM, true},  // [1]  F1 — derived dari F2
    {120.0f, 0.0f, 2, DEFAULT_SPEED_RPM, true},  // [2]  record
    {0.0f, 0.0f, 3, DEFAULT_SPEED_RPM, true},  // [3]  F3 — derived dari F2
    {0.0f, 0.0f, 2, DEFAULT_SPEED_RPM, true},  // [4]  F4 — derived dari F7
    {0.0f, 0.0f, 0, 0,                 false}, // [5]  invalid
    {240.0f, 120.0f, 4, DEFAULT_SPEED_RPM, true},  // [6]  record
    {0.0f, 240.0f, 3, DEFAULT_SPEED_RPM, true},  // [7]  record
    {0.0f, 0.0f, 0, 0,                 false}, // [8]  invalid
    {0.0f, 0.0f, 3, DEFAULT_SPEED_RPM, true},  // [9]  F9 — derived dari F6
    {0.0f, 0.0f, 2, DEFAULT_SPEED_RPM, true},  // [10] 10a — derived dari F7
    {120.0f, 360.0f, 3, DEFAULT_SPEED_RPM, true},  // [11] record
    {0.0f, 0.0f, 2, DEFAULT_SPEED_RPM, true},  // [12] F12 — derived dari F11
};

const ForestColApproach FOREST_AP_DEFAULT[4] = {
    {0.0f, 0.0f, false,  -90, true},
    {0.0f, 0.0f, false,   90, true},
    {0.0f, 0.0f, false,  180, true},
    {0.0f, 0.0f, false,  180, true},
};

static char forestAllianceChar(AllianceColor c) {
    return (c == AllianceColor::BLUE) ? 'b' : 'r';
}

static int8_t forestRecAnchorIdx(uint8_t forestId) {
    for (uint8_t k = 0; k < sizeof(FOREST_REC_IDS); k++) {
        if (FOREST_REC_IDS[k] == forestId) return static_cast<int8_t>(k);
    }
    return -1;
}

static void forestInitRecDefaults(ForestAllianceRec& rec) {
    for (int i = 0; i < 4; i++) {
        rec.approach[i] = FOREST_AP_DEFAULT[i];
        rec.approach[i].has_pre = false;
    }
    for (uint8_t k = 0; k < 4; k++) {
        const uint8_t id = FOREST_REC_IDS[k];
        rec.anchorX[k]  = FOREST_WP_DEFAULT[id].x_cm;
        rec.anchorY[k]  = FOREST_WP_DEFAULT[id].y_cm;
        rec.anchorOk[k] = false;
    }
}

void forestInitDefaults() {
    for (int i = 0; i < 13; i++) {
        gForestWp[i] = FOREST_WP_DEFAULT[i];
    }
    for (int i = 0; i < 4; i++) {
        gForestApproach[i] = FOREST_AP_DEFAULT[i];
    }
}

void forestSaveApproachNvs(AllianceColor c, uint8_t idx) {
    if (idx >= 4) return;
    const ForestColApproach& ap = gForestRec[allianceIdx(c)].approach[idx];
    char key[12];
    Preferences prefs;
    prefs.begin(FOREST_WP_NVS_NS, false);
    snprintf(key, sizeof(key), "ap_%c_%u_x", forestAllianceChar(c), idx);
    prefs.putFloat(key, ap.pre_x_cm);
    snprintf(key, sizeof(key), "ap_%c_%u_y", forestAllianceChar(c), idx);
    prefs.putFloat(key, ap.pre_y_cm);
    snprintf(key, sizeof(key), "ap_%c_%u_w", forestAllianceChar(c), idx);
    prefs.putShort(key, ap.yaw_deg);
    snprintf(key, sizeof(key), "ap_%c_%u_ok", forestAllianceChar(c), idx);
    prefs.putUChar(key, ap.has_pre ? 1 : 0);
    prefs.end();
}

void forestSaveForestWpNvs(AllianceColor c, uint8_t forestId) {
    const int8_t k = forestRecAnchorIdx(forestId);
    if (k < 0) return;
    const ForestAllianceRec& rec = gForestRec[allianceIdx(c)];
    char key[12];
    Preferences prefs;
    prefs.begin(FOREST_WP_NVS_NS, false);
    snprintf(key, sizeof(key), "f_%c_%u_x", forestAllianceChar(c), forestId);
    prefs.putFloat(key, rec.anchorX[k]);
    snprintf(key, sizeof(key), "f_%c_%u_y", forestAllianceChar(c), forestId);
    prefs.putFloat(key, rec.anchorY[k]);
    snprintf(key, sizeof(key), "f_%c_%u_ok", forestAllianceChar(c), forestId);
    prefs.putUChar(key, 1);
    prefs.end();
}

void forestLoadRecNvs(AllianceColor c) {
    forestInitRecDefaults(gForestRec[allianceIdx(c)]);
    Preferences prefs;
    prefs.begin(FOREST_WP_NVS_NS, true);
    const char ac = forestAllianceChar(c);
    ForestAllianceRec& rec = gForestRec[allianceIdx(c)];

    for (uint8_t i = 0; i < 4; i++) {
        char key[12];
        snprintf(key, sizeof(key), "ap_%c_%u_ok", ac, i);
        uint8_t ok = prefs.getUChar(key, 0);
        bool legacy = false;
        if (ok == 0 && c == AllianceColor::RED) {
            snprintf(key, sizeof(key), "ap%u_ok", i);
            ok = prefs.getUChar(key, 0);
            legacy = (ok != 0);
        }
        if (ok == 0) continue;

        ForestColApproach& ap = rec.approach[i];
        if (legacy) {
            snprintf(key, sizeof(key), "ap%u_x", i);
            ap.pre_x_cm = prefs.getFloat(key, 0.0f);
            snprintf(key, sizeof(key), "ap%u_y", i);
            ap.pre_y_cm = prefs.getFloat(key, 0.0f);
            snprintf(key, sizeof(key), "ap%u_w", i);
            ap.yaw_deg = prefs.getShort(key, FOREST_AP_DEFAULT[i].yaw_deg);
        } else {
            snprintf(key, sizeof(key), "ap_%c_%u_x", ac, i);
            ap.pre_x_cm = prefs.getFloat(key, 0.0f);
            snprintf(key, sizeof(key), "ap_%c_%u_y", ac, i);
            ap.pre_y_cm = prefs.getFloat(key, 0.0f);
            snprintf(key, sizeof(key), "ap_%c_%u_w", ac, i);
            ap.yaw_deg = prefs.getShort(key, FOREST_AP_DEFAULT[i].yaw_deg);
        }
        ap.has_pre = true;
        ap.has_yaw = FOREST_AP_DEFAULT[i].has_yaw;
    }

    for (uint8_t k = 0; k < 4; k++) {
        const uint8_t id = FOREST_REC_IDS[k];
        char key[12];
        snprintf(key, sizeof(key), "f_%c_%u_ok", ac, id);
        uint8_t ok = prefs.getUChar(key, 0);
        bool legacy = false;
        if (ok == 0 && c == AllianceColor::RED) {
            snprintf(key, sizeof(key), "f%u_ok", id);
            ok = prefs.getUChar(key, 0);
            legacy = (ok != 0);
        }
        if (ok == 0) continue;

        if (legacy) {
            snprintf(key, sizeof(key), "f%u_x", id);
            rec.anchorX[k] = prefs.getFloat(key, rec.anchorX[k]);
            snprintf(key, sizeof(key), "f%u_y", id);
            rec.anchorY[k] = prefs.getFloat(key, rec.anchorY[k]);
        } else {
            snprintf(key, sizeof(key), "f_%c_%u_x", ac, id);
            rec.anchorX[k] = prefs.getFloat(key, rec.anchorX[k]);
            snprintf(key, sizeof(key), "f_%c_%u_y", ac, id);
            rec.anchorY[k] = prefs.getFloat(key, rec.anchorY[k]);
        }
        rec.anchorOk[k] = true;
    }
    prefs.end();
}

// Anchor: F2,F6,F7,F11 (record). Sisanya offset ±FOREST_CELL_CM; BLUE negasi arah offset.
void forestDeriveWpFromRecorded(AllianceColor alliance) {
    const ForestWaypoint& f2  = gForestWp[2];
    const ForestWaypoint& f6  = gForestWp[6];
    const ForestWaypoint& f7  = gForestWp[7];
    const ForestWaypoint& f11 = gForestWp[11];
    if (!f2.valid || !f6.valid || !f7.valid || !f11.valid) return;

    const float cell = FOREST_CELL_CM;
    const float s    = (alliance == AllianceColor::BLUE) ? -1.0f : 1.0f;

    gForestWp[1].x_cm  = f2.x_cm;
    gForestWp[1].y_cm  = f2.y_cm + s * cell;
    gForestWp[3].x_cm  = f2.x_cm;
    gForestWp[3].y_cm  = f2.y_cm - s * cell;
    gForestWp[4].x_cm  = f7.x_cm - s * cell;
    gForestWp[4].y_cm  = f7.y_cm;
    gForestWp[9].x_cm  = f6.x_cm + s * cell;
    gForestWp[9].y_cm  = f6.y_cm;
    gForestWp[10].x_cm = f7.x_cm + s * cell;
    gForestWp[10].y_cm = f7.y_cm;
    gForestWp[0].x_cm  = f11.x_cm;
    gForestWp[0].y_cm  = f11.y_cm + s * cell;
    gForestWp[12].x_cm = f11.x_cm;
    gForestWp[12].y_cm = f11.y_cm - s * cell;
}

void forestApplyAlliance(AllianceColor c) {
    forestInitDefaults();
    const ForestAllianceRec& rec = gForestRec[allianceIdx(c)];
    for (uint8_t i = 0; i < 4; i++) {
        if (rec.approach[i].has_pre) {
            gForestApproach[i] = rec.approach[i];
            gForestApproach[i].has_yaw = FOREST_AP_DEFAULT[i].has_yaw;
        }
    }
    for (uint8_t k = 0; k < 4; k++) {
        if (!rec.anchorOk[k]) continue;
        const uint8_t id = FOREST_REC_IDS[k];
        gForestWp[id].x_cm = rec.anchorX[k];
        gForestWp[id].y_cm = rec.anchorY[k];
    }
    forestDeriveWpFromRecorded(c);
}

void forestClearApproachNvs(AllianceColor c, uint8_t idx) {
    char key[12];
    Preferences prefs;
    prefs.begin(FOREST_WP_NVS_NS, false);
    snprintf(key, sizeof(key), "ap_%c_%u_ok", forestAllianceChar(c), idx);
    prefs.remove(key);
    if (c == AllianceColor::RED) {
        snprintf(key, sizeof(key), "ap%u_ok", idx);
        prefs.remove(key);
    }
    prefs.end();
}

void forestClearForestWpNvs(AllianceColor c, uint8_t forestId) {
    char key[12];
    Preferences prefs;
    prefs.begin(FOREST_WP_NVS_NS, false);
    snprintf(key, sizeof(key), "f_%c_%u_ok", forestAllianceChar(c), forestId);
    prefs.remove(key);
    if (c == AllianceColor::RED) {
        snprintf(key, sizeof(key), "f%u_ok", forestId);
        prefs.remove(key);
    }
    prefs.end();
}

namespace {

uint8_t forestGroup(uint8_t id) {
    if (id <= 3)             return 0;
    if (id == 4 || id == 7) return 1;
    if (id == 6 || id == 9) return 2;
    if (id == 10)            return 4;
    return 3;
}

uint8_t forestRow(uint8_t id) { return (id - 1) / 3; }

float getCurrentX_cm() {
    if (gLastForestId > 0) return gForestWp[gLastForestId].x_cm;
    if (gLastApproachedCol == 0 || gLastApproachedCol == 2) return ORIGIN_X_CM - 0.5f * CELL_CM;
    if (gLastApproachedCol == 1 || gLastApproachedCol == 3) return ORIGIN_X_CM + 2.5f * CELL_CM;
    return GRID_CENTER_X_CM;
}

void computeApproachSeq(uint8_t tid, int8_t seq[4]) {
    seq[0] = seq[1] = seq[2] = seq[3] = -1;
    int k = 0;

    const uint8_t tg = forestGroup(tid);
    if (tg == 0) return;

    const uint8_t lg = (gLastForestId > 0) ? forestGroup(gLastForestId) : 255u;
    if (lg == tg) return;
    if (tg == 4 && (lg == 1 || lg == 3)) return;
    if (tg == 3 && lg == 4)             return;

    const bool isLD = (lg == 1 || lg == 4);
    const bool isRD = (lg == 2);

    const bool atAP0 = (gLastForestId == 0 && gLastApproachedCol == 0);
    const bool atAP1 = (gLastForestId == 0 && gLastApproachedCol == 1);
    const bool atAP2 = (gLastForestId == 0 && gLastApproachedCol == 2);
    const bool atAP3 = (gLastForestId == 0 && gLastApproachedCol == 3);

    if (tg == 1) {
        if (isRD) {
            if (forestRow(gLastForestId) <= 1) { seq[k++] = 1; seq[k++] = 0; }
            else                               { seq[k++] = 3; seq[k++] = 2; }
        } else if (!atAP0 && !isLD) seq[k++] = 0;
    } else if (tg == 2) {
        if (isLD) {
            if (forestRow(gLastForestId) <= 1) { seq[k++] = 0; seq[k++] = 1; }
            else                               { seq[k++] = 2; seq[k++] = 3; }
        } else if (!atAP1 && !isRD) seq[k++] = 1;
    } else if (tg == 4) {
        if (atAP0) return;
        if (atAP3) return;
        if (isRD)       seq[k++] = FOREST10B_AP;
        else if (!isLD) seq[k++] = FOREST10A_AP;
    } else {
        if (getCurrentX_cm() < GRID_CENTER_X_CM) {
            if (isRD) { seq[k++] = 1; seq[k++] = 0; }
            else if (!atAP0 && !atAP2 && !isLD) seq[k++] = 0;
            if (!atAP2) seq[k++] = 2;
        } else {
            if (isLD) { seq[k++] = 0; seq[k++] = 1; }
            else if (!atAP1 && !atAP3 && !isRD) seq[k++] = 1;
            if (!atAP3) seq[k++] = 3;
        }
    }
}

void computeExitSeq(int8_t seq[4]) {
    seq[0] = seq[1] = seq[2] = seq[3] = -1;
    int k = 0;

    switch (gLastApproachedCol) {
        case 1:
            seq[k++] = 3;
            seq[k++] = 2;
            break;
        case 0:
        case 3:
        case 2:
            seq[k++] = 2;
            break;
        default:
            seq[k++] = 2;
            break;
    }
}

uint8_t forestWpIdx(uint8_t forestId) {
    if (forestId == 10 && gLastApproachedCol == FOREST10B_AP) return FOREST10B_IDX;
    if (forestId == 10) return FOREST10A_IDX;
    return forestId;
}

char pickForestArmSide() {
    char side = 'l';
    const int8_t ap = gLastApproachedCol;
    if (ap == 0 || ap == 2) {
        if (gAllianceColor == AllianceColor::BLUE) {
            side = slave2ProxR() ? 'l' : 'r';
        } else {
            side = slave2ProxL() ? 'r' : 'l';
        }
    } else if (ap == 1 || ap == 3) {
        if (gAllianceColor == AllianceColor::BLUE) {
            side = slave2ProxL() ? 'r' : 'l';
        } else {
            side = slave2ProxR() ? 'l' : 'r';
        }
    }
    return side;
}

void deployForestArm(char side, long heightEnc) {
    if (side == 'l')
        motorYSetTarget(heightEnc);
    else if (side == 'r')
        sendSlave2Command("motortarget %ld", heightEnc);
}

enum class ForestPhase : uint8_t {
    COMPUTE,
    APPROACH_MOVE,
    APPROACH_YAW,
    FOREST_MOVE,
    FOREST_ARM_Y,
    FOREST_ARM_DEPLOY,
};

static constexpr uint8_t FOREST_TARGET_EXIT = 255;

uint8_t      sTarget          = 0;
uint8_t      sActiveForestId  = 0;
int8_t       sSeq[4]          = {-1, -1, -1, -1};
uint8_t      sSeqStep         = 0;
ForestPhase  sPhase           = ForestPhase::COMPUTE;
float        sApX             = 0.0f;
float        sApY             = 0.0f;
int16_t      sApYaw           = 0;
bool         sExitMode        = false;

void forestOnGotoMissionComplete() {
    if (sExitMode) return;
    if (sActiveSlotMission == 1) {
        gForestDest1Done = true;
        forestNotifySlave2("dest1_done");
    }
    sActiveSlotMission = 0;
}

bool forestNavigate() {
    while (true) {
        switch (sPhase) {

        case ForestPhase::COMPUTE: {
            const int8_t ap = sSeq[sSeqStep];
            if (ap == -1) {
                if (sExitMode) {
                    gLastForestId      = 0;
                    gLastApproachedCol = 2;
                    sExitMode          = false;
                    sTarget            = 0;
                    sPhase             = ForestPhase::COMPUTE;
                    return false;
                }

                const uint8_t forestId = sTarget;
                gLastForestId     = forestId;
                sActiveForestId   = forestId;
                sTarget           = 0;

                const uint8_t wpIdx = forestWpIdx(forestId);
                const ForestWaypoint& wp = gForestWp[wpIdx];

                gTargetX_cm         = wp.x_cm;
                gTargetY_cm         = wp.y_cm;
                gTargetSpeedRpm     = wp.speed_rpm;
                gMotionWaypointMode = true;
                sendGotoCommand(gTargetX_cm, gTargetY_cm, gYawTarget, gTargetSpeedRpm);
                sPhase              = ForestPhase::FOREST_MOVE;
                return true;
            }

            const int8_t prevApInSeq = (sSeqStep > 0) ? sSeq[sSeqStep - 1] : -1;
            int8_t nextApInSeq = -1;
            if (sSeqStep + 1 < 4) {
                const int8_t n = sSeq[sSeqStep + 1];
                if (n >= 0 && n <= 3) nextApInSeq = n;
            }
            const bool alreadyHere = (sSeqStep == 0 && ap == gLastApproachedCol);

            sSeqStep++;
            gLastApproachedCol = ap;

            if (!gForestApproach[ap].has_pre) {
                continue;
            }

            sApX   = gForestApproach[ap].pre_x_cm;
            sApY   = gForestApproach[ap].pre_y_cm;
            sApYaw = gForestApproach[ap].yaw_deg;

            if (sExitMode && ap == 2 && nextApInSeq == -1) {
                sApYaw = 180;
            } else {
                if (alreadyHere || ap == nextApInSeq) {
                    sApYaw = 0;
                } else if (ap == 2 && nextApInSeq == 3) {
                    sApYaw = 180;
                } else if (ap == 3 && nextApInSeq == 2) {
                    sApYaw = 180;
                } else if (ap == 3 && nextApInSeq == -1 && prevApInSeq == 2) {
                    sApYaw = 90;
                } else if (ap == 2 && nextApInSeq == -1 && prevApInSeq == 3) {
                    sApYaw = -90;
                } else if (ap == 3 && nextApInSeq == -1 && prevApInSeq != 2
                           && forestGroup(sTarget) == 0) {
                    sApYaw = 0;
                }
            }

            gTargetX_cm         = sApX;
            gTargetY_cm         = sApY;
            gTargetSpeedRpm     = sExitMode ? DEFAULT_SPEED_RPM
                                            : gForestWp[sTarget].speed_rpm;
            gMotionWaypointMode = true;
            sendGotoCommand(gTargetX_cm, gTargetY_cm, gYawTarget, gTargetSpeedRpm);
            sPhase              = ForestPhase::APPROACH_MOVE;
            return true;
        }

        case ForestPhase::APPROACH_MOVE:
            if (gMotionWaypointMode) return true;
            if (gForestApproach[gLastApproachedCol].has_yaw) {
                gTargetX_cm = sApX;
                gTargetY_cm = sApY;
                gMotionWaypointMode = true;
                sendGotoCommand(gTargetX_cm, gTargetY_cm, gYawTarget, gTargetSpeedRpm);
                sPhase = ForestPhase::APPROACH_YAW;
                return true;
            }
            sPhase = ForestPhase::COMPUTE;
            continue;

        case ForestPhase::APPROACH_YAW:
            if (gMotionWaypointMode) return true;
            gYawTarget = sApYaw;
            sendGotoCommand(gTargetX_cm, gTargetY_cm, gYawTarget, gTargetSpeedRpm);
            sPhase = ForestPhase::COMPUTE;
            continue;

        case ForestPhase::FOREST_MOVE:
            if (gMotionWaypointMode) return true;
            if (!sExitMode && sActiveForestId > 0) {
                motorYSetTarget(motorYLevelEnc(
                    gForestWp[forestWpIdx(sActiveForestId)].height_level));
                sPhase = ForestPhase::FOREST_ARM_Y;
                return true;
            }
            sPhase = ForestPhase::COMPUTE;
            return false;

        case ForestPhase::FOREST_ARM_Y:
            if (motorYIsActive()) return true;
            gForestArmSide = pickForestArmSide();
            deployForestArm(gForestArmSide,
                motorYLevelEnc(gForestWp[forestWpIdx(sActiveForestId)].height_level));
            if (gForestArmSide == 'l') {
                sPhase = ForestPhase::FOREST_ARM_DEPLOY;
                return true;
            }
            forestOnGotoMissionComplete();
            sActiveForestId = 0;
            sPhase          = ForestPhase::COMPUTE;
            return false;

        case ForestPhase::FOREST_ARM_DEPLOY:
            if (motorYIsActive()) return true;
            forestOnGotoMissionComplete();
            sActiveForestId = 0;
            sPhase          = ForestPhase::COMPUTE;
            return false;
        }
    }
}

} // namespace

constexpr const char* FOREST_DEST_NVS_NS = "forest_cfg";
constexpr const char* FOREST_KEY_D1      = "dest1";
constexpr const char* FOREST_KEY_D2      = "dest2";
constexpr uint8_t     FOREST_DEFAULT_D1  = 2;
constexpr uint8_t     FOREST_DEFAULT_D2  = 4;

void forestSaveDestNvs() {
    Preferences prefs;
    prefs.begin(FOREST_DEST_NVS_NS, false);
    prefs.putUChar(FOREST_KEY_D1, gForestDest1);
    prefs.putUChar(FOREST_KEY_D2, gForestDest2);
    prefs.end();
}

int8_t gLastApproachedCol = -1;
int8_t gLastForestId      = 0;
char   gForestArmSide     = 0;

void initForestDest() {
    forestLoadRecNvs(AllianceColor::BLUE);
    forestLoadRecNvs(AllianceColor::RED);
    forestApplyAlliance(gAllianceColor);

    Preferences prefs;
    prefs.begin(FOREST_DEST_NVS_NS, true);
    gForestDest1 = prefs.getUChar(FOREST_KEY_D1, FOREST_DEFAULT_D1);
    gForestDest2 = prefs.getUChar(FOREST_KEY_D2, FOREST_DEFAULT_D2);
    prefs.end();
    Serial.printf("[Forest] NVS dest1=%u dest2=%u active=%s\n",
                  gForestDest1, gForestDest2, allianceLabel(gAllianceColor));
}

void forestSetDestinations(uint8_t d1, uint8_t d2) {
    gForestDest1 = d1;
    gForestDest2 = d2;
    forestResetDest1Done();
    forestSaveDestNvs();
    Serial.printf("[Forest] cfg dest1=%u dest2=%u\n", gForestDest1, gForestDest2);
}

bool forestIsDest1Done() {
    return gForestDest1Done;
}

bool forestGotoSlot(uint8_t slot) {
    if (slot != 1 && slot != 2) return false;
    if (slot == 2 && !gForestDest1Done) return false;
    const uint8_t id = (slot == 1) ? gForestDest1 : gForestDest2;
    if (id < 1 || id > 12 || !gForestWp[id].valid) return false;
    sActiveSlotMission = slot;
    if (slot == 1) {
        forestResetDest1Done();
    }
    Serial.printf("[Forest] goto slot %u → forest %u\n", slot, id);
    return goForest(id);
}

void forestTriggerExit() {
    Serial.println("[Forest] exit");
    exitFromForest();
}

void forestRecordApproach(uint8_t idx, float x_cm, float y_cm, float yaw_deg) {
    if (idx >= 4) return;
    ForestAllianceRec& rec = gForestRec[allianceIdx()];
    rec.approach[idx].pre_x_cm = x_cm;
    rec.approach[idx].pre_y_cm = y_cm;
    rec.approach[idx].yaw_deg  = (int16_t)lroundf(yaw_deg);
    rec.approach[idx].has_pre  = true;
    rec.approach[idx].has_yaw  = FOREST_AP_DEFAULT[idx].has_yaw;
    forestSaveApproachNvs(gAllianceColor, idx);
    forestApplyAlliance(gAllianceColor);
    Serial.printf("[ForestRec] %s approach[%u] x=%.1f y=%.1f yaw=%d\n",
                  allianceLabel(gAllianceColor), idx, x_cm, y_cm, rec.approach[idx].yaw_deg);
}

void forestRecordWp(uint8_t forestId, float x_cm, float y_cm) {
    const int8_t k = forestRecAnchorIdx(forestId);
    if (k < 0 || forestId >= 13 || !FOREST_WP_DEFAULT[forestId].valid) return;
    ForestAllianceRec& rec = gForestRec[allianceIdx()];
    rec.anchorX[k]  = x_cm;
    rec.anchorY[k]  = y_cm;
    rec.anchorOk[k] = true;
    forestSaveForestWpNvs(gAllianceColor, forestId);
    forestApplyAlliance(gAllianceColor);
    Serial.printf("[ForestRec] %s forest %u x=%.1f y=%.1f\n",
                  allianceLabel(gAllianceColor), forestId, x_cm, y_cm);
}

void forestRecordClear(AllianceColor c) {
    forestInitRecDefaults(gForestRec[allianceIdx(c)]);
    for (uint8_t i = 0; i < 4; i++) {
        forestClearApproachNvs(c, i);
    }
    for (uint8_t k = 0; k < 4; k++) {
        forestClearForestWpNvs(c, FOREST_REC_IDS[k]);
    }
    if (c == gAllianceColor) {
        forestApplyAlliance(c);
    }
    Serial.printf("[ForestRec] %s cleared approach + forest 2/6/7/11\n", allianceLabel(c));
}

void forestRecordClear() {
    forestRecordClear(gAllianceColor);
}

void forestRecordPrint(Print& out) {
    out.printf("ForestRec (active=%s)\n", allianceLabel(gAllianceColor));
    for (uint8_t ai = 0; ai < 2; ai++) {
        const AllianceColor c = static_cast<AllianceColor>(ai);
        const ForestAllianceRec& rec = gForestRec[ai];
        out.printf("  %s approach:\n", allianceLabel(c));
        for (uint8_t i = 0; i < 4; i++) {
            const ForestColApproach& ap = rec.approach[i];
            if (ap.has_pre) {
                out.printf("    [%u] x=%.1f y=%.1f yaw=%d\n",
                           i, ap.pre_x_cm, ap.pre_y_cm, ap.yaw_deg);
            } else {
                out.printf("    [%u] (belum terekam)\n", i);
            }
        }
        out.printf("  %s wp:\n", allianceLabel(c));
        for (uint8_t k = 0; k < 4; k++) {
            const uint8_t id = FOREST_REC_IDS[k];
            if (rec.anchorOk[k]) {
                out.printf("    forest %u: x=%.1f y=%.1f\n",
                           id, rec.anchorX[k], rec.anchorY[k]);
            } else {
                out.printf("    forest %u: (belum terekam)\n", id);
            }
        }
    }
}

bool goForest(uint8_t id) {
    if (id == 0 || id > 12 || !gForestWp[id].valid) return false;

    const bool newMission = sExitMode || (sActiveForestId != id && sTarget != id);
    if (newMission) {
        sExitMode         = false;
        sTarget           = id;
        sActiveForestId   = id;
        sSeqStep          = 0;
        sPhase            = ForestPhase::COMPUTE;
        gForestArmSide    = 0;
        computeApproachSeq(id, sSeq);
    }

    return forestNavigate();
}

bool forestTick() {
    if (sExitMode || sTarget == FOREST_TARGET_EXIT) return exitFromForest();
    if (sActiveForestId > 0 || sTarget > 0) return goForest(sActiveForestId > 0 ? sActiveForestId : sTarget);
    return false;
}

bool exitFromForest() {
    if (sTarget != FOREST_TARGET_EXIT) {
        sExitMode = true;
        sTarget   = FOREST_TARGET_EXIT;
        sSeqStep  = 0;
        sPhase    = ForestPhase::COMPUTE;
        computeExitSeq(sSeq);
    }

    return forestNavigate();
}

void cancelForestGoto() {
    slave1Serial.println("wp cancel");
    gMotionWaypointMode = false;
    sTarget           = 0;
    sActiveForestId   = 0;
    sExitMode         = false;
    sPhase            = ForestPhase::COMPUTE;
    sSeqStep          = 0;
    gForestArmSide    = 0;
    sActiveSlotMission = 0;
    forestResetDest1Done();
}
