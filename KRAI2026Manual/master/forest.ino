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

// Globals — forest dest config (API + UART)
uint8_t gForestDest1     = 0;
uint8_t gForestDest2     = 0;
bool    gForestDest1Done = false;

static uint8_t sActiveSlotMission = 0;

static void forestNotifySlave2(const char* evt) {
    slave2Serial.printf("forest evt %s\n", evt);
}

static void forestResetDest1Done() {
    gForestDest1Done = false;
    forestNotifySlave2("dest1_reset");
}

namespace {

// ponytail: semua konstanta placeholder — ukur lapangan lalu update
static constexpr float   CELL_CM           = 120.0f;
static constexpr float   ORIGIN_X_CM       = 0.0f;   // offset X asal forest dari odom (0,0)
static constexpr float   ORIGIN_Y_CM       = 0.0f;   // offset Y asal forest dari odom (0,0)
static constexpr int16_t DEFAULT_SPEED_RPM = 100;

// Titik tengah horizontal grid — dipakai untuk putuskan jalur kiri/kanan ke bottom forest
static constexpr float   GRID_CENTER_X_CM  = ORIGIN_X_CM + 1.5f * CELL_CM;

//        col0   col1   col2
//  row0:  1      2      3
//  row1:  4     [5]     6
//  row2:  7     [8]     9
//  row3: 10     11     12

//  { x_cm,                    y_cm,                    height_level,    speed_rpm,         valid }
const ForestWaypoint FOREST_WP_BLUE[13] = {
    {0*CELL_CM + ORIGIN_X_CM, 3*CELL_CM + ORIGIN_Y_CM, 2, DEFAULT_SPEED_RPM, true},  // [0]  forest 10b — via approach[3]
    {0*CELL_CM + ORIGIN_X_CM, 0*CELL_CM + ORIGIN_Y_CM, 3, DEFAULT_SPEED_RPM, true},  // [1]  col0 row0
    {1*CELL_CM + ORIGIN_X_CM, 0*CELL_CM + ORIGIN_Y_CM, 2, DEFAULT_SPEED_RPM, true},  // [2]  col1 row0
    {2*CELL_CM + ORIGIN_X_CM, 0*CELL_CM + ORIGIN_Y_CM, 3, DEFAULT_SPEED_RPM, true},  // [3]  col2 row0
    {0*CELL_CM + ORIGIN_X_CM, 1*CELL_CM + ORIGIN_Y_CM, 2, DEFAULT_SPEED_RPM, true},  // [4]  col0 row1
    {0.0f,                    0.0f,                    0, 0,                 false}, // [5]  INVALID — forest tengah
    {2*CELL_CM + ORIGIN_X_CM, 1*CELL_CM + ORIGIN_Y_CM, 4, DEFAULT_SPEED_RPM, true},  // [6]  col2 row1
    {0*CELL_CM + ORIGIN_X_CM, 2*CELL_CM + ORIGIN_Y_CM, 3, DEFAULT_SPEED_RPM, true},  // [7]  col0 row2
    {0.0f,                    0.0f,                    0, 0,                 false}, // [8]  INVALID — forest tengah
    {2*CELL_CM + ORIGIN_X_CM, 2*CELL_CM + ORIGIN_Y_CM, 3, DEFAULT_SPEED_RPM, true},  // [9]  col2 row2
    {0*CELL_CM + ORIGIN_X_CM, 3*CELL_CM + ORIGIN_Y_CM, 2, DEFAULT_SPEED_RPM, true},  // [10] forest 10a — via approach[0]
    {1*CELL_CM + ORIGIN_X_CM, 3*CELL_CM + ORIGIN_Y_CM, 3, DEFAULT_SPEED_RPM, true},  // [11] col1 row3
    {2*CELL_CM + ORIGIN_X_CM, 3*CELL_CM + ORIGIN_Y_CM, 2, DEFAULT_SPEED_RPM, true},  // [12] col2 row3
};

// Forest 10 punya dua posisi — dipilih berdasarkan gLastApproachedCol saat goForest(10):
//   approach[0] → FOREST_WP_BLUE[10] (10a), langsung tanpa lewat [2]
//   approach[3] → FOREST_WP_BLUE[0]  (10b), langsung tanpa approach tambahan
static constexpr uint8_t FOREST10A_IDX = 10;
static constexpr uint8_t FOREST10B_IDX = 0;
static constexpr int8_t  FOREST10A_AP  = 0;
static constexpr int8_t  FOREST10B_AP  = 3;

// Approach corridor points — isi pre_x/y saat dimensi lapangan sudah diukur.
//
// Layout fisik:
//     [0] kiri       │ GRID │       [1] kanan
//    (kiri atas)     │      │      (kanan atas)
//                    │      │
//     [2] bot-kiri   │ GRID │   [3] bot-kanan
//    (kiri bawah)    │      │      (kanan bawah)
//
// [0] — koridor kiri atas  : dipakai untuk forest 4,7,10; transit keluar kiri
// [1] — koridor kanan atas : dipakai untuk forest 6,9;    transit keluar kanan
// [2] — koridor kiri bawah : dipakai untuk forest 11,12 via jalur kiri
// [3] — koridor kanan bawah: dipakai untuk forest 11,12 via jalur kanan
//
// yaw_deg: heading robot setelah tiba di approach (hadap ke dalam grid)
// ponytail: yaw placeholder — ukur lapangan lalu update
const ForestColApproach FOREST_COL_APPROACH[4] = {
    {0.0f, 0.0f, false,  -90, true},  // [0] kiri      — hadap kiri
    {0.0f, 0.0f, false,   90, true},  // [1] kanan     — hadap kanan
    {0.0f, 0.0f, false,  180, true},  // [2] bot-kiri  — hadap belakang
    {0.0f, 0.0f, false,  180, true},  // [3] bot-kanan — hadap belakang
};

// -------------------------------------------------------------------------
//  GROUP ROUTING
// -------------------------------------------------------------------------

// Group 0=TOP(1-3), 1=COL0(4,7), 2=COL2(6,9), 3=BOTTOM(11,12), 4=COL0_BOT(10)
uint8_t forestGroup(uint8_t id) {
    if (id <= 3)             return 0;
    if (id == 4 || id == 7) return 1;
    if (id == 6 || id == 9) return 2;
    if (id == 10)            return 4;
    return 3;
}

// Baris forest dalam grid (0-based): row = (id-1)/3
uint8_t forestRow(uint8_t id) { return (id - 1) / 3; }

// Estimasi posisi X robot saat ini dari state terakhir (sebelum ada odometri penuh).
float getCurrentX_cm() {
    if (gLastForestId > 0) return FOREST_WP_BLUE[gLastForestId].x_cm;
    if (gLastApproachedCol == 0 || gLastApproachedCol == 2) return ORIGIN_X_CM - 0.5f * CELL_CM;
    if (gLastApproachedCol == 1 || gLastApproachedCol == 3) return ORIGIN_X_CM + 2.5f * CELL_CM;
    return GRID_CENTER_X_CM;
}

// Isi seq[0..3] dengan urutan approach index. seq berakhir di -1 (sentinel).
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

// Urutan approach menuju [2] saat exitFromForest() — berdasarkan gLastApproachedCol saja.
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
            seq[k++] = 2;  // ponytail: lastApproached belum ada, fallback langsung [2]
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

// Arm kiri = master motor Y; arm kanan = slave2 motortarget (sama pola armBox.ino).
void deployForestArm(char side, long heightEnc) {
    if (side == 'l')
        motorYSetTarget(heightEnc);
    else if (side == 'r')
        sendSlave2Command("motortarget %ld", heightEnc);
}

// -------------------------------------------------------------------------
//  STATE MACHINE INTERNAL
// -------------------------------------------------------------------------

enum class ForestPhase : uint8_t {
    COMPUTE,
    APPROACH_MOVE,
    APPROACH_YAW,
    FOREST_MOVE,
    FOREST_ARM_Y,      // turun motor Y master ke height_enc forest
    FOREST_ARM_DEPLOY, // arm kiri: tunggu master Y sampai LEVEL_5
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

// Shared state-machine tick — dipakai goForest() dan exitFromForest().
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
                const ForestWaypoint& wp = FOREST_WP_BLUE[wpIdx];

                gTargetX_cm     = wp.x_cm;
                gTargetY_cm     = wp.y_cm;
                gTargetSpeedRpm = wp.speed_rpm;
                if (gAllianceColor == AllianceColor::BLUE) gTargetY_cm = -gTargetY_cm;

                gMotionWaypointMode = true;
                sPhase = ForestPhase::FOREST_MOVE;
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

            if (!FOREST_COL_APPROACH[ap].has_pre) {
                continue;
            }

            sApX   = FOREST_COL_APPROACH[ap].pre_x_cm;
            sApY   = FOREST_COL_APPROACH[ap].pre_y_cm;
            sApYaw = FOREST_COL_APPROACH[ap].yaw_deg;

            if (sExitMode && ap == 2 && nextApInSeq == -1) {
                sApYaw = 180;  // exit finish — kondisi khusus, selalu hadap atas
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
                    sApYaw = 0;  // [3]→forest TOP saja
                }
            }
            if (gAllianceColor == AllianceColor::BLUE
                && sApYaw != 180 && sApYaw != -180) {
                sApYaw = -sApYaw;
            }

            gTargetX_cm     = sApX;
            gTargetY_cm     = sApY;
            gTargetSpeedRpm = sExitMode ? DEFAULT_SPEED_RPM
                                        : FOREST_WP_BLUE[sTarget].speed_rpm;
            if (gAllianceColor == AllianceColor::BLUE) gTargetY_cm = -gTargetY_cm;

            gMotionWaypointMode = true;
            sPhase = ForestPhase::APPROACH_MOVE;
            return true;
        }

        case ForestPhase::APPROACH_MOVE:
            if (gMotionWaypointMode) return true;
            if (FOREST_COL_APPROACH[gLastApproachedCol].has_yaw) {
                sendGotoCommand(
                    (int16_t)lroundf(sApX),
                    (int16_t)lroundf(gAllianceColor == AllianceColor::BLUE ? -sApY : sApY),
                    sApYaw
                );
                gMotionWaypointMode = true;
                sPhase = ForestPhase::APPROACH_YAW;
                return true;
            }
            sPhase = ForestPhase::COMPUTE;
            continue;

        case ForestPhase::APPROACH_YAW:
            if (gMotionWaypointMode) return true;
            gYawTarget = sApYaw;
            sPhase = ForestPhase::COMPUTE;
            continue;

        case ForestPhase::FOREST_MOVE:
            if (gMotionWaypointMode) return true;
            if (!sExitMode && sActiveForestId > 0) {
                motorYSetTarget(motorYLevelEnc(
                    FOREST_WP_BLUE[forestWpIdx(sActiveForestId)].height_level));
                sPhase = ForestPhase::FOREST_ARM_Y;
                return true;
            }
            sPhase = ForestPhase::COMPUTE;
            return false;

        case ForestPhase::FOREST_ARM_Y:
            if (motorYIsActive()) return true;
            gForestArmSide = pickForestArmSide();
            deployForestArm(gForestArmSide,
                motorYLevelEnc(FOREST_WP_BLUE[forestWpIdx(sActiveForestId)].height_level));
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

// =====================================================================
//  GLOBAL STATE — forest navigation history
// =====================================================================

int8_t gLastApproachedCol = -1;
int8_t gLastForestId      = 0;
char   gForestArmSide     = 0;

namespace {

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

} // anonymous namespace

void initForestDest() {
    Preferences prefs;
    prefs.begin(FOREST_DEST_NVS_NS, true);
    gForestDest1 = prefs.getUChar(FOREST_KEY_D1, FOREST_DEFAULT_D1);
    gForestDest2 = prefs.getUChar(FOREST_KEY_D2, FOREST_DEFAULT_D2);
    prefs.end();
    Serial.printf("[Forest] NVS dest1=%u dest2=%u\n", gForestDest1, gForestDest2);
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
    if (id < 1 || id > 12 || !FOREST_WP_BLUE[id].valid) return false;
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

// =====================================================================
//  API (navigate)
// =====================================================================

bool goForest(uint8_t id) {
    if (id == 0 || id > 12 || !FOREST_WP_BLUE[id].valid) return false;

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
