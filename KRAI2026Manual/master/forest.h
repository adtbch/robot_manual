/*
 * =====================================================================
 * FILE    : forest.h
 * PERAN   : Forest waypoint lookup table — 12 posisi forest (4 baris × 3 kolom).
 *
 * LAYOUT:
 *        col0   col1   col2
 *   row0:  1      2      3
 *   row1:  4     [5]     6     ← posisi 5 invalid (forest tengah)
 *   row2:  7     [8]     9     ← posisi 8 invalid (forest tengah)
 *   row3: 10     11     12
 *
 * Koordinat absolut (odom frame) — kalibrasi via mode record odom.
 * gAllianceColor: pickForestArmSide(); forestNavigate() negasi Y untuk
 * forest 1, 3, 12, dan 10b saja saat BLUE.
 *
 * BOARD   : ESP32-S3 (Master)
 * =====================================================================
 */

#ifndef FOREST_H
#define FOREST_H

#include "config.h"

// =====================================================================
//  TIPE DATA
// =====================================================================

struct ForestWaypoint {
    float   x_cm;
    float   y_cm;
    uint8_t height_level;  // indeks 0..5 → gMotorYLevelEnc[]
    int16_t speed_rpm;
    bool    valid;
};

struct ForestColApproach {
    float   pre_x_cm;
    float   pre_y_cm;
    bool    has_pre;
    int16_t yaw_deg;
    bool    has_yaw;
};

// =====================================================================
//  SHARED STATE
// =====================================================================

extern ForestWaypoint    gForestWp[13];
extern ForestColApproach gForestApproach[4];

extern AllianceColor gAllianceColor;
extern int8_t        gLastApproachedCol;
extern int8_t        gLastForestId;
extern char          gForestArmSide;

extern uint8_t gForestDest1;
extern uint8_t gForestDest2;
extern bool    gForestDest1Done;

// =====================================================================
//  API
// =====================================================================

void initForestDest();
void forestSetDestinations(uint8_t d1, uint8_t d2);
bool forestGotoSlot(uint8_t slot);
bool forestIsDest1Done();
void forestTriggerExit();
void forestControlTick(const ControlPacket& pkt);

void forestRecordApproach(uint8_t idx, float x_cm, float y_cm, float yaw_deg);
void forestRecordWp(uint8_t forestId, float x_cm, float y_cm);
void forestRecordClear();
void forestRecordPrint(Print& out);

bool goForest(uint8_t id);
bool forestTick();
void cancelForestGoto();
bool exitFromForest();

#endif // FOREST_H
