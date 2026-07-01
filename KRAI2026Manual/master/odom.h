/*
 * =====================================================================
 * FILE    : odom.h
 * PERAN   : Record waypoint odometri (zone1, approach, forest) via joystick.
 *           Zone1 disimpan terpisah RED / BLUE.
 *
 * BOARD   : ESP32-S3 (Master)
 * =====================================================================
 */

#ifndef ODOM_H
#define ODOM_H

#include "config.h"

constexpr size_t  ODOM_WP_COUNT = 4;
constexpr int16_t ODOM_WP_DEFAULT_SPEED_RPM = 70;
constexpr uint32_t ODOM_MODE_HOLD_MS = 5000;

enum class OdomRecTarget : uint8_t {
    NONE = 0,
    ZONE1_0,
    ZONE1_1,
    ZONE1_2,
    ZONE1_3,
    APPROACH_0,
    APPROACH_1,
    APPROACH_2,
    APPROACH_3,
    FOREST_2,
    FOREST_6,
    FOREST_7,
    FOREST_11,
};

struct OdomWaypoint {
    float   x_cm;
    float   y_cm;
    float   yaw_deg;
    int16_t maxspeed_rpm;
    bool    valid;
};

extern OdomWaypoint gOdomZone1[2][ODOM_WP_COUNT];
extern OdomWaypoint gOdomWaypoints[ODOM_WP_COUNT];
extern bool         gOdomModeSave;

void initOdomRec();
void odomApplyAlliance();
void odomApplyAlliance(AllianceColor c);
void odomRecordTick(const ControlPacket& pkt);
void odomOnSampleReceived(float x_m, float y_m, float yaw_deg);
void odomRecordClear();
void odomRecordClear(AllianceColor c);
bool odomIsModeSave();
void odomRecordPrint(Print& out);
uint8_t odomZone1ValidCount();
uint8_t odomZone1ValidCount(AllianceColor c);
bool odomGoto(uint8_t slot);

#endif // ODOM_H
