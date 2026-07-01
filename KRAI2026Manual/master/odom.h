/*
 * =====================================================================
 * FILE    : odom.h
 * PERAN   : Record waypoint odometri (zone1, approach, forest) via joystick.
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

extern OdomWaypoint gOdomWaypoints[ODOM_WP_COUNT];
extern bool         gOdomModeSave;

void initOdomRec();
void odomRecordTick(const ControlPacket& pkt);
void odomOnSampleReceived(float x_m, float y_m, float yaw_deg);
void odomRecordClear();
bool odomIsModeSave();
void odomRecordPrint(Print& out);
uint8_t odomZone1ValidCount();
bool odomGoto(uint8_t slot);  // 1..4, set motion target dari gOdomWaypoints[]

#endif // ODOM_H
