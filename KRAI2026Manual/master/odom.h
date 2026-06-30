/*
 * =====================================================================
 * FILE    : odom.h
 * PERAN   : Record 4 waypoint odometri dari slave1 (mode_save joystick).
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

struct OdomWaypoint {
    float   x_cm;
    float   y_cm;
    float   yaw_deg;
    int16_t maxspeed_rpm;
};

extern OdomWaypoint gOdomWaypoints[ODOM_WP_COUNT];
extern uint8_t      gOdomWpFilled;
extern bool         gOdomModeSave;

void odomRecordTick(const ControlPacket& pkt);
void odomOnSampleReceived(float x_m, float y_m, float yaw_deg);
void odomRecordClear();
bool odomIsModeSave();
void odomRecordPrint(Print& out);
bool odomGoto(uint8_t slot);  // 1..4, set motion target dari gOdomWaypoints[]

#endif // ODOM_H
