/*
 * =====================================================================
 * FILE    : odom.h
 * PERAN   : Record waypoint odometri dari slave1 (trigger joystick).
 *
 * BOARD   : ESP32-S3 (Master)
 * =====================================================================
 */

#ifndef ODOM_H
#define ODOM_H

#include "config.h"

struct OdomSample {
    float x_m;
    float y_m;
    float yaw_deg;
};

void odomRecordTick(const ControlPacket& pkt);
void odomOnSampleReceived(float x_m, float y_m, float yaw_deg);
void odomRecordClear();
size_t odomRecordCount();
void odomRecordPrint(Print& out);

#endif // ODOM_H
