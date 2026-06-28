/*
 * =====================================================================
 * FILE    : waypoint.h
 * PERAN   : PID waypoint controller — gerak ke (x, y, yaw).
 *
 * BOARD   : ESP32-S3 (Slave1 Motion)
 * =====================================================================
 */

#ifndef WAYPOINT_H
#define WAYPOINT_H

#include "config.h"

// Default max RPM untuk waypoint
constexpr float WP_DEFAULT_MAX_RPM = 300.0f;

enum class WaypointState : uint8_t { IDLE, RUNNING, REACHED };

// NVS init/save
void initWaypointPid();
void saveWaypointPid();

// Control
void setWaypoint(float x_m, float y_m, float yaw_deg);
void cancelWaypoint();
void waypointTick(float x_cm, float y_cm, float yaw_deg, float maxRpm);

// Status
bool          isWaypointActive();
bool          isWaypointReached();
WaypointState getWaypointState();

// Tunable params (NVS-stored)
extern float wpKpXY;         // RPM per meter error
extern float wpTolPos_m;     // tolerance posisi (meter)
extern float wpTolYaw_deg;   // tolerance yaw (derajat)
extern float wpMaxSpeed;     // max RPM for waypoint movement

// Target saat ini (readonly — set via setWaypoint)
extern float wpTargetX_m;
extern float wpTargetY_m;
extern float wpTargetYaw_deg;

#endif // WAYPOINT_H
