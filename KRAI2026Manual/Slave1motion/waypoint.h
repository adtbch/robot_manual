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


// NVS init/save
void initWaypointPid();
void saveWaypointPid();

// Control
void cancelWaypoint();
void startWaypoint(float x_cm, float y_cm, float yaw_deg, float maxRpm);
void startWaypointCombo(float x1_cm, float y1_cm, float yaw1_deg,
                        float x2_cm, float y2_cm, float yaw2_deg, float maxRpm);
void waypointTick(float x_m, float y_m, float yaw_deg, float maxSpeed);
void wpNotifyReachedToMaster();  // UART ke master, max 20x per applyWaypointTarget

// Status
bool          isWaypointActive();
bool          isWaypointReached();
WaypointState getWaypointState();
bool          isWaypointComboActive();  // true saat combo P1→P2 jalan
uint8_t       getWaypointComboStep();   // 1 atau 2 saat combo; 0 jika tidak combo

// Tunable params (NVS-stored)
extern float wpKpXY;         // RPM per meter error
extern float wpTolPos_m;     // tolerance posisi (meter)
extern float wpTolYaw_deg;   // tolerance yaw (derajat)
extern float wpMaxSpeed;     // max RPM for waypoint movement

// Target saat ini (readonly — set via startWaypoint*)
extern float wpTargetX_m;
extern float wpTargetY_m;
extern float wpTargetYaw_deg;

#endif // WAYPOINT_H
