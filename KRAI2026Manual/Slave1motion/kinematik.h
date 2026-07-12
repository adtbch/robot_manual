/*
 * =====================================================================
 * FILE    : kinematik.h
 * PERAN   : Mecanum drive kinematics (robot-centric, field-centric).
 *
 * BOARD   : ESP32-S3 (Slave1 Motion)
 * =====================================================================
 */

#ifndef KINEMATIK_H
#define KINEMATIK_H

#include "config.h"

// =====================================================================
//  SHARED FUNCTION DECLARATIONS
// =====================================================================
void driveRobotCentric(int vx, int vy, int vtheta);
void driveFieldCentric(int vx, int vy, int vtheta);
void driveFieldCentricWithYawCorrection(int vx, int vy, int yawTarget);
void driveFieldCentricWithYawCorrectionPWM(int vx, int vy, int yawTarget);
void driveRobotCentricRpm(int vx, int vy, int vtheta);
void driveFieldCentricRpm(int vx, int vy, int vtheta);

#endif // KINEMATIK_H
