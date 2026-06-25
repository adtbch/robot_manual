/*
 * =====================================================================
 * FILE    : pid.h
 * PERAN   : PID controller (velocity RPM feedback).
 *
 * BOARD   : ESP32-S3 (Slave1 Motion)
 * =====================================================================
 */

#ifndef PID_H
#define PID_H

#include "config.h"

// =====================================================================
//  SHARED FUNCTION DECLARATIONS
// =====================================================================
void pidControllerInit();
int pidCompute(PIDState &pid, float target, float current, float dt);
int pidCompute(int motorIdx, float targetRPM, float dt);
int pidComputeYaw(PIDState &pid, float target, float current, float dt);
void pidSetGains(int motorIdx, float kp, float ki, float kd);
void pidResetOne(int motorIdx);
void pidReloadFromNVS();
void pidLoadFromNVS(int motorIdx, float &kp, float &ki, float &kd);
void pidSaveToNVS(int motorIdx, float kp, float ki, float kd);
void initYawPid();
void saveYawPid();
void showYawPid();

// RPM motor control
void rpmMotor(int rpm1, int rpm2, int rpm3, int rpm4);
void rpmMotorControl(int targetRPM0, int targetRPM1, int targetRPM2, int targetRPM3);

// Extern yaw PID
extern PIDState pidKinematicYaw;

#endif // PID_H
