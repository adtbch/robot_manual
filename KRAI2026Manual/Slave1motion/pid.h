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
void pidSetGains(int motorIdx, float kp, float ki, float kd, float kf);
void pidResetOne(int motorIdx);
void pidReloadFromNVS();
void pidLoadFromNVS(int motorIdx, float &kp, float &ki, float &kd, float &kf);
void pidSaveToNVS(int motorIdx, float kp, float ki, float kd, float kf);
void initYawPid();
void saveYawPid();
void showYawPid();

// RPM motor control
void rpmMotor(int rpm1, int rpm2, int rpm3, int rpm4);

// Extern yaw PID
extern PIDState pidKinematicYaw;
extern PIDState pidStates[MOTOR_COUNT];

#endif // PID_H
