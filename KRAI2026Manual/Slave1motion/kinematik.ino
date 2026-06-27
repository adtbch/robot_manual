/*
 * =====================================================================
 * FILE    : kinematik.ino
 * PERAN   : Mecanum drive kinematics.
 *
 * BOARD   : ESP32-S3 (Slave1 Motion)
 * =====================================================================
 */

#include "kinematik.h"

// =====================================================================
//  INTERNAL HELPERS
// =====================================================================

namespace {

void skalaKecepatanPWM(int m1, int m2, int m3, int m4) {
    int maxInput = max(max(abs(m1), abs(m2)), max(abs(m3), abs(m4)));
    if (maxInput > PWM_MAX) {
        m1 = (m1 * PWM_MAX) / maxInput;
        m2 = (m2 * PWM_MAX) / maxInput;
        m3 = (m3 * PWM_MAX) / maxInput;
        m4 = (m4 * PWM_MAX) / maxInput;
    }
    pwmMotor(0, m1);
    pwmMotor(1, m2);
    pwmMotor(2, m3);
    pwmMotor(3, m4);
}

} // anonymous namespace

// =====================================================================
//  ROBOT-CENTRIC
// =====================================================================

void driveRobotCentric(int vx, int vy, int vtheta) {
    int motorFR = vx + vy - vtheta;
    int motorFL = vx - vy + vtheta;
    int motorBR = vx - vy - vtheta;
    int motorBL = vx + vy + vtheta;
    skalaKecepatanPWM(motorFR, motorFL, motorBR, motorBL);
}

// =====================================================================
//  FIELD-CENTRIC
// =====================================================================

void driveFieldCentric(int vx, int vy, int vtheta) {
    float yawRad = getYaw() * (PI / 180.0f);
    float c = cosf(yawRad);
    float s = sinf(yawRad);

    int vxRot = roundf(vx * c - vy * s);
    int vyRot = roundf(vx * s + vy * c);

    driveRobotCentric(vxRot, vyRot, vtheta);
}

// =====================================================================
//  FIELD-CENTRIC WITH YAW CORRECTION
// =====================================================================

void driveFieldCentricWithYawCorrection(int vx, int vy, int yawTarget) {
    static uint32_t lastCallMs = 0;
    uint32_t nowMs = millis();
    float dt = (lastCallMs == 0) ? 0.04f : (nowMs - lastCallMs) * 0.001f;
    lastCallMs = nowMs;

    float currentYaw = getYaw();
    int correctionYaw = pidComputeYaw(pidKinematicYaw, (float)yawTarget, currentYaw, dt);
    driveFieldCentricRpm(vx, vy, correctionYaw);
}

// =====================================================================
//  ROBOT-CENTRIC RPM
// =====================================================================

void driveRobotCentricRpm(int vx, int vy, int vtheta) {
    int motorFR = vx + vy - vtheta;
    int motorFL = vx - vy + vtheta;
    int motorBR = vx - vy - vtheta;
    int motorBL = vx + vy + vtheta;

    int maxVal = max(max(abs(motorFR), abs(motorFL)), max(abs(motorBR), abs(motorBL)));
    if (maxVal > (int)RPM_MAX) {
        motorFR = motorFR * RPM_MAX / maxVal;
        motorFL = motorFL * RPM_MAX / maxVal;
        motorBR = motorBR * RPM_MAX / maxVal;
        motorBL = motorBL * RPM_MAX / maxVal;
    }

    rpmMotor(motorFR, motorFL, motorBR, motorBL);
}

// =====================================================================
//  FIELD-CENTRIC RPM
// =====================================================================

void driveFieldCentricRpm(int vx, int vy, int vtheta) {
    float yawRad = getYaw() * (PI / 180.0f);
    float c = cosf(yawRad);
    float s = sinf(yawRad);

    int vxRot = roundf(vx * c - vy * s);
    int vyRot = roundf(vx * s + vy * c);

    driveRobotCentricRpm(vxRot, vyRot, vtheta);
}
