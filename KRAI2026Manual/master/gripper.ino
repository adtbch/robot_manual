/*
 * =====================================================================
 * FILE    : gripper.ino
 * PERAN   : Servo homing + motor homing (blocking, setup saja).
 *
 * BOARD   : ESP32-S3 (Master)
 * =====================================================================
 */

#include "servo.h"
#include "proximity.h"
#include "motor.h"
#include "encoder.h"
#include "limit_switch.h"

// =====================================================================
//  CONFIG
// =====================================================================

constexpr int HOMING_PWM = 400;

// =====================================================================
//  HOMING — blocking, dipanggil dari setup() atau serial
// =====================================================================

void setServoHoming() {
    setServoAngle('t', 70);
    setServoAngle('b', 0);
    setServoAngle('d', 0);
    gGripperState = IDLE;
}

void setMotorHoming() {
    motorXStop();
    motorYStop();

    pwmMotor('y', HOMING_PWM);
    while (!readLimitSwitch(LIMIT_Y_BAWAH)) {
        delay(2);
    }
    pwmMotor('y', 0);
    resetEncoderCount('y');
    gripperMotorYSetLevel(0);
    
    pwmMotor('x', HOMING_PWM);
    while (!readLimitSwitch(LIMIT_X_MUNDUR)) {
        delay(2);
    }
    pwmMotor('x', 0);
    resetEncoderCount('x');
}

void setHomingAll() {
    setMotorHoming();
    setServoHoming();
}

void setArmHoming() {
    motorKSetDirection(-1);
    sendSlave2Command("motor k -255");
}
