#include "robot_config.h"

void setHoming() {
    if (limitSwitchAxisX == LOW) {
        MotorX.homing = true;
    }
    if (limitSwitchAxisY == LOW) {
        MotorZ.homing = true;
    }

    if (!MotorX.homing) {
        pwmMotor(0, 20);
    } else {
        pwmMotor(0, 0);
    }

    if (!MotorZ.homing) {
        pwmMotor(1, 20);
    } else {
        pwmMotor(1, 0);
    }
}
