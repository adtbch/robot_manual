#include "robot_config.h"

MotorState motorX = {false};
MotorState motorZ = {false};
bool readyArm = false;

bool setHoming() {
    // Check limit switches dan update status homing
    if (digitalRead(limitSwitchAxisX) == LOW) {
        motorX.homing = true;
        pwmMotor(0, 0); // Stop motor X
    }
    if (digitalRead(limitSwitchAxisY) == LOW) {
        motorZ.homing = true;
        pwmMotor(1, 0); // Stop motor Z
    }
    
    // Gerakkan motor yang belum homing
    if (!motorX.homing) {
        pwmMotor(0, -200);
    }

    if (!motorZ.homing) {
        pwmMotor(1, -200);
    }

    // Return true hanya jika kedua motor sudah selesai homing
    return (motorX.homing && motorZ.homing);
}

// Fungsi untuk gerakkan arm ke posisi tengah menggunakan encoder feedback
bool moveToCenter() {
    long currentX = getEncoderCount(0);  // Baca encoder motor X
    long currentZ = getEncoderCount(1);  // Baca encoder motor Z
    
    long errorX = CENTER_POSITION_X - currentX;
    long errorZ = CENTER_POSITION_Z - currentZ;
    
    bool xAtCenter = (abs(errorX) < CENTER_POSITION_TOLERANCE);
    bool zAtCenter = (abs(errorZ) < CENTER_POSITION_TOLERANCE);
    
    // Kontrol motor X
    if (!xAtCenter) {
        if (errorX > 0) {
            pwmMotor(0, CENTER_MOVE_SPEED);  // Maju
        } else {
            pwmMotor(0, -CENTER_MOVE_SPEED); // Mundur
        }
    } else {
        pwmMotor(0, 0);  // Stop motor X
    }
    
    // Kontrol motor Z
    if (!zAtCenter) {
        if (errorZ > 0) {
            pwmMotor(1, CENTER_MOVE_SPEED);  // Maju
        } else {
            pwmMotor(1, -CENTER_MOVE_SPEED); // Mundur
        }
    } else {
        pwmMotor(1, 0);  // Stop motor Z
    }
    
    // Return true jika kedua axis sudah di posisi tengah
    return (xAtCenter && zAtCenter);
}