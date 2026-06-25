/*
 * =====================================================================
 * FILE    : mpu.ino
 * PERAN   : MPU6050 IMU — Yaw/Gyro Reading (DMP MotionApps v6.12)
 *
 * LIBRARY : MPU6050v2 (Jeff Rowberg i2cdevlib)
 *
 * BOARD   : ESP32-S3 (Slave1 Motion)
 * =====================================================================
 */

#include "mpu.h"
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps612.h"
#include <Preferences.h>

// =====================================================================
//  STATE
// =====================================================================

namespace {

MPU6050 mpu;

constexpr const char* GYRO_NVS_NS = "gyro_cal";

float yawOffset = 0.0f;
float yaw = 0.0f;
float yawActive = 0.0f;

bool mpuReady = false;
bool dmpReady = false;
uint8_t devStatus;
uint16_t packetSize;
uint8_t fifoBuffer[64];

Quaternion q;
VectorFloat gravity;
float ypr[3];

volatile bool mpuInterrupt = false;

void IRAM_ATTR dmpDataReady() {
    mpuInterrupt = true;
}

} // anonymous namespace

// =====================================================================
//  NORMALIZE ANGLE
// =====================================================================

namespace {

float normalizeAngle(float angle, float minAngle, float maxAngle) {
    float range = maxAngle - minAngle;
    while (angle < minAngle) angle += range;
    while (angle >= maxAngle) angle -= range;
    return angle;
}

// =====================================================================
//  NVS CALIBRATION
// =====================================================================

void calibApplyDefaults() {
    mpu.setXAccelOffset(-4810);
    mpu.setYAccelOffset(7218);
    mpu.setZAccelOffset(11146);
    mpu.setXGyroOffset(-238);
    mpu.setYGyroOffset(136);
    mpu.setZGyroOffset(146);
    Serial.println("Applied default test-chip offsets");
}

void calibSaveToNVS() {
    Preferences prefs;
    prefs.begin(GYRO_NVS_NS, false);
    prefs.putInt("ax", mpu.getXAccelOffset());
    prefs.putInt("ay", mpu.getYAccelOffset());
    prefs.putInt("az", mpu.getZAccelOffset());
    prefs.putInt("gx", mpu.getXGyroOffset());
    prefs.putInt("gy", mpu.getYGyroOffset());
    prefs.putInt("gz", mpu.getZGyroOffset());
    prefs.putBool("cal", true);
    prefs.end();
    Serial.println("Calibration saved to NVS");
}

bool calibLoadFromNVS() {
    Preferences prefs;
    prefs.begin(GYRO_NVS_NS, true);
    bool found = prefs.getBool("cal", false);
    if (found) {
        mpu.setXAccelOffset(prefs.getInt("ax", 0));
        mpu.setYAccelOffset(prefs.getInt("ay", 0));
        mpu.setZAccelOffset(prefs.getInt("az", 0));
        mpu.setXGyroOffset(prefs.getInt("gx", 0));
        mpu.setYGyroOffset(prefs.getInt("gy", 0));
        mpu.setZGyroOffset(prefs.getInt("gz", 0));
    }
    prefs.end();
    return found;
}

} // anonymous namespace

void calibClearNVS() {
    Preferences prefs;
    prefs.begin(GYRO_NVS_NS, false);
    prefs.clear();
    prefs.end();
    Serial.println("Calibration NVS cleared.");
}

// =====================================================================
//  MPU INIT
// =====================================================================

namespace {

bool mpuInitCommon() {
    Serial.println(F("Initializing MPU6050 (DMP)..."));
    mpu.initialize();
    pinMode(MPU_INTERRUPT_PIN, INPUT);

    mpuReady = true;
    Serial.println(F("MPU6050 connection successful"));

    if (calibLoadFromNVS()) {
        Serial.println("Loaded calibration offsets from NVS:");
    } else {
        Serial.println("No calibration in NVS — using defaults:");
        calibApplyDefaults();
    }

    Serial.printf("  Offsets: Ax=%d Ay=%d Az=%d Gx=%d Gy=%d Gz=%d\n",
                  mpu.getXAccelOffset(), mpu.getYAccelOffset(), mpu.getZAccelOffset(),
                  mpu.getXGyroOffset(), mpu.getYGyroOffset(), mpu.getZGyroOffset());

    Serial.println(F("Initializing DMP..."));
    devStatus = mpu.dmpInitialize();

    if (devStatus == 0) {
        mpu.CalibrateAccel(25);
        mpu.CalibrateGyro(25);
        Serial.println(F("Active offsets after calibration:"));
        mpu.PrintActiveOffsets();

        Serial.println(F("Enabling DMP..."));
        mpu.setDMPEnabled(true);
        mpu.setDLPFMode(3);
        mpu.setRate(4);

        attachInterrupt(digitalPinToInterrupt(MPU_INTERRUPT_PIN), dmpDataReady, RISING);
        mpu.getIntStatus();

        dmpReady = true;
        packetSize = mpu.dmpGetFIFOPacketSize();
    } else {
        Serial.printf("DMP Initialization failed (code %d)\n", devStatus);
        dmpReady = false;
        return false;
    }

    yawOffset = 0.0f;
    yaw = 0.0f;
    Serial.printf("DMP Ready. yawOffset = %.2f\n", yawOffset);
    return true;
}

} // anonymous namespace

bool setupMPU() {
    return mpuInitCommon();
}

// =====================================================================
//  CALIBRATION
// =====================================================================

void calibrateGyro() {
    Serial.println("Calibrating MPU6050... KEEP ROBOT COMPLETELY STILL!");

    mpu.CalibrateAccel(25);
    mpu.CalibrateGyro(25);

    Serial.println("Calibration complete. Active offsets:");
    mpu.PrintActiveOffsets();

    calibSaveToNVS();

    yawOffset = 0.0f;
    unsigned long startMs = millis();
    while (millis() - startMs < 2000) {
        if (mpu.dmpGetCurrentFIFOPacket(fifoBuffer)) {
            mpu.dmpGetQuaternion(&q, fifoBuffer);
            mpu.dmpGetGravity(&gravity, &q);
            mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
            yawOffset = ypr[0] * (180.0f / M_PI);
        }
    }
    yaw = 0.0f;
    Serial.printf("Re-calibrated. New Reference Yaw: %.2f deg\n", yawOffset);
}

void calibrateGyroHot() {
    Serial.println("Hot recalibration: stop all motors first!");
    delay(500);
    calibrateGyro();
}

// =====================================================================
//  YAW GETTERS/SETTERS
// =====================================================================

float getYaw() {
    if (isnan(yaw) || isinf(yaw)) return 0.0f;
    return yaw;
}

void setYawReference(float targetYaw) {
    yawOffset = normalizeAngle(yaw + yawOffset - targetYaw, -180.0f, 180.0f);
    yaw = targetYaw;
    yawActive = targetYaw;
}

void snapYaw() {
    if (yaw > -20.0f && yaw < 20.0f) {
        setYawReference(0.0f);
    } else if (yaw > 70.0f && yaw < 110.0f) {
        setYawReference(90.0f);
    } else if (yaw > -110.0f && yaw < -70.0f) {
        setYawReference(-90.0f);
    } else if (yaw < -160.0f || yaw > 160.0f) {
        setYawReference(180.0f);
    }
}

// =====================================================================
//  UPDATE YAW — Pure DMP 6-Axis
// =====================================================================

void updateYaw() {
    if (!mpuReady || !dmpReady) return;

    static bool hasSeenPacket = false;
    static uint32_t lastPacketMs = 0;
    static uint32_t lastRecoverMs = 0;

    if (mpu.dmpGetCurrentFIFOPacket(fifoBuffer)) {
        mpu.dmpGetQuaternion(&q, fifoBuffer);
        mpu.dmpGetGravity(&gravity, &q);
        mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);

        float rawYaw = ypr[0] * (180.0f / M_PI);
        yaw = normalizeAngle(rawYaw - yawOffset, -180.0f, 180.0f);
        if (isnan(yaw) || isinf(yaw)) yaw = 0.0f;

        hasSeenPacket = true;
        lastPacketMs = millis();

        // Drift freeze
        static bool wasMoving = false;
        static uint32_t lastMoveMs = 0;

        int16_t gzRaw = mpu.getRotationZ();
        bool isMoving = (fabsf(gzRaw / 131.0f) > 0.3f);

        if (isMoving) {
            yawActive = yaw;
            lastMoveMs = millis();
            wasMoving = true;
        } else if (wasMoving) {
            yawActive = yaw;
            wasMoving = false;
        }

        if (isMoving || (millis() - lastMoveMs <= 500)) {
            yaw = yawActive;
        }
    } else {
        if (!hasSeenPacket) return;
        uint32_t now = millis();
        if ((now - lastPacketMs > 2000) && (now - lastRecoverMs > 2000)) {
            Serial.println("[MPU WATCHDOG] No DMP packet for >2s — resetting FIFO & DMP");
            mpu.resetFIFO();
            mpu.setDMPEnabled(false);
            delay(5);
            mpu.setDMPEnabled(true);
            mpuInterrupt = false;
            hasSeenPacket = false;
            lastRecoverMs = millis();
        }
    }
}
