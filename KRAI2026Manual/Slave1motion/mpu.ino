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
#include "i2c_bus.h"
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps612.h"
#include <Preferences.h>
#include <Wire.h>

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

constexpr uint32_t kWatchdogTimeoutMs = 2500;
constexpr uint32_t kRecoverCooldownMs = 3000;
constexpr uint32_t kRecoverySettleMs  = 15;   // jeda antar step recovery (non-blocking)
constexpr float kGyroMoveThresholdDps = 1.5f;
constexpr uint32_t kDriftFreezeHoldMs = 500;
constexpr uint32_t kPollMinIntervalMs = 10;  // throttle: max ~100Hz baca FIFO, tidak bergantung INT

// Tiga state recovery DMP — dieksekusi satu step per iterasi loop() agar tidak blocking.
// ponytail: state machine sederhana, tidak perlu FreeRTOS task.
enum class DmpRecovery : uint8_t { IDLE, BUS_RECOVER, WAIT_SETTLE, DMP_ENABLE };
DmpRecovery dmpRecoveryStep = DmpRecovery::IDLE;
uint32_t    dmpRecoveryStepMs = 0;

// Core 3.x: setelah NACK, slave bisa menahan SDA low / driver master stuck.
// Bebaskan bus secara manual: clock SCL 9x agar slave melepas SDA, kirim STOP,
// lalu re-init Wire. Tanpa ini, reset DMP via I2C percuma karena bus masih nyangkut.
void i2cBusRecover() {
    Wire.end();

    pinMode(I2C_SDA, INPUT_PULLUP);
    pinMode(I2C_SCL, OUTPUT_OPEN_DRAIN);
    digitalWrite(I2C_SCL, HIGH);
    delayMicroseconds(5);

    for (int i = 0; i < 9; i++) {
        digitalWrite(I2C_SCL, LOW);
        delayMicroseconds(5);
        digitalWrite(I2C_SCL, HIGH);
        delayMicroseconds(5);
    }

    // STOP condition: SDA transisi low->high saat SCL high
    pinMode(I2C_SDA, OUTPUT_OPEN_DRAIN);
    digitalWrite(I2C_SDA, LOW);
    delayMicroseconds(5);
    digitalWrite(I2C_SCL, HIGH);
    delayMicroseconds(5);
    digitalWrite(I2C_SDA, HIGH);
    delayMicroseconds(5);

    Wire.begin(I2C_SDA, I2C_SCL);
    Wire.setClock(100000);
    Wire.setTimeOut(20);
}

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
    float res = fmod(angle - minAngle, range);
    if (res < 0) res += range;
    return res + minAngle;
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

bool setupMPU() {
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

float getYawError(float target_deg) {
    return normalizeAngle(target_deg - getYaw(), -180.0f, 180.0f);
}

float getPitch() {
    float p = ypr[1] * (180.0f / M_PI);
    if (isnan(p) || isinf(p)) return 0.0f;
    return p;
}

float getRoll() {
    float r = ypr[2] * (180.0f / M_PI);
    if (isnan(r) || isinf(r)) return 0.0f;
    return r;
}

float getSlopeDeg() {
    return getRoll();
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
    static uint32_t lastReadMs = 0;
    static bool wasMoving = false;
    static uint32_t lastMoveMs = 0;

    uint32_t now = millis();

    // Non-blocking recovery: satu step per iterasi loop() → max blocking per step
    // = 3 I2C ops × 20ms timeout = 60ms, bukan 400ms sekaligus.
    if (dmpRecoveryStep != DmpRecovery::IDLE) {
        if (!I2cBus::acquire(I2cBus::Owner::MPU)) return;
        switch (dmpRecoveryStep) {
            case DmpRecovery::BUS_RECOVER:
                i2cBusRecover();
                mpu.resetFIFO();
                mpu.setDMPEnabled(false);
                dmpRecoveryStep = DmpRecovery::WAIT_SETTLE;
                dmpRecoveryStepMs = millis();
                break;
            case DmpRecovery::WAIT_SETTLE:
                if (millis() - dmpRecoveryStepMs >= kRecoverySettleMs) {
                    dmpRecoveryStep = DmpRecovery::DMP_ENABLE;
                }
                break;
            case DmpRecovery::DMP_ENABLE:
                mpu.setDMPEnabled(true);
                mpu.getIntStatus();
                mpuInterrupt = false;
                dmpRecoveryStep = DmpRecovery::IDLE;
                lastRecoverMs = millis();
                break;
            default: break;
        }
        I2cBus::release(I2cBus::Owner::MPU);
        return;  // skip baca FIFO selama recovery berlangsung
    }

    // Baca FIFO saat INT aktif ATAU throttle interval lewat (fallback tanpa kabel INT).
    bool shouldRead = mpuInterrupt || (now - lastReadMs >= kPollMinIntervalMs);
    if (!shouldRead) return;

    if (!I2cBus::acquire(I2cBus::Owner::MPU)) return;

    mpuInterrupt = false;
    lastReadMs = now;

    // dmpGetCurrentFIFOPacket() sudah overflow-proof & mengembalikan packet TERBARU.
    // Panggil SEKALI (if), bukan while — while bisa memblokir s/d 11ms menunggu packet
    // parsial & menembak I2C berulang (memperparah NACK saat motor bising).
    if (mpu.dmpGetCurrentFIFOPacket(fifoBuffer)) {
        mpu.dmpGetQuaternion(&q, fifoBuffer);
        mpu.dmpGetGravity(&gravity, &q);
        mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);

        float rawYaw = ypr[0] * (180.0f / M_PI);
        yaw = normalizeAngle(rawYaw - yawOffset, -180.0f, 180.0f);
        if (isnan(yaw) || isinf(yaw)) yaw = 0.0f;

        hasSeenPacket = true;
        lastPacketMs = millis();

        int16_t gyroSample[3];
        mpu.dmpGetGyro(gyroSample, fifoBuffer);
        bool isMoving = (fabsf(gyroSample[2] / 131.0f) > kGyroMoveThresholdDps);

        if (isMoving) {
            yawActive = yaw;
            lastMoveMs = millis();
            wasMoving = true;
        } else if (wasMoving) {
            yawActive = yaw;
            wasMoving = false;
        }

        if (isMoving || (millis() - lastMoveMs <= kDriftFreezeHoldMs)) {
            yaw = yawActive;
        }
    }

    // // Watchdog: tidak ada packet valid → mulai recovery non-blocking
    // if (hasSeenPacket &&
    //     (now - lastPacketMs > kWatchdogTimeoutMs) &&
    //     (now - lastRecoverMs > kRecoverCooldownMs)) {
    //     Serial.printf("[MPU WATCHDOG] No DMP packet for >%lums — starting non-blocking recovery\n",
    //                   (unsigned long)kWatchdogTimeoutMs);
    //     dmpRecoveryStep = DmpRecovery::BUS_RECOVER;
    //     hasSeenPacket = false;
    // }

    I2cBus::release(I2cBus::Owner::MPU);
}
