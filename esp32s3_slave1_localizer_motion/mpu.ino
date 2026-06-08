// ============================================================
// MPU6050 IMU - Yaw/Gyro Reading (DMP MotionApps v6.12)
// ============================================================
// Library: MPU6050v2 (Jeff Rowberg i2cdevlib)
// Menggunakan DMP (Digital Motion Processor) internal chip
// untuk estimasi Yaw 6-Axis (Gyro + Accel) secara pure.
// ============================================================

#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps612.h"
#include <Preferences.h>

static MPU6050 mpu;

// NVS namespace for calibration data
static constexpr const char* GYRO_NVS_NS = "gyro_cal";

// Yaw offset (heading reference dari posisi awal robot)
static float yawOffset = 0.0f;

// Yaw angle in degrees (relatif dari posisi awal robot)
static float yaw = 0.0f;

// Flag: true after mpu.setup succeeds
static bool mpuReady = false;

// DMP control/status vars
static bool dmpReady = false;
static uint8_t devStatus;      // status device (0 = success, !0 = error)
static uint16_t packetSize;    // ukuran packet FIFO DMP
static uint8_t fifoBuffer[64]; // buffer FIFO

// Orientation vars
static Quaternion q;           // [w, x, y, z]
static VectorFloat gravity;    // [x, y, z] gravity
static float ypr[3];           // [yaw, pitch, roll]

// Interrupt flag
static volatile bool mpuInterrupt = false;

void IRAM_ATTR dmpDataReady() {
    mpuInterrupt = true;
}

// ============================================================
// Default calibration values (sesuai offset chip Anda)
// ============================================================

static void calibApplyDefaults() {
    mpu.setXAccelOffset(-4810);
    mpu.setYAccelOffset(7218);
    mpu.setZAccelOffset(11146);
    mpu.setXGyroOffset(-238);
    mpu.setYGyroOffset(136);
    mpu.setZGyroOffset(146);

    Serial.println("Applied default test-chip offsets");
}

// ============================================================
// NVS save/load calibration
// ============================================================

static void calibSaveToNVS() {
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

static bool calibLoadFromNVS() {
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

static void calibClearNVS() {
    Preferences prefs;
    prefs.begin(GYRO_NVS_NS, false);
    prefs.clear();
    prefs.end();
    Serial.println("Calibration NVS cleared.");
}

// ============================================================
// Fungsi Normalize Angle (Wrapping)
// Di-overload dengan float agar presisi desimal yaw tidak hilang
// ============================================================
float normalizeAngle(float angle, float minAngle, float maxAngle) {
    float range = maxAngle - minAngle;
    while (angle < minAngle) {
        angle += range;
    }
    while (angle >= maxAngle) {
        angle -= range;
    }
    return angle;
}

// ============================================================
// Shared MPU init + konvergensi
// ============================================================

static bool mpuInitCommon() {
    Serial.println(F("Initializing MPU6050 (DMP)..."));
    mpu.initialize();
    pinMode(INTERRUPT_PIN, INPUT);

    mpuReady = true;
    Serial.println(F("MPU6050 connection successful"));

    // Muat offset dari NVS atau default
    if (calibLoadFromNVS()) {
        Serial.println("Loaded calibration offsets from NVS:");
    } else {
        Serial.println("No calibration in NVS — using defaults:");
        calibApplyDefaults();
    }
    
    Serial.printf("  Offsets: Ax=%d Ay=%d Az=%d Gx=%d Gy=%d Gz=%d\n",
                  mpu.getXAccelOffset(), mpu.getYAccelOffset(), mpu.getZAccelOffset(),
                  mpu.getXGyroOffset(), mpu.getYGyroOffset(), mpu.getZGyroOffset());

    // Inisialisasi DMP
    Serial.println(F("Initializing DMP..."));
    devStatus = mpu.dmpInitialize();

    if (devStatus == 0) {
        // Auto-kalibrasi internal untuk fine-tuning offset
        mpu.CalibrateAccel(15);
        mpu.CalibrateGyro(15);
        Serial.println(F("Active offsets after calibration:"));
        mpu.PrintActiveOffsets();

        Serial.println(F("Enabling DMP..."));
        mpu.setDMPEnabled(true);

        // Pasang hardware interrupt
        attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN), dmpDataReady, RISING);
        
        dmpReady = true;
        packetSize = mpu.dmpGetFIFOPacketSize();
    } else {
        Serial.printf("DMP Initialization failed (code %d)\n", devStatus);
        dmpReady = false;
        return false;
    }

    // Capture reference awal (tunggu 2 detik agar dmp stabil)
    yawOffset = 0.0f;
    unsigned long startMs = millis();
    while (millis() - startMs < 2000) {
        if (mpu.dmpGetCurrentFIFOPacket(fifoBuffer)) {
            mpu.dmpGetQuaternion(&q, fifoBuffer);
            mpu.dmpGetGravity(&gravity, &q);
            mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
            yawOffset = ypr[0] * (180.0f / M_PI);
        }
        delay(10);
    }
    
    yaw = 0.0f;
    Serial.printf("DMP Ready. Reference Yaw: %.2f deg\n", yawOffset);
    return true;
}

bool setupMPUWithMagnetic() {
    delay(150);
    return mpuInitCommon();
}

bool setupMPUGyro() {
    delay(150);
    return mpuInitCommon();
}

bool setupMPU() {
    return mpuInitCommon();
}

// ============================================================
// Calibration (hanya via serial CALIB_GYRO)
// ============================================================

void calibrateGyro() {
    Serial.println("Calibrating MPU6050... KEEP ROBOT COMPLETELY STILL!");
    
    mpu.CalibrateAccel(15);
    mpu.CalibrateGyro(15);
    
    Serial.println("Calibration complete. Active offsets:");
    mpu.PrintActiveOffsets();

    calibSaveToNVS();

    // Re-capture yaw reference (tunggu 2 detik)
    yawOffset = 0.0f;
    unsigned long startMs = millis();
    while (millis() - startMs < 2000) {
        if (mpu.dmpGetCurrentFIFOPacket(fifoBuffer)) {
            mpu.dmpGetQuaternion(&q, fifoBuffer);
            mpu.dmpGetGravity(&gravity, &q);
            mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
            yawOffset = ypr[0] * (180.0f / M_PI);
        }
        delay(10);
    }
    yaw = 0.0f;
    Serial.printf("Re-calibrated. New Reference Yaw: %.2f deg\n", yawOffset);
}

void calibrateGyroHot() {
    Serial.println("Hot recalibration: stop all motors first!");
    delay(500);
    calibrateGyro();
}

float getYaw() {
    if (isnan(yaw) || isinf(yaw)) return 0.0f;
    return yaw;
}

void resetYaw() {
    if (mpuReady && dmpReady && mpu.dmpGetCurrentFIFOPacket(fifoBuffer)) {
        mpu.dmpGetQuaternion(&q, fifoBuffer);
        mpu.dmpGetGravity(&gravity, &q);
        mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
        yawOffset = ypr[0] * (180.0f / M_PI);
    }
    yaw = 0.0f;
}

float getFilteredGyroZ() {
    return 0.0f; // placeholder karena filter dinonaktifkan
}

void setGyroFilterAlpha(float alpha) {
    // dummy karena filter dinonaktifkan
}

// ============================================================
// Update Yaw — Pure DMP 6-Axis (Fused)
// ============================================================

void updateYaw() {
    if (!mpuReady || !dmpReady) return;

    // Baca data terbaru dari FIFO DMP
    if (mpu.dmpGetCurrentFIFOPacket(fifoBuffer)) {
        mpu.dmpGetQuaternion(&q, fifoBuffer);
        mpu.dmpGetGravity(&gravity, &q);
        mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
        
        // ypr[0] = yaw (dalam radian). Konversi ke derajat.
        // Kiri = + (CCW), Kanan = - (CW)
        float rawYaw = ypr[0] * (180.0f / M_PI);
        
        if (!isnan(rawYaw) && !isinf(rawYaw)) {
            float tempYaw = rawYaw - yawOffset;
            
            // Normalize ke range [-180, 180] menggunakan normalizeAngle
            yaw = normalizeAngle(tempYaw, -180.0f, 180.0f);
        }
        
        // Safety guard
        if (isnan(yaw) || isinf(yaw)) {
            yaw = 0.0f;
        }
    }
}
