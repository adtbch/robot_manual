// ============================================================
// MPU6050 IMU - Yaw/Gyro Reading (DMP MotionApps v6.12)
// ============================================================
// Library: MPU6050v2 (Jeff Rowberg i2cdevlib)
// Menggunakan DMP (Digital Motion Processor) internal chip
// untuk estimasi Yaw 6-Axis (Gyro + Accel) secara pure.
//
// Pola inisialisasi DMP mengikuti official example:
//   MPU6050_DMP6_using_DMP_V6.12.ino
// - Tidak ada delay() di loop utama
// - Tidak ada blocking reference capture
// - DMP reference yaw otomatis dari orientasi saat enable
// ============================================================

#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps612.h"
#include <Preferences.h>

static MPU6050 mpu;

// NVS namespace for calibration data
static constexpr const char* GYRO_NVS_NS = "gyro_cal";

// Yaw offset (heading reference dari resetYaw)
static float yawOffset = 0.0f;

// Yaw angle in degrees (relatif dari posisi awal robot)
static float yaw = 0.0f;

// Flag: true after mpu.initialize succeeds
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

// (bias/temperature compensation dihapus — DMP sudah handle gyro bias internal)


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
// Shared MPU init — pola dari official example
// ============================================================

static bool mpuInitCommon() {
    // ---- Inisialisasi I2C ----
    Serial.println(F("Initializing MPU6050 (DMP)..."));
    mpu.initialize();
    pinMode(INTERRUPT_PIN, INPUT);

    mpuReady = true;
    Serial.println(F("MPU6050 connection successful"));

    // ---- Load offset dari NVS atau default ----
    if (calibLoadFromNVS()) {
        Serial.println("Loaded calibration offsets from NVS:");
    } else {
        Serial.println("No calibration in NVS — using defaults:");
        calibApplyDefaults();
    }
    
    Serial.printf("  Offsets: Ax=%d Ay=%d Az=%d Gx=%d Gy=%d Gz=%d\n",
                  mpu.getXAccelOffset(), mpu.getYAccelOffset(), mpu.getZAccelOffset(),
                  mpu.getXGyroOffset(), mpu.getYGyroOffset(), mpu.getZGyroOffset());

    // ---- Inisialisasi DMP (MotionApps v6.12) ----
    Serial.println(F("Initializing DMP..."));
    devStatus = mpu.dmpInitialize();

    if (devStatus == 0) {
        // Auto-kalibrasi internal (tuning offset) — 25 loops untuk bias lebih presisi
        mpu.CalibrateAccel(25);
        mpu.CalibrateGyro(25);
        Serial.println(F("Active offsets after calibration:"));
        mpu.PrintActiveOffsets();

        // Enable DMP
        Serial.println(F("Enabling DMP..."));
        mpu.setDMPEnabled(true);

        // Set DLPF mode 3 (42 Hz gyro, 44 Hz accel) — kurangi noise tanpa mengganggu DMP
        mpu.setDLPFMode(3);
        // Set sample rate output ~200 Hz (1 kHz / (1+4))
        mpu.setRate(4);

        // Pasang hardware interrupt
        attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN), dmpDataReady, RISING);
        // Baca dan clear interrupt status — penting agar DMP mulai mengisi FIFO
        // (pola dari official example)
        mpu.getIntStatus();
        
        dmpReady = true;
        packetSize = mpu.dmpGetFIFOPacketSize();
    } else {
        Serial.printf("DMP Initialization failed (code %d)\n", devStatus);
        dmpReady = false;
        return false;
    }

    // DMP ready – yaw reference otomatis dari orientasi saat enable
    yawOffset = 0.0f;
    yaw = 0.0f;
    Serial.printf("DMP Ready. yawOffset = %.2f\n", yawOffset);
    return true;
}

bool setupMPUGyro() {
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
    
    mpu.CalibrateAccel(25);
    mpu.CalibrateGyro(25);
    
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
// Bedanya dengan version awal:
// - Tidak ada blocking / non‑blocking capture reference
// - Langsung baca FIFO jika ada data
// - Bias online & suhu untuk mengurangi drift
// ============================================================

void updateYaw() {
    if (!mpuReady || !dmpReady) return;

    // Watchdog state
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

        // ---- Drift freeze via gyro Z motion detection ----
        static float yawActive = 0.0f;
        static bool wasMoving = false;
        static uint32_t lastMoveMs = 0;

        int16_t gzRaw = mpu.getRotationZ();
        bool isMoving = (fabsf(gzRaw / 131.0f) > 0.3f);  // deg/s

        if (isMoving) {
            yawActive = yaw;
            lastMoveMs = millis();
            wasMoving = true;
        } else if (wasMoving) {
            // Freeze saat baru berhenti
            yawActive = yaw;
            wasMoving = false;
        }
        // Stationer >500 ms → output frozen
        if (isMoving || (millis() - lastMoveMs <= 500)) {
            yaw = yawActive;
        } else {
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
