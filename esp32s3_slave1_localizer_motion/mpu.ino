// ============================================================
// MPU9250 IMU - Yaw/Gyro Reading (DMP-based)
// ============================================================
// Library: MPU9250 by hideakitai
// DMP menyediakan yaw/pitch/roll yang sudah fused accel+gyro
// Tidak perlu manual integrasi — DMP handle drift compensation
//
// Flow:
//   Boot → load NVS/default → konvergensi 5s di setup → yaw siap
//   CALIB_GYRO → kalibrasi full → simpan NVS → rekonvergensi
//   RESET_GYRO → hapus NVS → next boot pakai default lagi
// ============================================================

#include <Wire.h>
#include "MPU9250.h"
#include <Preferences.h>

static MPU9250 mpu;

// NVS namespace for calibration data
static constexpr const char* GYRO_NVS_NS = "gyro_cal";

// Yaw offset (heading reference dari posisi awal robot)
static float yawOffset = 0.0f;

// Yaw angle in degrees (relatif dari posisi awal robot)
static float yaw = 0.0f;

// Flag: true after mpu.setup succeeds
static bool mpuReady = false;

// Konvergensi filter quaternion
static constexpr unsigned long YAW_CONVERGE_MS = 10000; // 5 detik

// ============================================================
// Default calibration values (hasil kalibrasi dari contoh)
// ============================================================

static void calibApplyDefaults() {
    mpu.setGyroBias(7.31f, -4.46f, -1.32f);
    mpu.setAccBias(-473.41f, 667.36f, -256.87f);
    mpu.setMagBias(186.34f, 244.73f, -427.56f);
    mpu.setMagScale(0.91f, 1.00f, 1.11f);
    Serial.println("Applied default calibration values");
}

// ============================================================
// NVS save/load calibration
// ============================================================

static void calibSaveToNVS() {
    Preferences prefs;
    prefs.begin(GYRO_NVS_NS, false);
    prefs.putFloat("gbx", mpu.getGyroBiasX());
    prefs.putFloat("gby", mpu.getGyroBiasY());
    prefs.putFloat("gbz", mpu.getGyroBiasZ());
    prefs.putFloat("abx", mpu.getAccBiasX());
    prefs.putFloat("aby", mpu.getAccBiasY());
    prefs.putFloat("abz", mpu.getAccBiasZ());
    prefs.putFloat("mbx", mpu.getMagBiasX());
    prefs.putFloat("mby", mpu.getMagBiasY());
    prefs.putFloat("mbz", mpu.getMagBiasZ());
    prefs.putFloat("msx", mpu.getMagScaleX());
    prefs.putFloat("msy", mpu.getMagScaleY());
    prefs.putFloat("msz", mpu.getMagScaleZ());
    prefs.putBool("cal", true);
    prefs.end();
    Serial.println("Calibration saved to NVS");
}

static bool calibLoadFromNVS() {
    Preferences prefs;
    prefs.begin(GYRO_NVS_NS, true);
    bool found = prefs.getBool("cal", false);
    if (found) {
        mpu.setGyroBias(prefs.getFloat("gbx", 0.0f),
                        prefs.getFloat("gby", 0.0f),
                        prefs.getFloat("gbz", 0.0f));
        mpu.setAccBias(prefs.getFloat("abx", 0.0f),
                       prefs.getFloat("aby", 0.0f),
                       prefs.getFloat("abz", 0.0f));
        mpu.setMagBias(prefs.getFloat("mbx", 0.0f),
                       prefs.getFloat("mby", 0.0f),
                       prefs.getFloat("mbz", 0.0f));
        mpu.setMagScale(prefs.getFloat("msx", 1.0f),
                        prefs.getFloat("msy", 1.0f),
                        prefs.getFloat("msz", 1.0f));
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
// Setup — termasuk konvergensi filter quaternion
// ============================================================

bool setupMPU() {
    delay(150);

    MPU9250Setting setting;
    setting.accel_fs_sel = ACCEL_FS_SEL::A16G;
    setting.gyro_fs_sel = GYRO_FS_SEL::G2000DPS;
    setting.mag_output_bits = MAG_OUTPUT_BITS::M16BITS;
    setting.fifo_sample_rate = FIFO_SAMPLE_RATE::SMPL_200HZ;
    setting.gyro_fchoice = 0x03;
    setting.gyro_dlpf_cfg = GYRO_DLPF_CFG::DLPF_41HZ;
    setting.accel_fchoice = 0x01;
    setting.accel_dlpf_cfg = ACCEL_DLPF_CFG::DLPF_45HZ;

    const int maxRetries = 3;
    bool ok = false;
    for (int i = 0; i < maxRetries && !ok; i++) {
        if (mpu.setup(0x68, setting)) {
            ok = true;
        } else {
            Serial.println("MPU9250: not found – retrying...");
            delay(200);
        }
    }
    if (!ok) {
        Serial.println("MPU9250: not found after retries");
        mpuReady = false;
        return false;
    }
    mpuReady = true;
    Serial.println("MPU9250: OK");

    if (calibLoadFromNVS()) {
        Serial.println("Loaded calibration from NVS:");
    } else {
        Serial.println("No calibration in NVS — using defaults:");
        calibApplyDefaults();
    }
    Serial.printf("  Gyro bias: %.4f %.4f %.4f deg/s\n",
                  mpu.getGyroBiasX(), mpu.getGyroBiasY(), mpu.getGyroBiasZ());
    Serial.printf("  Acc bias:  %.4f %.4f %.4f mg\n",
                  mpu.getAccBiasX(), mpu.getAccBiasY(), mpu.getAccBiasZ());
    Serial.printf("  Mag bias:  %.4f %.4f %.4f mG\n",
                  mpu.getMagBiasX(), mpu.getMagBiasY(), mpu.getMagBiasZ());
    Serial.printf("  Mag scale: %.4f %.4f %.4f\n",
                  mpu.getMagScaleX(), mpu.getMagScaleY(), mpu.getMagScaleZ());

    // Konvergensi filter quaternion — blokir di setup supaya yaw langsung stabil
    Serial.printf("Quaternion filter converging (%lus)...", YAW_CONVERGE_MS / 1000);
    unsigned long startMs = millis();
    while (millis() - startMs < YAW_CONVERGE_MS) {
        mpu.update();
        delay(10);
    }
    // Capture heading awal sebagai reference → yaw mulai dari 0
    yawOffset = mpu.getYaw();
    yaw = 0.0f;
    Serial.printf(" done. Reference: %.2f deg\n", yawOffset);

    return true;
}

// ============================================================
// Calibration (hanya via serial CALIB_GYRO)
// ============================================================

void calibrateGyro() {
    Serial.println("Calibrating accel+gyro+mag... keep robot still!");
    mpu.verbose(true);
    mpu.calibrateAccelGyro();
    mpu.calibrateMag();
    mpu.verbose(false);

    Serial.println("Calibration complete:");
    Serial.printf("  Gyro bias: %.4f %.4f %.4f deg/s\n",
                  mpu.getGyroBiasX(), mpu.getGyroBiasY(), mpu.getGyroBiasZ());
    Serial.printf("  Acc bias:  %.4f %.4f %.4f mg\n",
                  mpu.getAccBiasX(), mpu.getAccBiasY(), mpu.getAccBiasZ());
    Serial.printf("  Mag bias:  %.4f %.4f %.4f mG\n",
                  mpu.getMagBiasX(), mpu.getMagBiasY(), mpu.getMagBiasZ());
    Serial.printf("  Mag scale: %.4f %.4f %.4f\n",
                  mpu.getMagScaleX(), mpu.getMagScaleY(), mpu.getMagScaleZ());

    calibSaveToNVS();

    // Rekonvergensi setelah kalibrasi
    Serial.printf("Reconverging filter (%lus)...", YAW_CONVERGE_MS / 1000);
    unsigned long startMs = millis();
    while (millis() - startMs < YAW_CONVERGE_MS) {
        mpu.update();
        delay(10);
    }
    yawOffset = mpu.getYaw();
    yaw = 0.0f;
    Serial.printf(" done. Reference: %.2f deg\n", yawOffset);
}

void calibrateGyroHot() {
    Serial.println("Hot recalibration: stop all motors first!");
    delay(500);
    calibrateGyro();
}

// ============================================================
// Update Yaw
// ============================================================

void updateYaw() {
    if (!mpuReady) return;

    if (mpu.update()) {
        yaw = mpu.getYaw() - yawOffset;

        // Wrap yaw di [-180, 180]
        if (yaw > 180.0f)  yaw -= 360.0f;
        if (yaw < -180.0f) yaw += 360.0f;
    }
}

float getYaw() {
    return yaw;
}

void resetYaw() {
    if (mpuReady && mpu.update()) {
        yawOffset = mpu.getYaw();
    }
    yaw = 0.0f;
}
