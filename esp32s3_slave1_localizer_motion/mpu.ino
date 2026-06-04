// ============================================================
// MPU6050 IMU - Yaw/Gyro Reading
// ============================================================
// Perbaikan 1: Hapus software LPF + hybrid deadzone yang bikin lockup
// Perbaikan 2: Deadzone langsung di RAW value (tidak korup state)
// Perbaikan 3: Threshold efektif sekarang = deadzone beneran, bukan 5x lipat
// Lihat analisa: deadzone-OR-filtered bikin threshold efektif jadi 0.05 rad/s
//   dan asimetri arah +/- akibat residual bias x interaksi LPF
// ============================================================

#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Preferences.h>

static Adafruit_MPU6050 mpu;
static sensors_event_t a, g, temp;

// NVS namespace for gyro calibration data
static constexpr const char* GYRO_NVS_NS = "gyro_cal";

// Estimated gyro Z bias — auto-load dari NVS saat startup
static float gyroZBias = 0.0f;

// Deadzone: nilai gyro di bawah threshold ini dianggap 0
// MPU6050 internal DLPF 21Hz sudah cukup redam noise getaran
static constexpr float GYRO_DEADZONE = 0.015f; // ≈0.86 °/s

// Yaw angle in degrees
static float yaw = 0.0f;
static unsigned long lastMpuTime = 0;

// ============================================================
// NVS save/load gyro bias
// ============================================================

static void gyroBiasSaveToNVS(float bias) {
    Preferences prefs;
    prefs.begin(GYRO_NVS_NS, false); // read-write
    prefs.putFloat("zbias", bias);
    prefs.end();
    Serial.printf("Gyro bias saved to NVS: %.6f rad/s\n", bias);
}

static bool gyroBiasLoadFromNVS(float &bias) {
    Preferences prefs;
    prefs.begin(GYRO_NVS_NS, true); // read-only
    bool found = prefs.isKey("zbias");
    if (found) {
        bias = prefs.getFloat("zbias", 0.0f);
    }
    prefs.end();
    return found;
}

static void gyroBiasClearNVS() {
    Preferences prefs;
    prefs.begin(GYRO_NVS_NS, false);
    prefs.clear();
    prefs.end();
    Serial.println("Gyro bias NVS cleared.");
}

// ============================================================
// Setup
// ============================================================

bool setupMPU() {
    if (!mpu.begin(0x68, &Wire, 0)) {
        Serial.println("MPU6050: not found");
        return false;
    }

    mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
    mpu.setGyroRange(MPU6050_RANGE_500_DEG);
    mpu.setFilterBandwidth(MPU6050_BAND_21_HZ); // Internal DLPF sudah cukup

    // Load bias dari NVS dulu
    if (gyroBiasLoadFromNVS(gyroZBias)) {
        Serial.printf("MPU6050: Loaded gyro bias from NVS: %.6f rad/s\n", gyroZBias);
        Serial.println("  Send CALIB_GYRO if bias has drifted.");
        yaw = 0.0f;
        lastMpuTime = millis();
    } else {
        // First boot — calibrate manually
        Serial.println("MPU6050: No saved bias found. Calibrating...");
        calibrateGyro();
    }

    return true;
}

// ============================================================
// Calibration
// ============================================================

void calibrateGyro() {
    const int samples = 3000;
    float sum = 0.0f;

    Serial.println("Keep robot still for 3 seconds...");
    for (int i = 0; i < samples; i++) {
        mpu.getEvent(&a, &g, &temp);
        sum += g.gyro.z;
        if (i % 200 == 0) Serial.print(".");
        delay(3);
    }
    Serial.println();

    gyroZBias = sum / samples;
    yaw = 0.0f;
    lastMpuTime = millis();
    Serial.printf("Gyro Z bias calibrated: %.6f rad/s\n", gyroZBias);

    // Auto-save ke NVS
    gyroBiasSaveToNVS(gyroZBias);
}

// Recalibrasi — panggil via serial CALIB_GYRO jika drift terasa (motor panas dll)
void calibrateGyroHot() {
    Serial.println("Hot recalibration: stop all motors first!");
    calibrateGyro(); // auto-save ke NVS
}

void updateYaw() {
    unsigned long now = millis();
    if (lastMpuTime == 0) {
        lastMpuTime = now;
        return;
    }

    // Baca sensor
    mpu.getEvent(&a, &g, &temp);

    float dt = (now - lastMpuTime) / 1000.0f;
    lastMpuTime = now;
    if (dt <= 0.0f || dt > 0.5f) return;

    // 1. Koreksi bias
    float gyroZ = g.gyro.z - gyroZBias;

    // 2. Deadzone langsung di RAW — tanpa LPF tambahan
    //    MPU6050 internal DLPF 21Hz sudah meredam noise getaran.
    //    Tidak ada LPF state corruption → threshold efektif = GYRO_DEADZONE beneran.
    if (fabsf(gyroZ) < GYRO_DEADZONE) {
        gyroZ = 0.0f;
    }

    // 3. Integrasi ke Yaw (derajat)
    //    GYRO_SCALE_FACTOR bisa ditune jika ada scale error
    const float GYRO_SCALE_FACTOR = 1.0f;
    yaw += gyroZ * GYRO_SCALE_FACTOR * dt * (180.0f / PI);

    // Wrap yaw di [-180, 180]
    if (yaw > 180.0f)  yaw -= 360.0f;
    if (yaw < -180.0f) yaw += 360.0f;
}

float getYaw() {
    return yaw;
}

void resetYaw() {
    yaw = 0.0f;
    lastMpuTime = millis();
}
