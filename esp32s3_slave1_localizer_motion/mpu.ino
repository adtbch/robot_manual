// ============================================================
// MPU6050 IMU - Yaw/Gyro Reading
// ============================================================

#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>

static Adafruit_MPU6050 mpu;
static sensors_event_t a, g, temp;

// Estimated gyro Z bias — calibrate manually or via init routine
static float gyroZBias = 0.0f;

// Low-pass filtered gyro Z for vibration suppression
static float gyroZFiltered = 0.0f;

// Tuning: smaller alpha = smoother but more lag
static constexpr float GYRO_LPF_ALPHA = 0.18f;
static constexpr float GYRO_DEADZONE = 0.008f;

// Yaw angle in degrees
static float yaw = 0.0f;
static unsigned long lastMpuTime = 0;

bool setupMPU() {
    if (!mpu.begin(0x68, &Wire, 0)) {
        Serial.println("MPU6050: not found");
        return false;
    }

    mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
    mpu.setGyroRange(MPU6050_RANGE_500_DEG);
    mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);

    Serial.println("MPU6050: OK. Calibrating...");
    calibrateGyro(); // WAJIB: Hitung bias saat startup agar tidak drift parah

    return true;
}

void calibrateGyro() {
    const int samples = 3000; // Naikkan jumlah sampel untuk presisi lebih tinggi
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
    gyroZFiltered = 0.0f;
    yaw = 0.0f;
    lastMpuTime = millis();
    Serial.printf("Gyro Z bias calibrated: %.6f rad/s\n", gyroZBias);
}

void updateYaw() {
    unsigned long now = millis();
    if (lastMpuTime == 0) {
        lastMpuTime = now;
        return;
    }

    mpu.getEvent(&a, &g, &temp);

    float dt = (now - lastMpuTime) / 1000.0f;
    lastMpuTime = now;
    if (dt <= 0.0f || dt > 0.5f) return;

    // 1. Ambil nilai Gyro Z murni dikurangi bias
    float gyroZRaw = g.gyro.z - gyroZBias;

    // 2. Low-pass filter untuk meredam getaran roda
    gyroZFiltered = (GYRO_LPF_ALPHA * gyroZRaw) + ((1.0f - GYRO_LPF_ALPHA) * gyroZFiltered);

    // 3. Deadzone kecil untuk noise sisa di sekitar nol
    if (abs(gyroZFiltered) < GYRO_DEADZONE) {
        gyroZFiltered = 0.0f;
    }

    // Skala perbaikan gyro (jika diputar 360 malah baca 350, naikkan nilai ini > 1.0)
    // Default 1.0f. Anda bisa tune ini nanti jika masih ada skala error
    const float GYRO_SCALE_FACTOR = 1.0f;

    // 4. Integrasi ke Yaw (derajat) dengan scaling
    yaw += (gyroZFiltered * GYRO_SCALE_FACTOR) * dt * (180.0f / PI);

    // Keep yaw in [-180, 180]
    if (yaw > 180.0f)  yaw -= 360.0f;
    if (yaw < -180.0f) yaw += 360.0f;
}

float getYaw() {
    return yaw;
}

void resetYaw() {
    yaw = 0.0f;
    gyroZFiltered = 0.0f;
    lastMpuTime = millis();
}
