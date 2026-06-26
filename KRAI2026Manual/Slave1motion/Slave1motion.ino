/*
 * =====================================================================
 * FILE    : Slave1motion.ino
 * BOARD   : ESP32-S3 (Slave1 Motion Board KRAI 2026)
 * PERAN   : Entry point — setup() + loop().
 *           Mecanum drive 4 motor, MPU6050 yaw, PID, serial relay.
 * =====================================================================
 */

#include "config.h"
#include "motor.h"
#include "encoder.h"
#include "mpu.h"
#include "kinematik.h"
#include "pid.h"
#include "serial.h"
#include "oled.h"
#include "button.h"
#include "ota.h"
#include "autoTuner.h"
#include <Wire.h>

// =====================================================================
//  SETUP
// =====================================================================

void setup() {
    Serial.begin(115200);
    Serial.println("=== KRAI 2026 Slave1 Motion Board ===");

    // Init serial relay
    setupSerial();
    Serial.println("Serial: READY");

    // Init button
    setupButton();
    Serial.println("Button: READY");

    // Init WiFi + OTA
    setupOTA();

    // Init I2C
    Wire.begin(I2C_SDA, I2C_SCL);
    Wire.setTimeOut(100);
    Wire.setClock(400000);

    // Init OLED
    if (!setupOLED()) {
        Serial.println("OLED: not ready");
    } else {
        Serial.println("OLED: READY");
    }

    // Init motors
    SetupMotors();
    Serial.println("Motors: READY");

    // Init PID
    pidControllerInit();
    initYawPid();
    Serial.println("PID: READY");

    // Init encoders
    setupEncoders();
    Serial.println("Encoders: READY");

    // Init MPU — BLOCK di setup sampai berhasil
    while (!setupMPU()) {
        Serial.println("MPU: FAILED, retrying...");
        oledShowStatus("MPU FAILED!", "Check wiring");
        delay(500);
    }
    Serial.println("MPU: READY");
    oledShowStatus("MPU: READY", "Starting...");
    delay(1000);
}

// =====================================================================
//  LOOP
// =====================================================================

void loop() {
    // Handle OTA update (WiFi)
    handleOTA();

    // Baca perintah USB Serial (Tuning PID)
    parseSerialCommand();

    // Update state button (debounce & durasi)
    updateButton();

    // Tombol tahan 3 detik -> trigger auto-tune
    if (isButtonLongPressed() && !isAutoTunerRunning()) {
        Serial.println("\n[AUTOTUNE] Button Long Press -> Starting!");
        startAutoTune(0); // Mulai tuning motor 0
    }

    // Jalankan Auto-Tuner (kalau sedang aktif)
    autoTunerTick();

    // Relay WSN-31 → Master
    serialRelayTick();

    // Convert encoder ke RPM (40ms interval)
    static Jeda jedaEncoder;
    if (jedaEncoder.check(RPM_INTERVAL_MS)) {
        convertEncoderToRPM();
    }

    // Update yaw dari MPU
    updateYaw();

    // Update odometry dari external encoder (setiap loop, sudah ada dt guard)
    updateOdometry();

    // Update OLED display
    updateOLED();
}
