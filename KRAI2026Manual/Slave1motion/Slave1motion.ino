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
#include <WebServer.h>
#include "webconfig.h"
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

    // Init Web Server (runs on core 0)
    setupWebServer();

    // Init I2C
    Wire.begin(I2C_SDA, I2C_SCL);
    Wire.setTimeOut(100);  // Timeout 100ms agar I2C tidak hang jika bus error
    Wire.setClock(100000); // ponytail: diturunkan ke 100kHz untuk meningkatkan toleransi terhadap noise/NACK dari BTS

    // Init OLED
    if (!setupOLED()) {
        Serial.println("OLED: not ready");
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

bool testYawMode = false;
int testYawTarget = 0;

void loop() {
    // Handle OTA update (WiFi)
    handleOTA();
    
    // Baca perintah USB Serial (Tuning PID)
    parseSerialCommand();

    // Update state button (debounce & durasi)
    updateButton();

    // Tombol tahan 3 detik -> trigger auto-tune
    if (isButtonLongPressed() && !isAutoTunerRunning()) {
        testYawMode = false; // Matikan mode test yaw jika menyala
        motorStopAll();
        Serial.println("\n[AUTOTUNE] Button Long Press -> Starting!");
        startAutoTuneAll();
    }

    // Jalankan Auto-Tuner (kalau sedang aktif)
    autoTunerTick(isButtonLongHolding());

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

    // Test Yaw PID Mode
    if (testYawMode && !isAutoTunerRunning()) {
        static Jeda jedaYawTest;
        if (jedaYawTest.check(20)) { // 50Hz update loop
            driveFieldCentricWithYawCorrection(0, 0, testYawTarget);
        }
    }

    // Update OLED display
    updateOLED();
}
