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

    setupSerial();
    Serial.println("Serial: READY");

    setupButton();
    Serial.println("Button: READY");

    setupOTA();

    setupWebServer();

    Wire.begin(I2C_SDA, I2C_SCL);
    Wire.setTimeOut(100);  // Timeout 100ms agar I2C tidak hang jika bus error
    Wire.setClock(100000); // ponytail: diturunkan ke 100kHz untuk meningkatkan toleransi terhadap noise/NACK dari BTS

    if (!setupOLED()) {
        Serial.println("OLED: not ready");
    }

    SetupMotors();
    Serial.println("Motors: READY");

    pidControllerInit();
    initYawPid();
    Serial.println("PID: READY");

    setupEncoders();
    Serial.println("Encoders: READY");

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
    handleOTA();

    // USB Serial (tuning) + UART master (rpm dari motion_control)
    serialCommandTick();

    updateButton();

    if (isButtonLongPressed() && !isAutoTunerRunning()) {
        testYawMode = false; // Matikan mode test yaw jika menyala
        motorStopAll();
        Serial.println("\n[AUTOTUNE] Button Long Press -> Starting!");
        startAutoTuneAll();
    }

    autoTunerTick(isButtonLongHolding());

    serialRelayTick();

    static Jeda jedaEncoder;
    if (jedaEncoder.check(RPM_INTERVAL_MS)) {
        convertEncoderToRPM();
    }

    // Saat autotune motor jalan: skip MPU agar bus I2C tidak berebut dengan OLED
    if (!isAutoTunerRunning()) {
        updateYaw();
    }
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
