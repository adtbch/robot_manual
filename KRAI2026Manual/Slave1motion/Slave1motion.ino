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
#include "waypoint.h"
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
    Wire.setTimeOut(20);  // 20ms cukup untuk 42-byte DMP packet di 100kHz; timeout cepat saat bus stuck
    Wire.setClock(400000); // ponytail: 400kHz — lebih cepat keluar dari NACK timeout, kurangi positive feedback loop

    if (!setupOLED()) {
        Serial.println("OLED: not ready");
    }

    SetupMotors();
    Serial.println("Motors: READY");

    pidControllerInit();
    initYawPid();
    initWaypointPid();
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

    // Pindahkan I2C (OLED & MPU) ke Core 0 agar tidak block Core 1 (UART/PID)
    xTaskCreatePinnedToCore(
        [](void* pvParam) {
            while (true) {
                if (!isAutoTunerRunning()) {
                    updateYaw();
                }
                updateOLED();
                vTaskDelay(pdMS_TO_TICKS(10)); // 10ms cukup (MPU 100Hz, OLED throttled 100ms)
            }
        },
        "I2cTask",
        4096,
        NULL,
        1,
        NULL,
        0 // Core 0
    );
}

// =====================================================================
//  LOOP
// =====================================================================

bool testYawMode = false;
int testYawTarget = 0;

void loop() {

    // USB Serial (tuning) + UART master (rpm dari motion_control)
    serialCommandTick();

    updateButton();

    if (isButtonLongPressed() && !isAutoTunerRunning()) {
        testYawMode = false; // Matikan mode test yaw jika menyala
        // motorStopAll();
        // Serial.println("\n[AUTOTUNE] Button Long Press -> Starting!");
        startAutoTuneAll();
    }

    autoTunerTick(isButtonLongHolding());

    serialRelayTick();

    static Jeda jedaEncoder;
    if (jedaEncoder.check(RPM_INTERVAL_MS)) {
        convertEncoderToRPM();
    }

    updateOdometry();
    
    // Waypoint dan TestYaw mutex - waypoint lebih prioritas
    if (!isAutoTunerRunning()) {
        // updateYaw() sudah pindah ke I2cTask di Core 0
        if (getWaypointState() != WaypointState::IDLE) {
            waypointTick(wpTargetX_m, wpTargetY_m, wpTargetYaw_deg, wpMaxSpeed);
        } 
        if (testYawMode) {
            static Jeda jedaYawTest;
            if (jedaYawTest.check(20)) {
                driveFieldCentricWithYawCorrection(0, 0, testYawTarget);
            }
        }
    }
}
