/*
 * =====================================================================
 * FILE    : serial.ino
 * PERAN   : Inisialisasi UART1 + UART2, relay WSN-31 ke master.
 *
 * BOARD   : ESP32-S3 (Slave1 Motion)
 * =====================================================================
 */

#include "serial.h"
#include "pid.h"
#include "motor.h"
#include "autoTuner.h"

// =====================================================================
//  SETUP
// =====================================================================

void setupSerial() {
    // WSN-31 SET HIGH = mode normal
    pinMode(WSN_SET_PIN, OUTPUT);
    digitalWrite(WSN_SET_PIN, HIGH);

    // UART ke WSN-31 (receiver)
    Serial2.begin(SERIAL_WSN_BAUD, SERIAL_8N1, SERIAL_WSN_RX, SERIAL_WSN_TX);
    // UART ke Master (forwarder)
    Serial1.begin(SERIAL_MASTER_BAUD, SERIAL_8N1, SERIAL_MASTER_RX, SERIAL_MASTER_TX);

    Serial.printf("[WSN-31] Relay init — RX=%d TX=%d → Master TX=%d RX=%d\n",
                  SERIAL_WSN_RX, SERIAL_WSN_TX, SERIAL_MASTER_TX, SERIAL_MASTER_RX);
}

// =====================================================================
//  RELAY — forwarding WSN-31 → Master
// =====================================================================

void serialRelayTick() {
    while (Serial2.available()) {
        uint8_t b = Serial2.read();
        Serial1.write(b);
    }
}

// =====================================================================
//  PARSER USB SERIAL (Tuning PID)
// =====================================================================

void parseSerialCommand() {
    if (!Serial.available()) return;

    String input = Serial.readStringUntil('\n');
    input.trim();
    if (input.length() == 0) return;

    // Format: "tune <motor_idx> <kp> <ki> <kd> <kf>"
    // Contoh: "tune 0 1.5 0.1 0.05 2.0"
    if (input.startsWith("tune ")) {
        int idx;
        float kp, ki, kd, kf;
        if (sscanf(input.c_str(), "tune %d %f %f %f %f", &idx, &kp, &ki, &kd, &kf) == 5) {
            pidSetGains(idx, kp, ki, kd, kf);
            Serial.printf("Motor %d PID updated: Kp=%.2f Ki=%.2f Kd=%.2f Kf=%.2f\n", idx, kp, ki, kd, kf);
        } else {
            Serial.println("Format salah! Gunakan: tune <motorIdx> <kp> <ki> <kd> <kf>");
        }
    }
    // Format: "save" -> Simpan semua PID ke NVS
    else if (input.equals("save")) {
        extern PIDState pidStates[]; // akses dari pid.ino
        for (int i = 0; i < 4; i++) {
            pidSaveToNVS(i, pidStates[i].kp, pidStates[i].ki, pidStates[i].kd, pidStates[i].kf);
        }
        Serial.println("Semua konstanta PID tersimpan ke NVS.");
    }
    // Format: "rpm <fr> <fl> <br> <bl>" -> Test RPM
    // Contoh: "rpm 100 100 100 100"
    else if (input.startsWith("rpm ")) {
        int r1, r2, r3, r4;
        if (sscanf(input.c_str(), "rpm %d %d %d %d", &r1, &r2, &r3, &r4) == 4) {
            rpmMotor(r1, r2, r3, r4);
            Serial.printf("Test RPM: FR=%d FL=%d BR=%d BL=%d\n", r1, r2, r3, r4);
        } else {
            Serial.println("Format salah! Gunakan: rpm <fr> <fl> <br> <bl>");
        }
    }
    // Stop semua
    else if (input.equals("stop")) {
        rpmMotor(0, 0, 0, 0);
        Serial.println("Semua motor BERHENTI.");
    }
    // AutoTune Command
    else if (input.startsWith("autotune ")) {
        int idx = input.substring(9).toInt();
        startAutoTune(idx);
    }
    else {
        Serial.println("Command: 'tune', 'save', 'rpm', 'stop', 'autotune <motor>'");
    }
}
