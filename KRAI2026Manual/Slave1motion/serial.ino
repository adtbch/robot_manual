/*
 * =====================================================================
 * FILE    : serial.ino
 * PERAN   : Inisialisasi UART1 + UART2, relay WSN-31 ke master,
 *           unified command handler (USB Serial + UART master).
 *
 * BOARD   : ESP32-S3 (Slave1 Motion)
 * =====================================================================
 */

#include "serial.h"
#include "pid.h"
#include "motor.h"
#include "autoTuner.h"

namespace {

constexpr size_t SERIAL_CMD_BUF_SIZE = 64;

char pcBuf[SERIAL_CMD_BUF_SIZE];
uint8_t pcBufIdx = 0;

char masterBuf[SERIAL_CMD_BUF_SIZE];
uint8_t masterBufIdx = 0;

void parseAndExecuteCommand(char* cmd, Print& out) {
    char* token = strtok(cmd, " ");
    if (token == nullptr) return;

    for (char* p = token; *p; ++p) *p = tolower(*p);

    if (strcmp(token, "tune") == 0) {
        char* idxStr = strtok(nullptr, " ");
        char* kpStr = strtok(nullptr, " ");
        char* kiStr = strtok(nullptr, " ");
        char* kdStr = strtok(nullptr, " ");
        char* kfStr = strtok(nullptr, " ");
        if (idxStr != nullptr && kpStr != nullptr && kiStr != nullptr &&
            kdStr != nullptr && kfStr != nullptr) {
            const int idx = atoi(idxStr);
            const float kp = atof(kpStr);
            const float ki = atof(kiStr);
            const float kd = atof(kdStr);
            const float kf = atof(kfStr);
            pidSetGains(idx, kp, ki, kd, kf);
            out.printf("Motor %d PID updated: Kp=%.2f Ki=%.2f Kd=%.2f Kf=%.2f\n",
                       idx, kp, ki, kd, kf);
        } else {
            out.println("Format salah! Gunakan: tune <motorIdx> <kp> <ki> <kd> <kf>");
        }
    } else if (strcmp(token, "save") == 0) {
        for (int i = 0; i < 4; i++) {
            pidSaveToNVS(i, pidStates[i].kp, pidStates[i].ki, pidStates[i].kd, pidStates[i].kf);
        }
        out.println("Semua konstanta PID tersimpan ke NVS.");
    } else if (strcmp(token, "rpm") == 0) {
        char* frStr = strtok(nullptr, " ");
        char* flStr = strtok(nullptr, " ");
        char* brStr = strtok(nullptr, " ");
        char* blStr = strtok(nullptr, " ");
        if (frStr != nullptr && flStr != nullptr && brStr != nullptr && blStr != nullptr) {
            const int fr = atoi(frStr);
            const int fl = atoi(flStr);
            const int br = atoi(brStr);
            const int bl = atoi(blStr);
            rpmMotor(fr, fl, br, bl);
            out.printf("RPM: FR=%d FL=%d BR=%d BL=%d\n", fr, fl, br, bl);
        } else {
            out.println("Format salah! Gunakan: rpm <fr> <fl> <br> <bl>");
        }
    } else if (strcmp(token, "stop") == 0) {
        rpmMotor(0, 0, 0, 0);
        out.println("Semua motor BERHENTI.");
    } else if (strcmp(token, "autotune") == 0) {
        char* idxStr = strtok(nullptr, " ");
        if (idxStr != nullptr) {
            startAutoTune(atoi(idxStr));
        } else {
            out.println("Format salah! Gunakan: autotune <motor>");
        }
    } else {
        out.println("Command: tune, save, rpm, stop, autotune");
    }
}

void readSerialLine(Stream& port, char* buf, uint8_t& idx, Print& out) {
    while (port.available() > 0) {
        const char c = port.read();
        if (c == '\n' || c == '\r') {
            if (idx > 0) {
                buf[idx] = '\0';
                parseAndExecuteCommand(buf, out);
                idx = 0;
            }
        } else if (idx < SERIAL_CMD_BUF_SIZE - 1) {
            buf[idx++] = c;
        } else {
            out.println("Error: command terlalu panjang");
            idx = 0;
        }
    }
}

}  // namespace

// =====================================================================
//  SETUP
// =====================================================================

void setupSerial() {
    pinMode(WSN_SET_PIN, OUTPUT);
    digitalWrite(WSN_SET_PIN, HIGH);

    Serial2.begin(SERIAL_WSN_BAUD, SERIAL_8N1, SERIAL_WSN_RX, SERIAL_WSN_TX);
    Serial1.begin(SERIAL_MASTER_BAUD, SERIAL_8N1, SERIAL_MASTER_RX, SERIAL_MASTER_TX);

    Serial.printf("[Serial] Master UART RX=%d TX=%d @ %lu baud\n",
                  SERIAL_MASTER_RX, SERIAL_MASTER_TX, (unsigned long)SERIAL_MASTER_BAUD);
}

// =====================================================================
//  RELAY — forwarding WSN-31 → Master
// =====================================================================

void serialRelayTick() {
    while (Serial2.available()) {
        const uint8_t b = Serial2.read();
        Serial1.write(b);
    }
}

// =====================================================================
//  COMMAND HANDLER — USB Serial + UART master
// =====================================================================

void serialCommandTick() {
    readSerialLine(Serial, pcBuf, pcBufIdx, Serial);
    readSerialLine(Serial1, masterBuf, masterBufIdx, Serial1);
}
