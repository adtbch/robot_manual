/*
 * =====================================================================
 * FILE    : serial_command.ino
 * PERAN   : 1) Helper functions untuk kirim serial command ke slave boards.
 *           2) Parse sensor data dari slave2arm.
 *
 * SLAVE1 (UART1): motion control — kn vx vy yaw
 * SLAVE2 (UART2): arm manipulator — sensor data
 *
 * SENSOR DATA dari slave2arm (format text):
 *   prox <r|l> <0|1>        — proximity detected/clear
 *   limit <name> <0|1>      — limit switch triggered/clear
 *   enc <x|y|k> <count>     — encoder posisi
 *   pne <r|l> <0|1>         — pneumatic state
 *
 * BOARD   : ESP32-S3 (Master)
 * =====================================================================
 */

#include "serial.h"
#include "proximity.h"
#include <cstdarg>

// =====================================================================
//  STATE — sensor data terakhir dari slave2
// =====================================================================

namespace {
struct Slave2SensorState {
    bool proxR = false;
    bool proxL = false;
    bool limitDepan = false;
    bool limitBelakang = false;
    bool limitTurun = false;
    long encX = 0;
    long encY = 0;
    long encK = 0;
    bool pneR = false;
    bool pneL = false;
};
Slave2SensorState gSlave2;
} // anonymous namespace

// =====================================================================
//  SLAVE2 SENSOR QUERY — baca status terakhir
// =====================================================================

bool slave2ProxR()           { return gSlave2.proxR; }
bool slave2ProxL()           { return gSlave2.proxL; }
bool slave2LimitDepan()      { return gSlave2.limitDepan; }
bool slave2LimitBelakang()   { return gSlave2.limitBelakang; }
bool slave2LimitTurun()      { return gSlave2.limitTurun; }
long slave2EncX()            { return gSlave2.encX; }
long slave2EncY()            { return gSlave2.encY; }
long slave2EncK()            { return gSlave2.encK; }
bool slave2PneR()            { return gSlave2.pneR; }
bool slave2PneL()            { return gSlave2.pneL; }

// =====================================================================
//  PARSE — dipanggil dari serialCommandTick() saat data dari slave2
// =====================================================================

bool parseSlave2Sensor(char* cmd) {
    char* token = strtok(cmd, " ");
    if (token == nullptr) return false;

    // ── prox <side> <0|1> ─────────────────────────────────────
    if (strcmp(token, "prox") == 0) {
        char* side = strtok(nullptr, " ");
        char* val = strtok(nullptr, " ");
        if (side && val) {
            bool detected = (atoi(val) == 1);
            if (side[0] == 'r') gSlave2.proxR = detected;
            else if (side[0] == 'l') gSlave2.proxL = detected;
            return true;
        }
    }

    // ── limit <name> <0|1> ────────────────────────────────────
    if (strcmp(token, "limit") == 0) {
        char* name = strtok(nullptr, " ");
        char* val = strtok(nullptr, " ");
        if (name && val) {
            bool triggered = (atoi(val) == 1);
            if (strcmp(name, "d") == 0)             gSlave2.limitDepan = triggered;
            else if (strcmp(name, "b") == 0)        gSlave2.limitBelakang = triggered;
            else if (strcmp(name, "t") == 0)        gSlave2.limitTurun = triggered;
            return true;
        }
    }

    // ── enc <id> <count> ──────────────────────────────────────
    if (strcmp(token, "enc") == 0) {
        char* id = strtok(nullptr, " ");
        char* val = strtok(nullptr, " ");
        if (id && val) {
            long count = atol(val);
            if (id[0] == 'x')      gSlave2.encX = count;
            else if (id[0] == 'y') gSlave2.encY = count;
            else if (id[0] == 'k') gSlave2.encK = count;
            return true;
        }
    }

    // ── pne <side> <0|1> ──────────────────────────────────────
    if (strcmp(token, "pne") == 0) {
        char* side = strtok(nullptr, " ");
        char* val = strtok(nullptr, " ");
        if (side && val) {
            bool state = (atoi(val) == 1);
            if (side[0] == 'r')      gSlave2.pneR = state;
            else if (side[0] == 'l') gSlave2.pneL = state;
            return true;
        }
    }

    return false;
}

// =====================================================================
//  SLAVE1 — Motion (Mecanum)
// =====================================================================

void sendKnCommand(int16_t vx, int16_t vy, int16_t yawTarget) {
    slave1Serial.printf("kn %d %d %d\n", vx, vy, yawTarget);
}

// =====================================================================
//  SLAVE2 — Arm Manipulator
// =====================================================================

void sendSlave2Command(const char* fmt, ...) {
    char buf[64];
    va_list args;
    va_start(args, fmt);
    vsnprintf(buf, sizeof(buf), fmt, args);
    va_end(args);
    slave2Serial.println(buf);
}
