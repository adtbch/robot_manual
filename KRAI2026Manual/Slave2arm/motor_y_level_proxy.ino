/*
 * =====================================================================
 * FILE    : motor_y_level_proxy.ino
 * PERAN   : Forward motorlevel get/cfg ke master via UART1.
 *           Preferences hanya di master — Slave2 tidak simpan.
 * BOARD   : ESP32-S3 (Slave2 Arm)
 * =====================================================================
 */

#include "motor_y_level_proxy.h"
#include "serial.h"
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

namespace {

constexpr uint32_t MOTOR_Y_LEVEL_ACK_MS = 400;

char gMotorYLevelLastAck[32] = "idle";
bool gMotorYLevelQueryReady = false;
long gMotorYLevelQueryBuf[6] = {0, 300, 600, 900, 1200, 1500};

void motorYLevelSetAck(const char* msg) {
    strncpy(gMotorYLevelLastAck, msg, sizeof(gMotorYLevelLastAck) - 1);
    gMotorYLevelLastAck[sizeof(gMotorYLevelLastAck) - 1] = '\0';
}

void sendMotorLevelLine(const char* line) {
    gMotorYLevelQueryReady = false;
    motorYLevelSetAck("idle");
    masterUartSendLine(line);
}

// ponytail: spin tunggu ack — serialCommandTick di loop() core 1 yang baca UART
bool waitMotorLevelAck(uint32_t timeoutMs) {
    const uint32_t t0 = millis();
    while (millis() - t0 < timeoutMs) {
        if (gMotorYLevelQueryReady) return true;
        if (strcmp(gMotorYLevelLastAck, "idle") != 0) return true;
        vTaskDelay(1);
    }
    return gMotorYLevelQueryReady || strcmp(gMotorYLevelLastAck, "idle") != 0;
}

} // anonymous namespace

const char* motorYLevelLastAck() {
    return gMotorYLevelLastAck;
}

bool parseMasterMotorLevelLine(char* line) {
    if (strncmp(line, "motorlevel lvl ", 15) == 0) {
        char* p = line + 15;
        for (int i = 0; i < 6; i++) {
            while (*p == ' ') p++;
            if (*p == '\0') return false;
            gMotorYLevelQueryBuf[i] = atol(p);
            while (*p && *p != ' ') p++;
        }
        gMotorYLevelQueryReady = true;
        motorYLevelSetAck("ok");
        return true;
    }
    if (strcmp(line, "motorlevel ok") == 0) {
        motorYLevelSetAck("ok");
        return true;
    }
    if (strncmp(line, "motorlevel err ", 15) == 0) {
        motorYLevelSetAck(line + 15);
        return true;
    }
    return false;
}

bool motorYLevelQueryMaster(long out[6]) {
    sendMotorLevelLine("motorlevel get");
    if (!waitMotorLevelAck(MOTOR_Y_LEVEL_ACK_MS) || !gMotorYLevelQueryReady) {
        if (strcmp(gMotorYLevelLastAck, "idle") == 0) {
            motorYLevelSetAck("timeout");
        }
        return false;
    }
    for (int i = 0; i < 6; i++) {
        out[i] = gMotorYLevelQueryBuf[i];
    }
    return true;
}

bool motorYLevelSaveToMaster(const long levels[6]) {
    char buf[80];
    snprintf(buf, sizeof(buf),
             "motorlevel cfg %ld %ld %ld %ld %ld %ld",
             levels[0], levels[1], levels[2], levels[3], levels[4], levels[5]);
    sendMotorLevelLine(buf);
    if (!waitMotorLevelAck(MOTOR_Y_LEVEL_ACK_MS)) {
        motorYLevelSetAck("timeout");
        return false;
    }
    return strstr(gMotorYLevelLastAck, "ok") != nullptr;
}
