/*
 * =====================================================================
 * FILE    : forest_config.ino
 * PERAN   : Forward forest * ke master via UART1. NVS hanya di master.
 * BOARD   : ESP32-S3 (Slave2 Arm)
 * =====================================================================
 */

#include "forest_config.h"
#include "serial.h"
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

namespace {

constexpr uint32_t FOREST_ACK_MS = 400;

char    gForestLastAck[32]  = "idle";
uint8_t gForestDest1Cache   = 4;
uint8_t gForestDest2Cache   = 6;
bool    gForestDest1DoneLocal = false;
bool    gForestQueryReady   = false;

void forestSetAck(const char* msg) {
    strncpy(gForestLastAck, msg, sizeof(gForestLastAck) - 1);
    gForestLastAck[sizeof(gForestLastAck) - 1] = '\0';
}

void sendForestLine(const char* line) {
    gForestQueryReady = false;
    forestSetAck("idle");
    masterUartSendLine(line);
    Serial.printf("[Forest TX] %s\n", line);
}

// ponytail: spin tunggu ack — serialCommandTick di loop() core 1 yang baca UART
bool waitForestAck(uint32_t timeoutMs) {
    const uint32_t t0 = millis();
    while (millis() - t0 < timeoutMs) {
        if (gForestQueryReady) return true;
        if (strcmp(gForestLastAck, "idle") != 0) return true;
        vTaskDelay(1);
    }
    return gForestQueryReady || strcmp(gForestLastAck, "idle") != 0;
}

} // anonymous namespace

uint8_t forestGetDest1() { return gForestDest1Cache; }
uint8_t forestGetDest2() { return gForestDest2Cache; }
bool forestIsDest1DoneLocal() { return gForestDest1DoneLocal; }
const char* forestLastAck() { return gForestLastAck; }

bool parseMasterForestLine(char* line) {
    if (strncmp(line, "forest dest ", 12) == 0) {
        char* p = line + 12;
        gForestDest1Cache = (uint8_t)atoi(p);
        while (*p && *p != ' ') p++;
        while (*p == ' ') p++;
        gForestDest2Cache = (uint8_t)atoi(p);
        while (*p && *p != ' ') p++;
        while (*p == ' ') p++;
        gForestDest1DoneLocal = atoi(p) != 0;
        gForestQueryReady = true;
        forestSetAck("ok");
        return true;
    }
    if (strncmp(line, "forest evt ", 11) == 0) {
        if (strstr(line, "dest1_done") != nullptr) {
            gForestDest1DoneLocal = true;
            forestSetAck("dest1_done");
        } else if (strstr(line, "dest1_reset") != nullptr) {
            gForestDest1DoneLocal = false;
            forestSetAck("dest1_reset");
        }
        return true;
    }
    if (strcmp(line, "forest ok") == 0) {
        forestSetAck("ok");
        return true;
    }
    if (strncmp(line, "forest err ", 11) == 0) {
        forestSetAck(line + 11);
        return true;
    }
    return false;
}

bool forestQueryDestFromMaster() {
    sendForestLine("forest get");
    if (!waitForestAck(FOREST_ACK_MS) || !gForestQueryReady) {
        if (strcmp(gForestLastAck, "idle") == 0) {
            forestSetAck("timeout");
        }
        return false;
    }
    return true;
}

bool forestSaveDestinations(uint8_t d1, uint8_t d2) {
    char buf[32];
    snprintf(buf, sizeof(buf), "forest cfg %u %u", d1, d2);
    sendForestLine(buf);
    if (!waitForestAck(FOREST_ACK_MS)) {
        forestSetAck("timeout");
        return false;
    }
    if (strstr(gForestLastAck, "ok") == nullptr) {
        return false;
    }
    gForestDest1Cache = d1;
    gForestDest2Cache = d2;
    gForestDest1DoneLocal = false;
    return true;
}

bool forestSendGoto(uint8_t slot) {
    if (slot != 1 && slot != 2) return false;
    if (slot == 2 && !gForestDest1DoneLocal) {
        forestSetAck("dest1_not_done");
        return false;
    }
    char buf[24];
    snprintf(buf, sizeof(buf), "forest goto %u", slot);
    sendForestLine(buf);
    if (!waitForestAck(FOREST_ACK_MS)) {
        forestSetAck("timeout");
        return false;
    }
    return strstr(gForestLastAck, "ok") != nullptr;
}

void forestSendExit() {
    sendForestLine("forest exit");
    waitForestAck(FOREST_ACK_MS);
}

void forestSendCancel() {
    sendForestLine("forest cancel");
    gForestDest1DoneLocal = false;
    waitForestAck(FOREST_ACK_MS);
}
