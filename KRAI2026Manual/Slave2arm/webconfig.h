/*
 * =====================================================================
 * FILE    : webconfig.h
 * PERAN   : WiFi AP + HTTP test panel untuk Slave2 Arm.
 * BOARD   : ESP32-S3 (Slave2 Arm)
 * =====================================================================
 */

#ifndef WEBCONFIG_H
#define WEBCONFIG_H

#include "config.h"
#include <WiFi.h>
#include <WebServer.h>

constexpr const char* WEB_AP_SSID = "KRAI_Slave2_Test";
constexpr const char* WEB_AP_PASS = "krai2026";

extern WebServer webServer;

void setupWebServer();
bool webHasClients();

#endif // WEBCONFIG_H
