/*
 * =====================================================================
 * FILE    : webserver.h
 * PERAN   : Local web configurator — PID tuning & auto-tune trigger.
 *           Runs on core 0 via FreeRTOS task.
 *
 * BOARD   : ESP32-S3 (Slave1 Motion)
 * =====================================================================
 */

#ifndef WEBCONFIG_H
#define WEBCONFIG_H

#include "config.h"
#include <WiFi.h>
#include <WebServer.h>

extern WebServer server;

extern bool testYawMode;
extern int testYawTarget;

void setupWebServer();
void webServerTick();

#endif // WEBCONFIG_H
