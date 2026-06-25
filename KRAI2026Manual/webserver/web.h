/*
 * =====================================================================
 * FILE    : web.h
 * PERAN   : Konfigurasi modul Web Server (HTTP).
 *
 * BOARD   : ESP32 (Web Server)
 * =====================================================================
 */

#ifndef WEB_H
#define WEB_H

#include "config.h"
#include <WebServer.h>

// =====================================================================
//  WEB SERVER INSTANCE
// =====================================================================

extern WebServer server;

// =====================================================================
//  HTTP HANDLER DECLARATIONS
// =====================================================================

void handleRoot();
void handleApiStatus();
void handleApiConfig();
void handleApiSerial();
void handleApiEspnow();
void handleNotFound();

#endif // WEB_H
