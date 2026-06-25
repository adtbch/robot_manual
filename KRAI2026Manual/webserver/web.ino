/*
 * =====================================================================
 * FILE    : web.ino
 * PERAN   : HTTP server — serve index.html dari PROGMEM + handle API.
 *
 * ISI FILE INI:
 *   - webServerInit()     Setup WiFi AP + HTTP routes + start server
 *   - webServerTick()     server.handleClient() — dipanggil di loop()
 *   - handleRoot()        Serve index.html dari PROGMEM
 *   - handleApiStatus()   GET  /api/status  → JSON status
 *   - handleApiSave()     POST /api/save    → per-section save, forward via ESP-NOW
 *   - handleApiConfig()   POST /api/config  → legacy full config (kept for import)
 *   - handleApiSerial()   POST /api/serial  → forward serial command ke master
 *   - handleApiEspnow()   GET  /api/espnow  → ESP-NOW status
 *   - handleNotFound()    404 handler
 *
 * ALIRAN DATA (per-section save):
 *   Phone → WiFi AP → HTTP POST /api/save {section, data}
 *   → handleApiSave() → espNowConfigSendJson()
 *   → Master parse section → apply to NVS / forward ke slave
 *
 * CATATAN:
 *   - index.html di-include sebagai PROGMEM (raw string literal)
 *   - WiFi AP dan ESP-NOW bisa jalan bareng (dual-mode)
 *   - CORS headers agar fetch() dari phone bisa jalan
 * =====================================================================
 */

#include "web.h"
#include "index.h"

WebServer server(80);

// =====================================================================
//  CORS HEADERS
// =====================================================================

static void sendCorsHeaders() {
    server.sendHeader("Access-Control-Allow-Origin", "*");
    server.sendHeader("Access-Control-Allow-Methods", "GET, POST, OPTIONS");
    server.sendHeader("Access-Control-Allow-Headers", "Content-Type");
}

// =====================================================================
//  HANDLERS
// =====================================================================

void handleRoot() {
    server.sendHeader("Content-Type", "text/html; charset=utf-8");
    server.send_P(200, "text/html", indexHtml);
}

void handleApiStatus() {
    sendCorsHeaders();

    // Build status JSON
    String json = "{";
    json += "\"espnow\":" + String(espNowConfigIsConnected() ? "true" : "false") + ",";
    json += "\"ap_ssid\":\"" + String(WIFI_AP_SSID) + "\",";
    json += "\"ap_ip\":\"" + WiFi.softAPIP().toString() + "\",";
    json += "\"master_mac\":\"";
    char macStr[18];
    snprintf(macStr, sizeof(macStr), "%02X:%02X:%02X:%02X:%02X:%02X",
             MASTER_MAC[0], MASTER_MAC[1], MASTER_MAC[2],
             MASTER_MAC[3], MASTER_MAC[4], MASTER_MAC[5]);
    json += macStr;
    json += "\",";
    json += "\"uptime_ms\":" + String(millis()) + ",";
    json += "\"free_heap\":" + String(ESP.getFreeHeap());
    json += "}";

    server.send(200, "application/json", json);
}

void handleApiConfig() {
    sendCorsHeaders();

    if (server.method() == HTTP_OPTIONS) {
        server.send(200);
        return;
    }

    if (server.method() != HTTP_POST) {
        server.send(405, "application/json", "{\"error\":\"POST only\"}");
        return;
    }

    String body = server.arg("plain");
    if (body.length() == 0) {
        server.send(400, "application/json", "{\"error\":\"empty body\"}");
        return;
    }

    Serial.printf("[WEB] Config diterima: %u bytes\n", (unsigned)body.length());

    // Forward via ESP-NOW ke master
    bool ok = espNowConfigSendJson(body);

    if (ok) {
        server.send(200, "application/json", "{\"ok\":true,\"bytes\":" + String(body.length()) + "}");
        Serial.println("[WEB] Config forwarding OK");
    } else {
        server.send(500, "application/json", "{\"ok\":false,\"error\":\"espnow_send_failed\"}");
        Serial.println("[WEB] Config forwarding GAGAL");
    }
}

void handleApiSerial() {
    sendCorsHeaders();

    if (server.method() == HTTP_OPTIONS) {
        server.send(200);
        return;
    }

    if (server.method() != HTTP_POST) {
        server.send(405, "application/json", "{\"error\":\"POST only\"}");
        return;
    }

    String body = server.arg("plain");
    if (body.length() == 0) {
        server.send(400, "application/json", "{\"error\":\"empty body\"}");
        return;
    }

    Serial.printf("[WEB] Serial command: %s\n", body.c_str());

    // Kirim ke master via ESP-NOW (serial relay packet)
    uint8_t packet[ESPNOW_HEADER_SIZE + ESPNOW_PAYLOAD_MAX];
    EspNowConfigHeader* hdr = reinterpret_cast<EspNowConfigHeader*>(packet);
    hdr->magic = ESPNOW_CONFIG_MAGIC;
    hdr->index = 0;
    hdr->total = 1;
    hdr->type = CONFIG_TYPE_SERIAL;

    size_t cmdLen = min(body.length(), (size_t)ESPNOW_PAYLOAD_MAX);
    memcpy(packet + ESPNOW_HEADER_SIZE, body.c_str(), cmdLen);

    bool ok = espNowConfigSendRaw(packet, ESPNOW_HEADER_SIZE + cmdLen);

    if (ok) {
        server.send(200, "text/plain", "OK");
    } else {
        server.send(500, "text/plain", "FAIL");
    }
}

void handleApiEspnow() {
    sendCorsHeaders();

    String json = "{";
    json += "\"connected\":" + String(espNowConfigIsConnected() ? "true" : "false");
    json += "}";

    server.send(200, "application/json", json);
}

// =====================================================================
//  PER-SECTION SAVE HANDLER
// =====================================================================

void handleApiSave() {
    sendCorsHeaders();

    if (server.method() == HTTP_OPTIONS) {
        server.send(200);
        return;
    }

    if (server.method() != HTTP_POST) {
        server.send(405, "application/json", "{\"error\":\"POST only\"}");
        return;
    }

    String body = server.arg("plain");
    if (body.length() == 0) {
        server.send(400, "application/json", "{\"error\":\"empty body\"}");
        return;
    }

    // Extract section name dari JSON
    // Contoh: {"section":"pid","data":{...}}
    int sectionIdx = body.indexOf("\"section\"");
    if (sectionIdx < 0) {
        server.send(400, "application/json", "{\"error\":\"missing section field\"}");
        return;
    }

    int colonIdx = body.indexOf(':', sectionIdx);
    int quoteStart = body.indexOf('"', colonIdx + 1);
    int quoteEnd = body.indexOf('"', quoteStart + 1);
    String section = body.substring(quoteStart + 1, quoteEnd);

    Serial.printf("[WEB] Save section: %s (%u bytes)\n", section.c_str(), (unsigned)body.length());

    // Forward via ESP-NOW ke master
    bool ok = espNowConfigSendJson(body);

    if (ok) {
        server.send(200, "application/json", "{\"ok\":true,\"section\":\"" + section + "\"}");
        Serial.printf("[WEB] Section '%s' forwarded OK\n", section.c_str());
    } else {
        server.send(500, "application/json", "{\"ok\":false,\"error\":\"espnow_send_failed\"}");
        Serial.printf("[WEB] Section '%s' forwarding GAGAL\n", section.c_str());
    }
}

void handleNotFound() {
    server.send(404, "text/plain", "404 Not Found");
}

// =====================================================================
//  PUBLIC API
// =====================================================================

void webServerInit() {
    // WiFi AP mode
    WiFi.mode(WIFI_AP);
    WiFi.softAP(WIFI_AP_SSID, WIFI_AP_PASS, WIFI_AP_CHANNEL, 0, WIFI_AP_MAX_CONN);

    IPAddress apIP = WiFi.softAPIP();
    Serial.printf("[WEB] WiFi AP: SSID=%s IP=%s\n", WIFI_AP_SSID, apIP.toString().c_str());

    // Setup HTTP routes
    server.on("/", HTTP_GET, handleRoot);
    server.on(API_STATUS, HTTP_GET, handleApiStatus);
    server.on("/api/save", HTTP_POST, handleApiSave);
    server.on("/api/save", HTTP_OPTIONS, handleApiSave);
    server.on(API_CONFIG, HTTP_POST, handleApiConfig);
    server.on(API_CONFIG, HTTP_OPTIONS, handleApiConfig);
    server.on(API_SERIAL, HTTP_POST, handleApiSerial);
    server.on(API_SERIAL, HTTP_OPTIONS, handleApiSerial);
    server.on(API_ESPNOW, HTTP_GET, handleApiEspnow);
    server.onNotFound(handleNotFound);

    // Start server
    server.begin();
    Serial.println("[WEB] HTTP server started");
}

void webServerTick() {
    server.handleClient();
}
