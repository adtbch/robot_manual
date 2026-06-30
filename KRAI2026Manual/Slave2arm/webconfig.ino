/*
 * =====================================================================
 * FILE    : webconfig.ino
 * PERAN   : WiFi AP + HTTP test panel — motor PWM, sensor read, pneu.
 *           FreeRTOS task core 0. Sensor read hanya saat HTTP dipanggil.
 * BOARD   : ESP32-S3 (Slave2 Arm)
 * =====================================================================
 */

#include "webconfig.h"
#include "webpage.h"
#include "motor.h"
#include "encoder.h"
#include "limit_switch.h"
#include "proximity.h"
#include "pneumatic.h"

WebServer webServer(80);
TaskHandle_t webTaskHandle = nullptr;

namespace {

float getJsonFloat(const String& json, const String& key) {
    const String search = "\"" + key + "\"";
    const int idx = json.indexOf(search);
    if (idx < 0) return 0.0f;
    const int colon = json.indexOf(':', idx);
    if (colon < 0) return 0.0f;
    return json.substring(colon + 1).toFloat();
}

int getJsonInt(const String& json, const String& key) {
    return (int)getJsonFloat(json, key);
}

char getJsonChar(const String& json, const String& key) {
    const String search = "\"" + key + "\"";
    const int idx = json.indexOf(search);
    if (idx < 0) return 0;
    const int q1 = json.indexOf('"', json.indexOf(':', idx) + 1);
    if (q1 < 0) return 0;
    const int q2 = json.indexOf('"', q1 + 1);
    if (q2 <= q1 + 1) return 0;
    return json.charAt(q1 + 1);
}

String getJsonString(const String& json, const String& key) {
    const String search = "\"" + key + "\"";
    const int idx = json.indexOf(search);
    if (idx < 0) return "";
    const int q1 = json.indexOf('"', json.indexOf(':', idx) + 1);
    if (q1 < 0) return "";
    const int q2 = json.indexOf('"', q1 + 1);
    if (q2 <= q1 + 1) return "";
    return json.substring(q1 + 1, q2);
}

void sendNoClient() {
    webServer.send(503, "application/json", "{\"error\":\"no wifi client\"}");
}

bool requireClient() {
    if (!webHasClients()) {
        sendNoClient();
        return false;
    }
    return true;
}

void handleRoot() {
    webServer.sendHeader("Content-Type", "text/html; charset=utf-8");
    webServer.send_P(200, "text/html", indexHtml);
}

void handleApiStatus() {
    if (!requireClient()) return;

    const String json = String("{")
        + "\"limit\":{\"depan\":" + String(readLimitSwitch(LIMIT_ARMBOX_DEPAN) ? "true" : "false")
        + ",\"belakang\":" + String(readLimitSwitch(LIMIT_ARMBOX_BELAKANG) ? "true" : "false")
        + ",\"turun\":" + String(readLimitSwitch(LIMIT_ARMBOX_TURUN) ? "true" : "false") + "}"
        + ",\"prox\":{\"r\":" + String(readProximity('r') ? "true" : "false")
        + ",\"l\":" + String(readProximity('l') ? "true" : "false") + "}"
        + ",\"pne\":{\"r\":" + String(pneumaticState('r') ? "true" : "false")
        + ",\"l\":" + String(pneumaticState('l') ? "true" : "false") + "}"
        + ",\"motorY\":{\"target\":" + String(motorYGetTarget())
        + ",\"active\":" + String(motorYIsActive() ? "true" : "false") + "}"
        + ",\"motorRun\":{\"x\":" + String(motorRunIsActive('x') ? "true" : "false")
        + ",\"k\":" + String(motorRunIsActive('k') ? "true" : "false") + "}"
        + ",\"motorPwm\":{\"x\":" + String(motorRunGetPwm('x'))
        + ",\"k\":" + String(motorRunGetPwm('k'))
        + ",\"y\":" + String(motorYGetLastPwm()) + "}"
        + ",\"clients\":" + String(WiFi.softAPgetStationNum())
        + ",\"heap\":" + String(ESP.getFreeHeap())
        + "}";
    webServer.send(200, "application/json", json);
}

void handleApiEnc() {
    if (!requireClient()) return;
    const String json = String("{\"x\":") + String(getEncoderCount('x'))
        + ",\"y\":" + String(getEncoderCount('y'))
        + ",\"k\":null}";
    webServer.send(200, "application/json", json);
}

void handleApiMotor() {
    if (webServer.method() != HTTP_POST) {
        webServer.send(405, "application/json", "{\"error\":\"POST only\"}");
        return;
    }
    const String body = webServer.arg("plain");
    const char id = getJsonChar(body, "id");
    const int pwm = getJsonInt(body, "pwm");
    if (id == 0) {
        webServer.send(400, "application/json", "{\"ok\":false,\"error\":\"bad id\"}");
        return;
    }
    if (!executeMotorCommand(id, pwm)) {
        webServer.send(400, "application/json", "{\"ok\":false,\"error\":\"bad id\"}");
        return;
    }
    Serial.printf("[WEB] motor %c pwm=%d\n", id, pwm);
    webServer.send(200, "application/json",
        String("{\"ok\":true,\"id\":\"") + String(id) + "\",\"pwm\":" + String(pwm) + "}");
}

void handleApiMotorStop() {
    motorStopAll();
    Serial.println("[WEB] motorstop");
    webServer.send(200, "application/json", "{\"ok\":true}");
}

void handleApiMotorRunStop() {
    motorRunStopAll();
    Serial.println("[WEB] motorrunstop");
    webServer.send(200, "application/json", "{\"ok\":true}");
}

void handleApiMotorTarget() {
    if (webServer.method() != HTTP_POST) {
        webServer.send(405, "application/json", "{\"error\":\"POST only\"}");
        return;
    }
    const long enc = (long)getJsonInt(webServer.arg("plain"), "enc");
    motorYSetTarget(enc);
    Serial.printf("[WEB] motortarget %ld\n", enc);
    webServer.send(200, "application/json", "{\"ok\":true}");
}

void handleApiMotorTargetStop() {
    motorYStop();
    Serial.println("[WEB] motortargetstop");
    webServer.send(200, "application/json", "{\"ok\":true}");
}

void handleApiPne() {
    if (webServer.method() != HTTP_POST) {
        webServer.send(405, "application/json", "{\"error\":\"POST only\"}");
        return;
    }
    const String body = webServer.arg("plain");
    const char side = getJsonChar(body, "side");
    const String state = getJsonString(body, "state");
    if (side != 'r' && side != 'l') {
        webServer.send(400, "application/json", "{\"ok\":false}");
        return;
    }
    if (state == "on") pneumaticOn(side);
    else if (state == "off") pneumaticOff(side);
    else pneumaticToggle(side);
    Serial.printf("[WEB] pne %c %s\n", side, state.c_str());
    webServer.send(200, "application/json", "{\"ok\":true}");
}

void handleApiPneAll() {
    pneumaticAllOff();
    Serial.println("[WEB] pneall");
    webServer.send(200, "application/json", "{\"ok\":true}");
}

void handleApiEncReset() {
    for (size_t i = 0; i < ENCODER_COUNT; i++) {
        resetEncoderCount((char)i);
    }
    Serial.println("[WEB] encreset");
    webServer.send(200, "application/json", "{\"ok\":true}");
}

void handleApiStop() {
    motorStopAll();
    pneumaticAllOff();
    Serial.println("[WEB] STOP");
    webServer.send(200, "application/json", "{\"ok\":true}");
}

void handleNotFound() {
    webServer.send(404, "text/plain", "404");
}

void webTask(void* param) {
    for (;;) {
        webServer.handleClient();
        vTaskDelay(1);
    }
}

} // anonymous namespace

bool webHasClients() {
    return WiFi.softAPgetStationNum() > 0;
}

void setupWebServer() {
    WiFi.mode(WIFI_AP);
    WiFi.softAP(WEB_AP_SSID, WEB_AP_PASS);

    Serial.print("[WEB] AP IP: ");
    Serial.println(WiFi.softAPIP());
    Serial.printf("[WEB] SSID: %s  pass: %s\n", WEB_AP_SSID, WEB_AP_PASS);

    webServer.on("/", HTTP_GET, handleRoot);
    webServer.on("/api/status", HTTP_GET, handleApiStatus);
    webServer.on("/api/enc", HTTP_GET, handleApiEnc);
    webServer.on("/api/motor", HTTP_POST, handleApiMotor);
    webServer.on("/api/motorstop", HTTP_POST, handleApiMotorStop);
    webServer.on("/api/motorrunstop", HTTP_POST, handleApiMotorRunStop);
    webServer.on("/api/motortarget", HTTP_POST, handleApiMotorTarget);
    webServer.on("/api/motortargetstop", HTTP_POST, handleApiMotorTargetStop);
    webServer.on("/api/pne", HTTP_POST, handleApiPne);
    webServer.on("/api/pneall", HTTP_POST, handleApiPneAll);
    webServer.on("/api/encreset", HTTP_POST, handleApiEncReset);
    webServer.on("/api/stop", HTTP_POST, handleApiStop);
    webServer.onNotFound(handleNotFound);

    webServer.begin();
    Serial.println("[WEB] HTTP server port 80");

    xTaskCreatePinnedToCore(
        webTask, "webTask", 8192, nullptr, 1, &webTaskHandle, 0
    );
}
