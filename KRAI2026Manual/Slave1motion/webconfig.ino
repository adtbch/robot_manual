/*
 * =====================================================================
 * FILE    : webserver.ino
 * PERAN   : Local web configurator — PID tuning & auto-tune trigger.
 *           Runs on FreeRTOS core 0 task. WiFi AP sudah dari ota.ino.
 *
 * ENDPOINTS:
 *   GET  /              → HTML config page (PROGMEM)
 *   GET  /api/pid       → semua motor PID + yaw PID (JSON)
 *   POST /api/pid       → set motor PID {idx, kp, ki, kf, db}
 *   POST /api/yawpid    → set yaw PID {kp, ki, kd}
 *   POST /api/save      → save semua PID ke NVS
 *   POST /api/autotune  → trigger auto-tune {idx: 0-3 atau "all"}
 *   GET  /api/status    → robot status (yaw, encoder, rpm)
 *
 * ARCHITECTURE:
 *   Core 1 (main loop): motor, PID, IMU, encoder
 *   Core 0 (task):      WiFi stack + HTTP server
 *   Shared: pidStates[], pidKinematicYaw (read-only dari web, write jarang)
 *
 * CATATAN:
 *   - WiFi AP sudah di-init oleh ota.ino — jangan panggil WiFi.softAP lagi
 * =====================================================================
 */

#include "webconfig.h"
#include "pid.h"
#include "mpu.h"
#include "encoder.h"
#include "autoTuner.h"

// =====================================================================
//  STATE
// =====================================================================

WebServer server(80);
TaskHandle_t webTaskHandle = nullptr;

// HTML page — inline di PROGMEM
static const char indexHtml[] PROGMEM = R"rawliteral(
<!DOCTYPE html>
<html lang="en">
<head>
<meta charset="UTF-8">
<meta name="viewport" content="width=device-width,initial-scale=1">
<title>KRAI 2026 - Slave1 PID Config</title>
<style>
*{box-sizing:border-box;margin:0;padding:0}
body{font-family:system-ui,sans-serif;background:#0f172a;color:#e2e8f0;padding:12px;max-width:600px;margin:0 auto}
h1{font-size:1.2em;margin-bottom:12px;color:#38bdf8}
.card{background:#1e293b;border-radius:8px;padding:12px;margin-bottom:10px}
.card h2{font-size:0.95em;margin-bottom:8px;color:#94a3b8}
.row{display:flex;gap:6px;align-items:center;margin-bottom:4px}
.row label{width:28px;font-size:0.75em;color:#64748b;text-align:right}
.row input{flex:1;background:#0f172a;border:1px solid #334155;border-radius:4px;color:#e2e8f0;padding:4px 6px;font-size:0.85em;width:60px}
.row input:focus{outline:none;border-color:#38bdf8}
.btn{display:inline-block;border:none;border-radius:6px;padding:6px 12px;font-size:0.8em;cursor:pointer;font-weight:600}
.btn-blue{background:#2563eb;color:#fff}
.btn-green{background:#16a34a;color:#fff}
.btn-orange{background:#ea580c;color:#fff}
.btn-red{background:#dc2626;color:#fff}
.btn:active{opacity:0.8}
.status{font-size:0.75em;color:#64748b;margin-top:6px}
.motor-label{font-size:0.8em;color:#38bdf8;font-weight:600;margin-bottom:2px}
#log{background:#0f172a;border:1px solid #334155;border-radius:4px;padding:6px;font-size:0.7em;color:#94a3b8;max-height:120px;overflow-y:auto;margin-top:8px;font-family:monospace}
</style>
</head>
<body>
<h1>KRAI 2026 - Slave1 PID Config</h1>
<div id="motor-pids"></div>
<div class="card">
<h2>Yaw PID</h2>
<div class="row"><label>Kp</label><input id="ykp" type="number" step="0.01"><label>Ki</label><input id="yki" type="number" step="0.001"><label>Kd</label><input id="ykd" type="number" step="0.01"></div>
<div style="margin-top:6px">
<button class="btn btn-blue" onclick="saveYawPid()">Save Yaw</button>
<button class="btn btn-green" onclick="saveAll()">Save All to NVS</button>
</div>
</div>
<div class="card">
<h2>Auto-Tune</h2>
<div style="display:flex;gap:4px;flex-wrap:wrap">
<button class="btn btn-orange" onclick="autoTune(0)">Tune M0</button>
<button class="btn btn-orange" onclick="autoTune(1)">Tune M1</button>
<button class="btn btn-orange" onclick="autoTune(2)">Tune M2</button>
<button class="btn btn-orange" onclick="autoTune(3)">Tune M3</button>
<button class="btn btn-red" onclick="autoTune('all')">Tune ALL</button>
</div>
</div>
<div class="card">
<h2>Status</h2>
<div id="st" class="status">Loading...</div>
</div>
<div id="log"></div>
<script>
const M=['FR','FL','BR','BL'];
function log(m){const d=document.getElementById('log');d.innerHTML+=new Date().toLocaleTimeString()+' '+m+'<br>';d.scrollTop=d.scrollHeight;}
function buildCards(){const c=document.getElementById('motor-pids');let h='';for(let i=0;i<4;i++){h+='<div class="card"><div class="motor-label">Motor '+i+' ('+M[i]+')</div>';h+='<div class="row"><label>Kp</label><input id="m'+i+'kp" type="number" step="0.1"><label>Ki</label><input id="m'+i+'ki" type="number" step="0.01"><label>Kf</label><input id="m'+i+'kf" type="number" step="0.01"></div>';h+='<div class="row"><label>Db</label><input id="m'+i+'db" type="number" step="1"></div>';h+='<div style="margin-top:4px"><button class="btn btn-blue" onclick="saveM('+i+')">Save M'+i+'</button></div></div>';}c.innerHTML=h;}
function loadPid(){fetch('/api/pid').then(r=>r.json()).then(d=>{for(let i=0;i<4;i++){const p=d.motors[i]||{};document.getElementById('m'+i+'kp').value=p.kp||0;document.getElementById('m'+i+'ki').value=p.ki||0;document.getElementById('m'+i+'kf').value=p.kf||0;document.getElementById('m'+i+'db').value=p.deadband||0;}document.getElementById('ykp').value=d.yaw?.kp||0;document.getElementById('yki').value=d.yaw?.ki||0;document.getElementById('ykd').value=d.yaw?.kd||0;log('PID loaded');}).catch(e=>log('ERR: '+e));}
function saveM(i){const b=JSON.stringify({idx:i,kp:parseFloat(document.getElementById('m'+i+'kp').value),ki:parseFloat(document.getElementById('m'+i+'ki').value),kf:parseFloat(document.getElementById('m'+i+'kf').value),db:parseFloat(document.getElementById('m'+i+'db').value)});fetch('/api/pid',{method:'POST',headers:{'Content-Type':'application/json'},body:b}).then(r=>r.json()).then(d=>log('M'+i+': '+d.ok)).catch(e=>log('ERR: '+e));}
function saveYawPid(){const b=JSON.stringify({kp:parseFloat(document.getElementById('ykp').value),ki:parseFloat(document.getElementById('yki').value),kd:parseFloat(document.getElementById('ykd').value)});fetch('/api/yawpid',{method:'POST',headers:{'Content-Type':'application/json'},body:b}).then(r=>r.json()).then(d=>log('Yaw: '+d.ok)).catch(e=>log('ERR: '+e));}
function saveAll(){saveM(0);saveM(1);saveM(2);saveM(3);saveYawPid();setTimeout(()=>fetch('/api/save',{method:'POST'}),200);log('All saved to NVS');}
function autoTune(i){if(!confirm('Auto-tune motor '+i+'? Robot harus diam.'))return;fetch('/api/autotune',{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify({idx:i})}).then(r=>r.json()).then(d=>log('AT: '+d.ok)).catch(e=>log('ERR: '+e));}
function loadStatus(){fetch('/api/status').then(r=>r.json()).then(d=>{let s='Yaw: '+Number(d.yaw||0).toFixed(1)+' | Enc: '+(d.enc||[]).join(', ')+' | RPM: '+(d.rpm||[]).join(', ');document.getElementById('st').textContent=s;}).catch(()=>{});}
buildCards();loadPid();setInterval(loadStatus,500);
</script>
</body>
</html>
)rawliteral";

// =====================================================================
//  HANDLERS
// =====================================================================

static void handleRoot() {
    server.sendHeader("Content-Type", "text/html; charset=utf-8");
    server.send_P(200, "text/html", indexHtml);
}

static void handleApiPidGet() {
    String json = "{\"motors\":[";
    for (int i = 0; i < 4; i++) {
        if (i > 0) json += ",";
        json += "{\"kp\":" + String(pidStates[i].kp, 3);
        json += ",\"ki\":" + String(pidStates[i].ki, 3);
        json += ",\"kf\":" + String(pidStates[i].kf, 3);
        json += ",\"deadband\":" + String(pidStates[i].deadband, 1) + "}";
    }
    json += "],\"yaw\":{\"kp\":" + String(pidKinematicYaw.kp, 3);
    json += ",\"ki\":" + String(pidKinematicYaw.ki, 3);
    json += ",\"kd\":" + String(pidKinematicYaw.kd, 3) + "}}";
    server.send(200, "application/json", json);
}

static void handleApiPidPost() {
    if (server.method() != HTTP_POST) {
        server.send(405, "application/json", "{\"error\":\"POST only\"}");
        return;
    }
    String body = server.arg("plain");
    int idx = -1;
    float kp = 0, ki = 0, kf = 0, db = 0;

    int idxIdx = body.indexOf("\"idx\"");
    if (idxIdx >= 0) idx = body.substring(idxIdx + 5).toInt();
    int kpIdx = body.indexOf("\"kp\"");
    if (kpIdx >= 0) kp = body.substring(kpIdx + 4).toFloat();
    int kiIdx = body.indexOf("\"ki\"");
    if (kiIdx >= 0) ki = body.substring(kiIdx + 4).toFloat();
    int kfIdx = body.indexOf("\"kf\"");
    if (kfIdx >= 0) kf = body.substring(kfIdx + 4).toFloat();
    int dbIdx = body.indexOf("\"db\"");
    if (dbIdx >= 0) db = body.substring(dbIdx + 4).toFloat();

    if (idx < 0 || idx > 3) {
        server.send(400, "application/json", "{\"error\":\"invalid idx\"}");
        return;
    }
    pidSetGains(idx, kp, ki, kf, db);
    Serial.printf("[WEB] Motor %d PID: Kp=%.3f Ki=%.3f Kf=%.3f Db=%.1f\n", idx, kp, ki, kf, db);
    server.send(200, "application/json", "{\"ok\":true}");
}

static void handleApiYawPid() {
    if (server.method() != HTTP_POST) {
        server.send(405, "application/json", "{\"error\":\"POST only\"}");
        return;
    }
    String body = server.arg("plain");
    float kp = 0, ki = 0, kd = 0;

    int kpIdx = body.indexOf("\"kp\"");
    if (kpIdx >= 0) kp = body.substring(kpIdx + 4).toFloat();
    int kiIdx = body.indexOf("\"ki\"");
    if (kiIdx >= 0) ki = body.substring(kiIdx + 4).toFloat();
    int kdIdx = body.indexOf("\"kd\"");
    if (kdIdx >= 0) kd = body.substring(kdIdx + 4).toFloat();

    pidKinematicYaw.kp = kp;
    pidKinematicYaw.ki = ki;
    pidKinematicYaw.kd = kd;
    pidKinematicYaw.reset();
    Serial.printf("[WEB] Yaw PID: Kp=%.3f Ki=%.3f Kd=%.3f\n", kp, ki, kd);
    server.send(200, "application/json", "{\"ok\":true}");
}

static void handleApiSave() {
    for (int i = 0; i < 4; i++) {
        pidSaveToNVS(i, pidStates[i].kp, pidStates[i].ki, pidStates[i].kf, pidStates[i].deadband);
    }
    saveYawPid();
    Serial.println("[WEB] All PID saved to NVS");
    server.send(200, "application/json", "{\"ok\":true}");
}

static void handleApiAutotune() {
    if (server.method() != HTTP_POST) {
        server.send(405, "application/json", "{\"error\":\"POST only\"}");
        return;
    }
    if (isAutoTunerRunning()) {
        server.send(409, "application/json", "{\"error\":\"already running\"}");
        return;
    }
    String body = server.arg("plain");
    int idx = -1;

    int idxIdx = body.indexOf("\"idx\"");
    if (idxIdx >= 0) {
        int q1 = body.indexOf('"', idxIdx + 5);
        int q2 = body.indexOf('"', q1 + 1);
        String val = body.substring(q1 + 1, q2);
        if (val == "all") {
            startAutoTuneAll();
            server.send(200, "application/json", "{\"ok\":true,\"target\":\"all\"}");
            Serial.println("[WEB] Auto-tune ALL started");
            return;
        }
        idx = val.toInt();
    }

    if (idx < 0 || idx > 3) {
        server.send(400, "application/json", "{\"error\":\"invalid idx\"}");
        return;
    }
    startAutoTune(idx);
    Serial.printf("[WEB] Auto-tune motor %d started\n", idx);
    server.send(200, "application/json", "{\"ok\":true,\"target\":" + String(idx) + "}");
}

static void handleApiStatus() {
    float yawVal = getYaw();
    String json = "{\"yaw\":" + String(yawVal, 1);
    json += ",\"enc\":[";
    for (int i = 0; i < 4; i++) {
        if (i > 0) json += ",";
        json += String((int32_t)getExtEncoderCount(i));
    }
    json += "],\"rpm\":[";
    for (int i = 0; i < 4; i++) {
        if (i > 0) json += ",";
        json += String(getEncoderVelocityRpm(i), 1);
    }
    json += "],\"uptime\":" + String(millis());
    json += ",\"autotune\":" + String(isAutoTunerRunning() ? "true" : "false");
    json += "}";
    server.send(200, "application/json", json);
}

static void handleNotFound() {
    server.send(404, "text/plain", "404");
}

// =====================================================================
//  FREE_RTOS TASK — CORE 0
// =====================================================================

static void webTask(void* param) {
    for (;;) {
        server.handleClient();
        vTaskDelay(1);
    }
}

// =====================================================================
//  PUBLIC API
// =====================================================================

void setupWebServer() {
    server.on("/", HTTP_GET, handleRoot);
    server.on("/api/pid", HTTP_GET, handleApiPidGet);
    server.on("/api/pid", HTTP_POST, handleApiPidPost);
    server.on("/api/yawpid", HTTP_POST, handleApiYawPid);
    server.on("/api/save", HTTP_POST, handleApiSave);
    server.on("/api/autotune", HTTP_POST, handleApiAutotune);
    server.on("/api/status", HTTP_GET, handleApiStatus);
    server.onNotFound(handleNotFound);

    server.begin();
    Serial.println("[WEB] HTTP server started on port 80");

    xTaskCreatePinnedToCore(
        webTask, "webTask", 8192, nullptr, 1, &webTaskHandle, 0
    );
    Serial.println("[WEB] Task pinned to core 0");
}

void webServerTick() {
    // Task handles it on core 0 — kept for API compat
}
