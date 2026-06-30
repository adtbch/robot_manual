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
#include "motor.h"
#include "encoder.h"
#include "autoTuner.h"
#include "kinematik.h"
#include "waypoint.h"

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
.btn-purple{background:#9333ea;color:#fff}
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
<div style="margin-top:10px">
<h2 style="font-size:0.8em;color:#cbd5e1">Test Yaw Target</h2>
<div style="display:flex;gap:4px;flex-wrap:wrap">
<button class="btn btn-purple" onclick="testYaw(0)">0°</button>
<button class="btn btn-purple" onclick="testYaw(90)">90°</button>
<button class="btn btn-purple" onclick="testYaw(-90)">-90°</button>
<button class="btn btn-purple" onclick="testYaw(180)">180°</button>
<button class="btn btn-red" onclick="testYaw(null)">STOP</button>
</div>
</div>
</div>
<div class="card">
<h2>Gravity FF (tanjakan)</h2>
<p class="status">Kg &times; sin(roll). S di OLED: maju nanjak +, mundur nanjak &minus;. Range 0&ndash;2500.</p>
<div class="row"><label>Kg</label><input id="motorKg" type="number" step="10" min="0" max="2500" style="flex:2"></div>
<div style="margin-top:6px">
<button class="btn btn-blue" onclick="saveGravity()">Save Kg</button>
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
<h2>Waypoint</h2>
<div class="row"><label>Kp</label><input id="wpKp" type="number" step="1"><label>Tol(cm)</label><input id="wpTol" type="number" step="0.5"><label>TolYaw</label><input id="wpTolYaw" type="number" step="0.5"></div>
<div style="display:flex;gap:4px;margin-top:6px">
<button class="btn btn-blue" onclick="saveWp()">Save WP</button>
<button class="btn btn-green" onclick="wpStatus()">Status</button>
</div>
<div style="margin-top:10px">
<h2 style="font-size:0.8em;color:#cbd5e1">Test Waypoint</h2>
<div style="display:flex;gap:4px;flex-wrap:wrap">
<input id="wpX" type="number" placeholder="X cm" value="0" style="width:60px;background:#0f172a;border:1px solid #334155;border-radius:4px;color:#e2e8f0;padding:4px">
<input id="wpY" type="number" placeholder="Y cm" value="0" style="width:60px;background:#0f172a;border:1px solid #334155;border-radius:4px;color:#e2e8f0;padding:4px">
<input id="wpYaw" type="number" placeholder="Yaw deg" value="0" style="width:60px;background:#0f172a;border:1px solid #334155;border-radius:4px;color:#e2e8f0;padding:4px">
<input id="wpSpeed" type="number" placeholder="RPM" value="300" style="width:60px;background:#0f172a;border:1px solid #334155;border-radius:4px;color:#e2e8f0;padding:4px">
<button class="btn btn-green" onclick="wpGo()">Go</button>
<button class="btn btn-red" onclick="wpStop()">Stop</button>
</div>
<div style="margin-top:10px">
<h2 style="font-size:0.8em;color:#cbd5e1">Combo P1 → P2</h2>
<div class="row" style="margin-bottom:2px"><span style="font-size:0.75em;color:#64748b;width:24px">P1</span><input id="wp1X" type="number" placeholder="X cm" value="0" style="width:55px"><input id="wp1Y" type="number" placeholder="Y cm" value="0" style="width:55px"><input id="wp1Yaw" type="number" placeholder="Yaw" value="0" style="width:55px"></div>
<div class="row"><span style="font-size:0.75em;color:#64748b;width:24px">P2</span><input id="wp2X" type="number" placeholder="X cm" value="100" style="width:55px"><input id="wp2Y" type="number" placeholder="Y cm" value="50" style="width:55px"><input id="wp2Yaw" type="number" placeholder="Yaw" value="90" style="width:55px"></div>
<div style="display:flex;gap:4px;flex-wrap:wrap;margin-top:4px;align-items:center">
<input id="wpComboSpeed" type="number" placeholder="RPM" value="300" style="width:60px;background:#0f172a;border:1px solid #334155;border-radius:4px;color:#e2e8f0;padding:4px">
<button class="btn btn-green" onclick="wpComboGo()">Combo Go</button>
</div>
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
function loadPid(){fetch('/api/pid').then(r=>r.json()).then(d=>{for(let i=0;i<4;i++){const p=d.motors[i]||{};document.getElementById('m'+i+'kp').value=p.kp||0;document.getElementById('m'+i+'ki').value=p.ki||0;document.getElementById('m'+i+'kf').value=p.kf||0;document.getElementById('m'+i+'db').value=p.deadband||0;}document.getElementById('ykp').value=d.yaw?.kp||0;document.getElementById('yki').value=d.yaw?.ki||0;document.getElementById('ykd').value=d.yaw?.kd||0;document.getElementById('motorKg').value=d.gravity?.kg||0;if(d.waypoint){document.getElementById('wpKp').value=d.waypoint.kp||0;document.getElementById('wpTol').value=d.waypoint.tol_pos||0;document.getElementById('wpTolYaw').value=d.waypoint.tol_yaw||0;}log('PID loaded');}).catch(e=>log('ERR: '+e));}
function saveGravity(){const b=JSON.stringify({kg:parseFloat(document.getElementById('motorKg').value)||0});fetch('/api/gravity',{method:'POST',headers:{'Content-Type':'application/json'},body:b}).then(r=>r.json()).then(d=>log('Kg: '+d.ok)).catch(e=>log('ERR: '+e));}
function saveM(i){const b=JSON.stringify({idx:i,kp:parseFloat(document.getElementById('m'+i+'kp').value),ki:parseFloat(document.getElementById('m'+i+'ki').value),kf:parseFloat(document.getElementById('m'+i+'kf').value),db:parseFloat(document.getElementById('m'+i+'db').value)});fetch('/api/pid',{method:'POST',headers:{'Content-Type':'application/json'},body:b}).then(r=>r.json()).then(d=>log('M'+i+': '+d.ok)).catch(e=>log('ERR: '+e));}
function saveYawPid(){const b=JSON.stringify({kp:parseFloat(document.getElementById('ykp').value),ki:parseFloat(document.getElementById('yki').value),kd:parseFloat(document.getElementById('ykd').value)});fetch('/api/yawpid',{method:'POST',headers:{'Content-Type':'application/json'},body:b}).then(r=>r.json()).then(d=>log('Yaw: '+d.ok)).catch(e=>log('ERR: '+e));}
function saveAll(){saveM(0);saveM(1);saveM(2);saveM(3);saveYawPid();saveGravity();setTimeout(()=>fetch('/api/save',{method:'POST'}),200);log('All saved to NVS');}
function testYaw(target){const b=JSON.stringify(target===null?{stop:true}:{stop:false,target:target});fetch('/api/testyaw',{method:'POST',headers:{'Content-Type':'application/json'},body:b}).then(r=>r.json()).then(d=>log(target===null?'Test Yaw STOP':'Test Yaw Target: '+target+'°')).catch(e=>log('ERR: '+e));}
function autoTune(i){if(!confirm('Auto-tune motor '+i+'? Robot harus diam.'))return;fetch('/api/autotune',{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify({idx:i})}).then(r=>r.json()).then(d=>log('AT: '+d.ok)).catch(e=>log('ERR: '+e));}
function saveWp(){const b=JSON.stringify({kp:parseFloat(document.getElementById('wpKp').value)||200,tol_pos:parseFloat(document.getElementById('wpTol').value)||5,tol_yaw:parseFloat(document.getElementById('wpTolYaw').value)||3});fetch('/api/waypoint',{method:'POST',headers:{'Content-Type':'application/json'},body:b}).then(r=>r.json()).then(d=>log('WP: '+d.ok)).catch(e=>log('ERR: '+e));}
function wpGo(){const x=parseFloat(document.getElementById('wpX').value)||0,y=parseFloat(document.getElementById('wpY').value)||0,yaw=parseFloat(document.getElementById('wpYaw').value)||0,spd=parseFloat(document.getElementById('wpSpeed').value)||300;fetch('/api/wpgoto',{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify({x:x,y:y,yaw:yaw,max_speed:spd})}).then(r=>r.json()).then(d=>log('WP Go: '+d.ok)).catch(e=>log('ERR: '+e));}
function wpComboGo(){const spd=parseFloat(document.getElementById('wpComboSpeed').value)||300;const b=JSON.stringify({x1:parseFloat(document.getElementById('wp1X').value)||0,y1:parseFloat(document.getElementById('wp1Y').value)||0,yaw1:parseFloat(document.getElementById('wp1Yaw').value)||0,x2:parseFloat(document.getElementById('wp2X').value)||0,y2:parseFloat(document.getElementById('wp2Y').value)||0,yaw2:parseFloat(document.getElementById('wp2Yaw').value)||0,max_speed:spd});fetch('/api/wpcombo',{method:'POST',headers:{'Content-Type':'application/json'},body:b}).then(r=>r.json()).then(d=>log('WP Combo: '+d.ok)).catch(e=>log('ERR: '+e));}
function wpStop(){fetch('/api/wpstop',{method:'POST'}).then(r=>r.json()).then(d=>log('WP Stop: '+d.ok)).catch(e=>log('ERR: '+e));}
function wpStatus(){fetch('/api/wpstatus').then(r=>r.json()).then(d=>{let m='WP: state='+d.state+' target=('+d.target_x+','+d.target_y+') yaw='+d.target_yaw+'°';if(d.combo_step)m+=' combo step='+d.combo_step;log(m);}).catch(e=>log('ERR: '+e));}
function loadStatus(){fetch('/api/status').then(r=>r.json()).then(d=>{let s='Yaw: '+Number(d.yaw||0).toFixed(1)+' S: '+Number(d.slope||0).toFixed(0)+' | Enc: '+(d.enc||[]).join(', ')+' | RPM: '+(d.rpm||[]).join(', ');document.getElementById('st').textContent=s;}).catch(()=>{});}
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
    json += ",\"kd\":" + String(pidKinematicYaw.kd, 3) + "}";
    json += ",\"waypoint\":{\"kp\":" + String(wpKpXY, 1);
    json += ",\"tol_pos\":" + String(wpTolPos_m * 100.0f, 1);  // m → cm
    json += ",\"tol_yaw\":" + String(wpTolYaw_deg, 1) + "}";
    json += ",\"gravity\":{\"kg\":" + String(motorKg, 1) + "}";
    json += "}";
    server.send(200, "application/json", json);
}

// Helper untuk parsing JSON secara manual
static float getJsonFloat(const String& json, const String& key) {
    String search = "\"" + key + "\"";
    int idx = json.indexOf(search);
    if (idx < 0) return 0.0f;
    
    int colonIdx = json.indexOf(':', idx + search.length());
    if (colonIdx < 0) return 0.0f;
    
    return json.substring(colonIdx + 1).toFloat();
}

static int getJsonInt(const String& json, const String& key) {
    String search = "\"" + key + "\"";
    int idx = json.indexOf(search);
    if (idx < 0) return -1;
    
    int colonIdx = json.indexOf(':', idx + search.length());
    if (colonIdx < 0) return -1;
    
    return json.substring(colonIdx + 1).toInt();
}

static void handleApiPidPost() {
    if (server.method() != HTTP_POST) {
        server.send(405, "application/json", "{\"error\":\"POST only\"}");
        return;
    }
    String body = server.arg("plain");
    
    int idx = getJsonInt(body, "idx");
    if (idx < 0 || idx > 3) {
        server.send(400, "application/json", "{\"error\":\"invalid idx\"}");
        return;
    }

    float kp = getJsonFloat(body, "kp");
    float ki = getJsonFloat(body, "ki");
    float kf = getJsonFloat(body, "kf");
    float db = getJsonFloat(body, "db");

    pidSetGains(idx, kp, ki, kf, db);
    pidSaveToNVS(idx, kp, ki, kf, db);
    Serial.printf("[WEB] Motor %d PID: Kp=%.3f Ki=%.3f Kf=%.3f Db=%.1f (NVS saved)\n", idx, kp, ki, kf, db);
    server.send(200, "application/json", "{\"ok\":true}");
}

static void handleApiGravity() {
    if (server.method() != HTTP_POST) {
        server.send(405, "application/json", "{\"error\":\"POST only\"}");
        return;
    }
    String body = server.arg("plain");
    float kg = getJsonFloat(body, "kg");
    setMotorKg(kg);
    saveMotorKg();
    Serial.printf("[WEB] Gravity Kg=%.1f (NVS saved)\n", motorKg);
    server.send(200, "application/json", "{\"ok\":true,\"kg\":" + String(motorKg, 1) + "}");
}

static void handleApiYawPid() {
    if (server.method() != HTTP_POST) {
        server.send(405, "application/json", "{\"error\":\"POST only\"}");
        return;
    }
    String body = server.arg("plain");
    
    float kp = getJsonFloat(body, "kp");
    float ki = getJsonFloat(body, "ki");
    float kd = getJsonFloat(body, "kd");

    pidKinematicYaw.kp = kp;
    pidKinematicYaw.ki = ki;
    pidKinematicYaw.kd = kd;
    pidKinematicYaw.reset();
    saveYawPid();
    
    Serial.printf("[WEB] Yaw PID: Kp=%.3f Ki=%.3f Kd=%.3f (NVS saved)\n", kp, ki, kd);
    server.send(200, "application/json", "{\"ok\":true}");
}

static void handleApiSave() {
    for (int i = 0; i < 4; i++) {
        pidSaveToNVS(i, pidStates[i].kp, pidStates[i].ki, pidStates[i].kf, pidStates[i].deadband);
    }
    saveYawPid();
    saveMotorKg();
    saveWaypointPid();
    Serial.println("[WEB] All PID + Waypoint saved to NVS");
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

    // Matikan testYawMode secara paksa saat mulai autotune
    if (testYawMode) {
        testYawMode = false;
        motorStopAll();
    }
    
    // Check if string "all"
    if (body.indexOf("\"all\"") >= 0) {
        startAutoTuneAll();
        server.send(200, "application/json", "{\"ok\":true,\"target\":\"all\"}");
        Serial.println("[WEB] Auto-tune ALL started");
        return;
    }
    
    int idx = getJsonInt(body, "idx");
    if (idx < 0 || idx > 3) {
        server.send(400, "application/json", "{\"error\":\"invalid idx\"}");
        return;
    }
    
    startAutoTune(idx);
    Serial.printf("[WEB] Auto-tune motor %d started\n", idx);
    server.send(200, "application/json", "{\"ok\":true,\"target\":" + String(idx) + "}");
}

static void handleApiTestYaw() {
    if (server.method() != HTTP_POST) {
        server.send(405, "application/json", "{\"error\":\"POST only\"}");
        return;
    }
    String body = server.arg("plain");

    if (body.indexOf("\"stop\":true") >= 0) {
        testYawMode = false;
        motorStopAll();
        Serial.println("[WEB] Test Yaw STOPPED");
        server.send(200, "application/json", "{\"ok\":true}");
        return;
    }

    int targetIdx = body.indexOf("\"target\"");
    if (targetIdx >= 0) {
        int valIdx = body.indexOf(':', targetIdx) + 1;
        testYawTarget = body.substring(valIdx).toInt();
        testYawMode = true;
        
        // Reset PID integral/state when starting test
        pidKinematicYaw.reset();
        
        Serial.printf("[WEB] Test Yaw STARTED: Target %d°\n", testYawTarget);
        server.send(200, "application/json", "{\"ok\":true,\"target\":" + String(testYawTarget) + "}");
    } else {
        server.send(400, "application/json", "{\"error\":\"missing target\"}");
    }
}

static void handleApiStatus() {
    float yawVal = getYaw();
    float slopeVal = getSlopeDeg();
    String json = "{\"yaw\":" + String(yawVal, 1);
    json += ",\"slope\":" + String(slopeVal, 1);
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

// =====================================================================
//  WAYPOINT HANDLERS
// =====================================================================

static void handleApiWaypoint() {
    if (server.method() != HTTP_POST) {
        server.send(405, "application/json", "{\"error\":\"POST only\"}");
        return;
    }
    String body = server.arg("plain");
    wpKpXY       = getJsonFloat(body, "kp");
    wpTolPos_m   = getJsonFloat(body, "tol_pos") * 0.01f;  // cm → m
    wpTolYaw_deg = getJsonFloat(body, "tol_yaw");
    saveWaypointPid();
    Serial.printf("[WEB] Waypoint: Kp=%.1f TolPos=%.2fcm TolYaw=%.1fdeg (NVS saved)\n",
                  wpKpXY, wpTolPos_m * 100.0f, wpTolYaw_deg);
    server.send(200, "application/json", "{\"ok\":true}");
}

static void handleApiWpGo() {
    if (server.method() != HTTP_POST) {
        server.send(405, "application/json", "{\"error\":\"POST only\"}");
        return;
    }
    String body = server.arg("plain");
    float x_cm = getJsonFloat(body, "x");
    float y_cm = getJsonFloat(body, "y");
    float yaw  = getJsonFloat(body, "yaw");
    float maxSpd = getJsonFloat(body, "max_speed");
    if (maxSpd < 1) maxSpd = wpMaxSpeed;

    testYawMode = false;
    startWaypoint(x_cm, y_cm, yaw, maxSpd);
    Serial.printf("[WEB] WP Go: (%.0f, %.0f)cm yaw=%.1f° speed=%.0fRPM\n", x_cm, y_cm, yaw, maxSpd);
    server.send(200, "application/json", "{\"ok\":true}");
}

static void handleApiWpCombo() {
    if (server.method() != HTTP_POST) {
        server.send(405, "application/json", "{\"error\":\"POST only\"}");
        return;
    }
    String body = server.arg("plain");
    float x1   = getJsonFloat(body, "x1");
    float y1   = getJsonFloat(body, "y1");
    float yaw1 = getJsonFloat(body, "yaw1");
    float x2   = getJsonFloat(body, "x2");
    float y2   = getJsonFloat(body, "y2");
    float yaw2 = getJsonFloat(body, "yaw2");
    float maxSpd = getJsonFloat(body, "max_speed");
    if (maxSpd < 1) maxSpd = wpMaxSpeed;

    testYawMode = false;
    startWaypointCombo(x1, y1, yaw1, x2, y2, yaw2, maxSpd);
    Serial.printf("[WEB] WP Combo: P1(%.0f,%.0f) P2(%.0f,%.0f) speed=%.0fRPM\n",
                  x1, y1, x2, y2, maxSpd);
    server.send(200, "application/json", "{\"ok\":true}");
}

static void handleApiWpStop() {
    cancelWaypoint();
    Serial.println("[WEB] WP Stop");
    server.send(200, "application/json", "{\"ok\":true}");
}

static void handleApiWpStatus() {
    const char* states[] = {"IDLE", "RUNNING", "REACHED"};
    String json = "{";
    json += "\"state\":\"" + String(states[(int)getWaypointState()]) + "\"";
    json += ",\"target_x\":" + String(wpTargetX_m * 100.0f, 1);
    json += ",\"target_y\":" + String(wpTargetY_m * 100.0f, 1);
    json += ",\"target_yaw\":" + String(wpTargetYaw_deg, 1);
    json += ",\"combo_active\":" + String(isWaypointComboActive() ? "true" : "false");
    json += ",\"combo_step\":" + String(getWaypointComboStep());
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
    server.on("/api/gravity", HTTP_POST, handleApiGravity);
    server.on("/api/testyaw", HTTP_POST, handleApiTestYaw);
    server.on("/api/save", HTTP_POST, handleApiSave);
    server.on("/api/autotune", HTTP_POST, handleApiAutotune);
    server.on("/api/waypoint", HTTP_POST, handleApiWaypoint);
    server.on("/api/wpgoto", HTTP_POST, handleApiWpGo);
    server.on("/api/wpcombo", HTTP_POST, handleApiWpCombo);
    server.on("/api/wpstop", HTTP_POST, handleApiWpStop);
    server.on("/api/wpstatus", HTTP_GET, handleApiWpStatus);
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
