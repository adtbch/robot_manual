#ifndef WEBPAGE_H
#define WEBPAGE_H

const char indexHtml[] PROGMEM = R"rawliteral(
<!DOCTYPE html>
<html lang="id">
<head>
<meta charset="UTF-8">
<meta name="viewport" content="width=device-width,initial-scale=1">
<title>KRAI 2026 — Slave2 Arm Test</title>
<style>
*{box-sizing:border-box;margin:0;padding:0}
body{font-family:system-ui,sans-serif;background:#0f172a;color:#e2e8f0;padding:12px;max-width:520px;margin:0 auto}
h1{font-size:1.15em;color:#38bdf8;margin-bottom:4px}
.sub{font-size:.72em;color:#64748b;margin-bottom:12px}
.card{background:#1e293b;border-radius:8px;padding:12px;margin-bottom:10px}
.card h2{font-size:.85em;color:#94a3b8;margin-bottom:8px;text-transform:uppercase;letter-spacing:.05em}
.row{display:flex;gap:6px;align-items:center;margin-bottom:6px;flex-wrap:wrap}
.badge{display:inline-block;padding:2px 8px;border-radius:4px;font-size:.72em;font-weight:600}
.badge-ok{background:#14532d;color:#86efac}
.badge-warn{background:#713f12;color:#fcd34d}
.badge-err{background:#7f1d1d;color:#fca5a5}
.badge-off{background:#334155;color:#94a3b8}
.motor-card{border-left:3px solid #38bdf8;padding-left:10px;margin-bottom:10px}
.motor-card.k{border-color:#a78bfa}
.motor-card.y{border-color:#34d399}
label{font-size:.72em;color:#64748b;min-width:36px}
input[type=range]{flex:1;accent-color:#38bdf8}
input[type=number]{width:72px;background:#0f172a;border:1px solid #334155;border-radius:4px;color:#e2e8f0;padding:4px 6px;font-size:.85em}
.btn{border:none;border-radius:6px;padding:6px 10px;font-size:.78em;font-weight:600;cursor:pointer}
.btn:active{opacity:.8}
.btn-blue{background:#2563eb;color:#fff}
.btn-green{background:#16a34a;color:#fff}
.btn-gray{background:#475569;color:#fff}
.btn-red{background:#dc2626;color:#fff}
.btn-orange{background:#ea580c;color:#fff}
.preset{display:flex;gap:4px;margin:4px 0}
.status-line{font-size:.75em;color:#64748b;margin-top:4px;font-family:monospace}
.sensor-grid{display:grid;grid-template-columns:1fr 1fr;gap:6px}
.sensor-item{font-size:.78em;padding:6px;border-radius:4px;background:#0f172a;text-align:center}
.enc-val{font-size:1.1em;font-weight:700;color:#38bdf8;font-family:monospace}
#log{font-size:.68em;color:#64748b;margin-top:8px;font-family:monospace;max-height:80px;overflow-y:auto}
.stop-row{margin-top:6px}
</style>
</head>
<body>
<h1>Slave2 Arm — Test Panel</h1>
<p class="sub" id="conn-status">WiFi: KRAI_Slave2_Test</p>

<div class="card">
<h2>Koneksi</h2>
<div class="row"><span class="badge badge-off" id="wifi-badge">—</span><span id="heap-txt" style="font-size:.75em;color:#64748b"></span></div>
</div>

<div class="card" id="motor-section">
<h2>Motor PWM</h2>
<div id="motors"></div>
</div>

<div class="card">
<h2>Motor Y — Positioning</h2>
<div class="row">
<label>Target</label>
<input type="number" id="y-target" value="500" min="0" max="5000">
<button class="btn btn-green" onclick="setYTarget()">Go</button>
<button class="btn btn-gray" onclick="apiPost('/api/motortargetstop')">Stop Y</button>
</div>
<div class="status-line" id="y-pos-status">target: —</div>
</div>

<div class="card">
<h2>Pneumatic</h2>
<div class="row">
<button class="btn btn-blue" onclick="setPne('r','on')">R ON</button>
<button class="btn btn-gray" onclick="setPne('r','off')">R OFF</button>
<button class="btn btn-blue" onclick="setPne('l','on')">L ON</button>
<button class="btn btn-gray" onclick="setPne('l','off')">L OFF</button>
<button class="btn btn-orange" onclick="apiPost('/api/pne',{side:'r',state:'toggle'})">R Toggle</button>
<button class="btn btn-orange" onclick="apiPost('/api/pne',{side:'l',state:'toggle'})">L Toggle</button>
</div>
<div class="row"><button class="btn btn-gray" onclick="apiPost('/api/pneall')">All OFF</button></div>
<div class="sensor-grid" style="margin-top:8px">
<div class="sensor-item">Pne R<br><span id="pne-r" class="badge badge-off">OFF</span></div>
<div class="sensor-item">Pne L<br><span id="pne-l" class="badge badge-off">OFF</span></div>
</div>
</div>

<div class="card">
<h2>Encoder</h2>
<div class="row" style="justify-content:space-between">
<div><span style="font-size:.75em">X</span> <span class="enc-val" id="enc-x">—</span></div>
<div><span style="font-size:.75em">Y</span> <span class="enc-val" id="enc-y">—</span></div>
<div><span style="font-size:.75em">K</span> <span class="enc-val" id="enc-k">—</span></div>
</div>
<div class="row" style="margin-top:8px">
<button class="btn btn-blue" onclick="readEnc()">Baca Encoder</button>
<button class="btn btn-gray" onclick="apiPost('/api/encreset')">Reset</button>
</div>
</div>

<div class="card">
<h2>Limit &amp; Proximity <span style="font-weight:400;font-size:.7em">(auto 500ms)</span></h2>
<div class="sensor-grid">
<div class="sensor-item">Depan<br><span id="lim-depan" class="badge badge-ok">clear</span></div>
<div class="sensor-item">Belakang<br><span id="lim-belakang" class="badge badge-ok">clear</span></div>
<div class="sensor-item">Turun<br><span id="lim-turun" class="badge badge-ok">clear</span></div>
<div class="sensor-item">Prox R<br><span id="prox-r" class="badge badge-ok">clear</span></div>
<div class="sensor-item">Prox L<br><span id="prox-l" class="badge badge-ok">clear</span></div>
</div>
</div>

<div class="card stop-row">
<button class="btn btn-red" style="width:100%;padding:12px;font-size:.95em" onclick="emergencyStop()">■ STOP SEMUA</button>
</div>
<div id="log"></div>

<script>
const MOCK_MODE = false;
const BASE = MOCK_MODE ? '' : ('http://' + location.hostname);

const mockState = {
  limit: {depan:false,belakang:false,turun:false},
  prox: {r:false,l:false},
  pne: {r:false,l:false},
  motorY: {target:0,active:false},
  motorRun: {x:false,k:false},
  motorPwm: {x:0,k:0,y:0},
  enc: {x:120,y:450,k:null},
  clients: 1,
  heap: 180000
};

const MOTORS = [
  {id:'x', label:'Motor X (kanan)', cls:'', presets:[-800,-400,-150,150,400,800]},
  {id:'k', label:'Motor K (kiri)', cls:'k', presets:[-800,-400,-150,150,400,800]},
  {id:'y', label:'Motor Y (jog PWM)', cls:'y', presets:[-800,-400,-150,150,400,800]}
];

function log(msg) {
  const el = document.getElementById('log');
  el.textContent = new Date().toLocaleTimeString() + ' ' + msg + '\n' + el.textContent;
}

function setBadge(id, on, onTxt, offTxt) {
  const el = document.getElementById(id);
  if (!el) return;
  el.textContent = on ? onTxt : offTxt;
  el.className = 'badge ' + (on ? 'badge-err' : 'badge-ok');
}

async function apiGet(path) {
  if (MOCK_MODE) {
    if (path === '/api/status') return {...mockState};
    if (path === '/api/enc') return {...mockState.enc};
    return {};
  }
  const r = await fetch(BASE + path);
  if (r.status === 503) throw new Error('no client');
  return r.json();
}

async function apiPost(path, body) {
  if (MOCK_MODE) {
    if (path === '/api/motor') {
      const {id, pwm} = body;
      mockState.motorPwm[id] = pwm;
      if (id === 'x' || id === 'k') {
        mockState.motorRun[id] = pwm !== 0;
        if (pwm === 0) mockState.motorPwm[id] = 0;
      }
      log('motor ' + id + ' pwm=' + pwm);
    } else if (path === '/api/motortarget') {
      mockState.motorY = {target: body.enc, active: true};
      log('Y target ' + body.enc);
    } else if (path === '/api/motortargetstop') {
      mockState.motorY.active = false;
    } else if (path === '/api/pne') {
      if (body.state === 'toggle') mockState.pne[body.side] = !mockState.pne[body.side];
      else mockState.pne[body.side] = body.state === 'on';
    } else if (path === '/api/pneall' || path === '/api/stop' || path === '/api/motorstop') {
      mockState.pne = {r:false,l:false};
      mockState.motorPwm = {x:0,k:0,y:0};
      mockState.motorRun = {x:false,k:false};
      mockState.motorY.active = false;
      log(path);
    } else if (path === '/api/encreset') {
      mockState.enc = {x:0,y:0,k:null};
      log('enc reset');
    }
    refreshUI(await apiGet('/api/status'));
    return {ok:true};
  }
  const r = await fetch(BASE + path, {
    method:'POST',
    headers:{'Content-Type':'application/json'},
    body: body ? JSON.stringify(body) : '{}'
  });
  return r.json();
}

function runMotor(id, pwm) {
  apiPost('/api/motor', {id, pwm});
}

function setPne(side, state) {
  apiPost('/api/pne', {side, state});
}

function setYTarget() {
  const enc = parseInt(document.getElementById('y-target').value, 10) || 0;
  apiPost('/api/motortarget', {enc});
}

function emergencyStop() {
  apiPost('/api/stop');
}

async function readEnc() {
  try {
    const d = await apiGet('/api/enc');
    document.getElementById('enc-x').textContent = d.x;
    document.getElementById('enc-y').textContent = d.y;
    document.getElementById('enc-k').textContent = d.k === null ? '—' : d.k;
    log('encoder read');
  } catch(e) { log('enc err: ' + e.message); }
}

function refreshUI(d) {
  if (!d) return;
  setBadge('lim-depan', d.limit?.depan, 'TRIG', 'clear');
  setBadge('lim-belakang', d.limit?.belakang, 'TRIG', 'clear');
  setBadge('lim-turun', d.limit?.turun, 'TRIG', 'clear');
  setBadge('prox-r', d.prox?.r, 'DETECT', 'clear');
  setBadge('prox-l', d.prox?.l, 'DETECT', 'clear');
  const pr = document.getElementById('pne-r');
  const pl = document.getElementById('pne-l');
  if (pr) { pr.textContent = d.pne?.r ? 'ON' : 'OFF'; pr.className = 'badge ' + (d.pne?.r ? 'badge-warn' : 'badge-off'); }
  if (pl) { pl.textContent = d.pne?.l ? 'ON' : 'OFF'; pl.className = 'badge ' + (d.pne?.l ? 'badge-warn' : 'badge-off'); }
  document.getElementById('y-pos-status').textContent =
    'target: ' + (d.motorY?.target ?? '—') + (d.motorY?.active ? ' ACTIVE' : ' idle');
  MOTORS.forEach(m => {
    const st = document.getElementById('st-' + m.id);
    if (!st) return;
    const pwm = d.motorPwm?.[m.id] ?? 0;
    const run = d.motorRun?.[m.id];
    st.textContent = (run || pwm !== 0) ? ('RUN pwm=' + pwm) : 'STOP';
  });
  const wb = document.getElementById('wifi-badge');
  if (wb) {
    wb.textContent = (d.clients > 0) ? ('WiFi ' + d.clients + ' client') : 'no client';
    wb.className = 'badge ' + (d.clients > 0 ? 'badge-ok' : 'badge-warn');
  }
  document.getElementById('heap-txt').textContent = d.heap ? ('heap ' + d.heap) : '';
}

async function pollStatus() {
  if (document.visibilityState !== 'visible') return;
  try {
    const d = await apiGet('/api/status');
    refreshUI(d);
  } catch(e) { /* 503 when no wifi client on real hardware */ }
}

function buildMotors() {
  const root = document.getElementById('motors');
  MOTORS.forEach(m => {
    const div = document.createElement('div');
    div.className = 'motor-card ' + m.cls;
    div.innerHTML = `
      <div style="font-size:.8em;font-weight:600;margin-bottom:4px">${m.label}</div>
      <div class="row">
        <label>PWM</label>
        <input type="range" id="sl-${m.id}" min="-1023" max="1023" value="400"
          oninput="document.getElementById('nm-${m.id}').value=this.value">
        <input type="number" id="nm-${m.id}" min="-1023" max="1023" value="400"
          oninput="document.getElementById('sl-${m.id}').value=this.value">
      </div>
      <div class="preset">
        ${m.presets.map(p => `<button class="btn btn-gray" onclick="document.getElementById('sl-${m.id}').value=${p};document.getElementById('nm-${m.id}').value=${p}">${p>0?'+':''}${p}</button>`).join('')}
      </div>
      <div class="row">
        <button class="btn btn-blue" onclick="runMotor('${m.id}',parseInt(document.getElementById('nm-${m.id}').value,10))">Jalankan</button>
        <button class="btn btn-gray" onclick="runMotor('${m.id}',0)">Stop</button>
      </div>
      <div class="status-line" id="st-${m.id}">STOP</div>`;
    root.appendChild(div);
  });
}

buildMotors();
setInterval(pollStatus, 500);
pollStatus();
if (!MOCK_MODE) document.getElementById('conn-status').textContent = 'Terhubung ke ' + location.hostname;
</script>
</body>
</html>

)rawliteral";

#endif // WEBPAGE_H
