/*
 * =====================================================================
 * FILE    : index.h
 * PERAN   : HTML content untuk web configurator — disimpan di PROGMEM.
 *
 * ISI     : index.html dari KRAI2026Manual/web/ (HTML+CSS+JS inline)
 *           Per-card save buttons — setiap section punya Save sendiri.
 *
 * CATATAN:
 *   - Menggunakan raw string literal R"rawliteral(...)rawliteral"
 *   - PROGMEM agar tidak makan RAM
 *   - Dipakai oleh web.ino: server.send_P(200, "text/html", indexHtml)
 *   - Update file ini jika index.html berubah
 * =====================================================================
 */

#ifndef INDEX_H
#define INDEX_H

#include <pgmspace.h>

const char indexHtml[] PROGMEM = R"rawliteral(
<!DOCTYPE html>
<html lang="id">
<head>
<meta charset="UTF-8">
<meta name="viewport" content="width=device-width,initial-scale=1">
<title>KRAI 2026 — Button & PID Config</title>
<style>
:root{--bg:#0f1117;--bg2:#1a1d27;--bg3:#252833;--bg4:#2a2d3a;--bgs:#1e2a4a;--bd:#2e3140;--bdf:#4a6cf7;--tx:#e4e6f0;--txd:#8b8fa3;--ac:#4a6cf7;--acd:#3a56d4;--gn:#34d399;--rd:#f87171;--or:#fb923c;--yw:#fbbf24;--r:8px;--rl:12px}
*{margin:0;padding:0;box-sizing:border-box}
body{font-family:'Segoe UI',-apple-system,sans-serif;background:var(--bg);color:var(--tx);line-height:1.6;min-height:100vh;display:flex;flex-direction:column}
header{display:flex;justify-content:space-between;align-items:center;padding:12px 20px;border-bottom:1px solid var(--bd);background:var(--bg2);position:sticky;top:0;z-index:100}
.hl{display:flex;align-items:center;gap:10px}.hl h1{font-size:1.15rem;font-weight:700}.hl .ac{color:var(--ac)}
.ver{font-size:.65rem;background:var(--bg3);color:var(--txd);padding:2px 7px;border-radius:4px}
.conn{display:flex;align-items:center;gap:5px;font-size:.78rem;color:var(--txd);padding:3px 9px;border-radius:16px;background:var(--bg3)}
.dot{width:7px;height:7px;border-radius:50%;display:inline-block}.dot.on{background:var(--gn);box-shadow:0 0 5px var(--gn)}.dot.off{background:var(--rd)}
.nav{display:flex;background:var(--bg2);border-bottom:1px solid var(--bd);padding:0 20px;position:sticky;top:46px;z-index:99}
.ni{display:flex;align-items:center;gap:6px;padding:10px 16px;border:none;background:0;color:var(--txd);font-size:.82rem;font-weight:500;cursor:pointer;border-bottom:2px solid transparent;margin-bottom:-1px;transition:.15s}
.ni:hover{color:var(--tx);background:var(--bg4)}.ni.on{color:var(--ac);border-bottom-color:var(--ac)}
main{flex:1;padding:16px 20px;max-width:1000px;width:100%;margin:0 auto}
.pg{display:flex;flex-direction:column;gap:14px}.pg.h{display:none}
.btn{padding:7px 14px;border:1px solid var(--bd);border-radius:var(--r);background:var(--bg3);color:var(--tx);font-size:.8rem;cursor:pointer;transition:.15s;white-space:nowrap}
.btn:hover{background:var(--bg4);border-color:var(--txd)}.btn:active{transform:scale(.97)}
.bp{background:var(--ac);border-color:var(--ac);color:#fff;font-weight:600}.bp:hover{background:var(--acd)}
.bo{background:0;border-color:var(--bd)}.bdn{background:var(--rd);border-color:var(--rd);color:#fff}
.ha{display:flex;gap:6px;align-items:center}
.cd{background:var(--bg2);border:1px solid var(--bd);border-radius:var(--rl);overflow:hidden}
.ch{display:flex;justify-content:space-between;align-items:center;padding:12px 16px;border-bottom:1px solid var(--bd)}
.ch h2{font-size:.9rem;font-weight:600}
.bg{font-size:.62rem;padding:2px 7px;border-radius:9px;background:var(--bg3);color:var(--txd);text-transform:uppercase;letter-spacing:.5px}
.cb{padding:14px 16px}
table{width:100%;border-collapse:collapse}
th{text-align:left;font-size:.7rem;color:var(--txd);font-weight:500;text-transform:uppercase;letter-spacing:.5px;padding:7px 10px;border-bottom:1px solid var(--bd)}
td{padding:7px 10px;font-size:.82rem;border-bottom:1px solid var(--bd)}
tr:last-child td{border-bottom:0}
.i{padding:7px 9px;border:1px solid var(--bd);border-radius:5px;background:var(--bg3);color:var(--tx);font-size:.82rem;font-family:'SF Mono',monospace;transition:border-color .15s;width:100%}
.i:focus{outline:0;border-color:var(--bdf);box-shadow:0 0 0 2px rgba(74,108,247,.15)}
select.i{appearance:none;background-image:url("data:image/svg+xml,%3Csvg width='10' height='6' viewBox='0 0 10 6' fill='none' xmlns='http://www.w3.org/2000/svg'%3E%3Cpath d='M1 1L5 5L9 1' stroke='%238b8fa3' stroke-width='1.5' stroke-linecap='round' stroke-linejoin='round'/%3E%3C/svg%3E");background-repeat:no-repeat;background-position:right 8px center;padding-right:26px;cursor:pointer}
.is{max-width:80px}
.fg{display:flex;flex-direction:column;gap:3px;flex:1;min-width:90px}
.fg label{font-size:.72rem;color:var(--txd);font-weight:500}
.fr{display:flex;gap:14px;flex-wrap:wrap;margin-bottom:10px}.fr:last-child{margin-bottom:0}
.mp-row{display:grid;grid-template-columns:140px 1fr 1fr 60px;gap:8px;align-items:center;padding:8px 0;border-bottom:1px solid var(--bd)}
.mp-row:last-child{border-bottom:0}
.mp-hdr{font-size:.7rem;color:var(--txd);font-weight:500;text-transform:uppercase}
.badge{font-size:.62rem;padding:2px 6px;border-radius:4px;display:inline-block}
.badge-grn{background:rgba(52,211,153,.15);color:var(--gn)}
.badge-or{background:rgba(251,146,60,.15);color:var(--or)}
.badge-bl{background:rgba(74,108,247,.15);color:var(--ac)}
.badge-rd{background:rgba(248,113,113,.15);color:var(--rd)}
.rem{background:none;border:0;color:var(--txd);cursor:pointer;padding:4px;border-radius:3px;font-size:.8rem}.rem:hover{background:var(--bg3);color:var(--rd)}
.sbar{display:flex;align-items:center;gap:8px;padding:6px 12px;background:var(--bg3);border-radius:6px;font-size:.78rem;color:var(--txd)}
.sbar .dot{width:6px;height:6px}
.ca{background:var(--bg);border:1px solid var(--bd);border-radius:5px;padding:10px;max-height:300px;min-height:150px;overflow-y:auto;font-family:'SF Mono',monospace;font-size:.75rem;line-height:1.8;margin-bottom:10px}
.ll{color:var(--txd)}.ll.li{color:var(--ac)}.ll.lo{color:var(--gn)}.ll.le{color:var(--rd)}.ll.lt{color:var(--or)}.ll.lr{color:var(--yw)}
.cir{display:flex;gap:7px}.cir .i{flex:1}
footer{display:flex;justify-content:space-between;align-items:center;padding:14px 20px;border-top:1px solid var(--bd);background:var(--bg2);position:sticky;bottom:0}
.fl{display:flex;gap:7px}.fi{font-size:.75rem;color:var(--txd)}
@media(max-width:700px){header{flex-direction:column;gap:8px}.mp-row{grid-template-columns:1fr;gap:4px}main{padding:10px}footer{flex-direction:column;gap:8px}.fl{flex-wrap:wrap;justify-content:center}}
::-webkit-scrollbar{width:5px}::-webkit-scrollbar-track{background:var(--bg)}::-webkit-scrollbar-thumb{background:var(--bd);border-radius:3px}
</style>
</head>
<body>

<header>
<div class="hl"><h1>KRAI 2026 <span class="ac">Config</span></h1><span class="ver">v2.0</span></div>
<div class="conn" id="connSt"><span class="dot off"></span><span id="connTx">WiFi: not connected</span></div>
</header>

<nav class="nav">
<button class="ni on" data-p="mapping" onclick="swP('mapping')">Button Mapping</button>
<button class="ni" data-p="pid" onclick="swP('pid')">PID Tuning</button>
<button class="ni" data-p="waypoint" onclick="swP('waypoint')">Waypoint</button>
<button class="ni" data-p="console" onclick="swP('console')">Console</button>
</nav>

<main>

<!-- PAGE: BUTTON MAPPING -->
<div class="pg" id="pg-mapping">

<div class="sbar" style="margin-bottom:12px">
<span class="dot on"></span>
<strong>Mode: <span id="curMode">GRIPPING</span></strong>
<span style="margin-left:12px">Mapping berubah per mode. Simpan per card.</span>
</div>

<!-- GRIPPING MODE -->
<div class="cd">
<div class="ch"><h2>GRIPPING Mode</h2><div class="ha"><span class="bg badge-grn">Active</span><button class="btn bp" style="padding:4px 10px;font-size:.72rem" onclick="saveSection('mapping_grip')">Save</button></div></div>
<div class="cb" id="mapGripping">
<div class="mp-row mp-hdr"><div>Button</div><div>Action</div><div>Value</div><div></div></div>
<div id="mapGripList"></div>
<button class="btn bo" style="margin-top:10px" onclick="mapAdd('grip')">+ Add Mapping</button>
</div>
</div>

<!-- ARM_BOX MODE -->
<div class="cd" style="margin-top:14px">
<div class="ch"><h2>ARM_BOX Mode</h2><div class="ha"><span class="bg badge-or">Standby</span><button class="btn bp" style="padding:4px 10px;font-size:.72rem" onclick="saveSection('mapping_arm')">Save</button></div></div>
<div class="cb" id="mapArmbox">
<div class="mp-row mp-hdr"><div>Button</div><div>Action</div><div>Value</div><div></div></div>
<div id="mapArmList"></div>
<button class="btn bo" style="margin-top:10px" onclick="mapAdd('armbox')">+ Add Mapping</button>
</div>
</div>

<!-- PRESETS -->
<div class="cd" style="margin-top:14px">
<div class="ch"><h2>Servo Presets</h2><div class="ha"><span class="bg badge-bl">Angles</span><button class="btn bp" style="padding:4px 10px;font-size:.72rem" onclick="saveSection('presets')">Save</button></div></div>
<div class="cb">
<table><thead><tr><th>Name</th><th>Angle (deg)</th><th></th></tr></thead><tbody id="presetList"></tbody></table>
<button class="btn bo" style="margin-top:10px" onclick="presetAdd()">+ Add Preset</button>
</div>
</div>

<!-- MOTOR POSITIONS -->
<div class="cd" style="margin-top:14px">
<div class="ch"><h2>Motor Positions (ARM_BOX)</h2><div class="ha"><span class="bg badge-bl">Encoder</span><button class="btn bp" style="padding:4px 10px;font-size:.72rem" onclick="saveSection('motor_pos')">Save</button></div></div>
<div class="cb">
<table><thead><tr><th>Name</th><th>Position (count)</th><th></th></tr></thead><tbody id="motorPosList"></tbody></table>
<button class="btn bo" style="margin-top:10px" onclick="motorPosAdd()">+ Add Position</button>
</div>
</div>

</div>

<!-- PAGE: PID -->
<div class="pg h" id="pg-pid">

<!-- PID CONTROLLER -->
<div class="cd">
<div class="ch"><h2>PID Controller</h2><div class="ha"><span class="bg badge-bl">Per Motor</span><button class="btn bp" style="padding:4px 10px;font-size:.72rem" onclick="saveSection('pid')">Save</button></div></div>
<div class="cb">
<table><thead><tr><th>Motor</th><th>Kp</th><th>Ki</th><th>Kd</th><th></th></tr></thead><tbody id="pidList"></tbody></table>
</div>
</div>

<!-- YAW PID -->
<div class="cd" style="margin-top:14px">
<div class="ch"><h2>Yaw PID</h2><div class="ha"><span class="bg badge-bl">IMU</span><button class="btn bp" style="padding:4px 10px;font-size:.72rem" onclick="saveSection('yaw_pid')">Save</button></div></div>
<div class="cb">
<div class="fr">
<div class="fg"><label>Kp</label><input type="number" class="i is" id="yawKp" step="0.01"></div>
<div class="fg"><label>Ki</label><input type="number" class="i is" id="yawKi" step="0.001"></div>
<div class="fg"><label>Kd</label><input type="number" class="i is" id="yawKd" step="0.001"></div>
</div>
</div>
</div>

</div>

<!-- PAGE: WAYPOINT -->
<div class="pg h" id="pg-waypoint">

<!-- WAYPOINT CONFIG -->
<div class="cd">
<div class="ch"><h2>Waypoint Config</h2><div class="ha"><span class="bg badge-bl">P-controller</span><button class="btn bp" style="padding:4px 10px;font-size:.72rem" onclick="saveSection('waypoint')">Save</button></div></div>
<div class="cb">
<div class="fr">
<div class="fg"><label>Kp (RPM/m)</label><input type="number" class="i is" id="wpKp" step="1"></div>
<div class="fg"><label>TolPos (cm)</label><input type="number" class="i is" id="wpTolPos" step="0.5"></div>
<div class="fg"><label>TolYaw (deg)</label><input type="number" class="i is" id="wpTolYaw" step="0.5"></div>
</div>
</div>
</div>

<!-- WAYPOINT TEST -->
<div class="cd" style="margin-top:14px">
<div class="ch"><h2>Waypoint Test</h2><div class="ha"><span class="bg badge-grn">Goto</span></div></div>
<div class="cb">
<div class="fr">
<div class="fg"><label>X (cm)</label><input type="number" class="i is" id="wpTestX" value="0" step="1"></div>
<div class="fg"><label>Y (cm)</label><input type="number" class="i is" id="wpTestY" value="0" step="1"></div>
<div class="fg"><label>Yaw (deg)</label><input type="number" class="i is" id="wpTestYaw" value="0" step="1"></div>
<div class="fg"><label>Speed (RPM)</label><input type="number" class="i is" id="wpTestSpeed" value="200" step="10"></div>
<div style="align-self:flex-end;display:flex;gap:6px">
<button class="btn bp" onclick="wpGo()">Go</button>
<button class="btn bdn" onclick="wpStop()">Stop</button>
</div>
</div>
</div>
</div>

</div>

<!-- PAGE: CONSOLE -->
<div class="pg h" id="pg-console">
<div class="cd"><div class="ch"><h2>Serial Console</h2></div><div class="cb">
<div class="ca" id="sLog"><div class="ll li">Ready.</div></div>
<div class="cir">
<input class="i" id="sInp" placeholder="Type command..." onkeydown="if(event.key==='Enter')sSendIn()">
<button class="btn bp" onclick="sSendIn()">Send</button>
</div>
</div></div>
</div>

</main>

<footer>
<div class="fl">
<button class="btn" onclick="expJSON()">Export JSON</button>
<button class="btn" onclick="document.getElementById('fI').click()">Import JSON</button>
<input type="file" id="fI" accept=".json" style="display:none" onchange="impJSON(event)">
</div>
<div class="fi" id="lAct"></div>
</footer>

<script>
var ESP_BASE='http://'+location.hostname;
var BTN_OPTS=[
{val:'BTN_CROSS',label:'X (Cross)'},{val:'BTN_CIRCLE',label:'O (Circle)'},
{val:'BTN_TRIANGLE',label:'Triangle'},{val:'BTN_SQUARE',label:'Square'},
{val:'BTN_L1',label:'L1'},{val:'BTN_R1',label:'R1'},
{val:'BTN_L2',label:'L2'},{val:'BTN_R2',label:'R2'},
{val:'BTN_UP',label:'D-pad Up'},{val:'BTN_DOWN',label:'D-pad Down'},
{val:'BTN_LEFT',label:'D-pad Left'},{val:'BTN_RIGHT',label:'D-pad Right'},
{val:'BTN_SHARE',label:'Share'},{val:'BTN_OPTIONS',label:'Options'},
{val:'BTN_L3',label:'L3 (Stick L)'},{val:'BTN_R3',label:'R3 (Stick R)'}
];
var MOD_OPTS=[
{val:'NONE',label:'(none)'},{val:'BTN_R2',label:'hold R2'},{val:'BTN_L2',label:'hold L2'},
{val:'BTN_R1',label:'hold R1'},{val:'BTN_L1',label:'hold L1'},
{val:'BTN_X',label:'hold X'},{val:'BTN_SQUARE',label:'hold Square'}
];
var ACTION_OPTS=[
{val:'none',label:'None',cat:'none'},
{val:'servo_toggle',label:'Servo Toggle',cat:'servo'},
{val:'servo_preset',label:'Servo Preset',cat:'servo'},
{val:'servo_step',label:'Servo Step +/-',cat:'servo'},
{val:'motor_jog',label:'Motor Jog',cat:'motor'},
{val:'motor_preset',label:'Motor Preset',cat:'motor'},
{val:'motor_speed_mod',label:'Speed Modifier',cat:'motor'},
{val:'relay_toggle',label:'Relay Toggle',cat:'io'},
{val:'relay_pulse',label:'Relay Pulse',cat:'io'},
{val:'send_serial',label:'Send Serial Cmd',cat:'comm'},
{val:'mode_toggle',label:'Mode Toggle',cat:'sys'},
{val:'input_toggle',label:'Input Toggle',cat:'sys'},
{val:'snap_yaw',label:'Snap Yaw',cat:'sys'},
{val:'rotate_abs',label:'Rotate Absolute',cat:'sys'}
];
var maps={grip:[],armbox:[]};var presets=[];var motorPos=[];var pid=[];
function swP(id){
document.querySelectorAll('.ni').forEach(function(n){n.classList.remove('on')});
document.querySelector('.ni[data-p="'+id+'"]').classList.add('on');
document.querySelectorAll('.pg').forEach(function(p){p.classList.add('h')});
document.getElementById('pg-'+id).classList.remove('h');
}
function v(id){var e=document.getElementById(id);return e?e.value:''}
function s(id,val){var e=document.getElementById(id);if(e)e.value=val;}
function log(msg,ty){
var a=document.getElementById('sLog'),d=document.createElement('div');
d.className='ll '+(ty||'');var t=new Date();
d.textContent='['+t.toTimeString().slice(0,8)+'] '+msg;
a.appendChild(d);a.scrollTop=a.scrollHeight;
while(a.children.length>200)a.removeChild(a.firstChild);
}
function setLA(t){var e=document.getElementById('lAct');if(e){e.textContent=t;setTimeout(function(){e.textContent='';},3000);}}
function selOpts(opts,selected){
return opts.map(function(o){return'<option value="'+o.val+'"'+(o.val===selected?' selected':'')+'>'+o.label+'</option>';}).join('');
}
function mapRender(mode,listId){
var container=document.getElementById(listId);var arr=maps[mode];
container.innerHTML=arr.map(function(m,i){
return'<div class="mp-row" data-i="'+i+'">'+
'<div><select class="i" onchange="mapChg(\''+mode+'\','+i+',\'btn\',this.value)">'+selOpts(BTN_OPTS,m.btn)+'</select></div>'+
'<div style="display:flex;gap:6px"><select class="i" style="max-width:140px" onchange="mapChg(\''+mode+'\','+i+',\'mod\',this.value)">'+selOpts(MOD_OPTS,m.mod)+'</select>'+
'<select class="i" onchange="mapChg(\''+mode+'\','+i+',\'action\',this.value)">'+selOpts(ACTION_OPTS,m.action)+'</select></div>'+
'<div><input class="i" value="'+esc(m.value)+'" onchange="mapChg(\''+mode+'\','+i+',\'value\',this.value)" placeholder="'+(m.action==='servo_preset'?'preset name':'value')+'"></div>'+
'<div><button class="rem" onclick="mapRem(\''+mode+'\','+i+')">x</button></div>'+
'</div>';
}).join('');
}
function mapAdd(mode){
maps[mode].push({btn:'BTN_CROSS',mod:'NONE',action:'none',value:''});
mapRender(mode,mode==='grip'?'mapGripList':'mapArmList');
}
function mapRem(mode,i){maps[mode].splice(i,1);mapRender(mode,mode==='grip'?'mapGripList':'mapArmList');}
function mapChg(mode,i,field,val){maps[mode][i][field]=val;}
function presetRender(){
var tb=document.getElementById('presetList');
tb.innerHTML=presets.map(function(p,i){
return'<tr><td><input class="i" value="'+esc(p.name)+'" onchange="presets['+i+'].name=this.value" style="max-width:160px"></td>'+
'<td><input type="number" class="i is" value="'+p.angle+'" onchange="presets['+i+'].angle=parseInt(this.value)"></td>'+
'<td><button class="rem" onclick="presets.splice('+i+',1);presetRender()">x</button></td></tr>';
}).join('');
}
function presetAdd(){presets.push({name:'Preset '+(presets.length+1),angle:90});presetRender();}
function motorPosRender(){
var tb=document.getElementById('motorPosList');
tb.innerHTML=motorPos.map(function(p,i){
return'<tr><td><input class="i" value="'+esc(p.name)+'" onchange="motorPos['+i+'].name=this.value" style="max-width:160px"></td>'+
'<td><input type="number" class="i is" value="'+p.pos+'" onchange="motorPos['+i+'].pos=parseInt(this.value)"></td>'+
'<td><button class="rem" onclick="motorPos.splice('+i+',1);motorPosRender()">x</button></td></tr>';
}).join('');
}
function motorPosAdd(){motorPos.push({name:'Pos '+(motorPos.length+1),pos:0});motorPosRender();}
function pidRender(){
var tb=document.getElementById('pidList');var names=['Motor FR','Motor FL','Motor BR','Motor BL'];
tb.innerHTML=pid.map(function(p,i){
return'<tr><td><strong>'+(names[i]||('M'+i))+'</strong></td>'+
'<td><input type="number" class="i is" step="0.01" value="'+p.kp+'" onchange="pid['+i+'].kp=parseFloat(this.value)"></td>'+
'<td><input type="number" class="i is" step="0.001" value="'+p.ki+'" onchange="pid['+i+'].ki=parseFloat(this.value)"></td>'+
'<td><input type="number" class="i is" step="0.001" value="'+p.kd+'" onchange="pid['+i+'].kd=parseFloat(this.value)"></td>'+
'<td></td></tr>';
}).join('');
}
function esc(s){return String(s).replace(/&/g,'&amp;').replace(/</g,'&lt;').replace(/>/g,'&gt;');}
function collectSection(section){
switch(section){
case 'mapping_grip':return{section:'mapping_grip',data:maps.grip};
case 'mapping_arm':return{section:'mapping_arm',data:maps.armbox};
case 'presets':return{section:'presets',data:presets};
case 'motor_pos':return{section:'motor_pos',data:motorPos};
case 'pid':return{section:'pid',data:{motors:pid}};
case 'waypoint':return{section:'waypoint',data:{kp:parseFloat(v('wpKp'))||200,tol_pos:parseFloat(v('wpTolPos'))||5,tol_yaw:parseFloat(v('wpTolYaw'))||3}};
case 'yaw_pid':return{section:'yaw_pid',data:{kp:parseFloat(v('yawKp'))||0,ki:parseFloat(v('yawKi'))||0,kd:parseFloat(v('yawKd'))||0}};
default:return null;
}
}
function saveSection(section){
var cfg=collectSection(section);
if(!cfg){log('[ERR] Unknown section: '+section,'le');return;}
log('[TX] Saving '+section+'...','lt');
fetch(ESP_BASE+'/api/save',{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify(cfg)})
.then(function(r){return r.json();}).then(function(d){
log('[RX] OK: '+section,'lo');setLA(section+' saved');
}).catch(function(e){
log('[ERR] '+e.message,'le');setLA('Save failed');
});
}
function collectAll(){
return{version:'2.0',ts:new Date().toISOString(),
mappings:{gripping:maps.grip,armbox:maps.armbox},
presets:presets,motorPositions:motorPos,
pid:{motors:pid,yaw:{kp:parseFloat(v('yawKp'))||0,ki:parseFloat(v('yawKi'))||0,kd:parseFloat(v('yawKd'))||0}},
waypoint:{kp:parseFloat(v('wpKp'))||200,tol_pos:parseFloat(v('wpTolPos'))||5,tol_yaw:parseFloat(v('wpTolYaw'))||3}};
}
function aplC(c){
if(c.mappings){if(c.mappings.gripping)maps.grip=c.mappings.gripping;if(c.mappings.armbox)maps.armbox=c.mappings.armbox;}
if(c.presets)presets=c.presets;if(c.motorPositions)motorPos=c.motorPositions;
if(c.pid){if(c.pid.motors)pid=c.pid.motors;if(c.pid.yaw){s('yawKp',c.pid.yaw.kp);s('yawKi',c.pid.yaw.ki);s('yawKd',c.pid.yaw.kd);}}
if(c.waypoint){s('wpKp',c.waypoint.kp);s('wpTolPos',c.waypoint.tol_pos);s('wpTolYaw',c.waypoint.tol_yaw);}
mapRender('grip','mapGripList');mapRender('armbox','mapArmList');
presetRender();motorPosRender();pidRender();
}
function expJSON(){
var c=collectAll();var b=new Blob([JSON.stringify(c,null,2)],{type:'application/json'});
var u=URL.createObjectURL(b),a=document.createElement('a');
a.href=u;a.download='krai2026_config_'+Date.now()+'.json';a.click();URL.revokeObjectURL(u);
setLA('Exported');log('Config exported.','lo');
}
function impJSON(ev){
var f=ev.target.files[0];if(!f)return;
var r=new FileReader();r.onload=function(e){
try{aplC(JSON.parse(e.target.result));setLA('Imported');log('Imported '+f.name,'lo');}
catch(err){log('Error: '+err.message,'le');}
};r.readAsText(f);ev.target.value='';
}
function sSendIn(){
var inp=document.getElementById('sInp'),t=inp.value.trim();if(!t)return;
log('[TX] '+t,'lt');
fetch(ESP_BASE+'/api/serial',{method:'POST',headers:{'Content-Type':'text/plain'},body:t})
.then(function(r){return r.text();}).then(function(d){if(d)log('[RX] '+d,'lr');})
.catch(function(e){log('Error: '+e.message,'le');});
inp.value='';inp.focus();
}
function wpGo(){
var x=parseFloat(v('wpTestX'))||0,y=parseFloat(v('wpTestY'))||0,yaw=parseFloat(v('wpTestYaw'))||0;
if(!x&&!y&&!yaw){log('[ERR] Set X/Y/Yaw dulu','le');return;}
var cmd='goto '+x+' '+y+' '+yaw;
log('[TX] '+cmd,'lt');
fetch(ESP_BASE+'/api/serial',{method:'POST',headers:{'Content-Type':'text/plain'},body:cmd})
.then(function(r){return r.text();}).then(function(d){if(d)log('[RX] '+d,'lr');})
.catch(function(e){log('Error: '+e.message,'le');});
}
function wpStop(){
log('[TX] wp cancel','lt');
fetch(ESP_BASE+'/api/serial',{method:'POST',headers:{'Content-Type':'text/plain'},body:'wp cancel'})
.then(function(r){return r.text();}).then(function(d){if(d)log('[RX] '+d,'lr');})
.catch(function(e){log('Error: '+e.message,'le');});
}
function pollESP(){
fetch(ESP_BASE+'/api/status').then(function(r){return r.json();}).then(function(d){
document.getElementById('connSt').querySelector('.dot').className='dot on';
document.getElementById('connTx').textContent='WiFi: connected';
if(d.mode)document.getElementById('curMode').textContent=d.mode;
}).catch(function(){
document.getElementById('connSt').querySelector('.dot').className='dot off';
document.getElementById('connTx').textContent='WiFi: not connected';
});
}
function initDefaults(){
maps.grip=[
{btn:'BTN_TRIANGLE',mod:'NONE',action:'servo_toggle',value:'gripper'},
{btn:'BTN_CROSS',mod:'BTN_UP',action:'servo_preset',value:'center'},
{btn:'BTN_CROSS',mod:'BTN_LEFT',action:'servo_preset',value:'left'},
{btn:'BTN_CROSS',mod:'BTN_RIGHT',action:'servo_preset',value:'right'},
{btn:'BTN_R2',mod:'BTN_LEFT',action:'servo_step',value:'-5'},
{btn:'BTN_R2',mod:'BTN_RIGHT',action:'servo_step',value:'5'},
{btn:'BTN_UP',mod:'NONE',action:'motor_jog',value:'axisX,+300'},
{btn:'BTN_DOWN',mod:'NONE',action:'motor_jog',value:'axisX,-300'},
{btn:'BTN_LEFT',mod:'NONE',action:'motor_jog',value:'axisZ,+300'},
{btn:'BTN_RIGHT',mod:'NONE',action:'motor_jog',value:'axisZ,-300'},
{btn:'BTN_R1',mod:'NONE',action:'motor_speed_mod',value:'600'},
{btn:'BTN_L1',mod:'NONE',action:'motor_speed_mod',value:'200'}
];
maps.armbox=[
{btn:'BTN_TRIANGLE',mod:'NONE',action:'relay_toggle',value:'relay1'},
{btn:'BTN_CIRCLE',mod:'NONE',action:'relay_pulse',value:'relay0,100'},
{btn:'BTN_X',mod:'BTN_UP',action:'motor_jog',value:'motorZ,+350'},
{btn:'BTN_X',mod:'BTN_DOWN',action:'motor_jog',value:'motorZ,-350'},
{btn:'BTN_SQUARE',mod:'BTN_UP',action:'motor_jog',value:'motorY,+350'},
{btn:'BTN_SQUARE',mod:'BTN_DOWN',action:'motor_jog',value:'motorY,-350'},
{btn:'BTN_R2',mod:'BTN_UP',action:'servo_step',value:'-5'},
{btn:'BTN_R2',mod:'BTN_DOWN',action:'servo_step',value:'5'},
{btn:'BTN_UP',mod:'NONE',action:'motor_preset',value:'w_pos_back'},
{btn:'BTN_DOWN',mod:'NONE',action:'motor_preset',value:'w_pos_front'},
{btn:'BTN_LEFT',mod:'NONE',action:'motor_preset',value:'w_pos_left'},
{btn:'BTN_RIGHT',mod:'NONE',action:'motor_preset',value:'w_pos_right'}
];
presets=[
{name:'left',angle:26},{name:'center',angle:90},{name:'right',angle:154},
{name:'gripper_open',angle:0},{name:'gripper_closed',angle:95}
];
motorPos=[
{name:'w_pos_front',pos:0},{name:'w_pos_left',pos:1290},
{name:'w_pos_back',pos:2640},{name:'w_pos_right',pos:3940}
];
pid=[{kp:0.1,ki:0,kd:0},{kp:0.1,ki:0,kd:0},{kp:0.1,ki:0,kd:0},{kp:0.1,ki:0,kd:0}];
s('yawKp','2.5');s('yawKi','0.01');s('yawKd','0.1');
s('wpKp','200');s('wpTolPos','5');s('wpTolYaw','3');
}
initDefaults();
mapRender('grip','mapGripList');mapRender('armbox','mapArmList');
presetRender();motorPosRender();pidRender();
pollESP();setInterval(pollESP,5000);
</script>
</body>
</html>
)rawliteral";

#endif // INDEX_H
