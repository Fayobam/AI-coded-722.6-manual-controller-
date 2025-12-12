/*
  WebDashboard.h - V5.0 (Dual Core Support)
  - Dashboard runs on Core 0
  - Reads volatile shared variables from Core 1
*/

#ifndef WEB_DASH_H
#define WEB_DASH_H

#include <WiFi.h>
#include <WebServer.h>

extern struct ShiftParameters shiftMaps[8]; 
extern struct GeneralMaps generalMaps;
extern void persistMapsBinary();
extern String getExtendedTelemetryJSON();

// Temp scale externs
extern int16_t temp_bp[4];
extern int16_t temp_gain[4];
extern void saveTempScale();

// Observed shift times (last good) extern
extern uint16_t observedShiftMs[8][8][8];

// Web server instance
WebServer server(80);

// HTML Content
const char index_html[] PROGMEM = R"rawliteral(
<!DOCTYPE html>
<html>
<head>
  <meta name="viewport" content="width=device-width, initial-scale=1">
  <title>TCU V5</title>
  <style>
    body { font-family: monospace; background: #121212; color: #fff; text-align: center; margin: 0; padding: 0; }
    .tab-bar { display: flex; border-bottom: 2px solid #333; background: #1e1e1e; position: relative; }
    .tab-btn { flex: 1; padding: 15px; background: #1e1e1e; color: #888; border: none; font-size: 16px; cursor: pointer; }
    .tab-btn.active { background: #333; color: #00d4ff; font-weight: bold; border-bottom: 3px solid #00d4ff; }
    .page { display: none; padding: 10px; }
    .page.active { display: block; }
    .grid { display: grid; grid-template-columns: repeat(2, 1fr); gap: 8px; margin-bottom: 15px; }
    .card { background: #1e1e1e; padding: 10px; border-radius: 4px; border: 1px solid #333; }
    .val { font-size: 20px; font-weight: bold; color: #00ff88; }
    .lbl { font-size: 10px; color: #888; text-transform: uppercase; }
    .big { font-size: 50px; color: #00d4ff; grid-column: span 2; }
    .wide { grid-column: span 2; }
    .bar-cont { width: 100%; background: #333; height: 10px; margin-top: 5px; border-radius: 2px; }
    .bar-fill { height: 100%; background: #ff0055; width: 0%; transition: width 0.2s; }
    select, button { background: #333; color: white; border: 1px solid #555; padding: 8px; margin: 5px; width: 95%; font-size: 16px; }
    button.save { background: #006622; }
    table { width: 100%; border-collapse: collapse; font-size: 11px; margin-top: 5px; }
    td { border: 1px solid #444; padding: 6px 2px; text-align: center; cursor: pointer; color: #fff; }
    .act { border: 2px solid #fff !important; box-shadow: 0 0 5px white; transform: scale(1.1); z-index:10; }
    #uptime { position:absolute; top:5px; right:10px; font-size:12px; color:#00ff88; pointer-events:none; }
    #editModal { display:none; position:fixed; top:20%; left:10%; right:10%; background:#222; padding:20px; border:2px solid #00d4ff; z-index:99; }
    input { width: 80%; padding: 10px; background: #000; color: #fff; border: 1px solid #555; font-size: 18px; text-align: center; }
    .card input { width: 60px; margin: 3px; padding: 6px; }
  </style>
</head>
<body>
  <div class="tab-bar">
    <button class="tab-btn active" onclick="showTab('dash')">DASHBOARD</button>
    <button class="tab-btn" onclick="showTab('tune')">TUNING</button>
    <div id="uptime">CONN...</div>
  </div>

  <div id="dash" class="page active">
    <div class="grid">
      <div class="card big"><span id="gear">N</span><div class="lbl">GEAR</div></div>
      <div class="card"><div class="val" id="rpm">0</div><div class="lbl">ENG RPM</div></div>
      <div class="card"><div class="val" id="spd">0</div><div class="lbl">KPH</div></div>
      <div class="card"><div class="val" id="inRpm">0</div><div class="lbl">INPUT</div></div>
      <div class="card"><div class="val" id="outRpm">0</div><div class="lbl">OUTPUT</div></div>
      <div class="card"><div class="val" id="tps">0</div><div class="lbl">TPS %</div></div>
      <div class="card"><div class="val" id="tmp">0</div><div class="lbl">TEMP C</div></div>
      <div class="card wide"><div class="val" id="sState" style="color:#ffcc00">IDLE</div><div class="lbl">SHIFT STATE</div></div>
      <div class="card"><div class="val" id="lTime">0</div><div class="lbl">LAST ms</div></div>
      <div class="card"><div class="val" id="tcc">0</div><div class="lbl">TCC</div></div>
    </div>
    <div class="card" style="text-align:left">
      <div class="lbl">MPC: <span id="mpcVal">0</span></div><div class="bar-cont"><div class="bar-fill" id="mpcBar"></div></div>
      <br>
      <div class="lbl">SPC: <span id="spcVal">0</span></div><div class="bar-cont"><div class="bar-fill" id="spcBar" style="background:#00d4ff"></div></div>
    </div>
  </div>

  <div id="tune" class="page">
    <div style="background:#1a1a1a; padding:10px;">
      <select id="mapSel" onchange="refreshMap()">
        <option value="0">Upshift 1 -> 2</option>
        <option value="1">Upshift 2 -> 3</option>
        <option value="2">Upshift 3 -> 4</option>
        <option value="3">Upshift 4 -> 5</option>
        <option value="4">Downshift 2 -> 1</option>
        <option value="5">Downshift 3 -> 2</option>
        <option value="6">Downshift 4 -> 3</option>
        <option value="7">Downshift 5 -> 4</option>
        <option value="99">Global Maps (Line/TCC)</option>
      </select>
      <select id="tblSel" onchange="refreshMap()"></select>
    </div>
    <table id="mapTable"></table>
    <br>
    <button class="save" onclick="saveToFlash()">SAVE TO FLASH</button>

    <div class="card" style="margin-top:10px;">
      <div class="lbl">TEMP SCALER (°C breakpoints / gain x100)</div>
      <div>BP: <input id="bp0"><input id="bp1"><input id="bp2"><input id="bp3"></div>
      <div>GN: <input id="gn0"><input id="gn1"><input id="gn2"><input id="gn3"></div>
      <button onclick="pushTemp()">SET TEMP SCALE</button>
    </div>
  </div>

  <div id="editModal">
    <h3>Edit Value</h3>
    <input type="number" id="newVal">
    <br><br>
    <button onclick="commit()">SET</button>
    <button onclick="closeEdit()" style="background:#552222">CANCEL</button>
  </div>

<script>
let curT=-1, curR=-1;
function showTab(id){
  document.querySelectorAll('.page').forEach(d=>d.classList.remove('active'));
  document.querySelectorAll('.tab-btn').forEach(b=>b.classList.remove('active'));
  document.getElementById(id).classList.add('active');
  event.target.classList.add('active');
}

setInterval(() => {
  fetch('/data').then(r=>r.json()).then(d=>{
    document.getElementById('rpm').innerText = d.rpm;
    document.getElementById('spd').innerText = d.spd;
    document.getElementById('tps').innerText = d.tps;
    document.getElementById('tmp').innerText = d.tmp;
    let g = d.gr; if(g==99) g="N"; else if(g==0) g="P"; else if(g<0) g="R";
    document.getElementById('gear').innerText = g;
    document.getElementById('inRpm').innerText = d.inRpm;
    document.getElementById('outRpm').innerText = d.outRpm;
    document.getElementById('sState').innerText = d.sState;
    document.getElementById('lTime').innerText = d.lTime;
    document.getElementById('tcc').innerText = d.tcc;
    let s = Math.floor(d.uptime/1000);
    let m = Math.floor(s/60); let h = Math.floor(m/60);
    document.getElementById('uptime').innerText = `${h}h ${m%60}m ${s%60}s`;
    document.getElementById('mpcVal').innerText = d.mpc;
    document.getElementById('mpcBar').style.width = (d.mpc/2.55)+'%';
    document.getElementById('spcVal').innerText = d.spc;
    document.getElementById('spcBar').style.width = (d.spc/2.55)+'%';
    document.querySelectorAll('td').forEach(c=>c.classList.remove('act'));
    let c = document.getElementById(`c_${d.ti}_${d.ri}`);
    if(c) c.classList.add('act');
  });
}, 100); // Fast UI updates (100ms)

function updateTblOpts() {
  let m = document.getElementById('mapSel').value;
  let t = document.getElementById('tblSel');
  t.innerHTML = '';
  if(m == 99) {
    t.innerHTML += '<option value="0">Line Duty (Steady MPC)</option>';
    t.innerHTML += '<option value="1">TCC Duty</option>';
  } else {
    t.innerHTML += '<option value="0">Fill Time (ms)</option>';
    t.innerHTML += '<option value="1">Fill Duty (PWM)</option>';
    t.innerHTML += '<option value="2">Shift Duty (PWM)</option>';
    t.innerHTML += '<option value="3">Shift MPC Duty (PWM)</option>';
    t.innerHTML += '<option value="4">Target Shift Time (ms)</option>';
    t.innerHTML += '<option value="5">Observed Shift Time (last good, ms)</option>';
  }
}
document.getElementById('mapSel').addEventListener('change', () => { updateTblOpts(); refreshMap(); });
updateTblOpts();

function refreshMap() {
  let m = document.getElementById('mapSel').value;
  let t = document.getElementById('tblSel').value;
  fetch(`/getmap?m=${m}&t=${t}`).then(r=>r.json()).then(res=>{
    let h = '<tr><td></td>';
    for(let i=0;i<8;i++) h+=`<td style="color:#888">${i*1000}</td>`;
    h += '</tr>';
    let data = res.data;
    let minV = 9999, maxV = -9999;
    for(let v of data) { if(v < minV) minV = v; if(v > maxV) maxV = v; }
    let range = maxV - minV; if(range === 0) range = 1;
    for(let r=0; r<8; r++) {
      h += `<tr><td style="color:#888">${Math.round((r/7)*100)}%</td>`;
      for(let c=0; c<8; c++) {
        let val = data[(r*8)+c];
        let ratio = (val - minV) / range;
        let hue = 240 - (ratio * 240);
        h += `<td id="c_${r}_${c}" style="background:hsl(${hue},60%,30%)" onclick="openEdit(${r},${c},${val})">${val}</td>`;
      }
      h += '</tr>';
    }
    document.getElementById('mapTable').innerHTML = h;
  });
}
function openEdit(r,c,old) {
  curT = r; curR = c;
  document.getElementById('newVal').value = old;
  document.getElementById('editModal').style.display = 'block';
  document.getElementById('newVal').focus();
}
function closeEdit() { document.getElementById('editModal').style.display = 'none'; }
function commit() {
  let m = document.getElementById('mapSel').value;
  let t = document.getElementById('tblSel').value;
  let v = document.getElementById('newVal').value;
  fetch(`/setmap?m=${m}&t=${t}&r=${curT}&c=${curR}&v=${v}`).then(r=>{
    if(r.ok) { refreshMap(); closeEdit(); }
    else alert("Read-only table");
  });
}
function saveToFlash() { fetch('/save').then(r=>r.text()).then(t=>alert(t)); }

// Temp scaler
function loadTemp() {
  fetch('/tempscale').then(r=>r.json()).then(d=>{
    for(let i=0;i<4;i++){
      document.getElementById('bp'+i).value = d.bp[i];
      document.getElementById('gn'+i).value = d.gn[i];
    }
  });
}
function pushTemp() {
  let qs = [];
  for(let i=0;i<4;i++){
    qs.push(`b${i}=${encodeURIComponent(document.getElementById('bp'+i).value)}`);
    qs.push(`g${i}=${encodeURIComponent(document.getElementById('gn'+i).value)}`);
  }
  fetch('/tempscale?'+qs.join('&')).then(r=>r.text()).then(alert);
}
loadTemp();

refreshMap();
</script>
</body>
</html>
)rawliteral";

void handleRoot() { server.send(200, "text/html", index_html); }
void handleData() { server.send(200, "application/json", getExtendedTelemetryJSON()); }

void handleTempScaleGet() {
  String j = "{\"bp\":[";
  for(int i=0;i<4;i++){ j += String(temp_bp[i]); if(i!=3) j+=','; }
  j += "],\"gn\":[";
  for(int i=0;i<4;i++){ j += String(temp_gain[i]); if(i!=3) j+=','; }
  j += "]}";
  server.send(200, "application/json", j);
}
void handleTempScaleSet() {
  for(int i=0;i<4;i++){
    if(server.hasArg("b"+String(i))) temp_bp[i]   = (int16_t)server.arg("b"+String(i)).toInt();
    if(server.hasArg("g"+String(i))) temp_gain[i] = (int16_t)server.arg("g"+String(i)).toInt();
  }
  saveTempScale();
  server.send(200, "text/plain", "Temp scale updated");
}

void handleGetMap() {
  if(!server.hasArg("m") || !server.hasArg("t")) { server.send(400); return; }
  int m = server.arg("m").toInt();
  int t = server.arg("t").toInt();
  String j = "{\"data\":[";
  for(int row=0; row<8; row++) {
    for(int col=0; col<8; col++) {
      int val = 0;
      if(m == 99) {
        if(t == 0) val = generalMaps.LINE_duty[row][col];
        else if(t == 1) val = generalMaps.TCC_duty[row][col];
      } else {
        if(t == 0) val = shiftMaps[m].FILL_time[row][col];
        else if(t == 1) val = shiftMaps[m].FILL_duty[row][col];
        else if(t == 2) val = shiftMaps[m].SHIFT_duty[row][col];
        else if(t == 3) val = shiftMaps[m].SHIFT_MPC_duty[row][col];
        else if(t == 4) val = shiftMaps[m].TARGET_SHIFT_time[row][col];
        else if(t == 5) val = observedShiftMs[m][row][col]; // read-only observed
      }
      j += String(val);
      if(row!=7 || col!=7) j += ",";
    }
  }
  j += "]}";
  server.send(200, "application/json", j);
}
void handleSetMap() {
  if(server.hasArg("m") && server.hasArg("t") && server.hasArg("v")) {
    int m = server.arg("m").toInt();
    int t = server.arg("t").toInt();
    int r = server.arg("r").toInt();
    int c = server.arg("c").toInt();
    int v = server.arg("v").toInt();
    if(t == 5) { server.send(403, "text/plain", "Read-only"); return; } // observed table read-only
    if(m == 99) {
        if(t == 0) generalMaps.LINE_duty[r][c] = v;
        else if(t == 1) generalMaps.TCC_duty[r][c] = v;
    } else if(m>=0 && m<8) {
        if(t == 0) shiftMaps[m].FILL_time[r][c] = v;
        else if(t == 1) shiftMaps[m].FILL_duty[r][c] = v;
        else if(t == 2) shiftMaps[m].SHIFT_duty[r][c] = v;
        else if(t == 3) shiftMaps[m].SHIFT_MPC_duty[r][c] = v;
        else if(t == 4) shiftMaps[m].TARGET_SHIFT_time[r][c] = v;
    }
    server.send(200, "text/plain", "OK");
    return;
  }
  server.send(400, "text/plain", "Error");
}
void handleSave() { persistMapsBinary(); server.send(200, "text/plain", "V5 Maps Saved!"); }

void setupWebInterface() {
  WiFi.mode(WIFI_AP);
  WiFi.softAP("MercedesTCU_V5", "72267226");
  server.on("/", handleRoot);
  server.on("/data", handleData);
  server.on("/getmap", handleGetMap);
  server.on("/setmap", handleSetMap);
  server.on("/save", handleSave);
  server.on("/tempscale", HTTP_GET, handleTempScaleGet);
  server.on("/tempscale", HTTP_ANY, handleTempScaleSet); // allow GET with params or POST
  server.begin();
  Serial.println("Web Dashboard V5 Ready (Core 0)");
}
void handleWebInterface() { server.handleClient(); }

#endif