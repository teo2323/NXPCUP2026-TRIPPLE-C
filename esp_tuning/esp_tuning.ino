//192.168.4.1

#include <WiFi.h>
#include <WebServer.h>
#include <ESPmDNS.h>
#include <Preferences.h>

/* Access Point Configuration */
const char* AP_SSID   = "ESP32_Robot_AP2";  // Network Name
const char* AP_PASS   = "12345678";        // Minimum 8 characters
const char* MDNS_NAME = "esp32_2";           // Hostname (http://esp32_2.local)

WebServer server(80);
Preferences preferences;

// Local stored PID & Motor values (default values)
float p_right       = 0.40f;
float p_left        = 0.40f;
float d_right       = 0.20f;
float d_left        = 0.20f;
float motor_speed   = 70.0f;
float decay_factor  = 0.90f;
bool engine_enabled = false;

/* Telemetry Data Structures & Ring Buffer */
struct TelemetryFrame {
    unsigned long id;
    unsigned long timestamp;
    uint8_t lineCount;
    char whichLines[16];
    uint16_t numVectors;
    uint32_t horizVectorCount;
    float steeringAngle;
    float motorSpeed;
    bool engineEnabled;
    float batteryVolts;
    int lx0, ly0, lx1, ly1;
    int rx0, ry0, rx1, ry1;
};

#define TELEMETRY_BUFFER_SIZE 300
TelemetryFrame telemetryRingBuffer[TELEMETRY_BUFFER_SIZE];
int telemetryHead = 0;
int telemetryCount = 0;
unsigned long telemetrySeqCounter = 0;

void processTelemetryLine(const String& line) {
    // Format: TELEM:lines=2|which=BOTH|num_vec=3|horiz_cnt=5|steer=-12.50|speed=70|eng=1|batt=7.40|lx0=15|ly0=48|lx1=25|ly1=10|rx0=60|ry0=48|rx1=50|ry1=10
    if (!line.startsWith("TELEM:")) return;

    TelemetryFrame frame;
    frame.id = ++telemetrySeqCounter;
    frame.timestamp = millis();
    frame.lineCount = 0;
    strcpy(frame.whichLines, "NONE");
    frame.numVectors = 0;
    frame.horizVectorCount = 0;
    frame.steeringAngle = 0.0f;
    frame.motorSpeed = 0.0f;
    frame.engineEnabled = false;
    frame.batteryVolts = 7.40f;
    frame.lx0 = 0; frame.ly0 = 0; frame.lx1 = 0; frame.ly1 = 0;
    frame.rx0 = 0; frame.ry0 = 0; frame.rx1 = 0; frame.ry1 = 0;

    String content = line.substring(6);
    int start = 0;
    while (start < content.length()) {
        int delim = content.indexOf('|', start);
        if (delim == -1) delim = content.length();
        String token = content.substring(start, delim);
        token.trim();
        start = delim + 1;

        int eq = token.indexOf('=');
        if (eq != -1) {
            String key = token.substring(0, eq);
            String val = token.substring(eq + 1);
            if (key == "lines") frame.lineCount = (uint8_t)val.toInt();
            else if (key == "which") strncpy(frame.whichLines, val.c_str(), sizeof(frame.whichLines) - 1);
            else if (key == "num_vec") frame.numVectors = (uint16_t)val.toInt();
            else if (key == "horiz_cnt") frame.horizVectorCount = (uint32_t)val.toInt();
            else if (key == "steer") frame.steeringAngle = val.toFloat();
            else if (key == "speed") frame.motorSpeed = val.toFloat();
            else if (key == "eng") frame.engineEnabled = (val.toInt() > 0);
            else if (key == "batt") frame.batteryVolts = val.toFloat();
            else if (key == "lx0") frame.lx0 = val.toInt();
            else if (key == "ly0") frame.ly0 = val.toInt();
            else if (key == "lx1") frame.lx1 = val.toInt();
            else if (key == "ly1") frame.ly1 = val.toInt();
            else if (key == "rx0") frame.rx0 = val.toInt();
            else if (key == "ry0") frame.ry0 = val.toInt();
            else if (key == "rx1") frame.rx1 = val.toInt();
            else if (key == "ry1") frame.ry1 = val.toInt();
        }
    }

    telemetryRingBuffer[telemetryHead] = frame;
    telemetryHead = (telemetryHead + 1) % TELEMETRY_BUFFER_SIZE;
    if (telemetryCount < TELEMETRY_BUFFER_SIZE) {
        telemetryCount++;
    }
}

void processSerial2Input() {
    static String rxLine = "";
    while (Serial2.available()) {
        char c = (char)Serial2.read();
        Serial.write(c); // Forward byte to USB serial debug

        if (c == '\n' || c == '\r') {
            rxLine.trim();
            if (rxLine.length() > 0) {
                if (rxLine.startsWith("TELEM:")) {
                    processTelemetryLine(rxLine);
                }
                rxLine = "";
            }
        } else {
            if (rxLine.length() < 256) {
                rxLine += c;
            } else {
                rxLine = ""; // buffer overflow safeguard
            }
        }
    }
}

/* Embedded HTML Dashboard */
const char INDEX_HTML[] PROGMEM = R"rawliteral(
<!DOCTYPE html>
<html lang="ro">
<head>
  <meta charset="UTF-8">
  <meta name="viewport" content="width=device-width, initial-scale=1.0">
  <title>NXP Robot - PID & Telemetrie Control Panel</title>
  <!-- PyScript Engine import for in-browser Python execution -->
  <script type="module" src="https://pyscript.net/releases/2024.1.1/core.js"></script>
  <link rel="stylesheet" href="https://pyscript.net/releases/2024.1.1/core.css">
  <style>
    :root {
      --bg: #0f172a;
      --card: #1e293b;
      --accent: #38bdf8;
      --accent-hover: #0284c7;
      --text: #f8fafc;
      --muted: #94a3b8;
      --border: #334155;
      --success: #22c55e;
      --danger: #ef4444;
      --danger-hover: #dc2626;
      --warning: #f59e0b;
    }
    * { box-sizing: border-box; margin: 0; padding: 0; font-family: -apple-system, BlinkMacSystemFont, "Segoe UI", Roboto, sans-serif; }
    body { background: var(--bg); color: var(--text); display: flex; flex-direction: column; align-items: center; min-height: 100vh; padding: 20px; }
    .container { width: 100%; max-width: 620px; }
    .header { text-align: center; margin-bottom: 20px; }
    .header h1 { font-size: 1.75rem; color: var(--accent); margin-bottom: 6px; }
    .header p { color: var(--muted); font-size: 0.9rem; }
    .badge { display: inline-block; background: #0369a1; color: #e0f2fe; font-size: 0.75rem; font-weight: bold; padding: 4px 10px; border-radius: 12px; margin-top: 6px; }
    
    .card { background: var(--card); border: 1px solid var(--border); border-radius: 14px; padding: 20px; margin-bottom: 18px; box-shadow: 0 4px 12px rgba(0,0,0,0.3); }
    .card-title { font-size: 1.1rem; color: var(--text); margin-bottom: 16px; border-bottom: 1px solid var(--border); padding-bottom: 8px; font-weight: 600; display: flex; justify-content: space-between; align-items: center; }
    
    .estop-btn { width: 100%; background: var(--danger); color: #fff; border: none; font-size: 1.25rem; font-weight: 800; padding: 16px; border-radius: 12px; cursor: pointer; transition: all 0.2s; box-shadow: 0 4px 14px rgba(239, 68, 68, 0.4); text-transform: uppercase; letter-spacing: 1px; margin-bottom: 18px; }
    .estop-btn:hover { background: var(--danger-hover); transform: scale(1.01); }
    .estop-btn:active { transform: scale(0.98); }

    .engine-controls { display: flex; gap: 10px; margin-top: 14px; }
    .btn-start { flex: 1; background: var(--success); color: #052e16; font-size: 1rem; padding: 12px; font-weight: bold; border: none; border-radius: 8px; cursor: pointer; }
    .btn-start:hover { background: #16a34a; color: #fff; }
    .btn-pause { flex: 1; background: var(--warning); color: #451a03; font-size: 1rem; padding: 12px; font-weight: bold; border: none; border-radius: 8px; cursor: pointer; }
    .btn-pause:hover { background: #d97706; color: #fff; }

    .status-pill { font-size: 0.85rem; padding: 4px 10px; border-radius: 10px; font-weight: bold; }
    .status-on { background: #14532d; color: #4ade80; border: 1px solid #22c55e; }
    .status-off { background: #7f1d1d; color: #fca5a5; border: 1px solid #ef4444; }

    .param-group { margin-bottom: 18px; }
    .param-group:last-child { margin-bottom: 0; }
    .param-header { display: flex; justify-content: space-between; align-items: center; margin-bottom: 8px; }
    .param-name { font-weight: 600; font-size: 0.95rem; color: #e2e8f0; }
    .param-val { font-family: monospace; font-size: 1rem; color: var(--accent); background: #090d16; padding: 2px 8px; border-radius: 6px; border: 1px solid var(--border); }
    .controls { display: flex; gap: 10px; align-items: center; margin-top: 6px; }
    input[type=range] { flex: 1; accent-color: var(--accent); cursor: pointer; height: 6px; }
    input[type=number] { width: 85px; padding: 6px 8px; background: #0f172a; border: 1px solid var(--border); border-radius: 6px; color: #fff; font-size: 0.95rem; text-align: center; }
    .btn { background: var(--accent); color: #0f172a; border: none; font-weight: bold; padding: 8px 14px; border-radius: 6px; cursor: pointer; transition: all 0.2s; }
    .btn:hover { background: var(--accent-hover); color: #fff; }
    .btn-all { width: 100%; padding: 12px; font-size: 1rem; margin-top: 10px; background: #0284c7; color: #fff; }
    
    /* Telemetry & Terminal UI Styles */
    .term-box { background: #090d16; border: 1px solid var(--border); border-radius: 8px; font-family: monospace; font-size: 0.82rem; height: 230px; overflow-y: auto; padding: 10px; color: #38bdf8; margin-top: 10px; display: flex; flex-direction: column; gap: 4px; }
    .term-line { white-space: pre-wrap; word-break: break-all; }
    .term-line .ts { color: #64748b; margin-right: 6px; }
    .term-line .tag { color: #e2e8f0; font-weight: bold; }
    .term-line .val { color: #4ade80; }
    .term-line .warn { color: #f59e0b; }
    .term-line .err { color: #ef4444; }
    .term-controls { display: flex; gap: 8px; flex-wrap: wrap; align-items: center; justify-content: space-between; margin-top: 8px; }
    .term-controls-left { display: flex; gap: 8px; align-items: center; font-size: 0.85rem; color: var(--muted); }
    .btn-sm { padding: 6px 12px; font-size: 0.8rem; border-radius: 6px; }
    .btn-json { background: #10b981; color: #042f2e; font-weight: bold; border: none; padding: 8px 14px; border-radius: 8px; cursor: pointer; transition: all 0.2s; }
    .btn-json:hover { background: #059669; color: #fff; }
    .grid-2 { display: grid; grid-template-columns: 1fr 1fr; gap: 10px; margin-bottom: 12px; }
    .metric-box { background: #090d16; border: 1px solid var(--border); border-radius: 8px; padding: 10px; text-align: center; }
    .metric-title { font-size: 0.75rem; color: var(--muted); text-transform: uppercase; margin-bottom: 4px; }
    .metric-val { font-size: 1.15rem; font-weight: bold; color: var(--accent); font-family: monospace; }
    .line-pill { display: inline-block; padding: 3px 8px; border-radius: 6px; font-size: 0.8rem; font-weight: bold; }
    .pill-both { background: #14532d; color: #4ade80; border: 1px solid #22c55e; }
    .pill-single { background: #78350f; color: #fde047; border: 1px solid #f59e0b; }
    .pill-turn { background: #0e7490; color: #67e8f9; border: 1px solid #06b6d4; }
    .pill-none { background: #7f1d1d; color: #fca5a5; border: 1px solid #ef4444; }

    /* Pixy 2D Visualizer & PyScript Styling */
    .pixy-canvas-container { position: relative; width: 100%; background: #030712; border: 1px solid var(--border); border-radius: 10px; padding: 8px; text-align: center; margin-bottom: 8px; }
    #pixyCanvas { width: 100%; max-width: 500px; height: 255px; background: #090d16; border-radius: 8px; display: block; margin: 0 auto; }
    .coords-badge-container { display: flex; flex-direction: column; gap: 6px; font-family: monospace; font-size: 0.82rem; margin-top: 6px; background: #0f172a; padding: 10px; border-radius: 8px; border: 1px solid var(--border); }
    .coords-left { color: #38bdf8; font-weight: bold; }
    .coords-right { color: #4ade80; font-weight: bold; }
    .py-analytics-box { background: #090d16; border: 1px solid #0284c7; border-radius: 8px; padding: 10px; font-family: monospace; font-size: 0.82rem; color: #38bdf8; margin-top: 10px; }

    #toast { position: fixed; bottom: 20px; left: 50%; transform: translateX(-50%); background: var(--success); color: #052e16; padding: 12px 24px; border-radius: 8px; font-weight: bold; opacity: 0; transition: opacity 0.3s; pointer-events: none; box-shadow: 0 4px 12px rgba(0,0,0,0.5); z-index: 1000; }
    #toast.show { opacity: 1; }
    #toast.danger-toast { background: var(--danger); color: #fff; }
  </style>
</head>
<body>
  <div class="container">
    <div class="header">
      <h1>NXP Robot Control Panel</h1>
      <p>Tuning PID & Telemetrie Consola (Port 80)</p>
      <div class="badge">AP: ESP32_Robot_AP2 | http://esp32_2.local</div>
    </div>

    <!-- Emergency Stop Button -->
    <button class="estop-btn" onclick="triggerEmergencyStop()">FRANA URGENTA (E-STOP)</button>

    <!-- 2D Pixy Camera Frame Visualizer & Line Coords Card -->
    <div class="card">
      <div class="card-title">
        <span>Vizualizator 2D Pixy Camera (78x51 px)</span>
        <span id="pixy_state_badge" class="line-pill pill-none">0 (NONE)</span>
      </div>

      <div class="pixy-canvas-container">
        <canvas id="pixyCanvas" width="390" height="255"></canvas>
      </div>

      <div class="coords-badge-container">
        <div class="coords-left" id="coord_left_text">Linia Stanga: (lx0=0, ly0=0) -> (lx1=0, ly1=0) [NEDETECTAT]</div>
        <div class="coords-right" id="coord_right_text">Linia Dreapta: (rx0=0, ry0=0) -> (rx1=0, ry1=0) [NEDETECTAT]</div>
      </div>
    </div>

    <!-- Python PyScript Analytics & Steering Diagram Card -->
    <div class="card">
      <div class="card-title">
        <span>Diagrame Live & Analiza Python (PyScript Engine)</span>
        <button class="btn btn-sm" onclick="if(window.triggerPyAnalysis) window.triggerPyAnalysis();">Ruleaza Python</button>
      </div>

      <div style="width: 100%; height: 150px; background: #090d16; border: 1px solid var(--border); border-radius: 8px; padding: 6px; position: relative;">
        <canvas id="steerChartCanvas" width="550" height="138" style="width: 100%; height: 100%;"></canvas>
      </div>

      <div id="py_analytics_box" class="py-analytics-box">
        Engine-ul Python PyScript este gata. Se asteapta cadru telemetrie...
      </div>
    </div>

    <!-- Telemetry & Terminal Simulation Card -->
    <div class="card">
      <div class="card-title">
        <span>Telemetrie & Consola Terminal</span>
        <button class="btn-json" onclick="downloadJSONHistory()">Descarca JSON</button>
      </div>

      <div class="grid-2">
        <div class="metric-box">
          <div class="metric-title">Linii Descoperite</div>
          <div id="metric_lines" class="metric-val"><span class="line-pill pill-none">0 (NONE)</span></div>
        </div>
        <div class="metric-box">
          <div class="metric-title">Unghi Directie (Steer)</div>
          <div id="metric_steer" class="metric-val">0.0 deg</div>
        </div>
        <div class="metric-box">
          <div class="metric-title">Vectori Detectati</div>
          <div id="metric_vecs" class="metric-val">0</div>
        </div>
        <div class="metric-box">
          <div class="metric-title">Vectori Orizontali</div>
          <div id="metric_horiz_cnt" class="metric-val">0</div>
        </div>
        <div class="metric-box">
          <div class="metric-title">Tensiune Baterie</div>
          <div id="metric_batt" class="metric-val">7.40 V</div>
        </div>
      </div>

      <div class="term-controls">
        <div class="term-controls-left">
          <label><input type="checkbox" id="term_autoscroll" checked> Auto-scroll</label>
        </div>
        <div style="display:flex; gap:6px;">
          <button class="btn btn-sm" id="btn_pause_feed" onclick="togglePauseFeed()">Pauza</button>
          <button class="btn btn-sm" style="background:#475569; color:#fff;" onclick="clearTerminal()">Goleste</button>
        </div>
      </div>

      <div id="terminal_box" class="term-box">
        <div class="term-line"><span class="ts">[SYS]</span> <span class="tag">Consola Telemetrie NXP -> ESP32 conectata. Se asteapta pachete TELEM...</span></div>
      </div>
    </div>

    <!-- Engine & Speed Control Card -->
    <div class="card">
      <div class="card-title">
        <span>Control Viteza Motoare</span>
        <span id="engine_status" class="status-pill status-off">STOPPED</span>
      </div>

      <div class="param-group">
        <div class="param-header">
          <span class="param-name">MOTOR_SPEED (Viteza %)</span>
          <span class="param-val" id="val_SPEED">70%</span>
        </div>
        <div class="controls">
          <input type="range" id="slider_SPEED" min="0" max="100" step="1" value="70" oninput="syncSpeed(this.value)">
          <input type="number" id="num_SPEED" min="0" max="100" step="1" value="70" oninput="syncSpeedSlider(this.value)">
          <button class="btn" onclick="saveSpeed()">Set Speed</button>
        </div>
      </div>

      <div class="engine-controls">
        <button class="btn-start" onclick="setEngine(1)">PORNESTE MOTOARELE</button>
        <button class="btn-pause" onclick="setEngine(0)">OPRESTE MOTOARELE</button>
      </div>
    </div>

    <!-- PID Proportional Card -->
    <div class="card">
      <div class="card-title">Proportional (P Gains)</div>
      
      <div class="param-group">
        <div class="param-header">
          <span class="param-name">STEERING_P_RIGHT</span>
          <span class="param-val" id="val_P_RIGHT">0.400</span>
        </div>
        <div class="controls">
          <input type="range" id="slider_P_RIGHT" min="0" max="10" step="0.005" value="0.4" oninput="syncVal('P_RIGHT', this.value)">
          <input type="number" id="num_P_RIGHT" min="0" max="10" step="0.005" value="0.4" oninput="syncSlider('P_RIGHT', this.value)">
          <button class="btn" onclick="saveParam('STEERING_P_RIGHT', 'P_RIGHT')">Save</button>
        </div>
      </div>

      <div class="param-group">
        <div class="param-header">
          <span class="param-name">STEERING_P_LEFT</span>
          <span class="param-val" id="val_P_LEFT">0.400</span>
        </div>
        <div class="controls">
          <input type="range" id="slider_P_LEFT" min="0" max="10" step="0.005" value="0.4" oninput="syncVal('P_LEFT', this.value)">
          <input type="number" id="num_P_LEFT" min="0" max="10" step="0.005" value="0.4" oninput="syncSlider('P_LEFT', this.value)">
          <button class="btn" onclick="saveParam('STEERING_P_LEFT', 'P_LEFT')">Save</button>
        </div>
      </div>
    </div>

    <!-- PID Derivative Card -->
    <div class="card">
      <div class="card-title">Derivativ (D Gains)</div>
      
      <div class="param-group">
        <div class="param-header">
          <span class="param-name">STEERING_D_RIGHT</span>
          <span class="param-val" id="val_D_RIGHT">0.200</span>
        </div>
        <div class="controls">
          <input type="range" id="slider_D_RIGHT" min="0" max="10" step="0.005" value="0.2" oninput="syncVal('D_RIGHT', this.value)">
          <input type="number" id="num_D_RIGHT" min="0" max="10" step="0.005" value="0.2" oninput="syncSlider('D_RIGHT', this.value)">
          <button class="btn" onclick="saveParam('STEERING_D_RIGHT', 'D_RIGHT')">Save</button>
        </div>
      </div>

      <div class="param-group">
        <div class="param-header">
          <span class="param-name">STEERING_D_LEFT</span>
          <span class="param-val" id="val_D_LEFT">0.200</span>
        </div>
        <div class="controls">
          <input type="range" id="slider_D_LEFT" min="0" max="10" step="0.005" value="0.2" oninput="syncVal('D_LEFT', this.value)">
          <input type="number" id="num_D_LEFT" min="0" max="10" step="0.005" value="0.2" oninput="syncSlider('D_LEFT', this.value)">
          <button class="btn" onclick="saveParam('STEERING_D_LEFT', 'D_LEFT')">Save</button>
        </div>
      </div>
    </div>

    <!-- Decay Factor Card -->
    <div class="card">
      <div class="card-title">Decay Factor</div>
      <div class="param-group">
        <div class="param-header">
          <span class="param-name">DECAY_FACTOR</span>
          <span class="param-val" id="val_DECAY">0.900</span>
        </div>
        <div class="controls">
          <input type="range" id="slider_DECAY" min="0.5" max="1" step="0.005" value="0.9" oninput="syncVal('DECAY', this.value)">
          <input type="number" id="num_DECAY" min="0.5" max="1" step="0.005" value="0.9" oninput="syncSlider('DECAY', this.value)">
          <button class="btn" onclick="saveParam('DECAY_FACTOR', 'DECAY')">Save</button>
        </div>
      </div>
    </div>

    <button class="btn btn-all" onclick="saveAllPID()">Salveaza Toti Parametrii PID</button>
  </div>

  <div id="toast">Salvat cu succes!</div>

  <script>
    window.telemetryHistory = [];
    let lastTelemetryId = 0;
    let isFeedPaused = false;

    const syncVal = (id, val) => {
      document.getElementById('num_' + id).value = val;
      document.getElementById('val_' + id).innerText = parseFloat(val).toFixed(3);
    }
    const syncSlider = (id, val) => {
      document.getElementById('slider_' + id).value = val;
      document.getElementById('val_' + id).innerText = parseFloat(val).toFixed(3);
    }
    const syncSpeed = (val) => {
      document.getElementById('num_SPEED').value = val;
      document.getElementById('val_SPEED').innerText = parseInt(val) + '%';
    }
    const syncSpeedSlider = (val) => {
      document.getElementById('slider_SPEED').value = val;
      document.getElementById('val_SPEED').innerText = parseInt(val) + '%';
    }
    const showToast = (msg, isDanger) => {
      const t = document.getElementById('toast');
      t.innerText = msg;
      if (isDanger) t.classList.add('danger-toast');
      else t.classList.remove('danger-toast');
      t.classList.add('show');
      setTimeout(function() { t.classList.remove('show'); }, 2200);
    }
    const updateEngineBadge = (running) => {
      const b = document.getElementById('engine_status');
      if (running) {
        b.innerText = 'RUNNING (' + document.getElementById('num_SPEED').value + '%)';
        b.className = 'status-pill status-on';
      } else {
        b.innerText = 'STOPPED';
        b.className = 'status-pill status-off';
      }
    }
    const saveParam = (paramName, id) => {
      const val = document.getElementById('num_' + id).value;
      fetch('/set?param=' + encodeURIComponent(paramName) + '&val=' + encodeURIComponent(val))
        .then(function(r) {
          if (!r.ok) throw new Error('HTTP ' + r.status);
          return r.json();
        })
        .then(function(d) { showToast('Transmis pe NXP: ' + paramName + ' = ' + val, false); })
        .catch(function(e) { showToast('Eroare salvare ' + paramName, true); });
    }
    const saveSpeed = () => {
      const val = document.getElementById('num_SPEED').value;
      fetch('/set?param=MOTOR_SPEED&val=' + encodeURIComponent(val))
        .then(function(r) {
          if (!r.ok) throw new Error('HTTP ' + r.status);
          return r.json();
        })
        .then(function(d) { showToast('Viteza setata la: ' + val + '%', false); })
        .catch(function(e) { showToast('Eroare setare viteza!', true); });
    }
    const setEngine = (state) => {
      fetch('/set?param=ENGINE_ENABLED&val=' + state)
        .then(function(r) {
          if (!r.ok) throw new Error('HTTP ' + r.status);
          return r.json();
        })
        .then(function(d) {
          updateEngineBadge(state === 1);
          showToast(state === 1 ? 'Motoare PORNITE!' : 'Motoare OPRITE', false);
        })
        .catch(function(e) { showToast('Eroare comutare motoare!', true); });
    }
    const triggerEmergencyStop = () => {
      fetch('/set?param=EMERGENCY_STOP&val=1')
        .then(function(r) {
          if (!r.ok) throw new Error('HTTP ' + r.status);
          return r.json();
        })
        .then(function(d) {
          updateEngineBadge(false);
          showToast('EMERGENCY STOP ACTIVAT!', true);
        })
        .catch(function(e) { showToast('EMERGENCY STOP TRIMIS!', true); });
    }
    const saveAllPID = () => {
      saveParam('STEERING_P_RIGHT', 'P_RIGHT');
      setTimeout(() => { saveParam('STEERING_P_LEFT', 'P_LEFT'); }, 250);
      setTimeout(() => { saveParam('STEERING_D_RIGHT', 'D_RIGHT'); }, 500);
      setTimeout(() => { saveParam('STEERING_D_LEFT', 'D_LEFT'); }, 750);
      setTimeout(() => { saveParam('DECAY_FACTOR', 'DECAY'); }, 1000);
    }

    /* Telemetry & Terminal Functions */
    const togglePauseFeed = () => {
      isFeedPaused = !isFeedPaused;
      const btn = document.getElementById('btn_pause_feed');
      btn.innerText = isFeedPaused ? 'Reia' : 'Pauza';
      showToast(isFeedPaused ? 'Feed terminal pus pe pauza' : 'Feed terminal reluat', false);
    }

    const clearTerminal = () => {
      document.getElementById('terminal_box').innerHTML = '<div class="term-line"><span class="ts">[SYS]</span> <span class="tag">Terminal curatat.</span></div>';
    }

    /* 2D Pixy Frame & Coordinates Canvas Visualizer */
    const drawPixyFrame2D = (item) => {
      const cvs = document.getElementById('pixyCanvas');
      if (!cvs) return;
      const ctx = cvs.getContext('2d');
      const W = cvs.width;   // 390
      const H = cvs.height;  // 255
      const scaleX = W / 78.0; // 5
      const scaleY = H / 51.0; // 5

      // Background
      ctx.fillStyle = '#090d16';
      ctx.fillRect(0, 0, W, H);

      // Grid lines (every 10 units)
      ctx.strokeStyle = '#1e293b';
      ctx.lineWidth = 1;
      for (let x = 0; x <= 78; x += 10) {
        ctx.beginPath();
        ctx.moveTo(x * scaleX, 0);
        ctx.lineTo(x * scaleX, H);
        ctx.stroke();
      }
      for (let y = 0; y <= 51; y += 10) {
        ctx.beginPath();
        ctx.moveTo(0, y * scaleY);
        ctx.lineTo(W, y * scaleY);
        ctx.stroke();
      }

      // Centerline X=39
      ctx.strokeStyle = '#475569';
      ctx.setLineDash([4, 4]);
      ctx.beginPath();
      ctx.moveTo(39 * scaleX, 0);
      ctx.lineTo(39 * scaleX, H);
      ctx.stroke();
      ctx.setLineDash([]);

      // Axis labels
      ctx.fillStyle = '#64748b';
      ctx.font = '10px monospace';
      ctx.fillText('TOP (Y=0)', 10, 14);
      ctx.fillText('VEHICLE (Y=51)', 10, H - 8);
      ctx.fillText('X=39 (CENTER)', 39 * scaleX - 35, 14);

      // Left Line (x0, y0) -> (x1, y1)
      const lText = document.getElementById('coord_left_text');
      if (item.lx0 !== undefined && (item.lx0 > 0 || item.ly0 > 0 || item.lx1 > 0 || item.ly1 > 0)) {
        const lx0_p = item.lx0 * scaleX;
        const ly0_p = item.ly0 * scaleY;
        const lx1_p = item.lx1 * scaleX;
        const ly1_p = item.ly1 * scaleY;

        ctx.strokeStyle = '#38bdf8';
        ctx.lineWidth = 3.5;
        ctx.beginPath();
        ctx.moveTo(lx0_p, ly0_p);
        ctx.lineTo(lx1_p, ly1_p);
        ctx.stroke();

        ctx.fillStyle = '#0284c7';
        ctx.beginPath(); ctx.arc(lx0_p, ly0_p, 5, 0, 2 * Math.PI); ctx.fill();
        ctx.fillStyle = '#e0f2fe';
        ctx.beginPath(); ctx.arc(lx1_p, ly1_p, 5, 0, 2 * Math.PI); ctx.fill();

        ctx.fillStyle = '#38bdf8';
        ctx.font = 'bold 11px monospace';
        ctx.fillText('L0(' + item.lx0 + ',' + item.ly0 + ')', lx0_p + 6, ly0_p - 4);
        ctx.fillText('L1(' + item.lx1 + ',' + item.ly1 + ')', lx1_p + 6, ly1_p - 4);

        if (lText) lText.innerText = 'Linia Stanga: (x0=' + item.lx0 + ', y0=' + item.ly0 + ') -> (x1=' + item.lx1 + ', y1=' + item.ly1 + ')';
      } else {
        if (lText) lText.innerText = 'Linia Stanga: [NEDETECTAT / INACTIV]';
      }

      // Right Line (x0, y0) -> (x1, y1)
      const rText = document.getElementById('coord_right_text');
      if (item.rx0 !== undefined && (item.rx0 > 0 || item.ry0 > 0 || item.rx1 > 0 || item.ry1 > 0)) {
        const rx0_p = item.rx0 * scaleX;
        const ry0_p = item.ry0 * scaleY;
        const rx1_p = item.rx1 * scaleX;
        const ry1_p = item.rx1 * scaleY;

        ctx.strokeStyle = '#22c55e';
        ctx.lineWidth = 3.5;
        ctx.beginPath();
        ctx.moveTo(rx0_p, ry0_p);
        ctx.lineTo(rx1_p, ry1_p);
        ctx.stroke();

        ctx.fillStyle = '#15803d';
        ctx.beginPath(); ctx.arc(rx0_p, ry0_p, 5, 0, 2 * Math.PI); ctx.fill();
        ctx.fillStyle = '#dcfce7';
        ctx.beginPath(); ctx.arc(rx1_p, ry1_p, 5, 0, 2 * Math.PI); ctx.fill();

        ctx.fillStyle = '#4ade80';
        ctx.font = 'bold 11px monospace';
        ctx.fillText('R0(' + item.rx0 + ',' + item.ry0 + ')', rx0_p + 6, ry0_p - 4);
        ctx.fillText('R1(' + item.rx1 + ',' + item.ry1 + ')', rx1_p + 6, ry1_p - 4);

        if (rText) rText.innerText = 'Linia Dreapta: (x0=' + item.rx0 + ', y0=' + item.ry0 + ') -> (x1=' + item.rx1 + ', y1=' + item.ry1 + ')';
      } else {
        if (rText) rText.innerText = 'Linia Dreapta: [NEDETECTAT / INACTIV]';
      }

      // Steering Heading Arrow
      const steerRad = (item.steer * Math.PI) / 180.0;
      const originX = 39 * scaleX;
      const originY = H;
      const arrowLen = 60;
      const endX = originX + arrowLen * Math.sin(steerRad);
      const endY = originY - arrowLen * Math.cos(steerRad);

      ctx.strokeStyle = '#f59e0b';
      ctx.lineWidth = 2.5;
      ctx.beginPath();
      ctx.moveTo(originX, originY);
      ctx.lineTo(endX, endY);
      ctx.stroke();

      ctx.fillStyle = '#f59e0b';
      ctx.font = 'bold 10px monospace';
      ctx.fillText('STEER: ' + item.steer.toFixed(1) + ' deg', endX - 25, endY - 6);

      // State Badge
      const b = document.getElementById('pixy_state_badge');
      if (b) {
        let pillClass = 'pill-none';
        if (item.which === 'BOTH') pillClass = 'pill-both';
        else if (item.which === 'LEFT' || item.which === 'RIGHT') pillClass = 'pill-single';
        else if (item.which.indexOf('TURN') !== -1) pillClass = 'pill-turn';
        b.className = 'line-pill ' + pillClass;
        b.innerText = item.lines + ' (' + item.which + ')';
      }
    }

    /* Steering Angle Time Series Chart */
    const updateSteerChart = (item) => {
      const cvs = document.getElementById('steerChartCanvas');
      if (!cvs || !window.telemetryHistory) return;
      const ctx = cvs.getContext('2d');
      const W = cvs.width;
      const H = cvs.height;

      ctx.fillStyle = '#090d16';
      ctx.fillRect(0, 0, W, H);

      const zeroY = H / 2;
      ctx.strokeStyle = '#334155';
      ctx.lineWidth = 1;
      ctx.beginPath();
      ctx.moveTo(0, zeroY);
      ctx.lineTo(W, zeroY);
      ctx.stroke();

      ctx.fillStyle = '#64748b';
      ctx.font = '9px monospace';
      ctx.fillText('+45 deg (RIGHT)', 4, 12);
      ctx.fillText('0 deg (CENTER)', 4, zeroY - 3);
      ctx.fillText('-45 deg (LEFT)', 4, H - 4);

      const data = window.telemetryHistory.slice(-60);
      if (data.length < 2) return;

      const dx = W / 60;
      ctx.strokeStyle = '#38bdf8';
      ctx.lineWidth = 2;
      ctx.beginPath();

      data.forEach(function(pt, idx) {
        const x = idx * dx;
        const y = zeroY - (pt.steer / 45.0) * (H / 2 - 10);
        if (idx === 0) ctx.moveTo(x, y);
        else ctx.lineTo(x, y);
      });

      ctx.stroke();
    }

    const updateTelemetryUI = (item) => {
      if (!item) return;

      // Update UI cards
      const linesElem = document.getElementById('metric_lines');
      let pillClass = 'pill-none';
      if (item.which === 'BOTH') pillClass = 'pill-both';
      else if (item.which === 'LEFT' || item.which === 'RIGHT') pillClass = 'pill-single';
      else if (item.which.indexOf('TURN') !== -1) pillClass = 'pill-turn';

      linesElem.innerHTML = '<span class="line-pill ' + pillClass + '">' + item.lines + ' (' + item.which + ')</span>';
      document.getElementById('metric_steer').innerText = (item.steer > 0 ? '+' : '') + item.steer.toFixed(1) + ' deg';
      document.getElementById('metric_vecs').innerText = item.num_vec;
      if (document.getElementById('metric_horiz_cnt')) {
        document.getElementById('metric_horiz_cnt').innerText = item.horiz_cnt !== undefined ? item.horiz_cnt : 0;
      }
      document.getElementById('metric_batt').innerText = item.batt.toFixed(2) + ' V';

      // Update 2D Pixy Canvas & Steering Chart
      drawPixyFrame2D(item);
      updateSteerChart(item);

      // Trigger PyScript Python engine analysis if available
      if (window.triggerPyAnalysis) {
        try { window.triggerPyAnalysis(); } catch(e) {}
      }

      // Update simulated terminal box
      if (!isFeedPaused) {
        const box = document.getElementById('terminal_box');
        const lineDiv = document.createElement('div');
        lineDiv.className = 'term-line';

        const sec = (item.ts / 1000).toFixed(1);
        let statusColor = item.lines === 2 ? 'val' : (item.lines === 1 ? 'warn' : 'err');

        lineDiv.innerHTML = '<span class="ts">[' + sec + 's #' + item.id + ']</span> <span class="tag">TELEM</span> | Linii: <span class="' + statusColor + '">' + item.lines + ' (' + item.which + ')</span> | Vecs: <span class="val">' + item.num_vec + '</span> | Horiz: <span class="val">' + (item.horiz_cnt !== undefined ? item.horiz_cnt : 0) + '</span> | Steer: <span class="val">' + (item.steer > 0 ? '+' : '') + item.steer.toFixed(1) + ' deg</span> | Speed: ' + item.speed + '% | Batt: ' + item.batt.toFixed(2) + 'V';

        box.appendChild(lineDiv);

        if (document.getElementById('term_autoscroll').checked) {
          box.scrollTop = box.scrollHeight;
        }

        while (box.children.length > 200) {
          box.removeChild(box.firstChild);
        }
      }
    }

    const pollTelemetry = () => {
      fetch('/api/telemetry?since=' + lastTelemetryId)
        .then(function(r) { return r.json(); })
        .then(function(d) {
          if (d.new_items && d.new_items.length > 0) {
            d.new_items.forEach(function(item) {
              window.telemetryHistory.push(item);
              if (item.id > lastTelemetryId) {
                lastTelemetryId = item.id;
              }
              updateTelemetryUI(item);
            });
          } else if (d.latest) {
            updateTelemetryUI(d.latest);
          }
        })
        .catch(function(e) {})
        .finally(function() {
          setTimeout(pollTelemetry, 150);
        });
    }

    const downloadJSONHistory = () => {
      if (!window.telemetryHistory || window.telemetryHistory.length === 0) {
        showToast('Niciun pachet de telemetrie inregistrat inca!', true);
        return;
      }

      const exportData = {
        exported_at: new Date().toISOString(),
        total_records: window.telemetryHistory.length,
        records: window.telemetryHistory
      };

      const jsonStr = JSON.stringify(exportData, null, 2);
      const blob = new Blob([jsonStr], { type: 'application/json' });
      const url = URL.createObjectURL(blob);

      const nowStr = new Date().toISOString().replace(/[:.]/g, '-').slice(0, 19);
      const a = document.createElement('a');
      a.href = url;
      a.download = 'nxp_telemetry_' + nowStr + '.json';
      document.body.appendChild(a);
      a.click();
      document.body.removeChild(a);
      URL.revokeObjectURL(url);

      showToast('S-au descarcat ' + window.telemetryHistory.length + ' inregistrari JSON!', false);
    }

    window.onload = function() {
      fetch('/api/params')
        .then(function(r) { return r.json(); })
        .then(function(d) {
          if (d.STEERING_P_RIGHT !== undefined) { syncVal('P_RIGHT', d.STEERING_P_RIGHT); syncSlider('P_RIGHT', d.STEERING_P_RIGHT); }
          if (d.STEERING_P_LEFT !== undefined) { syncVal('P_LEFT', d.STEERING_P_LEFT); syncSlider('P_LEFT', d.STEERING_P_LEFT); }
          if (d.STEERING_D_RIGHT !== undefined) { syncVal('D_RIGHT', d.STEERING_D_RIGHT); syncSlider('D_RIGHT', d.STEERING_D_RIGHT); }
          if (d.STEERING_D_LEFT !== undefined) { syncVal('D_LEFT', d.STEERING_D_LEFT); syncSlider('D_LEFT', d.STEERING_D_LEFT); }
          if (d.MOTOR_SPEED !== undefined) { syncSpeed(d.MOTOR_SPEED); syncSpeedSlider(d.MOTOR_SPEED); }
          if (d.DECAY_FACTOR !== undefined) { syncVal('DECAY', d.DECAY_FACTOR); syncSlider('DECAY', d.DECAY_FACTOR); }
          if (d.ENGINE_ENABLED !== undefined) { updateEngineBadge(d.ENGINE_ENABLED === 1); }
        })
        .catch(function(e) { console.log('Init fetch error:', e); });

      pollTelemetry();
    }
  </script>

  <!-- PyScript Python Script Engine Block -->
  <script type="py">
    from js import window, document

    def run_py_analysis():
        try:
            history = window.telemetryHistory
            if not history or len(history) == 0:
                return
            n = len(history)
            both = sum(1 for item in history if getattr(item, 'which', '') == 'BOTH')
            left = sum(1 for item in history if getattr(item, 'which', '') == 'LEFT')
            right = sum(1 for item in history if getattr(item, 'which', '') == 'RIGHT')
            turn = sum(1 for item in history if 'TURN' in str(getattr(item, 'which', '')))
            
            steers = [float(getattr(x, 'steer', 0)) for x in history]
            avg_s = sum(steers) / len(steers) if steers else 0.0
            max_s = max(steers) if steers else 0.0
            min_s = min(steers) if steers else 0.0

            out = document.getElementById('py_analytics_box')
            if out:
                out.innerHTML = f"<b>Python PyScript Engine Analytics:</b><br>" \
                                f"Total Cadre Procesate: {n} | Ambe Linii: {both} ({both*100//n}%) | Stanga: {left} | Dreapta: {right} | Viraj: {turn}<br>" \
                                f"Unghi Mediu Steer: {avg_s:.2f} deg | Max Steer Dreapta: +{max_s:.1f} deg | Max Steer Stanga: {min_s:.1f} deg"
        except Exception as e:
            pass

    window.triggerPyAnalysis = run_py_analysis
  </script>
</body>
</html>
)rawliteral";

/* Load persisted PID & Motor params from Preferences */
void loadStoredParams() {
    preferences.begin("pid_tuning", false);
    p_right       = preferences.getFloat("P_RIGHT", 0.40f);
    p_left        = preferences.getFloat("P_LEFT",  0.40f);
    d_right       = preferences.getFloat("D_RIGHT", 0.20f);
    d_left        = preferences.getFloat("D_LEFT",  0.20f);
    motor_speed   = preferences.getFloat("SPEED",   70.0f);
    decay_factor  = preferences.getFloat("DECAY",   0.90f);
    engine_enabled = false; // Always start stopped for safety
}

/* Transmit current params to NXP over UART */
void sendParamsToNXP() {
    Serial2.printf("STEERING_P_RIGHT = %.4f\n", p_right);
    delay(40);
    Serial2.printf("STEERING_P_LEFT = %.4f\n", p_left);
    delay(40);
    Serial2.printf("STEERING_D_RIGHT = %.4f\n", d_right);
    delay(40);
    Serial2.printf("STEERING_D_LEFT = %.4f\n", d_left);
    delay(40);
    Serial2.printf("MOTOR_SPEED = %.4f\n", motor_speed);
    delay(40);
    Serial2.printf("DECAY_FACTOR = %.4f\n", decay_factor);
    delay(40);
    Serial2.printf("ENGINE_ENABLED = 0\n"); // Ensure NXP starts stopped
    delay(40);
}

/* HTTP Handlers */
void handleRoot() {
    server.sendHeader("Access-Control-Allow-Origin", "*");
    server.send(200, "text/html", INDEX_HTML);
}

void handleGetParams() {
    server.sendHeader("Access-Control-Allow-Origin", "*");
    String json = "{";
    json += "\"STEERING_P_RIGHT\":" + String(p_right, 4) + ",";
    json += "\"STEERING_P_LEFT\":" + String(p_left, 4) + ",";
    json += "\"STEERING_D_RIGHT\":" + String(d_right, 4) + ",";
    json += "\"STEERING_D_LEFT\":" + String(d_left, 4) + ",";
    json += "\"MOTOR_SPEED\":" + String(motor_speed, 1) + ",";
    json += "\"DECAY_FACTOR\":" + String(decay_factor, 4) + ",";
    json += "\"ENGINE_ENABLED\":" + String(engine_enabled ? 1 : 0);
    json += "}";
    server.send(200, "application/json", json);
}

void handleGetTelemetry() {
    server.sendHeader("Access-Control-Allow-Origin", "*");
    unsigned long sinceId = 0;
    if (server.hasArg("since")) {
        sinceId = (unsigned long)server.arg("since").toInt();
    }

    String json = "{\"latest\":";
    if (telemetryCount == 0) {
        json += "null";
    } else {
        int latestIdx = (telemetryHead - 1 + TELEMETRY_BUFFER_SIZE) % TELEMETRY_BUFFER_SIZE;
        TelemetryFrame &f = telemetryRingBuffer[latestIdx];
        json += "{\"id\":" + String(f.id);
        json += ",\"ts\":" + String(f.timestamp);
        json += ",\"lines\":" + String(f.lineCount);
        json += ",\"which\":\"" + String(f.whichLines) + "\"";
        json += ",\"num_vec\":" + String(f.numVectors);
        json += ",\"horiz_cnt\":" + String(f.horizVectorCount);
        json += ",\"steer\":" + String(f.steeringAngle, 2);
        json += ",\"speed\":" + String(f.motorSpeed, 1);
        json += ",\"eng\":" + String(f.engineEnabled ? 1 : 0);
        json += ",\"batt\":" + String(f.batteryVolts, 2);
        json += ",\"lx0\":" + String(f.lx0);
        json += ",\"ly0\":" + String(f.ly0);
        json += ",\"lx1\":" + String(f.lx1);
        json += ",\"ly1\":" + String(f.ly1);
        json += ",\"rx0\":" + String(f.rx0);
        json += ",\"ry0\":" + String(f.ry0);
        json += ",\"rx1\":" + String(f.rx1);
        json += ",\"ry1\":" + String(f.ry1);
        json += "}";
    }

    json += ",\"new_items\":[";
    bool first = true;
    int startIdx = (telemetryCount < TELEMETRY_BUFFER_SIZE) ? 0 : telemetryHead;
    for (int i = 0; i < telemetryCount; i++) {
        int idx = (startIdx + i) % TELEMETRY_BUFFER_SIZE;
        TelemetryFrame &f = telemetryRingBuffer[idx];
        if (f.id > sinceId) {
            if (!first) json += ",";
            first = false;
            json += "{\"id\":" + String(f.id);
            json += ",\"ts\":" + String(f.timestamp);
            json += ",\"lines\":" + String(f.lineCount);
            json += ",\"which\":\"" + String(f.whichLines) + "\"";
            json += ",\"num_vec\":" + String(f.numVectors);
            json += ",\"horiz_cnt\":" + String(f.horizVectorCount);
            json += ",\"steer\":" + String(f.steeringAngle, 2);
            json += ",\"speed\":" + String(f.motorSpeed, 1);
            json += ",\"eng\":" + String(f.engineEnabled ? 1 : 0);
            json += ",\"batt\":" + String(f.batteryVolts, 2);
            json += ",\"lx0\":" + String(f.lx0);
            json += ",\"ly0\":" + String(f.ly0);
            json += ",\"lx1\":" + String(f.lx1);
            json += ",\"ly1\":" + String(f.ly1);
            json += ",\"rx0\":" + String(f.rx0);
            json += ",\"ry0\":" + String(f.ry0);
            json += ",\"rx1\":" + String(f.rx1);
            json += ",\"ry1\":" + String(f.ry1);
            json += "}";
        }
    }
    json += "]}";

    server.send(200, "application/json", json);
}

void handleSetParam() {
    server.sendHeader("Access-Control-Allow-Origin", "*");

    if (!server.hasArg("param") || !server.hasArg("val")) {
        server.send(400, "application/json", "{\"status\":\"error\",\"message\":\"Missing arguments\"}");
        return;
    }
    String param = server.arg("param");
    float val = server.arg("val").toFloat();

    if (param == "STEERING_P_RIGHT") {
        p_right = val;
        preferences.putFloat("P_RIGHT", val);
    } else if (param == "STEERING_P_LEFT") {
        p_left = val;
        preferences.putFloat("P_LEFT", val);
    } else if (param == "STEERING_D_RIGHT") {
        d_right = val;
        preferences.putFloat("D_RIGHT", val);
    } else if (param == "STEERING_D_LEFT") {
        d_left = val;
        preferences.putFloat("D_LEFT", val);
    } else if (param == "MOTOR_SPEED") {
        motor_speed = val;
        preferences.putFloat("SPEED", val);
    } else if (param == "DECAY_FACTOR") {
        decay_factor = val;
        preferences.putFloat("DECAY", val);
    } else if (param == "ENGINE_ENABLED") {
        engine_enabled = (val > 0.5f);
    } else if (param == "EMERGENCY_STOP") {
        engine_enabled = false;
    }

    // Format message to NXP board: "NUME_PARAMETRU = VAL\n"
    String msg = param + " = " + String(val, 4) + "\n";
    Serial2.print(msg);
    Serial2.flush();

    Serial.print("Transmis catre NXP: ");
    Serial.print(msg);

    String jsonResponse = "{\"status\":\"ok\",\"param\":\"" + param + "\",\"val\":" + String(val, 4) + "}";
    server.send(200, "application/json", jsonResponse);
}

void setup() {
    delay(2000);
    Serial.begin(115200);                        // USB Serial Debug
    Serial2.begin(115200, SERIAL_8N1, 16, 17);  // RX=GPIO16 (from NXP K3/P2_4), TX=GPIO17 (to NXP J3/P2_3)

    Serial.println("Pornire ESP32 PID & Engine Control Web Server...");

    loadStoredParams();

    /* Configure ESP32 as Access Point */
    WiFi.mode(WIFI_AP);
    if (WiFi.softAP(AP_SSID, AP_PASS)) {
        Serial.print("Access Point pornit cu SSID: ");
        Serial.println(AP_SSID);
        Serial.print("IP Server Web: http://");
        Serial.println(WiFi.softAPIP());
    } else {
        Serial.println("Eroare la pornirea Access Point-ului!");
    }

    /* Initialize mDNS */
    if (MDNS.begin(MDNS_NAME)) {
        Serial.print("mDNS responder pornit: http://");
        Serial.print(MDNS_NAME);
        Serial.println(".local");
        MDNS.addService("http", "tcp", 80);
    } else {
        Serial.println("Eroare la pornirea mDNS!");
    }

    /* Setup HTTP Server Routes */
    server.on("/", handleRoot);
    server.on("/api/params", handleGetParams);
    server.on("/api/telemetry", handleGetTelemetry);
    server.on("/set", handleSetParam);

    server.begin();
    Serial.println("Server Web HTTP pornit pe portul 80!");

    // Trimitere parametri initiali catre NXP pe UART
    sendParamsToNXP();
}

/* Send a placeholder heartbeat message to NXP every 10 seconds */
void sendHeartbeat() {
    static unsigned long lastHeartbeatTime = 0;
    unsigned long now = millis();
    if (now - lastHeartbeatTime >= 10000UL) {
        lastHeartbeatTime = now;
        Serial2.print("HEARTBEAT = 1\n");
        Serial2.flush();
        Serial.println("[ESP32 -> NXP] Placeholder heartbeat message sent (every 10s)");
    }
}

void loop() {
    server.handleClient();

    sendHeartbeat();

    /* Process incoming UART data from NXP board */
    processSerial2Input();
}