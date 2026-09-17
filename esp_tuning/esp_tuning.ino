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

// Local stored PID & Motor values (default values matching bootcamp car)
float kp_straight        = 1.50f;
float kd_straight        = 0.50f;
float kp_curve           = 3.00f;
float kd_curve           = 1.20f;
float curve_threshold    = 10.0f;
float motor_speed        = 90.0f;
float sharp_coeff        = 0.20f;
float decay_factor       = 0.90f;
bool engine_enabled      = false;

/* Minimal Telemetry Data Structure & Ring Buffer */
struct TelemetryFrame {
    unsigned long id;
    unsigned long timestamp;
    uint8_t lineCount;
    char whichLines[16];
    uint16_t numVectors;
    uint32_t horizVectorCount;
    float steeringAngle;
    int lx0, ly0, lx1, ly1;
    int rx0, ry0, rx1, ry1;
};

#define TELEMETRY_BUFFER_SIZE 300
TelemetryFrame telemetryRingBuffer[TELEMETRY_BUFFER_SIZE];
int telemetryHead = 0;
int telemetryCount = 0;
unsigned long telemetrySeqCounter = 0;

void processTelemetryLine(const String& line) {
    if (!line.startsWith("TELEM:")) return;

    TelemetryFrame frame;
    frame.id = ++telemetrySeqCounter;
    frame.timestamp = millis();
    frame.lineCount = 0;
    strcpy(frame.whichLines, "NONE");
    frame.numVectors = 0;
    frame.horizVectorCount = 0;
    frame.steeringAngle = 0.0f;
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
                } else if (rxLine.startsWith("PING=")) {
                    String seq = rxLine.substring(5);
                    Serial2.print("PONG=" + seq + "\n");
                    Serial2.flush();
                    Serial.println("\n[ESP32 DUPLEX] RX from FRDM: PING=" + seq + " -> Replied with PONG=" + seq);
                } else if (rxLine.startsWith("PONG=")) {
                    String seq = rxLine.substring(5);
                    Serial.println("\n[ESP32 DUPLEX] RX from FRDM: PONG=" + seq + " (Round-trip confirmed!)");
                } else if (rxLine.startsWith("ACK:")) {
                    Serial.println("\n[ESP32 DUPLEX] RX from FRDM: " + rxLine);
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
  <title>NXP Robot - PID Control Panel</title>
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
    .container { width: 100%; max-width: 1200px; }
    .header { text-align: center; margin-bottom: 20px; }
    .header h1 { font-size: 1.75rem; color: var(--accent); margin-bottom: 6px; }
    .header p { color: var(--muted); font-size: 0.9rem; }
    .badge { display: inline-block; background: #0369a1; color: #e0f2fe; font-size: 0.75rem; font-weight: bold; padding: 4px 10px; border-radius: 12px; margin-top: 6px; }
    
    .dashboard-layout { display: grid; grid-template-columns: 1.1fr 0.9fr; gap: 20px; width: 100%; align-items: start; }
    @media (max-width: 900px) {
      .dashboard-layout { grid-template-columns: 1fr; }
    }

    .card { background: var(--card); border: 1px solid var(--border); border-radius: 14px; padding: 20px; margin-bottom: 18px; box-shadow: 0 4px 12px rgba(0,0,0,0.3); }
    .card-title { font-size: 1.1rem; color: var(--text); margin-bottom: 16px; border-bottom: 1px solid var(--border); padding-bottom: 8px; font-weight: 600; display: flex; justify-content: space-between; align-items: center; }
    
    .top-controls-card { margin-bottom: 20px; }
    .estop-btn { width: 100%; background: var(--danger); color: #fff; border: none; font-size: 1.25rem; font-weight: 800; padding: 16px; border-radius: 12px; cursor: pointer; transition: all 0.2s; box-shadow: 0 4px 14px rgba(239, 68, 68, 0.4); text-transform: uppercase; letter-spacing: 1px; margin-bottom: 12px; }
    .estop-btn:hover { background: var(--danger-hover); transform: scale(1.01); }
    .estop-btn:active { transform: scale(0.98); }

    .engine-controls { display: flex; gap: 10px; }
    .btn-start { flex: 1; background: var(--success); color: #052e16; font-size: 1rem; padding: 14px; font-weight: bold; border: none; border-radius: 8px; cursor: pointer; }
    .btn-start:hover { background: #16a34a; color: #fff; }
    .btn-pause { flex: 1; background: var(--warning); color: #451a03; font-size: 1rem; padding: 14px; font-weight: bold; border: none; border-radius: 8px; cursor: pointer; }
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
    .btn-all { width: 100%; padding: 12px; font-size: 1rem; margin-top: 10px; background: #0284c7; color: #fff; border: none; border-radius: 8px; cursor: pointer; font-weight: bold; }
    
    .grid-2 { display: grid; grid-template-columns: 1fr 1fr; gap: 10px; margin-bottom: 12px; }
    .metric-box { background: #090d16; border: 1px solid var(--border); border-radius: 8px; padding: 10px; text-align: center; }
    .metric-title { font-size: 0.75rem; color: var(--muted); text-transform: uppercase; margin-bottom: 4px; }
    .metric-val { font-size: 1.15rem; font-weight: bold; color: var(--accent); font-family: monospace; }
    .line-pill { display: inline-block; padding: 3px 8px; border-radius: 6px; font-size: 0.8rem; font-weight: bold; }
    .pill-both { background: #14532d; color: #4ade80; border: 1px solid #22c55e; }
    .pill-single { background: #78350f; color: #fde047; border: 1px solid #f59e0b; }
    .pill-turn { background: #0e7490; color: #67e8f9; border: 1px solid #06b6d4; }
    .pill-none { background: #7f1d1d; color: #fca5a5; border: 1px solid #ef4444; }

    /* Pixy 2D Visualizer Styling */
    .pixy-canvas-container { position: relative; width: 100%; background: #030712; border: 1px solid var(--border); border-radius: 10px; padding: 8px; text-align: center; margin-bottom: 8px; }
    #pixyCanvas { width: 100%; max-width: 500px; height: 255px; background: #090d16; border-radius: 8px; display: block; margin: 0 auto; }
    .coords-badge-container { display: flex; flex-direction: column; gap: 6px; font-family: monospace; font-size: 0.82rem; margin-top: 6px; background: #0f172a; padding: 10px; border-radius: 8px; border: 1px solid var(--border); }
    .coords-left { color: #38bdf8; font-weight: bold; }
    .coords-right { color: #4ade80; font-weight: bold; }

    #toast { position: fixed; bottom: 20px; left: 50%; transform: translateX(-50%); background: var(--success); color: #052e16; padding: 12px 24px; border-radius: 8px; font-weight: bold; opacity: 0; transition: opacity 0.3s; pointer-events: none; box-shadow: 0 4px 12px rgba(0,0,0,0.5); z-index: 1000; }
    #toast.show { opacity: 1; }
    #toast.danger-toast { background: var(--danger); color: #fff; }
  </style>
</head>
<body>
  <div class="container">
    <div class="header">
      <h1>NXP Robot Control Panel</h1>
      <p>Tuning PID Control Panel (Port 80)</p>
      <div class="badge">AP: ESP32_Robot_AP2 | http://esp32_2.local</div>
    </div>

    <!-- Top Section: Start, Stop & Emergency Controls -->
    <div class="card top-controls-card">
      <div class="card-title">
        <span>Control Motoare & Stare Robot</span>
        <span id="engine_status" class="status-pill status-off">STOPPED</span>
      </div>
      <button class="estop-btn" onclick="triggerEmergencyStop()">FRANA URGENTA (E-STOP)</button>
      <div class="engine-controls">
        <button class="btn-start" onclick="setEngine(1)">PORNESTE MOTOARELE (START)</button>
        <button class="btn-pause" onclick="setEngine(0)">OPRESTE MOTOARELE (STOP)</button>
      </div>
    </div>

    <!-- Dashboard 2-Column Grid Layout -->
    <div class="dashboard-layout">
      <!-- LEFT SIDE: Plots & Metrics -->
      <div class="left-column">
        <!-- 2D Pixy Camera Frame Visualizer Plot Card -->
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

        <!-- Live Steering Diagram Plot Card -->
        <div class="card">
          <div class="card-title">
            <span>Diagrame Live</span>
          </div>

          <div style="width: 100%; height: 150px; background: #090d16; border: 1px solid var(--border); border-radius: 8px; padding: 6px; position: relative;">
            <canvas id="steerChartCanvas" width="550" height="138" style="width: 100%; height: 100%;"></canvas>
          </div>
        </div>

        <!-- Telemetry Summary Card -->
        <div class="card">
          <div class="card-title">
            <span>Telemetrie</span>
          </div>

          <div class="grid-2">
            <div class="metric-box">
              <div class="metric-title">Vectori Detectati</div>
              <div id="metric_vecs" class="metric-val">0</div>
            </div>
            <div class="metric-box">
              <div class="metric-title">Vectori Orizontali</div>
              <div id="metric_horiz_cnt" class="metric-val">0</div>
            </div>
          </div>
        </div>
      </div>

      <!-- RIGHT SIDE: Parameter Sliders -->
      <div class="right-column">
        <!-- Motor Speed Control Card -->
        <div class="card">
          <div class="card-title">
            <span>Control Viteza Motoare</span>
          </div>

          <div class="param-group">
            <div class="param-header">
              <span class="param-name">MOTOR_SPEED (Viteza %)</span>
              <span class="param-val" id="val_SPEED">90%</span>
            </div>
            <div class="controls">
              <input type="range" id="slider_SPEED" min="0" max="100" step="1" value="90" oninput="syncSpeed(this.value)">
              <input type="number" id="num_SPEED" min="0" max="100" step="1" value="90" oninput="syncSpeedSlider(this.value)">
              <button class="btn" onclick="saveSpeed()">Set Speed</button>
            </div>
          </div>
        </div>

        <!-- Zone 1: Straight Line PID Card -->
        <div class="card">
          <div class="card-title">Zone 1: Linie Dreapt&#x103; (Straight PID)</div>
          
          <div class="param-group">
            <div class="param-header">
              <span class="param-name">KP_STRAIGHT (Propor&#x21B;ional)</span>
              <span class="param-val" id="val_KP_STR">1.500</span>
            </div>
            <div class="controls">
              <input type="range" id="slider_KP_STR" min="0" max="10" step="0.05" value="1.5" oninput="syncVal('KP_STR', this.value)">
              <input type="number" id="num_KP_STR" min="0" max="10" step="0.05" value="1.5" oninput="syncSlider('KP_STR', this.value)">
              <button class="btn" onclick="saveParam('KP_STRAIGHT', 'KP_STR')">Save</button>
            </div>
          </div>

          <div class="param-group">
            <div class="param-header">
              <span class="param-name">KD_STRAIGHT (Derivativ)</span>
              <span class="param-val" id="val_KD_STR">0.500</span>
            </div>
            <div class="controls">
              <input type="range" id="slider_KD_STR" min="0" max="5" step="0.05" value="0.5" oninput="syncVal('KD_STR', this.value)">
              <input type="number" id="num_KD_STR" min="0" max="5" step="0.05" value="0.5" oninput="syncSlider('KD_STR', this.value)">
              <button class="btn" onclick="saveParam('KD_STRAIGHT', 'KD_STR')">Save</button>
            </div>
          </div>
        </div>

        <!-- Zone 2: Curve & Sharp Turn PID Card -->
        <div class="card">
          <div class="card-title">Zone 2: Curb&#x103; / Viraj (Curve PID)</div>
          
          <div class="param-group">
            <div class="param-header">
              <span class="param-name">KP_CURVE (Propor&#x21B;ional)</span>
              <span class="param-val" id="val_KP_CRV">3.000</span>
            </div>
            <div class="controls">
              <input type="range" id="slider_KP_CRV" min="0" max="15" step="0.05" value="3.0" oninput="syncVal('KP_CRV', this.value)">
              <input type="number" id="num_KP_CRV" min="0" max="15" step="0.05" value="3.0" oninput="syncSlider('KP_CRV', this.value)">
              <button class="btn" onclick="saveParam('KP_CURVE', 'KP_CRV')">Save</button>
            </div>
          </div>

          <div class="param-group">
            <div class="param-header">
              <span class="param-name">KD_CURVE (Derivativ)</span>
              <span class="param-val" id="val_KD_CRV">1.200</span>
            </div>
            <div class="controls">
              <input type="range" id="slider_KD_CRV" min="0" max="10" step="0.05" value="1.2" oninput="syncVal('KD_CRV', this.value)">
              <input type="number" id="num_KD_CRV" min="0" max="10" step="0.05" value="1.2" oninput="syncSlider('KD_CRV', this.value)">
              <button class="btn" onclick="saveParam('KD_CURVE', 'KD_CRV')">Save</button>
            </div>
          </div>
        </div>

        <!-- Zone Transition & Decay Card -->
        <div class="card">
          <div class="card-title">Tranzi&#x21B;ie Zon&#x103; &amp; Dec&#x103;dere</div>

          <div class="param-group">
            <div class="param-header">
              <span class="param-name">CURVE_THRES (Prag unghi &deg;)</span>
              <span class="param-val" id="val_CURV_TH">10.0&deg;</span>
            </div>
            <div class="controls">
              <input type="range" id="slider_CURV_TH" min="2" max="30" step="0.5" value="10" oninput="syncVal('CURV_TH', this.value)">
              <input type="number" id="num_CURV_TH" min="2" max="30" step="0.5" value="10" oninput="syncSlider('CURV_TH', this.value)">
              <button class="btn" onclick="saveParam('CURVE_THRES', 'CURV_TH')">Save</button>
            </div>
          </div>

          <div class="param-group">
            <div class="param-header">
              <span class="param-name">SHARP_COEFF (Coeficient vitez&#x103; viraj)</span>
              <span class="param-val" id="val_SHARP_CF">0.200</span>
            </div>
            <div class="controls">
              <input type="range" id="slider_SHARP_CF" min="0.05" max="1" step="0.05" value="0.2" oninput="syncVal('SHARP_CF', this.value)">
              <input type="number" id="num_SHARP_CF" min="0.05" max="1" step="0.05" value="0.2" oninput="syncSlider('SHARP_CF', this.value)">
              <button class="btn" onclick="saveParam('SHARP_COEFF', 'SHARP_CF')">Save</button>
            </div>
          </div>

          <div class="param-group">
            <div class="param-header">
              <span class="param-name">DECAY_FACTOR (Dec&#x103;dere f&#x103;r&#x103; linie)</span>
              <span class="param-val" id="val_DECAY">0.900</span>
            </div>
            <div class="controls">
              <input type="range" id="slider_DECAY" min="0.5" max="1" step="0.005" value="0.9" oninput="syncVal('DECAY', this.value)">
              <input type="number" id="num_DECAY" min="0.5" max="1" step="0.005" value="0.9" oninput="syncSlider('DECAY', this.value)">
              <button class="btn" onclick="saveParam('DECAY_FACTOR', 'DECAY')">Save</button>
            </div>
          </div>
        </div>

        <button class="btn btn-all" onclick="saveAllPID()">Salveaza Toti Parametrii</button>
      </div>
    </div>
  </div>

  <div id="toast">Salvat cu succes!</div>

  <script>
    window.telemetryHistory = [];
    let lastTelemetryId = 0;

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
      saveParam('KP_STRAIGHT', 'KP_STR');
      setTimeout(() => { saveParam('KD_STRAIGHT', 'KD_STR'); }, 150);
      setTimeout(() => { saveParam('KP_CURVE', 'KP_CRV'); }, 300);
      setTimeout(() => { saveParam('KD_CURVE', 'KD_CRV'); }, 450);
      setTimeout(() => { saveParam('CURVE_THRES', 'CURV_TH'); }, 600);
      setTimeout(() => { saveParam('SHARP_COEFF', 'SHARP_CF'); }, 750);
      setTimeout(() => { saveParam('DECAY_FACTOR', 'DECAY'); }, 900);
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
      document.getElementById('metric_vecs').innerText = item.num_vec;
      if (document.getElementById('metric_horiz_cnt')) {
        document.getElementById('metric_horiz_cnt').innerText = item.horiz_cnt !== undefined ? item.horiz_cnt : 0;
      }

      // Update 2D Pixy Canvas & Steering Chart
      drawPixyFrame2D(item);
      updateSteerChart(item);
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

    window.onload = function() {
      fetch('/api/params')
        .then(function(r) { return r.json(); })
        .then(function(d) {
          if (d.KP_STRAIGHT !== undefined) { syncVal('KP_STR', d.KP_STRAIGHT); syncSlider('KP_STR', d.KP_STRAIGHT); }
          if (d.KD_STRAIGHT !== undefined) { syncVal('KD_STR', d.KD_STRAIGHT); syncSlider('KD_STR', d.KD_STRAIGHT); }
          if (d.KP_CURVE !== undefined) { syncVal('KP_CRV', d.KP_CURVE); syncSlider('KP_CRV', d.KP_CURVE); }
          if (d.KD_CURVE !== undefined) { syncVal('KD_CRV', d.KD_CURVE); syncSlider('KD_CRV', d.KD_CURVE); }
          if (d.CURVE_THRES !== undefined) { syncVal('CURV_TH', d.CURVE_THRES); syncSlider('CURV_TH', d.CURVE_THRES); }
          if (d.SHARP_COEFF !== undefined) { syncVal('SHARP_CF', d.SHARP_COEFF); syncSlider('SHARP_CF', d.SHARP_COEFF); }
          if (d.MOTOR_SPEED !== undefined) { syncSpeed(d.MOTOR_SPEED); syncSpeedSlider(d.MOTOR_SPEED); }
          if (d.DECAY_FACTOR !== undefined) { syncVal('DECAY', d.DECAY_FACTOR); syncSlider('DECAY', d.DECAY_FACTOR); }
          if (d.ENGINE_ENABLED !== undefined) { updateEngineBadge(d.ENGINE_ENABLED === 1); }
        })
        .catch(function(e) { console.log('Init fetch error:', e); });

      pollTelemetry();
    }
  </script>
</body>
</html>
)rawliteral";

/* Load persisted PID & Motor params from Preferences */
void loadStoredParams() {
    preferences.begin("pid_tuning", false);
    kp_straight     = preferences.getFloat("KP_STR", 1.50f);
    kd_straight     = preferences.getFloat("KD_STR", 0.50f);
    kp_curve        = preferences.getFloat("KP_CRV", 3.00f);
    kd_curve        = preferences.getFloat("KD_CRV", 1.20f);
    curve_threshold = preferences.getFloat("CURV_TH", 10.0f);
    motor_speed     = preferences.getFloat("SPEED", 90.0f);
    sharp_coeff     = preferences.getFloat("SHARP_CF", 0.20f);
    decay_factor    = preferences.getFloat("DECAY", 0.90f);
    engine_enabled  = false; // Always start stopped for safety
}

/* Transmit current params to NXP over UART */
void sendParamsToNXP() {
    Serial2.printf("KP_STRAIGHT = %.4f\n", kp_straight);
    delay(40);
    Serial2.printf("KD_STRAIGHT = %.4f\n", kd_straight);
    delay(40);
    Serial2.printf("KP_CURVE = %.4f\n", kp_curve);
    delay(40);
    Serial2.printf("KD_CURVE = %.4f\n", kd_curve);
    delay(40);
    Serial2.printf("CURVE_THRES = %.4f\n", curve_threshold);
    delay(40);
    Serial2.printf("MOTOR_SPEED = %.4f\n", motor_speed);
    delay(40);
    Serial2.printf("SHARP_COEFF = %.4f\n", sharp_coeff);
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
    json += "\"KP_STRAIGHT\":" + String(kp_straight, 4) + ",";
    json += "\"KD_STRAIGHT\":" + String(kd_straight, 4) + ",";
    json += "\"KP_CURVE\":" + String(kp_curve, 4) + ",";
    json += "\"KD_CURVE\":" + String(kd_curve, 4) + ",";
    json += "\"CURVE_THRES\":" + String(curve_threshold, 2) + ",";
    json += "\"MOTOR_SPEED\":" + String(motor_speed, 1) + ",";
    json += "\"SHARP_COEFF\":" + String(sharp_coeff, 2) + ",";
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

    if (param == "KP_STRAIGHT") {
        kp_straight = val;
        preferences.putFloat("KP_STR", val);
    } else if (param == "KD_STRAIGHT") {
        kd_straight = val;
        preferences.putFloat("KD_STR", val);
    } else if (param == "KP_CURVE") {
        kp_curve = val;
        preferences.putFloat("KP_CRV", val);
    } else if (param == "KD_CURVE") {
        kd_curve = val;
        preferences.putFloat("KD_CRV", val);
    } else if (param == "CURVE_THRES") {
        curve_threshold = val;
        preferences.putFloat("CURV_TH", val);
    } else if (param == "MOTOR_SPEED") {
        motor_speed = val;
        preferences.putFloat("SPEED", val);
    } else if (param == "SHARP_COEFF") {
        sharp_coeff = val;
        preferences.putFloat("SHARP_CF", val);
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
    Serial2.begin(115200, SERIAL_8N1, 16, 17);  // RX=GPIO16 (from NXP P3_3 TX), TX=GPIO17 (to NXP P3_2 RX)

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

/* Send a test ping to FRDM every 2 seconds to verify bidirectional communication */
void sendHeartbeat() {
    static unsigned long lastHeartbeatTime = 0;
    static unsigned long hbSeq = 0;
    unsigned long now = millis();
    if (now - lastHeartbeatTime >= 2000UL) {
        lastHeartbeatTime = now;
        hbSeq++;
        Serial2.print("PING=" + String(hbSeq) + "\n");
        Serial2.flush();
        Serial.println("[ESP32 DUPLEX] TX -> FRDM: PING #" + String(hbSeq));
    }
}

void loop() {
    server.handleClient();

    sendHeartbeat();

    /* Process incoming UART data from NXP board */
    processSerial2Input();
}