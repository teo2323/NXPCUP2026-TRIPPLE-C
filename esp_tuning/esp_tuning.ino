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
bool engine_enabled = false;

/* Embedded HTML Dashboard */
const char INDEX_HTML[] PROGMEM = R"rawliteral(
<!DOCTYPE html>
<html lang="ro">
<head>
  <meta charset="UTF-8">
  <meta name="viewport" content="width=device-width, initial-scale=1.0">
  <title>NXP Robot - PID & Engine Control Panel</title>
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
    .container { width: 100%; max-width: 560px; }
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
    
    #toast { position: fixed; bottom: 20px; left: 50%; transform: translateX(-50%); background: var(--success); color: #052e16; padding: 12px 24px; border-radius: 8px; font-weight: bold; opacity: 0; transition: opacity 0.3s; pointer-events: none; box-shadow: 0 4px 12px rgba(0,0,0,0.5); z-index: 1000; }
    #toast.show { opacity: 1; }
    #toast.danger-toast { background: var(--danger); color: #fff; }
  </style>
</head>
<body>
  <div class="container">
    <div class="header">
      <h1>🏎️ NXP Robot Control Panel</h1>
      <p>Tuning PID & Control Viteză Graduală (Port 80)</p>
      <div class="badge">AP: ESP32_Robot_AP2 | http://esp32_2.local</div>
    </div>

    <!-- Emergency Stop Button -->
    <button class="estop-btn" onclick="triggerEmergencyStop()">🚨 FRÂNĂ URGENȚĂ (E-STOP)</button>

    <!-- Engine & Speed Control Card -->
    <div class="card">
      <div class="card-title">
        <span>⚙️ Control Viteză Motoare</span>
        <span id="engine_status" class="status-pill status-off">STOPPED</span>
      </div>

      <div class="param-group">
        <div class="param-header">
          <span class="param-name">MOTOR_SPEED (Viteză %)</span>
          <span class="param-val" id="val_SPEED">70%</span>
        </div>
        <div class="controls">
          <input type="range" id="slider_SPEED" min="0" max="100" step="1" value="70" oninput="syncSpeed(this.value)">
          <input type="number" id="num_SPEED" min="0" max="100" step="1" value="70" oninput="syncSpeedSlider(this.value)">
          <button class="btn" onclick="saveSpeed()">Set Speed</button>
        </div>
      </div>

      <div class="engine-controls">
        <button class="btn-start" onclick="setEngine(1)">▶️ PORNEȘTE MOTOARELE</button>
        <button class="btn-pause" onclick="setEngine(0)">⏸️ OPREȘTE MOTOARELE</button>
      </div>
    </div>

    <!-- PID Proportional Card -->
    <div class="card">
      <div class="card-title">🎯 Proporțional (P Gains)</div>
      
      <div class="param-group">
        <div class="param-header">
          <span class="param-name">STEERING_P_RIGHT</span>
          <span class="param-val" id="val_P_RIGHT">0.400</span>
        </div>
        <div class="controls">
          <input type="range" id="slider_P_RIGHT" min="0" max="3" step="0.005" value="0.4" oninput="syncVal('P_RIGHT', this.value)">
          <input type="number" id="num_P_RIGHT" min="0" max="3" step="0.005" value="0.4" oninput="syncSlider('P_RIGHT', this.value)">
          <button class="btn" onclick="saveParam('STEERING_P_RIGHT', 'P_RIGHT')">Save</button>
        </div>
      </div>

      <div class="param-group">
        <div class="param-header">
          <span class="param-name">STEERING_P_LEFT</span>
          <span class="param-val" id="val_P_LEFT">0.400</span>
        </div>
        <div class="controls">
          <input type="range" id="slider_P_LEFT" min="0" max="3" step="0.005" value="0.4" oninput="syncVal('P_LEFT', this.value)">
          <input type="number" id="num_P_LEFT" min="0" max="3" step="0.005" value="0.4" oninput="syncSlider('P_LEFT', this.value)">
          <button class="btn" onclick="saveParam('STEERING_P_LEFT', 'P_LEFT')">Save</button>
        </div>
      </div>
    </div>

    <!-- PID Derivative Card -->
    <div class="card">
      <div class="card-title">📈 Derivativ (D Gains)</div>
      
      <div class="param-group">
        <div class="param-header">
          <span class="param-name">STEERING_D_RIGHT</span>
          <span class="param-val" id="val_D_RIGHT">0.200</span>
        </div>
        <div class="controls">
          <input type="range" id="slider_D_RIGHT" min="0" max="3" step="0.005" value="0.2" oninput="syncVal('D_RIGHT', this.value)">
          <input type="number" id="num_D_RIGHT" min="0" max="3" step="0.005" value="0.2" oninput="syncSlider('D_RIGHT', this.value)">
          <button class="btn" onclick="saveParam('STEERING_D_RIGHT', 'D_RIGHT')">Save</button>
        </div>
      </div>

      <div class="param-group">
        <div class="param-header">
          <span class="param-name">STEERING_D_LEFT</span>
          <span class="param-val" id="val_D_LEFT">0.200</span>
        </div>
        <div class="controls">
          <input type="range" id="slider_D_LEFT" min="0" max="3" step="0.005" value="0.2" oninput="syncVal('D_LEFT', this.value)">
          <input type="number" id="num_D_LEFT" min="0" max="3" step="0.005" value="0.2" oninput="syncSlider('D_LEFT', this.value)">
          <button class="btn" onclick="saveParam('STEERING_D_LEFT', 'D_LEFT')">Save</button>
        </div>
      </div>
    </div>

    <button class="btn btn-all" onclick="saveAllPID()">💾 Salvează Toți Parametrii PID</button>
  </div>

  <div id="toast">Salvat cu succes!</div>

  <script>
    function syncVal(id, val) {
      document.getElementById('num_' + id).value = val;
      document.getElementById('val_' + id).innerText = parseFloat(val).toFixed(3);
    }
    function syncSlider(id, val) {
      document.getElementById('slider_' + id).value = val;
      document.getElementById('val_' + id).innerText = parseFloat(val).toFixed(3);
    }
    function syncSpeed(val) {
      document.getElementById('num_SPEED').value = val;
      document.getElementById('val_SPEED').innerText = parseInt(val) + '%';
    }
    function syncSpeedSlider(val) {
      document.getElementById('slider_SPEED').value = val;
      document.getElementById('val_SPEED').innerText = parseInt(val) + '%';
    }
    function showToast(msg, isDanger = false) {
      const t = document.getElementById('toast');
      t.innerText = msg;
      if (isDanger) t.classList.add('danger-toast');
      else t.classList.remove('danger-toast');
      t.classList.add('show');
      setTimeout(() => t.classList.remove('show'), 2200);
    }
    function updateEngineBadge(running) {
      const b = document.getElementById('engine_status');
      if (running) {
        b.innerText = 'RUNNING (' + document.getElementById('num_SPEED').value + '%)';
        b.className = 'status-pill status-on';
      } else {
        b.innerText = 'STOPPED';
        b.className = 'status-pill status-off';
      }
    }
    function saveParam(paramName, id) {
      const val = document.getElementById('num_' + id).value;
      fetch('/set?param=' + encodeURIComponent(paramName) + '&val=' + encodeURIComponent(val))
        .then(r => {
          if (!r.ok) throw new Error('HTTP ' + r.status);
          return r.json();
        })
        .then(d => showToast('Transmis pe NXP: ' + paramName + ' = ' + val))
        .catch(e => showToast('Eroare salvare ' + paramName, true));
    }
    function saveSpeed() {
      const val = document.getElementById('num_SPEED').value;
      fetch('/set?param=MOTOR_SPEED&val=' + encodeURIComponent(val))
        .then(r => {
          if (!r.ok) throw new Error('HTTP ' + r.status);
          return r.json();
        })
        .then(d => showToast('Viteză setată la: ' + val + '%'))
        .catch(e => showToast('Eroare setare viteză!', true));
    }
    function setEngine(state) {
      fetch('/set?param=ENGINE_ENABLED&val=' + state)
        .then(r => {
          if (!r.ok) throw new Error('HTTP ' + r.status);
          return r.json();
        })
        .then(d => {
          updateEngineBadge(state === 1);
          showToast(state === 1 ? '▶️ Motoare PORNITE!' : '⏸️ Motoare OPRITE');
        })
        .catch(e => showToast('Eroare comutare motoare!', true));
    }
    function triggerEmergencyStop() {
      fetch('/set?param=EMERGENCY_STOP&val=1')
        .then(r => {
          if (!r.ok) throw new Error('HTTP ' + r.status);
          return r.json();
        })
        .then(d => {
          updateEngineBadge(false);
          showToast('🚨 EMERGENCY STOP ACTIVAT!', true);
        })
        .catch(e => showToast('🚨 EMERGENCY STOP TRIMIS!', true));
    }
    function saveAllPID() {
      saveParam('STEERING_P_RIGHT', 'P_RIGHT');
      setTimeout(() => saveParam('STEERING_P_LEFT', 'P_LEFT'), 150);
      setTimeout(() => saveParam('STEERING_D_RIGHT', 'D_RIGHT'), 300);
      setTimeout(() => saveParam('STEERING_D_LEFT', 'D_LEFT'), 450);
    }
    window.onload = function() {
      fetch('/api/params')
        .then(r => r.json())
        .then(d => {
          if (d.STEERING_P_RIGHT !== undefined) { syncVal('P_RIGHT', d.STEERING_P_RIGHT); syncSlider('P_RIGHT', d.STEERING_P_RIGHT); }
          if (d.STEERING_P_LEFT !== undefined) { syncVal('P_LEFT', d.STEERING_P_LEFT); syncSlider('P_LEFT', d.STEERING_P_LEFT); }
          if (d.STEERING_D_RIGHT !== undefined) { syncVal('D_RIGHT', d.STEERING_D_RIGHT); syncSlider('D_RIGHT', d.STEERING_D_RIGHT); }
          if (d.STEERING_D_LEFT !== undefined) { syncVal('D_LEFT', d.STEERING_D_LEFT); syncSlider('D_LEFT', d.STEERING_D_LEFT); }
          if (d.MOTOR_SPEED !== undefined) { syncSpeed(d.MOTOR_SPEED); syncSpeedSlider(d.MOTOR_SPEED); }
          if (d.ENGINE_ENABLED !== undefined) { updateEngineBadge(d.ENGINE_ENABLED === 1); }
        })
        .catch(e => console.log('Init fetch error:', e));
    }
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
    json += "\"ENGINE_ENABLED\":" + String(engine_enabled ? 1 : 0);
    json += "}";
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
    } else if (param == "ENGINE_ENABLED") {
        engine_enabled = (val > 0.5f);
    } else if (param == "EMERGENCY_STOP") {
        engine_enabled = false;
    }

    // Format message to NXP board: "NUME_PARAMETRU = VAL\n"
    String msg = param + " = " + String(val, 4) + "\n";
    Serial2.print(msg);

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
    server.on("/set", handleSetParam);

    server.begin();
    Serial.println("Server Web HTTP pornit pe portul 80!");

    // Trimitere parametri initiali catre NXP pe UART
    sendParamsToNXP();
}

void loop() {
    server.handleClient();

    /* Afisare debug in consola USB cand NXP trimite raspuns (ex: ACK) pe Serial2 */
    while (Serial2.available()) {
        Serial.write(Serial2.read());
    }
}