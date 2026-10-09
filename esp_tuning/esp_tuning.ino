/*
 * NXP CUP - ESP32 Modular Parameter Tuning Web Server
 * 
 * Hardware Connections:
 *   ESP32 GND          <---> FRDM J9 Pin 22 (GND) or J7 Pin 10
 *   ESP32 GPIO17 (TX2) ----> FRDM J9 Pin 20 (P3_2 / RX) or J7 Pin 8
 *   ESP32 GPIO16 (RX2) <---- FRDM J9 Pin 19 (P3_3 / TX) or J7 Pin 6
 *
 * Network:
 *   Access Point SSID: NXP_Robot_Tuning
 *   Password:          12345678
 *   IP Address:        http://192.168.4.1
 *   mDNS URL:          http://nxp-robot.local
 *
 * Protocol:
 *   ESP -> FRDM:  "NAME_OF_THE_PARAMETER=NEW_VALUE\n"
 *   FRDM -> ESP:  "ACK:NAME_OF_THE_PARAMETER=NEW_VALUE\r\n"
 */

#include <WiFi.h>
#include <WebServer.h>
#include <ESPmDNS.h>
#include <Preferences.h>

/* ========================================================================= */
/* Access Point Configuration                                                */
/* ========================================================================= */
const char* AP_SSID   = "NXP_Robot_Tuning";
const char* AP_PASS   = "12345678";
const char* MDNS_NAME = "nxp-robot";

WebServer server(80);
Preferences preferences;

/* ========================================================================= */
/* Modular Parameter Definition Structure                                    */
/* Easily add, modify, or remove parameters for other branches here!         */
/* ========================================================================= */
struct ParamDef {
    const char* key;          // Exact command name sent to FRDM (matches Config.h)
    const char* label;        // Human-readable title
    const char* group;        // Card grouping in UI
    float minVal;             // Slider min
    float maxVal;             // Slider max
    float step;               // Step resolution
    float defaultVal;         // Factory default
    float currentVal;         // Live value in RAM
};

ParamDef params[] = {
    // Group: Steering PID (2-Zone Variable PID)
    {"STEERING_KP_STRAIGHT",        "Straight Kp",             "Steering PID",        0.0f,  10.0f, 0.05f, 1.50f, 1.50f},
    {"STEERING_KD_STRAIGHT",        "Straight Kd",             "Steering PID",        0.0f,   5.0f, 0.05f, 0.50f, 0.50f},
    {"STEERING_KP_CURVE",           "Curve Kp",                "Steering PID",        0.0f,  15.0f, 0.05f, 3.00f, 3.00f},
    {"STEERING_KD_CURVE",           "Curve Kd",                "Steering PID",        0.0f,  10.0f, 0.05f, 1.20f, 1.20f},
    {"CURVE_ERROR_THRESHOLD",       "Curve Error Threshold",   "Steering PID",        1.0f,  45.0f, 0.50f, 10.0f, 10.0f},

    // Group: Steering Calibration & Limits
    {"STEERING_LIMIT_RIGHT",        "Steering Limit Right",    "Steering Limits",     0.0f,  90.0f, 1.00f, 65.0f, 65.0f},
    {"STEERING_LIMIT_LEFT",         "Steering Limit Left",     "Steering Limits",   -90.0f,   0.0f, 1.00f, -60.0f,-60.0f},
    {"STEERING_OFFSET",             "Steering Center Offset",  "Steering Limits",   -30.0f,  30.0f, 0.50f,  0.0f,  0.0f},

    // Group: Motor Speed & Dynamics
    {"MOTOR_SPEED",                 "Base Motor Speed (%)",    "Motor & Speed",       0.0f, 100.0f, 1.00f, 90.0f, 90.0f},
    {"SHARP_TURN_SPEED_COEFF",      "Sharp Turn Speed Coeff",  "Motor & Speed",       0.0f,   1.0f, 0.05f, 0.20f, 0.20f},
    {"SPEED_LEFT",                  "Left Wheel Base Speed",   "Motor & Speed",       0.0f, 100.0f, 1.00f, 40.0f, 40.0f},
    {"SPEED_RIGHT",                 "Right Wheel Base Speed",  "Motor & Speed",       0.0f, 100.0f, 1.00f, 40.0f, 40.0f},

    // Group: Thresholds & Decay
    {"SHARP_TURN_ANGLE_THRESHOLD",  "Sharp Turn Threshold (\xC2\xB0)", "Thresholds & Decay",  5.0f,  60.0f, 1.00f, 30.0f, 30.0f},
    {"DECAY_FACTOR",                "Steering Decay Factor",   "Thresholds & Decay",  0.1f,   1.0f, 0.01f, 0.90f, 0.90f},
};
const size_t NUM_PARAMS = sizeof(params) / sizeof(params[0]);

// Engine state
bool engine_enabled = false;

// Diagnostics
unsigned long lastPingTime = 0;
uint32_t pingSequence = 0;
uint32_t lastPongReceived = 0;
String lastRxMessage = "NONE";
uint32_t rxPacketCount = 0;

/* ========================================================================= */
/* Preferences Storage                                                       */
/* ========================================================================= */
void loadStoredParams() {
    preferences.begin("nxp_tuning", false);
    for (size_t i = 0; i < NUM_PARAMS; i++) {
        // Use short key for NVS (max 15 chars)
        char nvsKey[16];
        snprintf(nvsKey, sizeof(nvsKey), "p_%d", (int)i);
        params[i].currentVal = preferences.getFloat(nvsKey, params[i].defaultVal);
    }
    engine_enabled = false; // Always safe stopped on boot
}

void saveStoredParam(size_t index) {
    if (index >= NUM_PARAMS) return;
    char nvsKey[16];
    snprintf(nvsKey, sizeof(nvsKey), "p_%d", (int)index);
    preferences.putFloat(nvsKey, params[index].currentVal);
}

/* ========================================================================= */
/* UART Helpers to FRDM                                                      */
/* ========================================================================= */
void sendParamToFRDM(const char* name, float val) {
    // Protocol requirement: "NAME_OF_THE_PARAMETER=NEW_VALUE\n"
    Serial2.printf("%s=%.4f\n", name, val);
    Serial2.flush();
    Serial.printf("[ESP32 TX -> FRDM] %s=%.4f\n", name, val);
}

void syncAllParamsToFRDM() {
    Serial.println("[ESP32] Syncing all parameters to FRDM board...");
    for (size_t i = 0; i < NUM_PARAMS; i++) {
        sendParamToFRDM(params[i].key, params[i].currentVal);
        delay(35);
    }
    // Always start with engine stopped
    Serial2.print("ENGINE_ENABLED=0\n");
    Serial2.flush();
    delay(20);
}

/* ========================================================================= */
/* Incoming UART Data Processing                                             */
/* ========================================================================= */
void processSerial2Input() {
    static String rxLine = "";
    while (Serial2.available()) {
        char c = (char)Serial2.read();
        Serial.write(c); // Mirror to USB Serial

        if (c == '\n' || c == '\r') {
            rxLine.trim();
            if (rxLine.length() > 0) {
                rxPacketCount++;
                lastRxMessage = rxLine;

                if (rxLine.startsWith("PING=")) {
                    String seq = rxLine.substring(5);
                    Serial2.print("PONG=" + seq + "\n");
                    Serial2.flush();
                    Serial.println("\n[ESP32 DUPLEX] RX PING=" + seq + " -> Replied PONG=" + seq);
                }
                else if (rxLine.startsWith("PONG=")) {
                    lastPongReceived = rxLine.substring(5).toInt();
                    Serial.printf("\n[ESP32 DUPLEX] Round-trip confirmed! PONG=%lu\n", (unsigned long)lastPongReceived);
                }
                else if (rxLine.startsWith("ACK:")) {
                    Serial.println("\n[ESP32 ACK] Confirmation from FRDM: " + rxLine);
                }
                rxLine = "";
            }
        } else {
            if (rxLine.length() < 256) {
                rxLine += c;
            } else {
                rxLine = ""; // Overflow guard
            }
        }
    }
}

/* ========================================================================= */
/* Periodic Heartbeat                                                        */
/* ========================================================================= */
void sendHeartbeat() {
    unsigned long now = millis();
    if (now - lastPingTime >= 2000UL) {
        lastPingTime = now;
        pingSequence++;
        Serial2.printf("PING=%lu\n", (unsigned long)pingSequence);
        Serial2.flush();
    }
}

/* ========================================================================= */
/* Embedded HTML / JS Dashboard                                              */
/* ========================================================================= */
const char INDEX_HTML[] PROGMEM = R"rawliteral(
<!DOCTYPE html>
<html lang="en">
<head>
  <meta charset="UTF-8">
  <meta name="viewport" content="width=device-width, initial-scale=1.0">
  <title>NXP Robot - Parameters Tuning</title>
  <style>
    :root {
      --bg: #090d16;
      --card-bg: #131c2e;
      --card-border: #1e2e4a;
      --accent: #38bdf8;
      --accent-hover: #0284c7;
      --text: #f1f5f9;
      --muted: #94a3b8;
      --success: #22c55e;
      --danger: #ef4444;
      --danger-hover: #dc2626;
      --warning: #f59e0b;
    }
    * { box-sizing: border-box; margin: 0; padding: 0; font-family: -apple-system, BlinkMacSystemFont, "Segoe UI", Roboto, sans-serif; }
    body { background: var(--bg); color: var(--text); padding: 18px; display: flex; flex-direction: column; align-items: center; min-height: 100vh; }
    .container { width: 100%; max-width: 1050px; }
    
    header { text-align: center; margin-bottom: 20px; }
    header h1 { font-size: 1.8rem; color: var(--accent); margin-bottom: 6px; letter-spacing: 0.5px; }
    header p { color: var(--muted); font-size: 0.9rem; }
    .badge-bar { display: flex; justify-content: center; gap: 10px; margin-top: 10px; flex-wrap: wrap; }
    .badge { background: #1e293b; color: #cbd5e1; border: 1px solid #334155; padding: 4px 12px; border-radius: 20px; font-size: 0.8rem; font-family: monospace; }
    .badge-active { background: #14532d; color: #86efac; border-color: #22c55e; }

    /* Top Safety Controls */
    .safety-panel { background: var(--card-bg); border: 1px solid var(--card-border); border-radius: 14px; padding: 18px; margin-bottom: 22px; box-shadow: 0 4px 16px rgba(0,0,0,0.4); }
    .safety-header { display: flex; justify-content: space-between; align-items: center; margin-bottom: 14px; }
    .safety-title { font-weight: 700; font-size: 1.05rem; }
    .status-pill { font-size: 0.85rem; font-weight: bold; padding: 5px 12px; border-radius: 12px; }
    .status-off { background: #7f1d1d; color: #fca5a5; border: 1px solid #ef4444; }
    .status-on { background: #14532d; color: #86efac; border: 1px solid #22c55e; }

    .estop-btn { width: 100%; background: var(--danger); color: #fff; border: none; font-size: 1.25rem; font-weight: 800; padding: 16px; border-radius: 10px; cursor: pointer; transition: all 0.2s; box-shadow: 0 4px 14px rgba(239,68,68,0.4); text-transform: uppercase; letter-spacing: 1px; margin-bottom: 12px; }
    .estop-btn:hover { background: var(--danger-hover); transform: translateY(-1px); }
    .estop-btn:active { transform: translateY(1px); }

    .engine-btn-group { display: flex; gap: 12px; }
    .btn-start { flex: 1; background: var(--success); color: #022c22; font-size: 1rem; padding: 12px; font-weight: bold; border: none; border-radius: 8px; cursor: pointer; transition: 0.2s; }
    .btn-start:hover { background: #16a34a; color: #fff; }
    .btn-stop { flex: 1; background: var(--warning); color: #451a03; font-size: 1rem; padding: 12px; font-weight: bold; border: none; border-radius: 8px; cursor: pointer; transition: 0.2s; }
    .btn-stop:hover { background: #d97706; color: #fff; }

    /* Cards Grid */
    .grid { display: grid; grid-template-columns: repeat(auto-fit, minmax(480px, 1fr)); gap: 18px; margin-bottom: 20px; }
    @media (max-width: 600px) {
      .grid { grid-template-columns: 1fr; }
    }

    .card { background: var(--card-bg); border: 1px solid var(--card-border); border-radius: 14px; padding: 18px; box-shadow: 0 4px 12px rgba(0,0,0,0.3); }
    .card-title { font-size: 1.1rem; color: var(--accent); margin-bottom: 16px; border-bottom: 1px solid var(--card-border); padding-bottom: 8px; font-weight: 600; }

    .param-row { margin-bottom: 16px; }
    .param-row:last-child { margin-bottom: 0; }
    .param-meta { display: flex; justify-content: space-between; align-items: baseline; margin-bottom: 6px; }
    .param-name { font-weight: 600; font-size: 0.95rem; color: #e2e8f0; }
    .param-val { font-family: monospace; font-size: 1rem; color: var(--accent); background: #0b1120; padding: 2px 8px; border-radius: 6px; border: 1px solid var(--card-border); }

    .param-controls { display: flex; gap: 10px; align-items: center; }
    input[type=range] { flex: 1; accent-color: var(--accent); cursor: pointer; height: 6px; }
    input[type=number] { width: 90px; padding: 7px; background: #0b1120; border: 1px solid var(--card-border); border-radius: 6px; color: #fff; font-size: 0.95rem; text-align: center; }
    .btn-set { background: var(--accent); color: #090d16; border: none; font-weight: bold; padding: 8px 14px; border-radius: 6px; cursor: pointer; transition: 0.2s; }
    .btn-set:hover { background: var(--accent-hover); color: #fff; }

    .btn-save-all { width: 100%; padding: 14px; font-size: 1.05rem; background: #0284c7; color: #fff; border: none; border-radius: 10px; cursor: pointer; font-weight: bold; margin-top: 10px; transition: 0.2s; box-shadow: 0 4px 14px rgba(2,132,199,0.3); }
    .btn-save-all:hover { background: #0369a1; }

    /* Toast Notification */
    #toast { position: fixed; bottom: 20px; left: 50%; transform: translateX(-50%); background: var(--success); color: #022c22; padding: 12px 24px; border-radius: 8px; font-weight: bold; opacity: 0; transition: opacity 0.3s; pointer-events: none; box-shadow: 0 4px 16px rgba(0,0,0,0.5); z-index: 1000; font-size: 0.95rem; }
    #toast.show { opacity: 1; }
    #toast.toast-danger { background: var(--danger); color: #fff; }
  </style>
</head>
<body>
  <div class="container">
    <header>
      <h1>NXP Robot - Parameters Tuning</h1>
      <p>Modular Algorithm Parameter Interface</p>
      <div class="badge-bar">
        <span class="badge">AP: NXP_Robot_Tuning</span>
        <span class="badge">IP: 192.168.4.1</span>
        <span id="comm_badge" class="badge">UART: CONNECTING...</span>
      </div>
    </header>

    <!-- Top Safety & Engine Controls -->
    <div class="safety-panel">
      <div class="safety-header">
        <span class="safety-title">Motor & Safety Controls</span>
        <span id="engine_badge" class="status-pill status-off">STOPPED</span>
      </div>
      <button class="estop-btn" onclick="triggerEStop()">EMERGENCY STOP (FRANA URGENTA)</button>
      <div class="engine-btn-group">
        <button class="btn-start" onclick="setEngine(1)">START MOTORS (PORNESTE)</button>
        <button class="btn-stop" onclick="setEngine(0)">STOP MOTORS (OPRESTE)</button>
      </div>
    </div>

    <!-- Dynamic Parameter Cards Grid -->
    <div id="cards_grid" class="grid">
      <!-- Generated dynamically from /api/params -->
    </div>

    <button class="btn-save-all" onclick="saveAllParams()">Save All Parameters to Robot</button>
  </div>

  <div id="toast">Value Updated!</div>

  <script>
    let paramList = [];

    function showToast(msg, isDanger = false) {
      const t = document.getElementById('toast');
      t.innerText = msg;
      if (isDanger) t.classList.add('toast-danger');
      else t.classList.remove('toast-danger');
      t.classList.add('show');
      setTimeout(() => { t.classList.remove('show'); }, 2200);
    }

    function syncControls(key, val, fromInput = false) {
      const slider = document.getElementById('sl_' + key);
      const num = document.getElementById('num_' + key);
      const valDisp = document.getElementById('val_' + key);
      if (slider && !fromInput) slider.value = val;
      if (num && fromInput) num.value = val;
      if (valDisp) valDisp.innerText = parseFloat(val).toFixed(2);
    }

    function setParam(key) {
      const num = document.getElementById('num_' + key);
      if (!num) return;
      const val = num.value;
      fetch('/set?param=' + encodeURIComponent(key) + '&val=' + encodeURIComponent(val))
        .then(r => r.json())
        .then(d => {
          showToast('Updated: ' + key + ' = ' + val, false);
        })
        .catch(err => {
          showToast('Error sending ' + key, true);
        });
    }

    function setEngine(state) {
      fetch('/engine?state=' + state)
        .then(r => r.json())
        .then(d => {
          updateEngineUI(state === 1);
          showToast(state === 1 ? 'Motors STARTED' : 'Motors STOPPED', false);
        })
        .catch(err => showToast('Error toggling engine', true));
    }

    function triggerEStop() {
      fetch('/estop')
        .then(r => r.json())
        .then(d => {
          updateEngineUI(false);
          showToast('EMERGENCY STOP TRIGGERED!', true);
        })
        .catch(err => showToast('E-STOP Transmitted!', true));
    }

    function updateEngineUI(running) {
      const b = document.getElementById('engine_badge');
      if (running) {
        b.innerText = 'RUNNING';
        b.className = 'status-pill status-on';
      } else {
        b.innerText = 'STOPPED';
        b.className = 'status-pill status-off';
      }
    }

    function saveAllParams() {
      paramList.forEach((p, idx) => {
        setTimeout(() => {
          setParam(p.key);
        }, idx * 120);
      });
    }

    function renderUI(data) {
      paramList = data.params || [];
      const grid = document.getElementById('cards_grid');
      grid.innerHTML = '';

      // Group parameters by their 'group' property
      const groups = {};
      paramList.forEach(p => {
        if (!groups[p.group]) groups[p.group] = [];
        groups[p.group].push(p);
      });

      // Render each group as a card
      for (const [groupName, pGroup] of Object.entries(groups)) {
        const card = document.createElement('div');
        card.className = 'card';
        card.innerHTML = `<div class="card-title">${groupName}</div>`;

        pGroup.forEach(p => {
          const row = document.createElement('div');
          row.className = 'param-row';
          row.innerHTML = `
            <div class="param-meta">
              <span class="param-name">${p.label} <small style="color:var(--muted);">(${p.key})</small></span>
              <span class="param-val" id="val_${p.key}">${parseFloat(p.val).toFixed(2)}</span>
            </div>
            <div class="param-controls">
              <input type="range" id="sl_${p.key}" min="${p.min}" max="${p.max}" step="${p.step}" value="${p.val}"
                     oninput="syncControls('${p.key}', this.value, true)">
              <input type="number" id="num_${p.key}" min="${p.min}" max="${p.max}" step="${p.step}" value="${p.val}"
                     oninput="syncControls('${p.key}', this.value, false)">
              <button class="btn-set" onclick="setParam('${p.key}')">Set</button>
            </div>
          `;
          card.appendChild(row);
        });

        grid.appendChild(card);
      }

      updateEngineUI(data.engine_enabled === 1);
    }

    function pollStatus() {
      fetch('/api/status')
        .then(r => r.json())
        .then(d => {
          const badge = document.getElementById('comm_badge');
          if (d.duplex_ok) {
            badge.innerText = 'UART: FULL-DUPLEX OK';
            badge.className = 'badge badge-active';
          } else {
            badge.innerText = 'UART: TX ONLY (WAITING RX)';
            badge.className = 'badge';
          }
          updateEngineUI(d.engine_enabled === 1);
        })
        .catch(err => {})
        .finally(() => {
          setTimeout(pollStatus, 1500);
        });
    }

    window.onload = function() {
      fetch('/api/params')
        .then(r => r.json())
        .then(renderUI)
        .catch(e => console.error('Failed to load params:', e));

      pollStatus();
    };
  </script>
</body>
</html>
)rawliteral";

/* ========================================================================= */
/* HTTP Handlers                                                             */
/* ========================================================================= */
void handleRoot() {
    server.sendHeader("Access-Control-Allow-Origin", "*");
    server.send(200, "text/html", INDEX_HTML);
}

void handleGetParams() {
    server.sendHeader("Access-Control-Allow-Origin", "*");
    String json = "{\"engine_enabled\":";
    json += String(engine_enabled ? 1 : 0);
    json += ",\"params\":[";

    for (size_t i = 0; i < NUM_PARAMS; i++) {
        if (i > 0) json += ",";
        json += "{";
        json += "\"key\":\"" + String(params[i].key) + "\",";
        json += "\"label\":\"" + String(params[i].label) + "\",";
        json += "\"group\":\"" + String(params[i].group) + "\",";
        json += "\"min\":" + String(params[i].minVal, 2) + ",";
        json += "\"max\":" + String(params[i].maxVal, 2) + ",";
        json += "\"step\":" + String(params[i].step, 2) + ",";
        json += "\"val\":" + String(params[i].currentVal, 4);
        json += "}";
    }
    json += "]}";

    server.send(200, "application/json", json);
}

void handleSetParam() {
    server.sendHeader("Access-Control-Allow-Origin", "*");
    if (!server.hasArg("param") || !server.hasArg("val")) {
        server.send(400, "application/json", "{\"status\":\"error\",\"message\":\"Missing param or val\"}");
        return;
    }

    String paramName = server.arg("param");
    float val = server.arg("val").toFloat();

    int foundIdx = -1;
    for (size_t i = 0; i < NUM_PARAMS; i++) {
        if (paramName == params[i].key) {
            params[i].currentVal = val;
            foundIdx = (int)i;
            break;
        }
    }

    if (foundIdx != -1) {
        saveStoredParam(foundIdx);
        sendParamToFRDM(paramName.c_str(), val);
        server.send(200, "application/json", "{\"status\":\"ok\",\"param\":\"" + paramName + "\",\"val\":" + String(val, 4) + "}");
    } else {
        server.send(404, "application/json", "{\"status\":\"error\",\"message\":\"Unknown parameter\"}");
    }
}

void handleEngine() {
    server.sendHeader("Access-Control-Allow-Origin", "*");
    int state = server.hasArg("state") ? server.arg("state").toInt() : 0;
    engine_enabled = (state == 1);

    Serial2.printf("ENGINE_ENABLED=%d\n", engine_enabled ? 1 : 0);
    Serial2.flush();
    Serial.printf("[ESP32 TX -> FRDM] ENGINE_ENABLED=%d\n", engine_enabled ? 1 : 0);

    server.send(200, "application/json", "{\"status\":\"ok\",\"engine_enabled\":" + String(engine_enabled ? 1 : 0) + "}");
}

void handleEStop() {
    server.sendHeader("Access-Control-Allow-Origin", "*");
    engine_enabled = false;

    Serial2.print("EMERGENCY_STOP=1\n");
    Serial2.flush();
    Serial.println("[ESP32 TX -> FRDM] EMERGENCY_STOP=1");

    server.send(200, "application/json", "{\"status\":\"ok\",\"estop\":1}");
}

void handleStatus() {
    server.sendHeader("Access-Control-Allow-Origin", "*");
    bool duplexOk = (lastPongReceived > 0 && (pingSequence - lastPongReceived) <= 2);

    String json = "{";
    json += "\"engine_enabled\":" + String(engine_enabled ? 1 : 0) + ",";
    json += "\"duplex_ok\":" + String(duplexOk ? "true" : "false") + ",";
    json += "\"rx_packets\":" + String(rxPacketCount) + ",";
    json += "\"last_rx\":\"" + lastRxMessage + "\"";
    json += "}";

    server.send(200, "application/json", json);
}

/* ========================================================================= */
/* Arduino Setup & Main Loop                                                 */
/* ========================================================================= */
void setup() {
    delay(1500);
    Serial.begin(115200);                        // USB Serial Debug Console
    Serial2.begin(115200, SERIAL_8N1, 16, 17);  // RX=GPIO16, TX=GPIO17 to FRDM P3_3/P3_2

    Serial.println("\n==================================================");
    Serial.println("   NXP CUP - ESP32 Modular Parameter Tuning Server ");
    Serial.println("==================================================");

    loadStoredParams();

    // Start Wi-Fi Access Point
    WiFi.mode(WIFI_AP);
    if (WiFi.softAP(AP_SSID, AP_PASS)) {
        Serial.printf("[ESP32] Access Point created! SSID: %s\n", AP_SSID);
        Serial.printf("[ESP32] Web Dashboard URL: http://%s\n", WiFi.softAPIP().toString().c_str());
    } else {
        Serial.println("[ESP32 ERROR] Failed to start Access Point!");
    }

    // Start mDNS Responder
    if (MDNS.begin(MDNS_NAME)) {
        Serial.printf("[ESP32] mDNS Responder active: http://%s.local\n", MDNS_NAME);
        MDNS.addService("http", "tcp", 80);
    }

    // Configure Web Server HTTP Routes
    server.on("/", handleRoot);
    server.on("/api/params", handleGetParams);
    server.on("/api/status", handleStatus);
    server.on("/set", handleSetParam);
    server.on("/engine", handleEngine);
    server.on("/estop", handleEStop);

    server.begin();
    Serial.println("[ESP32] HTTP Web Server running on port 80.\n");

    // Transmit all saved parameters to FRDM board on startup
    syncAllParamsToFRDM();
}

void loop() {
    server.handleClient();
    sendHeartbeat();
    processSerial2Input();
}