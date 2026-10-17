/*
 * NXP CUP - ESP32 Modular Parameter Tuning Web Server
 * Branch: tuning_bootcamp
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

WebServer   server(80);
Preferences preferences;

/* ========================================================================= */
/* Modular Parameter Definition Structure                                    */
/* Add, modify, or remove parameters here to match Config.h                 */
/* ========================================================================= */
struct ParamDef {
    const char* key;          // Exact command name sent to FRDM (must match wifi.c registry)
    const char* label;        // Human-readable title shown in the web UI
    const char* group;        // Card grouping in the web UI
    float minVal;             // Slider minimum
    float maxVal;             // Slider maximum
    float step;               // Step resolution
    float defaultVal;         // Factory default (matches wifi.c initializers)
    float currentVal;         // Live value in RAM (loaded from NVS on boot)
};

ParamDef params[] = {
    // ── Steering PID (2-Zone Variable PID) ──────────────────────────────────
    {"STEERING_KP_STRAIGHT",       "Straight Kp",              "Steering PID",       0.0f,  10.0f, 0.05f,   1.0f,   1.0f},
    {"STEERING_KD_STRAIGHT",       "Straight Kd",              "Steering PID",       0.0f,  50.0f,  0.5f,  10.0f,  10.0f},
    {"STEERING_KP_CURVE",          "Curve Kp",                 "Steering PID",       0.0f,  15.0f, 0.05f,   2.0f,   2.0f},
    {"STEERING_KD_CURVE",          "Curve Kd",                 "Steering PID",       0.0f,  10.0f, 0.05f,   2.0f,   2.0f},
    {"STEERING_DEADBAND_STRAIGHT", "Deadband Straight (\xC2\xB0)", "Steering PID",   0.0f,  15.0f,  0.1f,   2.5f,   2.5f},
    {"STEERING_DEADBAND_CURVE",    "Deadband Curve (\xC2\xB0)",    "Steering PID",   0.0f,  10.0f,  0.1f,   1.0f,   1.0f},

    // ── Curve Hysteresis ────────────────────────────────────────────────────
    {"CURVE_ENTER_ANGLE_DEG",      "Curve Enter Angle (\xC2\xB0)", "Curve Hysteresis", 5.0f, 70.0f, 1.0f,  35.0f,  35.0f},
    {"CURVE_EXIT_ANGLE_DEG",       "Curve Exit Angle (\xC2\xB0)",  "Curve Hysteresis", 1.0f, 40.0f, 1.0f,   5.0f,   5.0f},
    {"CURVE_EXIT_FRAMES",          "Curve Exit Frames",        "Curve Hysteresis",   1.0f,  10.0f,  1.0f,   2.0f,   2.0f},

    // ── Camera ──────────────────────────────────────────────────────────────
    {"PERSPECTIVE_ANGLE_OFFSET",   "Perspective Offset (\xC2\xB0)","Camera",          0.0f,  20.0f,  0.5f,   5.0f,   5.0f},

    // ── Steering Limits ─────────────────────────────────────────────────────
    {"STEERING_LIMIT_RIGHT",       "Limit Right (\xC2\xB0)",   "Steering Limits",    0.0f,  90.0f,  1.0f,  65.0f,  65.0f},
    {"STEERING_LIMIT_LEFT",        "Limit Left (\xC2\xB0)",    "Steering Limits",  -90.0f,   0.0f,  1.0f, -65.0f, -65.0f},
    {"STEERING_OFFSET",            "Center Offset (\xC2\xB0)", "Steering Limits",  -30.0f,  30.0f,  0.5f,  15.0f,  15.0f},

    // ── Motor & Speed ────────────────────────────────────────────────────────
    {"SPEED_RIGHT",                "Right Base Speed (%)",     "Motor & Speed",      0.0f, 100.0f,  1.0f, 100.0f, 100.0f},
    {"SPEED_LEFT",                 "Left Base Speed (%)",      "Motor & Speed",      0.0f, 100.0f,  1.0f, 100.0f, 100.0f},
    {"CORNER_BRAKE_COEFF",         "Corner Brake Coeff",       "Motor & Speed",      0.0f,   5.0f, 0.05f,   1.7f,   1.7f},
    {"DIFF_SPEED_COEFF",           "Diff Speed Coeff",         "Motor & Speed",      0.0f,   2.0f, 0.01f,  0.38f,  0.38f},

    // ── Detection ────────────────────────────────────────────────────────────
    {"VERTICAL_MIN_SLOPE",         "Vertical Min Slope",       "Detection",          0.1f,   3.0f,0.025f, 0.375f, 0.375f},
    {"DECAY_FACTOR",               "Steering Decay Factor",    "Detection",          0.0f,   1.0f, 0.01f,   0.5f,   0.5f},
    {"NO_LINE_MAX_LOST_FRAMES",    "Max Lost Frames",          "Detection",          1.0f,  30.0f,  1.0f,   5.0f,   5.0f},
};
const size_t NUM_PARAMS = sizeof(params) / sizeof(params[0]);

// Engine state (always starts stopped for safety)
bool engine_enabled = false;

// Diagnostics
unsigned long lastPingTime     = 0;
uint32_t      pingSequence     = 0;
uint32_t      lastPongReceived = 0;
String        lastRxMessage    = "NONE";
uint32_t      rxPacketCount    = 0;

/* ========================================================================= */
/* NVS Persistent Storage                                                    */
/* ========================================================================= */
void loadStoredParams() {
    preferences.begin("nxp_tuning", false);
    for (size_t i = 0; i < NUM_PARAMS; i++) {
        char nvsKey[16];
        snprintf(nvsKey, sizeof(nvsKey), "p_%d", (int)i);
        params[i].currentVal = preferences.getFloat(nvsKey, params[i].defaultVal);
    }
    engine_enabled = false;  // Always safe-stopped on boot
}

void saveStoredParam(size_t index) {
    if (index >= NUM_PARAMS) return;
    char nvsKey[16];
    snprintf(nvsKey, sizeof(nvsKey), "p_%d", (int)index);
    preferences.putFloat(nvsKey, params[index].currentVal);
}

/* ========================================================================= */
/* UART Helpers — communicate with FRDM over Serial2                        */
/* ========================================================================= */
void sendParamToFRDM(const char* name, float val) {
    // Protocol: "NAME_OF_THE_PARAMETER=NEW_VALUE\n"
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
/* Incoming UART Data Processing (FRDM -> ESP32)                            */
/* ========================================================================= */
void processSerial2Input() {
    static String rxLine = "";
    while (Serial2.available()) {
        char c = (char)Serial2.read();
        Serial.write(c);  // Mirror to USB serial monitor

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
                rxLine = "";  // Overflow guard
            }
        }
    }
}

/* ========================================================================= */
/* Periodic Heartbeat (PING every 2 s)                                       */
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
/* Web Dashboard HTML (single-page app, auto-generated from params[])       */
/* ========================================================================= */
static const char INDEX_HTML[] PROGMEM = R"rawliteral(
<!DOCTYPE html>
<html lang="en">
<head>
  <meta charset="UTF-8">
  <meta name="viewport" content="width=device-width, initial-scale=1.0">
  <title>NXP CUP — Parameter Tuning</title>
  <style>
    :root {
      --bg: #0f1117; --card: #1a1d27; --accent: #00d4ff;
      --text: #e0e6f0; --muted: #6b7280; --danger: #ef4444;
      --success: #22c55e; --border: #2d3148;
    }
    * { box-sizing: border-box; margin: 0; padding: 0; }
    body { background: var(--bg); color: var(--text); font-family: 'Segoe UI', sans-serif; padding: 16px; }
    header { display: flex; align-items: center; justify-content: space-between; margin-bottom: 20px; flex-wrap: wrap; gap: 10px; }
    h1 { font-size: 1.3rem; color: var(--accent); }
    .badge { background: var(--card); border: 1px solid var(--border); border-radius: 20px; padding: 4px 12px; font-size: 0.75rem; color: var(--muted); }
    .badge-active { border-color: var(--success); color: var(--success); }
    .controls { display: flex; gap: 8px; flex-wrap: wrap; }
    button { cursor: pointer; border: none; border-radius: 8px; padding: 8px 16px; font-size: 0.85rem; font-weight: 600; }
    .btn-run  { background: var(--success); color: #fff; }
    .btn-stop { background: var(--danger);  color: #fff; }
    .btn-sync { background: var(--accent);  color: #000; }
    .status-pill { display: inline-block; padding: 4px 10px; border-radius: 12px; font-size: 0.8rem; font-weight: 700; }
    .status-on  { background: #14532d; color: var(--success); }
    .status-off { background: #450a0a; color: var(--danger); }
    #cards_grid { display: grid; grid-template-columns: repeat(auto-fill, minmax(320px, 1fr)); gap: 16px; }
    .card { background: var(--card); border: 1px solid var(--border); border-radius: 12px; padding: 16px; }
    .card-title { font-size: 0.8rem; font-weight: 700; text-transform: uppercase; letter-spacing: .08em; color: var(--accent); margin-bottom: 12px; }
    .param-row { margin-bottom: 14px; }
    .param-meta { display: flex; justify-content: space-between; align-items: baseline; margin-bottom: 6px; }
    .param-name { font-size: 0.88rem; font-weight: 600; }
    .param-val  { font-size: 1rem; font-weight: 700; color: var(--accent); min-width: 48px; text-align: right; }
    .param-controls { display: flex; gap: 6px; align-items: center; }
    input[type=range] { flex: 1; accent-color: var(--accent); }
    input[type=number] { width: 72px; background: var(--bg); border: 1px solid var(--border); color: var(--text); border-radius: 6px; padding: 4px 6px; font-size: 0.82rem; }
    .btn-set { background: var(--accent); color: #000; border-radius: 6px; padding: 4px 10px; font-size: 0.78rem; font-weight: 700; }
  </style>
</head>
<body>
  <header>
    <div>
      <h1>&#x1F916; NXP CUP Parameter Tuning</h1>
      <small style="color:var(--muted)">tuning_bootcamp &bull; 20 parameters</small>
    </div>
    <div class="controls">
      <span id="comm_badge" class="badge">UART: Connecting...</span>
      <span id="engine_status" class="status-pill status-off">STOPPED</span>
      <button class="btn-run"  onclick="setEngine(1)">&#x25B6; RUN</button>
      <button class="btn-stop" onclick="setEngine(0)">&#x23F9; STOP</button>
      <button class="btn-stop" onclick="eStop()"    >&#x26A0; E-STOP</button>
      <button class="btn-sync" onclick="saveAllParams()">&#x21BA; Sync All</button>
    </div>
  </header>

  <div id="cards_grid"></div>

  <script>
    let paramList = [];

    function syncControls(key, value, fromSlider) {
      document.getElementById('val_'  + key).innerText = parseFloat(value).toFixed(2);
      if (fromSlider) document.getElementById('num_' + key).value = value;
      else            document.getElementById('sl_'  + key).value = value;
    }

    function setParam(key) {
      const val = document.getElementById('sl_' + key).value;
      fetch('/set?param=' + encodeURIComponent(key) + '&val=' + encodeURIComponent(val))
        .then(r => r.json())
        .then(d => { if (d.status === 'ok') document.getElementById('val_' + key).innerText = parseFloat(d.val).toFixed(2); })
        .catch(e => console.error('setParam error:', e));
    }

    function setEngine(state) {
      fetch('/engine?state=' + state).then(r => r.json()).then(d => updateEngineUI(d.engine_enabled === 1));
    }

    function eStop() {
      fetch('/estop').then(r => r.json()).then(d => updateEngineUI(false));
    }

    function updateEngineUI(running) {
      const b = document.getElementById('engine_status');
      if (running) { b.innerText = 'RUNNING'; b.className = 'status-pill status-on'; }
      else         { b.innerText = 'STOPPED'; b.className = 'status-pill status-off'; }
    }

    function saveAllParams() {
      paramList.forEach((p, idx) => { setTimeout(() => setParam(p.key), idx * 120); });
    }

    function renderUI(data) {
      paramList = data.params || [];
      const grid = document.getElementById('cards_grid');
      grid.innerHTML = '';
      const groups = {};
      paramList.forEach(p => { if (!groups[p.group]) groups[p.group] = []; groups[p.group].push(p); });
      for (const [groupName, pGroup] of Object.entries(groups)) {
        const card = document.createElement('div');
        card.className = 'card';
        card.innerHTML = `<div class="card-title">${groupName}</div>`;
        pGroup.forEach(p => {
          const row = document.createElement('div');
          row.className = 'param-row';
          row.innerHTML = `
            <div class="param-meta">
              <span class="param-name">${p.label} <small style="color:var(--muted)">(${p.key})</small></span>
              <span class="param-val" id="val_${p.key}">${parseFloat(p.val).toFixed(2)}</span>
            </div>
            <div class="param-controls">
              <input type="range"  id="sl_${p.key}"  min="${p.min}" max="${p.max}" step="${p.step}" value="${p.val}"
                     oninput="syncControls('${p.key}', this.value, true)">
              <input type="number" id="num_${p.key}" min="${p.min}" max="${p.max}" step="${p.step}" value="${p.val}"
                     oninput="syncControls('${p.key}', this.value, false)">
              <button class="btn-set" onclick="setParam('${p.key}')">Set</button>
            </div>`;
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
          if (d.duplex_ok) { badge.innerText = 'UART: FULL-DUPLEX OK'; badge.className = 'badge badge-active'; }
          else             { badge.innerText = 'UART: TX ONLY (WAITING RX)'; badge.className = 'badge'; }
          updateEngineUI(d.engine_enabled === 1);
        })
        .catch(() => {})
        .finally(() => setTimeout(pollStatus, 1500));
    }

    window.onload = function() {
      fetch('/api/params').then(r => r.json()).then(renderUI).catch(e => console.error('Failed to load params:', e));
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
        json += "\"key\":\""   + String(params[i].key)   + "\",";
        json += "\"label\":\"" + String(params[i].label) + "\",";
        json += "\"group\":\"" + String(params[i].group) + "\",";
        json += "\"min\":"     + String(params[i].minVal, 2)     + ",";
        json += "\"max\":"     + String(params[i].maxVal, 2)     + ",";
        json += "\"step\":"    + String(params[i].step, 4)       + ",";
        json += "\"val\":"     + String(params[i].currentVal, 4);
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
    float  val       = server.arg("val").toFloat();

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
        server.send(200, "application/json",
                    "{\"status\":\"ok\",\"param\":\"" + paramName + "\",\"val\":" + String(val, 4) + "}");
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
    server.send(200, "application/json",
                "{\"status\":\"ok\",\"engine_enabled\":" + String(engine_enabled ? 1 : 0) + "}");
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
    json += "\"engine_enabled\":"  + String(engine_enabled ? 1 : 0) + ",";
    json += "\"duplex_ok\":"       + String(duplexOk ? "true" : "false") + ",";
    json += "\"rx_packets\":"      + String(rxPacketCount) + ",";
    json += "\"last_rx\":\""       + lastRxMessage + "\"";
    json += "}";
    server.send(200, "application/json", json);
}

/* ========================================================================= */
/* Arduino Setup & Main Loop                                                 */
/* ========================================================================= */
void setup() {
    delay(1500);
    Serial.begin(115200);                        // USB Serial Debug Console
    Serial2.begin(115200, SERIAL_8N1, 16, 17);  // RX=GPIO16, TX=GPIO17 -> FRDM P3_3/P3_2

    Serial.println("\n==================================================");
    Serial.println("   NXP CUP - ESP32 Parameter Tuning Server");
    Serial.println("   Branch: tuning_bootcamp | 20 parameters");
    Serial.println("==================================================");

    loadStoredParams();

    // Start Wi-Fi Access Point with stability optimizations
    WiFi.mode(WIFI_AP);
    WiFi.setSleep(false); // Disable modem sleep to prevent beacon dropouts & disconnects

    // Configure AP on Channel 6, SSID broadcasted, max 4 clients
    if (WiFi.softAP(AP_SSID, AP_PASS, 6, 0, 4)) {
        // Set TX power to 17dBm to prevent violent current spikes/brownouts while keeping strong signal
        WiFi.setTxPower(WIFI_POWER_17dBm);
        Serial.printf("[ESP32] AP created! SSID: %s (Channel 6)\n", AP_SSID);
        Serial.printf("[ESP32] Dashboard: http://%s\n", WiFi.softAPIP().toString().c_str());
    } else {
        Serial.println("[ESP32 ERROR] Failed to start Access Point!");
    }

    // Start mDNS
    if (MDNS.begin(MDNS_NAME)) {
        Serial.printf("[ESP32] mDNS active: http://%s.local\n", MDNS_NAME);
        MDNS.addService("http", "tcp", 80);
    }

    // Configure HTTP routes
    server.on("/",           handleRoot);
    server.on("/api/params", handleGetParams);
    server.on("/api/status", handleStatus);
    server.on("/set",        handleSetParam);
    server.on("/engine",     handleEngine);
    server.on("/estop",      handleEStop);

    server.begin();
    Serial.println("[ESP32] HTTP server running on port 80.\n");

    // Push all saved parameter values to FRDM on startup
    syncAllParamsToFRDM();
}

void loop() {
    server.handleClient();
    sendHeartbeat();
    processSerial2Input();
}
