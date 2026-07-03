#include <Arduino.h>
#include <WiFi.h>
#include <WebServer.h>
#include <WiFiUdp.h>
#include <esp_now.h>
#include <Wire.h>
#include <Adafruit_INA219.h>

// ── WiFi credentials ──────────────────────────────────────────────────────────
const char* WIFI_SSID = "Airtel_Manu catering";
const char* WIFI_PASS = "Mano@888555";

// ── Pin definitions ───────────────────────────────────────────────────────────
#define PIN_SSR        26
#define PIN_OPTO       33

// ── Voltage thresholds — 4S LiPo (4.2 V/cell) ───────────────────────────────
#define LIPO_CONNECTED 12.0f
#define LIPO_MIN       13.2f
#define LIPO_MAX       16.8f
#define TARGET_VOLTAGE 15.5f

// ── Timing ────────────────────────────────────────────────────────────────────
#define START_PRESS_MS      3000
#define START_PRESS_GAP_MS  1000
#define MONITOR_INTERVAL   10000
#define DISPLAY_INTERVAL    2000

// ── Noise rejection ───────────────────────────────────────────────────────────
#define SAMPLE_COUNT    20
#define TRIM_COUNT       4
#define TARGET_CONFIRM   1
#define MAX_CONFIRM      2

// ── Sensor fault ─────────────────────────────────────────────────────────────
#define VOLTAGE_SENSOR_FAULT  -1.0f
#define SENSOR_FAULT_LIMIT     3

// ── Log buffer ────────────────────────────────────────────────────────────────
#define LOG_LINES     200
#define LOG_LINE_LEN  120
char     logBuf[LOG_LINES][LOG_LINE_LEN];
uint16_t logHead    = 0;   // next write index (circular)
uint16_t logCount   = 0;   // total lines written (caps at LOG_LINES)
portMUX_TYPE logMux = portMUX_INITIALIZER_UNLOCKED;

// ── INA219 ────────────────────────────────────────────────────────────────────
Adafruit_INA219 ina219;

// ── Web server ────────────────────────────────────────────────────────────────
WebServer server(80);

// ── UDP mirror ────────────────────────────────────────────────────────────────
WiFiUDP udp;
const IPAddress udpTargetIp(255, 255, 255, 255);
constexpr uint16_t UDP_PORT = 12345;
constexpr size_t UDP_CHUNK_SIZE = 512;

// ── ESP-NOW ───────────────────────────────────────────────────────────────────
uint8_t droneMacAddress[] = {0x08, 0xF9, 0xE0, 0x67, 0xDB, 0x5D};
typedef struct struct_message { char msg[32]; } struct_message;
struct_message myData;

// ── State ─────────────────────────────────────────────────────────────────────
float    startVoltage     = 0.0f;
bool     charging         = false;
uint8_t  targetHits       = 0;
uint8_t  maxHits          = 0;
uint8_t  sensorFaultCount = 0;
unsigned long lastMonitor = 0;
unsigned long lastDisplay = 0;

// ═════════════════════════════════════════════════════════════════════════════
// Logging — Saves to internal buffer for the web page & prints to Serial
// ═════════════════════════════════════════════════════════════════════════════
void tLog(const char* msg) {
  Serial.println(msg);
  portENTER_CRITICAL(&logMux);
  strncpy(logBuf[logHead], msg, LOG_LINE_LEN - 1);
  logBuf[logHead][LOG_LINE_LEN - 1] = '\0';
  logHead = (logHead + 1) % LOG_LINES;
  if (logCount < LOG_LINES) logCount++;
  portEXIT_CRITICAL(&logMux);
}

void tLogf(const char* fmt, ...) {
  char tmp[LOG_LINE_LEN];
  va_list args;
  va_start(args, fmt);
  vsnprintf(tmp, sizeof(tmp), fmt, args);
  va_end(args);
  tLog(tmp);
}

void sendLogsOverUdp(const String& payload) {
  if (WiFi.status() != WL_CONNECTED || payload.length() == 0) return;

  for (size_t offset = 0; offset < payload.length(); offset += UDP_CHUNK_SIZE) {
    size_t len = min(UDP_CHUNK_SIZE, payload.length() - offset);
    udp.beginPacket(udpTargetIp, UDP_PORT);
    udp.write(reinterpret_cast<const uint8_t*>(payload.c_str() + offset), len);
    udp.endPacket();
    delay(1);
  }
}

// ═════════════════════════════════════════════════════════════════════════════
// Web server handlers
// ═════════════════════════════════════════════════════════════════════════════

// GET /logs  — returns all buffered lines as plain text
void handleLogs() {
  String out;
  out.reserve(LOG_LINES * 60);
  portENTER_CRITICAL(&logMux);
  uint16_t start = (logCount < LOG_LINES) ? 0 : logHead;
  uint16_t total = (logCount < LOG_LINES) ? logCount : LOG_LINES;
  for (uint16_t i = 0; i < total; i++) {
    uint16_t idx = (start + i) % LOG_LINES;
    out += logBuf[idx];
    out += '\n';
  }
  portEXIT_CRITICAL(&logMux);
  sendLogsOverUdp(out);
  server.send(200, "text/plain", out);
}

// GET /  — serves the dashboard page
void handleRoot() {
  String html = R"rawhtml(
<!DOCTYPE html>
<html lang="en">
<head>
<meta charset="UTF-8">
<meta name="viewport" content="width=device-width,initial-scale=1">
<title>ESP32 Charger Log</title>
<style>
  :root {
    --bg:      #0d1117;
    --surface: #161b22;
    --border:  #30363d;
    --green:   #3fb950;
    --yellow:  #d29922;
    --red:     #f85149;
    --text:    #c9d1d9;
    --dim:     #6e7681;
    --accent:  #58a6ff;
    --font:    'Courier New', Courier, monospace;
  }
  * { box-sizing: border-box; margin: 0; padding: 0; }
  body { background: var(--bg); color: var(--text); font-family: var(--font); font-size: 13px; height: 100vh; display: flex; flex-direction: column; }
  header { background: var(--surface); border-bottom: 1px solid var(--border); padding: 10px 16px; display: flex; align-items: center; gap: 12px; flex-shrink: 0; }
  header h1 { font-size: 14px; font-weight: bold; color: var(--accent); letter-spacing: 0.05em; }
  #status-dot { width: 9px; height: 9px; border-radius: 50%; background: var(--dim); flex-shrink: 0; transition: background 0.3s; }
  #status-dot.live  { background: var(--green); box-shadow: 0 0 6px var(--green); }
  #status-dot.error { background: var(--red);   box-shadow: 0 0 6px var(--red); }
  #status-text { color: var(--dim); font-size: 12px; }
  .spacer { flex: 1; }
  #auto-label { color: var(--dim); font-size: 12px; }
  #auto-scroll-btn { background: none; border: 1px solid var(--border); color: var(--text); font-family: var(--font); font-size: 12px; padding: 3px 10px; cursor: pointer; border-radius: 4px; }
  #auto-scroll-btn.on { border-color: var(--accent); color: var(--accent); }
  #clear-btn { background: none; border: 1px solid var(--border); color: var(--dim); font-family: var(--font); font-size: 12px; padding: 3px 10px; cursor: pointer; border-radius: 4px; }
  #log-container { flex: 1; overflow-y: auto; padding: 10px 14px; }
  #log-container::-webkit-scrollbar { width: 6px; }
  #log-container::-webkit-scrollbar-track { background: var(--bg); }
  #log-container::-webkit-scrollbar-thumb { background: var(--border); border-radius: 3px; }
  .line { white-space: pre-wrap; word-break: break-all; line-height: 1.6; padding: 1px 0; }
  .line.emergency { color: var(--red); font-weight: bold; }
  .line.warning   { color: var(--yellow); }
  .line.ok        { color: var(--green); }
  .line.live      { color: var(--accent); }
  .line.sep       { color: var(--border); }
  footer { background: var(--surface); border-top: 1px solid var(--border); padding: 5px 16px; color: var(--dim); font-size: 11px; display: flex; gap: 20px; flex-shrink: 0; }
</style>
</head>
<body>
<header>
  <div id="status-dot"></div>
  <h1>ESP32 · LiPo Charger Log</h1>
  <span id="status-text">connecting…</span>
  <div class="spacer"></div>
  <span id="auto-label">auto-scroll</span>
  <button id="auto-scroll-btn" class="on" onclick="toggleAutoScroll()">ON</button>
  <button id="clear-btn" onclick="clearLocal()">clear</button>
</header>
<div id="log-container"></div>
<footer>
  <span id="line-count">0 lines</span>
  <span id="last-update">—</span>
  <span>poll: 2 s</span>
</footer>

<script>
  let autoScroll = true;
  let prevText   = "";
  let localClear = false;

  const container   = document.getElementById('log-container');
  const dot         = document.getElementById('status-dot');
  const statusText  = document.getElementById('status-text');
  const lineCount   = document.getElementById('line-count');
  const lastUpdate  = document.getElementById('last-update');
  const autoBtn     = document.getElementById('auto-scroll-btn');

  function classify(line) {
    const u = line.toUpperCase();
    if (u.includes('EMERGENCY') || u.includes('FATAL') || u.includes('FAULT')) return 'emergency';
    if (u.includes('WARNING') || u.includes('ERROR') || u.includes('SAFETY')) return 'warning';
    if (u.includes('OK') || u.includes('CONFIRMED') || u.includes('STARTED') || u.includes('CONNECTED')) return 'ok';
    if (u.includes('[LIVE]')) return 'live';
    if (u.startsWith('===') || u.startsWith('---')) return 'sep';
    return '';
  }

  function renderLines(text) {
    if (localClear) return;
    if (text === prevText) return;
    prevText = text;
    container.innerHTML = '';
    const lines = text.split('\n').filter(l => l.length > 0);
    lines.forEach(l => {
      const div = document.createElement('div');
      div.className = 'line ' + classify(l);
      div.textContent = l;
      container.appendChild(div);
    });
    lineCount.textContent = lines.length + ' lines';
    if (autoScroll) container.scrollTop = container.scrollHeight;
  }

  async function fetchLogs() {
    try {
      const r = await fetch('/logs');
      if (!r.ok) throw new Error('HTTP ' + r.status);
      const text = await r.text();
      dot.className = 'live';
      statusText.textContent = 'live';
      lastUpdate.textContent = 'updated ' + new Date().toLocaleTimeString();
      renderLines(text);
    } catch(e) {
      dot.className = 'error';
      statusText.textContent = 'unreachable';
    }
  }

  function toggleAutoScroll() {
    autoScroll = !autoScroll;
    autoBtn.textContent = autoScroll ? 'ON' : 'OFF';
    autoBtn.className   = autoScroll ? 'on' : '';
    if (autoScroll) container.scrollTop = container.scrollHeight;
  }

  function clearLocal() {
    localClear = true;
    container.innerHTML = '';
    prevText = '';
    lineCount.textContent = '0 lines';
    setTimeout(() => { localClear = false; }, 100);
  }

  fetchLogs();
  setInterval(fetchLogs, 2000);
</script>
</body>
</html>
)rawhtml";
  server.send(200, "text/html", html);
}

// ═════════════════════════════════════════════════════════════════════════════
// ESP-NOW send callback
// ═════════════════════════════════════════════════════════════════════════════
void onDataSent(const wifi_tx_info_t *info, esp_now_send_status_t status) {
  tLogf("ESP-NOW send: %s", status == ESP_NOW_SEND_SUCCESS ? "OK" : "FAILED");
}

// ═════════════════════════════════════════════════════════════════════════════
// Voltage reads
// ═════════════════════════════════════════════════════════════════════════════
float quickRead() {
  float bus_v    = ina219.getBusVoltage_V();
  float shunt_mv = ina219.getShuntVoltage_mV();
  return bus_v + (shunt_mv / 1000.0f);
}

float readVoltage() {
  float buf[SAMPLE_COUNT];
  for (int i = 0; i < SAMPLE_COUNT; i++) {
    float bus_v    = ina219.getBusVoltage_V();
    float shunt_mv = ina219.getShuntVoltage_mV();
    buf[i] = bus_v + (shunt_mv / 1000.0f);
    delay(5);
  }

  for (int i = 1; i < SAMPLE_COUNT; i++) {
    float key = buf[i]; int j = i - 1;
    while (j >= 0 && buf[j] > key) { buf[j+1] = buf[j]; j--; }
    buf[j+1] = key;
  }

  float total = 0.0f;
  for (int i = TRIM_COUNT; i < SAMPLE_COUNT - TRIM_COUNT; i++) total += buf[i];
  float batt_v = total / (float)(SAMPLE_COUNT - 2 * TRIM_COUNT);

  if (batt_v < 8.0f || batt_v > 18.0f) {
    sensorFaultCount++;
    tLogf("  [SENSOR FAULT %d/%d — implausible reading: %.3fV]",
          sensorFaultCount, SENSOR_FAULT_LIMIT, batt_v);
    return VOLTAGE_SENSOR_FAULT;
  }

  sensorFaultCount = 0;
  return batt_v;
}

// ═════════════════════════════════════════════════════════════════════════════
// Charge control helpers
// ═════════════════════════════════════════════════════════════════════════════
void sendEspNow(const char* message) {
  strcpy(myData.msg, message);
  esp_now_send(droneMacAddress, (uint8_t *)&myData, sizeof(myData));
  tLogf("ESP-NOW sent: %s", message);
}

void emergencyStop(const char* reason) {
  tLog("========================================");
  tLogf("EMERGENCY STOP: %s", reason);
  tLog("========================================");
  digitalWrite(PIN_SSR,  LOW);
  digitalWrite(PIN_OPTO, LOW);
  sendEspNow("CHARGING_FAULT");
  charging         = false;
  targetHits       = 0;
  maxHits          = 0;
  sensorFaultCount = 0;
  tLog("SSR cut. Reset ESP32 to retry.");
  while (true) { server.handleClient(); delay(100); }  // keep web server alive
}

void pressStart() {
  tLog("  Pressing START...");
  digitalWrite(PIN_OPTO, HIGH);
  delay(START_PRESS_MS);
  digitalWrite(PIN_OPTO, LOW);
  tLog("  START released.");
}

void startCharging() {
  tLog("========================================");
  tLog("STARTING CHARGE SEQUENCE");
  tLog("========================================");
  tLog("[1/4] Notifying drone...");
  sendEspNow("CHARGING_START");
  tLog("[2/4] SSR ON");
  digitalWrite(PIN_SSR, HIGH);
  tLog("[3/4] Waiting for iMAX boot...");
  for (int i = 5; i > 0; i--) {
    tLogf("  %ds...", i);
    server.handleClient();
    delay(1000);
  }
  tLog("[4/4] START presses...");
  pressStart();
  delay(START_PRESS_GAP_MS);
  pressStart();
  targetHits   = 0;
  maxHits      = 0;
  charging     = true;
  lastMonitor  = millis();
  lastDisplay  = millis();
  tLog("========================================");
  tLogf("Charging started | Start: %.3fV | Target: %.3fV | Confirm: %d consecutive",
        startVoltage, TARGET_VOLTAGE, TARGET_CONFIRM);
  tLog("========================================");
}

void stopCharging() {
  tLog("========================================");
  tLog("STOPPING CHARGE");
  tLog("========================================");
  pressStart();
  delay(1000);
  tLog("SSR OFF");
  digitalWrite(PIN_SSR, LOW);
  sendEspNow("CHARGING_STOP");
  charging   = false;
  targetHits = 0;
  maxHits    = 0;
  float finalVoltage = readVoltage();
  tLog("========================================");
  tLogf("Started : %.3fV", startVoltage);
  tLogf("Finished: %.3fV", finalVoltage);
  tLogf("Target  : %.3fV", TARGET_VOLTAGE);
  tLog("========================================");
}

// ═════════════════════════════════════════════════════════════════════════════
// Setup
// ═════════════════════════════════════════════════════════════════════════════
void setup() {
  Serial.begin(115200);
  // Wait up to 5 s for serial monitor to open
  unsigned long _t = millis();
  while (!Serial && millis() - _t < 5000);
  delay(2000);

  pinMode(PIN_SSR,  OUTPUT); digitalWrite(PIN_SSR,  LOW);
  pinMode(PIN_OPTO, OUTPUT); digitalWrite(PIN_OPTO, LOW);

  // ── INA219 ─────────────────────────────────────────────────────────────
  Wire.begin(21, 22);
  if (!ina219.begin()) {
    Serial.println("FATAL: INA219 not found — check wiring.");
    while (true) delay(500);
  }
  tLog("INA219 OK.");

  // ── WiFi (STA) ─────────────────────────────────────────────────────────
  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PASS);
  Serial.print("Connecting to WiFi");
  unsigned long wStart = millis();
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
    if (millis() - wStart > 15000) {
      Serial.println("\nWiFi timeout — continuing without web log.");
      break;
    }
  }
  if (WiFi.status() == WL_CONNECTED) {
    Serial.println();
    // Print IP directly via Serial (not tLog) — cannot be missed
    Serial.println("=============================");
    Serial.print("   IP: http://");
    Serial.println(WiFi.localIP());
    Serial.println("=============================");
    server.on("/",     handleRoot);
    server.on("/logs", handleLogs);
    server.begin();
    tLog("Web log server started.");
    // Print IP again into the log buffer so browser also shows it
    tLogf("WiFi connected. Open browser: http://%s", WiFi.localIP().toString().c_str());
    // And once more after a short pause so it's not buried
    delay(500);
    Serial.println("=============================");
    Serial.print("   IP: http://");
    Serial.println(WiFi.localIP());
    Serial.println("=============================");
  } else {
    Serial.println();
    Serial.println("WiFi FAILED — check SSID/password.");
    tLog("WiFi FAILED — check SSID/password.");
  }

  // ── ESP-NOW ────────────────────────────────────────────────────────────
  if (esp_now_init() != ESP_OK) {
    tLog("ESP-NOW init failed — continuing without it.");
  } else {
    esp_now_register_send_cb(onDataSent);
    esp_now_peer_info_t peerInfo = {};
    memcpy(peerInfo.peer_addr, droneMacAddress, 6);
    peerInfo.channel = 0;
    peerInfo.encrypt = false;
    if (esp_now_add_peer(&peerInfo) != ESP_OK)
      tLog("ESP-NOW add peer failed.");
  }

  tLog("========================================");
  tLogf("LiPo Charging Controller | 4S 5200mAh | Target: %.1fV", TARGET_VOLTAGE);
  tLog("========================================");

  // ── Wait for battery ───────────────────────────────────────────────────
  tLog("Waiting for battery...");
  while (true) {
    startVoltage = quickRead();
    tLogf("[LIVE] %.3fV", startVoltage);
    server.handleClient();          // keep serving while we wait
    if (startVoltage >= LIPO_CONNECTED) {
      tLog("Battery connected.");
      break;
    }
    delay(2000);
  }

  tLog("Taking averaged reading...");
  startVoltage = readVoltage();
  if (startVoltage == VOLTAGE_SENSOR_FAULT) {
    tLog("FATAL: INA219 returned bad reading at startup.");
    while (true) { server.handleClient(); delay(100); }
  }

  tLogf("Battery voltage: %.3fV", startVoltage);

  if (startVoltage < LIPO_MIN) {
    tLog("ERROR: Voltage too low — over-discharged. Check battery, then reset.");
    return;
  }
  if (startVoltage >= LIPO_MAX) {
    tLog("Battery already fully charged — no action.");
    return;
  }
  if (startVoltage >= TARGET_VOLTAGE) {
    tLog("Already at target voltage — no action.");
    return;
  }

  startCharging();
}

// ═════════════════════════════════════════════════════════════════════════════
// Loop
// ═════════════════════════════════════════════════════════════════════════════
void loop() {
  server.handleClient();          // must be called every loop iteration

  if (!charging) return;

  unsigned long now = millis();

  // ── Live display every DISPLAY_INTERVAL ──────────────────────────────
  if (now - lastDisplay >= DISPLAY_INTERVAL) {
    lastDisplay = now;
    float live = quickRead();
    tLogf("[LIVE] %.3fV | Target: %.3fV | Rise: +%.3fV | tHits: %d/%d | Time: %lum %lus",
          live, TARGET_VOLTAGE, live - startVoltage,
          targetHits, TARGET_CONFIRM,
          (now / 1000) / 60, (now / 1000) % 60);
  }

  // ── Charge control every MONITOR_INTERVAL ────────────────────────────
  if (now - lastMonitor >= MONITOR_INTERVAL) {
    lastMonitor = now;

    float v = readVoltage();

    if (v == VOLTAGE_SENSOR_FAULT) {
      if (sensorFaultCount >= SENSOR_FAULT_LIMIT)
        emergencyStop("INA219 unresponsive — 3 consecutive bad reads");
      return;
    }

    tLog("----------------------------------------");
    tLogf("[AVG] %.3fV | Rise: +%.3fV | tHits: %d/%d",
          v, v - startVoltage, targetHits, TARGET_CONFIRM);

    if (v >= LIPO_MAX) {
      maxHits++;
      tLogf("  SAFETY WARNING: %d/%d", maxHits, MAX_CONFIRM);
      if (maxHits >= MAX_CONFIRM) {
        tLog("SAFETY STOP — max voltage exceeded!");
        targetHits = 0;
        stopCharging();
        return;
      }
    } else {
      maxHits = 0;
    }

    if (v >= TARGET_VOLTAGE) {
      targetHits++;
      tLogf("  At/above target: %d/%d", targetHits, TARGET_CONFIRM);
      if (targetHits >= TARGET_CONFIRM) {
        tLog("Target confirmed — stopping charge.");
        stopCharging();
      }
    } else {
      targetHits = 0;
    }

    tLog("----------------------------------------");
  }
}