#include "web_control_panel.h"
#include <WiFi.h>

WebControlPanel::WebControlPanel()
    : server(nullptr), pitchPID(nullptr), rollPID(nullptr), wifiEnabled(false),
      kp_pitch(1), ki_pitch(0), kd_pitch(0), kp_roll(1), ki_roll(0),
      kd_roll(0) {}

WebControlPanel::~WebControlPanel() {
  if (server) {
    delete server;
  }
}

void WebControlPanel::init(PID *pitch, PID *roll) {
  pitchPID = pitch;
  rollPID = roll;
  prefs.begin("pid", false);
  server = new AsyncWebServer(80);
  setupWebHandlers();
}

void WebControlPanel::loadPIDGains(float default_kp_pitch,
                                   float default_ki_pitch,
                                   float default_kd_pitch,
                                   float default_kp_roll, float default_ki_roll,
                                   float default_kd_roll) {
  kp_pitch = prefs.getFloat("kp_pitch", default_kp_pitch);
  ki_pitch = prefs.getFloat("ki_pitch", default_ki_pitch);
  kd_pitch = prefs.getFloat("kd_pitch", default_kd_pitch);
  kp_roll = prefs.getFloat("kp_roll", default_kp_roll);
  ki_roll = prefs.getFloat("ki_roll", default_ki_roll);
  kd_roll = prefs.getFloat("kd_roll", default_kd_roll);

  updatePIDGains();
}

void WebControlPanel::update(uint16_t configModeSwitch) {
  bool configModeEnabled = configModeSwitch > 1500;

  if (configModeEnabled && !wifiEnabled) {
    enableWiFi();
  } else if (!configModeEnabled && wifiEnabled) {
    disableWiFi();
  }
}

void WebControlPanel::setupWebHandlers() {
  server->on("/", HTTP_GET, [this](AsyncWebServerRequest *request) {
    request->send(200, "text/html", buildWebPage());
  });

  server->on("/update", HTTP_POST, [this](AsyncWebServerRequest *request) {
    if (request->hasParam("kp_pitch", true))
      kp_pitch = request->getParam("kp_pitch", true)->value().toFloat();
    if (request->hasParam("ki_pitch", true))
      ki_pitch = request->getParam("ki_pitch", true)->value().toFloat();
    if (request->hasParam("kd_pitch", true))
      kd_pitch = request->getParam("kd_pitch", true)->value().toFloat();
    if (request->hasParam("kp_roll", true))
      kp_roll = request->getParam("kp_roll", true)->value().toFloat();
    if (request->hasParam("ki_roll", true))
      ki_roll = request->getParam("ki_roll", true)->value().toFloat();
    if (request->hasParam("kd_roll", true))
      kd_roll = request->getParam("kd_roll", true)->value().toFloat();

    updatePIDGains();
    savePIDGains();

    request->redirect("/");
  });

  server->on("/api/gains", HTTP_GET, [this](AsyncWebServerRequest *request) {
    String json = "{\"kp_pitch\":" + String(kp_pitch, 4) +
                  ",\"ki_pitch\":" + String(ki_pitch, 4) +
                  ",\"kd_pitch\":" + String(kd_pitch, 4) +
                  ",\"kp_roll\":" + String(kp_roll, 4) +
                  ",\"ki_roll\":" + String(ki_roll, 4) +
                  ",\"kd_roll\":" + String(kd_roll, 4) + "}";
    request->send(200, "application/json", json);
  });
}

void WebControlPanel::enableWiFi() {
  WiFi.mode(WIFI_AP);
  WiFi.softAP("ESP32-Stabilizer", "password123");
  server->begin();
  wifiEnabled = true;
  Serial.println("WiFi AP started: ESP32-Stabilizer, password: password123");
  Serial.print("IP: ");
  Serial.println(WiFi.softAPIP());
}

void WebControlPanel::disableWiFi() {
  server->end();
  WiFi.mode(WIFI_OFF);
  wifiEnabled = false;
  Serial.println("WiFi disabled");
}

void WebControlPanel::updatePIDGains() {
  if (pitchPID)
    pitchPID->setGains(kp_pitch, ki_pitch, kd_pitch);
  if (rollPID)
    rollPID->setGains(kp_roll, ki_roll, kd_roll);
}

void WebControlPanel::savePIDGains() {
  prefs.putFloat("kp_pitch", kp_pitch);
  prefs.putFloat("ki_pitch", ki_pitch);
  prefs.putFloat("kd_pitch", kd_pitch);
  prefs.putFloat("kp_roll", kp_roll);
  prefs.putFloat("ki_roll", ki_roll);
  prefs.putFloat("kd_roll", kd_roll);
}

String WebControlPanel::buildWebPage() {
  String html = "<html><head><style>";
  html += "body { font-family: Arial; margin: 20px; background: #f5f5f5; }";
  html += "h1 { color: #333; }";
  html += "input { padding: 5px; margin: 5px 0; width: 150px; }";
  html += ".section { margin: 20px 0; padding: 10px; border: 1px solid #ddd; "
          "background: white; }";
  html += "button { padding: 10px 20px; background: #4CAF50; color: white; "
          "border: none; cursor: pointer; }";
  html += "button:hover { background: #45a049; }";
  html += "</style></head><body>";
  html += "<h1>Stabilizer Control Panel</h1>";
  html += "<form action='/update' method='POST'>";

  html += buildPIDTuningSection();

  html += "<button type='submit'>Update</button>";
  html += "</form></body></html>";

  return html;
}

String WebControlPanel::buildPIDTuningSection() {
  String html = "<div class='section'><h2>PID Tuning</h2>";

  html += "<div class='section'><h3>Pitch PID</h3>";
  html += "Kp: <input type='number' step='0.01' name='kp_pitch' value='" +
          String(kp_pitch, 2) + "'><br>";
  html += "Ki: <input type='number' step='0.01' name='ki_pitch' value='" +
          String(ki_pitch, 2) + "'><br>";
  html += "Kd: <input type='number' step='0.01' name='kd_pitch' value='" +
          String(kd_pitch, 2) + "'><br>";
  html += "</div>";

  html += "<div class='section'><h3>Roll PID</h3>";
  html += "Kp: <input type='number' step='0.01' name='kp_roll' value='" +
          String(kp_roll, 2) + "'><br>";
  html += "Ki: <input type='number' step='0.01' name='ki_roll' value='" +
          String(ki_roll, 2) + "'><br>";
  html += "Kd: <input type='number' step='0.01' name='kd_roll' value='" +
          String(kd_roll, 2) + "'><br>";
  html += "</div>";

  html += "</div>";

  return html;
}
