#pragma once

#include "pid/pid.h"
#include <ESPAsyncWebServer.h>
#include <Preferences.h>

class WebControlPanel {
public:
  WebControlPanel();
  ~WebControlPanel();

  void init(PID *pitchPID, PID *rollPID);
  void update(uint16_t switchValue);

  float getKpPitch() const { return kp_pitch; }
  float getKiPitch() const { return ki_pitch; }
  float getKdPitch() const { return kd_pitch; }
  float getKpRoll() const { return kp_roll; }
  float getKiRoll() const { return ki_roll; }
  float getKdRoll() const { return kd_roll; }

  void loadPIDGains(float default_kp_pitch, float default_ki_pitch,
                    float default_kd_pitch, float default_kp_roll,
                    float default_ki_roll, float default_kd_roll);

private:
  AsyncWebServer *server;
  Preferences prefs;

  PID *pitchPID;
  PID *rollPID;

  float kp_pitch, ki_pitch, kd_pitch;
  float kp_roll, ki_roll, kd_roll;

  bool wifiEnabled;

  void setupWebHandlers();
  void enableWiFi();
  void disableWiFi();
  void updatePIDGains();
  void savePIDGains();
  String buildWebPage();
  String buildPIDTuningSection();
};
