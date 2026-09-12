#pragma once

#include <Arduino.h>

class PID {
public:
  PID(float kp, float ki, float kd);
  void reset();
  float update(float setpoint, float measurement, float dt, bool log = false);
  void setGains(float kp, float ki, float kd);

private:
  float kp, ki, kd;
  float integral;
  float derivative;
  float prevMeasurement;
  bool hasPreviousMeasurement;

  static constexpr float INTEGRAL_MAX = 25.0f;
  static constexpr float ERROR_THRESHOLD = 15.0f;
  static constexpr float OUTPUT_SATURATION = 100.0f;
};