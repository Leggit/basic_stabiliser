#pragma once

#include <Arduino.h>

class PID {
public:
  PID(float kp, float ki, float kd);
  void reset();
  float update(float setpoint, float measurement, bool log = false);
  void setGains(float kp, float ki, float kd);

private:
  float kp, ki, kd;
  float integral;
  float prevError;
};