#include "PID.h"

PID::PID(float kp, float ki, float kd)
    : kp(kp), ki(ki), kd(kd), integral(0.0f), prevError(0.0f) {}

void PID::reset() {
  integral = 0.0f;
  prevError = 0.0f;
}

void PID::setGains(float kp, float ki, float kd) {
  this->kp = kp;
  this->ki = ki;
  this->kd = kd;
}

float PID::update(float setpoint, float measurement, bool log) {
  float error = setpoint - measurement;

  if (error > -20 && error < 20) {
    integral = integral + error;
  } else {
    integral = 0;
  }

  // --- Derivative ---
  float derivative = error - prevError;

  if (log) {
    Serial.print("P:");
    Serial.print(kp * error);
    Serial.print(",I:");
    Serial.print(ki * integral);
    Serial.print(",D:");
    Serial.print(kd * derivative);
    Serial.print(",Total:");
    Serial.println((kp * error) + (ki * integral) + (kd * derivative));
  }
  // --- PID output ---
  float output = (kp * error) + (ki * integral) + (kd * derivative);

  prevError = error;

  return output;
}