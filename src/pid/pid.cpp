#include "PID.h"

PID::PID(float kp, float ki, float kd)
    : kp(kp), ki(ki), kd(kd), integral(0.0f), prevMeasurement(0.0f) {}

void PID::reset() {
  integral = 0.0f;
  prevMeasurement = 0.0f;
}

void PID::setGains(float kp, float ki, float kd) {
  this->kp = kp;
  this->ki = ki;
  this->kd = kd;
}

float PID::update(float setpoint, float measurement, float dt, bool log) {
  float error = setpoint - measurement;

  // --- Integral ---
  if (fabs(error) < 20.0f) {
    integral += error;
    integral = constrain(integral, -INTEGRAL_MAX, INTEGRAL_MAX);
  }

  // --- Derivative (on measurement) ---
  float rawDerivative = -(measurement - prevMeasurement);

  // Optional: filter it (HIGHLY recommended)
  derivative = 0.9f * derivative + 0.1f * rawDerivative;

  // --- PID output ---
  float output = (kp * error) + (ki * integral) + (kd * derivative);

  prevMeasurement = measurement;

  return output;
}