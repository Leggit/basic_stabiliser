#include "PID.h"

PID::PID(float kp, float ki, float kd)
    : kp(kp), ki(ki), kd(kd), integral(0.0f), derivative(0),
      prevMeasurement(0.0f), hasPreviousMeasurement(false) {}

void PID::reset() {
  integral = 0.0f;
  derivative = 0.0f;
  prevMeasurement = 0.0f;
  hasPreviousMeasurement = false;
}

void PID::setGains(float kp, float ki, float kd) {
  this->kp = kp;
  this->ki = ki;
  this->kd = kd;
}

float PID::update(float setpoint, float measurement, float dt, bool log) {
  (void)log;

  float error = setpoint - measurement;

  if (!hasPreviousMeasurement) {
    prevMeasurement = measurement;
    hasPreviousMeasurement = true;
  }

  // Reject invalid timing rather than allowing a bad sample to destabilize I/D.
  if (dt > 0.0f && isfinite(dt)) {
    const float rawDerivative = -(measurement - prevMeasurement) / dt;
    derivative = 0.9f * derivative + 0.1f * rawDerivative;

    const float proportionalAndDerivative = (kp * error) + (kd * derivative);
    const float candidateIntegral =
        constrain(integral + error * dt, -INTEGRAL_MAX, INTEGRAL_MAX);
    const float candidateOutput =
        proportionalAndDerivative + (ki * candidateIntegral);

    const bool outputSaturated = fabs(candidateOutput) >= OUTPUT_SATURATION;
    const bool errorDrivesFurther =
        (candidateOutput > OUTPUT_SATURATION && error > 0.0f) ||
        (candidateOutput < -OUTPUT_SATURATION && error < 0.0f);
    if (!outputSaturated || !errorDrivesFurther) {
      integral = candidateIntegral;
    }
  }

  float output = (kp * error) + (ki * integral) + (kd * derivative);
  output = constrain(output, -OUTPUT_SATURATION, OUTPUT_SATURATION);

  prevMeasurement = measurement;

  return output;
}