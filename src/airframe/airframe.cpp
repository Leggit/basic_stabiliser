#include "airframe.h"

#include "../config.h"
#include <Arduino.h>
#include <ESP32Servo.h>

namespace {
constexpr int MIN_SERVO_US = 1000;
constexpr int MAX_SERVO_US = 2000;
constexpr int CENTER_SERVO_US = 1500;
constexpr float PID_TO_SERVO_SCALE = -5.0f;

int servoMicroseconds(float pidOutput) {
  return constrain(
      static_cast<int>(CENTER_SERVO_US + pidOutput * PID_TO_SERVO_SCALE),
      MIN_SERVO_US, MAX_SERVO_US);
}

int manualElevonMicroseconds(uint16_t elevator, uint16_t aileron,
                             int8_t pitchSign, int8_t rollSign) {
  const int pitchInput =
      static_cast<int>(constrain(elevator, MIN_SERVO_US, MAX_SERVO_US)) -
      CENTER_SERVO_US;
  const int rollInput =
      static_cast<int>(constrain(aileron, MIN_SERVO_US, MAX_SERVO_US)) -
      CENTER_SERVO_US;

  return constrain(CENTER_SERVO_US + pitchInput * pitchSign +
                       rollInput * rollSign,
                   MIN_SERVO_US, MAX_SERVO_US);
}

int stabilizedElevonMicroseconds(float pitch, float roll, int8_t pitchSign,
                                 int8_t rollSign) {
  return constrain(CENTER_SERVO_US + (pitch * pitchSign + roll * rollSign) *
                                         PID_TO_SERVO_SCALE,
                   MIN_SERVO_US, MAX_SERVO_US);
}

void configureServo(Servo &servo, uint8_t pin) {
  servo.setPeriodHertz(50);
  servo.attach(pin, MIN_SERVO_US, MAX_SERVO_US);
}

void allocateServoTimers() {
  ESP32PWM::allocateTimer(0);
  ESP32PWM::allocateTimer(1);
  ESP32PWM::allocateTimer(2);
  ESP32PWM::allocateTimer(3);
}
} // namespace

struct StandardAirframe::ServoOutput {
  Servo elevator;
  Servo leftAileron;
  Servo rightAileron;
};

StandardAirframe::~StandardAirframe() { delete outputs; }

void StandardAirframe::begin() {
  allocateServoTimers();
  outputs = new ServoOutput;
  configureServo(outputs->elevator, ELEVATOR_SERVO_PIN);
  configureServo(outputs->leftAileron, AILERON_1_SERVO_PIN);
  configureServo(outputs->rightAileron, AILERON_2_SERVO_PIN);
}

void StandardAirframe::writeManual(const AirframeInputs &inputs) {
  const uint16_t aileron =
      constrain(inputs.aileron, MIN_SERVO_US, MAX_SERVO_US);
  int flaperonOffset = 0;
  if (inputs.flaps > FLAPS_FULL_US) {
    flaperonOffset = FLAPS_FULL_OFFSET_US;
  } else if (inputs.flaps > FLAPS_HALF_US) {
    flaperonOffset = FLAPS_HALF_OFFSET_US;
  }

  outputs->elevator.writeMicroseconds(
      constrain(inputs.elevator, MIN_SERVO_US, MAX_SERVO_US));
  outputs->leftAileron.writeMicroseconds(
      constrain(aileron + flaperonOffset, MIN_SERVO_US, MAX_SERVO_US));
  outputs->rightAileron.writeMicroseconds(
      constrain(aileron - flaperonOffset, MIN_SERVO_US, MAX_SERVO_US));
}

void StandardAirframe::writeStabilized(const StabilizationOutput &output) {
  outputs->elevator.writeMicroseconds(servoMicroseconds(output.pitch));
  const int rollCommand = servoMicroseconds(output.roll * -1.0f);
  outputs->leftAileron.writeMicroseconds(rollCommand);
  outputs->rightAileron.writeMicroseconds(rollCommand);
}

ElevonAirframe::ElevonAirframe(int8_t leftPitchSign, int8_t leftRollSign,
                               int8_t rightPitchSign, int8_t rightRollSign)
    : leftPitchSign(leftPitchSign), leftRollSign(leftRollSign),
      rightPitchSign(rightPitchSign), rightRollSign(rightRollSign) {}

struct ElevonAirframe::ServoOutput {
  Servo left;
  Servo right;
};

ElevonAirframe::~ElevonAirframe() { delete outputs; }

void ElevonAirframe::begin() {
  allocateServoTimers();
  outputs = new ServoOutput;
  configureServo(outputs->left, AILERON_1_SERVO_PIN);
  configureServo(outputs->right, AILERON_2_SERVO_PIN);
}

void ElevonAirframe::writeManual(const AirframeInputs &inputs) {
  outputs->left.writeMicroseconds(manualElevonMicroseconds(
      inputs.elevator, inputs.aileron, leftPitchSign, leftRollSign));
  outputs->right.writeMicroseconds(manualElevonMicroseconds(
      inputs.elevator, inputs.aileron, rightPitchSign, rightRollSign));
}

void ElevonAirframe::writeStabilized(const StabilizationOutput &output) {
  outputs->left.writeMicroseconds(stabilizedElevonMicroseconds(
      output.pitch, output.roll, leftPitchSign, leftRollSign));
  outputs->right.writeMicroseconds(stabilizedElevonMicroseconds(
      output.pitch, output.roll, rightPitchSign, rightRollSign));
}
