#include "control_input.h"

#include "../config.h"
#include <Arduino.h>

ControlInput::ControlInput(HardwareSerial &serial, uint8_t receiverPin)
    : receiver(serial, receiverPin) {}

void ControlInput::begin() {
  receiver.begin();
  Serial.println("Receiver initialized");
}

float ControlInput::desiredPitch() { return desiredAngle(elevator()); }

float ControlInput::desiredRoll() { return desiredAngle(aileron()) * -1.0f; }

uint16_t ControlInput::elevator() {
  return receiver.getChannel(ELEVATOR_CHANNEL);
}

uint16_t ControlInput::aileron() {
  return receiver.getChannel(AILERON_CHANNEL);
}

uint16_t ControlInput::flaps() { return receiver.getChannel(FLAPS_CHANNEL); }

bool ControlInput::stabilizationEnabled() {
  return receiver.getChannel(SWITCH_STABILIZATION) > SWITCH_THRESHOLD_US;
}

bool ControlInput::configModeEnabled() {
  return receiver.getChannel(SWITCH_CONFIG_MODE) > SWITCH_THRESHOLD_US;
}

float ControlInput::desiredAngle(uint16_t channelValue) const {
  const uint16_t lowerDeadband = CENTER_US - DEADBAND_US;
  const uint16_t upperDeadband = CENTER_US + DEADBAND_US;

  if (channelValue > lowerDeadband && channelValue < upperDeadband) {
    return 0.0f;
  }

  return (static_cast<float>(channelValue) - CENTER_US) / DEGREES_PER_INPUT_US *
         -1.0f;
}
