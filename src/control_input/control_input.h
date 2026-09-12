#pragma once

#include <FlyskyIBUS.h>
#include <stdint.h>

class ControlInput {
public:
  ControlInput(HardwareSerial &serial, uint8_t receiverPin);

  void begin();

  float desiredPitch();
  float desiredRoll();

  uint16_t elevator();
  uint16_t aileron();
  uint16_t flaps();

  bool stabilizationEnabled();
  bool configModeEnabled();

private:
  static constexpr uint16_t CENTER_US = 1500;
  static constexpr uint16_t DEADBAND_US = 50;
  static constexpr float DEGREES_PER_INPUT_US = 15.0f;
  static constexpr uint16_t SWITCH_THRESHOLD_US = 1500;

  FlyskyIBUS receiver;

  float desiredAngle(uint16_t channelValue) const;
};
