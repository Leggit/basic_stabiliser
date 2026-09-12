#pragma once

#include <stdint.h>

struct AirframeInputs {
  uint16_t elevator;
  uint16_t aileron;
  uint16_t flaps;
};

struct StabilizationOutput {
  float pitch;
  float roll;
};

class Airframe {
public:
  virtual ~Airframe() = default;

  virtual void begin() = 0;
  virtual void writeManual(const AirframeInputs &inputs) = 0;
  virtual void writeStabilized(const StabilizationOutput &output) = 0;
};

class StandardAirframe final : public Airframe {
public:
  ~StandardAirframe() override;

  void begin() override;
  void writeManual(const AirframeInputs &inputs) override;
  void writeStabilized(const StabilizationOutput &output) override;

private:
  struct ServoOutput;
  ServoOutput *outputs = nullptr;
};

class ElevonAirframe final : public Airframe {
public:
  ElevonAirframe(int8_t leftPitchSign, int8_t leftRollSign,
                 int8_t rightPitchSign, int8_t rightRollSign);
  ~ElevonAirframe() override;

  void begin() override;
  void writeManual(const AirframeInputs &inputs) override;
  void writeStabilized(const StabilizationOutput &output) override;

private:
  int8_t leftPitchSign;
  int8_t leftRollSign;
  int8_t rightPitchSign;
  int8_t rightRollSign;

  struct ServoOutput;
  ServoOutput *outputs = nullptr;
};
