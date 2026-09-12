#pragma once

#include <stdint.h>

class StatusIndicator {
public:
  virtual ~StatusIndicator() = default;

  virtual void begin() = 0;
  virtual void showSetupInProgress() = 0;
  virtual void showSetupComplete() = 0;
  [[noreturn]] virtual void showError() = 0;
};

class SingleLED final : public StatusIndicator {
public:
  explicit SingleLED(uint8_t pin);

  void begin() override;
  void showSetupInProgress() override;
  void showSetupComplete() override;
  [[noreturn]] void showError() override;

private:
  uint8_t pin;
};
