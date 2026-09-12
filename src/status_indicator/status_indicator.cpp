#include "status_indicator.h"

#include <Arduino.h>

SingleLED::SingleLED(uint8_t pin) : pin(pin) {}

void SingleLED::begin() { pinMode(pin, OUTPUT); }

void SingleLED::showSetupInProgress() {
  digitalWrite(pin, HIGH);
  delay(1000);
  digitalWrite(pin, LOW);
  delay(1000);
}

void SingleLED::showSetupComplete() { digitalWrite(pin, HIGH); }

[[noreturn]] void SingleLED::showError() {
  while (true) {
    digitalWrite(pin, HIGH);
    delay(100);
    digitalWrite(pin, LOW);
    delay(100);
  }
}
