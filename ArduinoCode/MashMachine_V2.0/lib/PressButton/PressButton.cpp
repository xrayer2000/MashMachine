#include "PressButton.h"

PressButton::PressButton(uint8_t pin, uint32_t debounceMs):
  _pin(pin),
  _debounceMs(debounceMs),
  _lastReading(HIGH),
  _lastPressMs(0)
{
    pinMode(_pin, INPUT_PULLUP);
}

bool PressButton::Pressed() {
    bool reading = digitalRead(_pin);
    uint32_t now = millis();

    bool pressed = false;

    // FALLING EDGE
    if (_lastReading == HIGH && reading == LOW) {
        if (now - _lastPressMs >= _debounceMs) {
            pressed = true;
            _lastPressMs = now;
        }
    }

    _lastReading = reading;
    return pressed;
}