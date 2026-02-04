#pragma once
#include <Arduino.h>

class PressButton {
public:
    PressButton(uint8_t pin, uint32_t debounceMs);
    bool Pressed();

private:
    uint8_t  _pin;
    uint32_t _debounceMs;
    uint32_t _lastPressMs;
    bool     _lastReading;
};

