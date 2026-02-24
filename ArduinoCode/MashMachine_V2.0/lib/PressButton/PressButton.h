#pragma once
#include <Arduino.h>

class PressButton {
public:
    PressButton(uint8_t pin, uint32_t debounceMs);
    void init();  // Call this from setup() to safely attach interrupt
    bool Pressed();
    // consume and return the timestamp (millis) of the last touch/press captured by ISR
    uint32_t consumeTouchMs();

private:
    uint8_t  _pin;
    uint32_t _debounceMs;
    uint32_t _lastPressMs;
    bool     _lastReading;
    // ISR-driven state
    volatile bool _latched;
    volatile uint32_t _lastIsrMs;
    volatile uint32_t _touchMs;
    volatile bool _isPressed;  // Track current button state to ignore bounces while held

    static PressButton* _instance; // single-instance helper for ISR
    static void IRAM_ATTR isr();
};

