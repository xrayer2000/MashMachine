#include "PressButton.h"
PressButton* PressButton::_instance = nullptr;

PressButton::PressButton(uint8_t pin, uint32_t debounceMs):
  _pin(pin),
  _debounceMs(debounceMs),
  _lastReading(HIGH),
  _lastPressMs(0),
  _latched(false),
  _lastIsrMs(0),
  _touchMs(0),
  _isPressed(false)
{
    pinMode(_pin, INPUT_PULLUP);
    // register single instance used by ISR
    _instance = this;
    // Note: Interrupt will be attached in init() method, not here, to avoid
    // race conditions during global variable initialization
}

void PressButton::init() {
    // attach ISR to capture presses even during blocking operations
    // Safe to call from setup() after ESP32 boot stabilizes
    attachInterrupt(digitalPinToInterrupt(_pin), PressButton::isr, FALLING);
}

bool PressButton::Pressed() {
    // Prefer ISR-latched press if available
    if (_latched) {
        _latched = false;
        _lastPressMs = millis();
        return true;
    }

    // Fallback to polling edge detection for compatibility
    bool reading = digitalRead(_pin);
    uint32_t now = millis();

    bool pressed = false;
    if (_lastReading == HIGH && reading == LOW) {
        if (now - _lastPressMs >= _debounceMs) {
            pressed = true;
            _lastPressMs = now;
        }
    }

    _lastReading = reading;
    return pressed;
}

uint32_t PressButton::consumeTouchMs()
{
    uint32_t v = _touchMs;
    _touchMs = 0;
    return v;
}

void IRAM_ATTR PressButton::isr()
{
    // route to the single instance
    if (!_instance) return;
    
    uint32_t now = millis();
    bool pinState = digitalRead(_instance->_pin);
    
    // Debounce check
    if (now - _instance->_lastIsrMs >= _instance->_debounceMs) {
        // Only register a new press if:
        // 1. Button is currently LOW (pressed)
        // 2. It wasn't already marked as pressed (prevents bounces while held)
        if (pinState == LOW && !_instance->_isPressed) {
            _instance->_isPressed = true;  // Mark button as pressed
            _instance->_lastIsrMs = now;
            _instance->_latched = true;
            _instance->_touchMs = now;
        }
        // Mark button as released when pin goes HIGH
        else if (pinState == HIGH && _instance->_isPressed) {
            _instance->_isPressed = false;  // Reset for next press
        }
    }
}