#pragma once

#include <stdint.h>

class MCP23017 {
public:
  MCP23017() : address_(0) {
    for (int i = 0; i < 16; ++i) {
      pinStates_[i] = 1; // High (not pressed / pulled up)
    }
  }
  MCP23017(uint8_t address) : address_(address) {
    for (int i = 0; i < 16; ++i) {
      pinStates_[i] = 1; // High
    }
  }
  bool begin(bool dummy = false) { return true; }
  void pinMode1(uint8_t pin, uint8_t mode) {}
  void setPullup(uint8_t pin, bool pullup) {}
  void write1(uint8_t pin, uint8_t val) {
    if (pin < 16) pinStates_[pin] = val;
  }
  uint8_t read1(uint8_t pin) {
    if (pin < 16) return pinStates_[pin];
    return 1;
  }
  void reverse16ByteOrder(bool dummy) {}

  // Helper for tests to simulate inputs
  void set_mock_pin_state(uint8_t pin, uint8_t val) {
    if (pin < 16) pinStates_[pin] = val;
  }

private:
  uint8_t address_;
  uint8_t pinStates_[16];
};
