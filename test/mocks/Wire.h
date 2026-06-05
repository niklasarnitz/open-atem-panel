#pragma once

#include <stdint.h>

class TwoWire {
public:
  void begin(int sda, int scl) {}
  void setClock(uint32_t frequency) {}
  void beginTransmission(uint8_t address) {}
  uint8_t endTransmission() { return 0; }
};

extern TwoWire Wire;
