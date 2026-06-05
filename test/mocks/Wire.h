#pragma once

#include <stdint.h>

class TwoWire {
public:
  void begin(int sda, int scl) {}
  void setClock(uint32_t frequency) {}
  void beginTransmission(uint8_t address) {}
  uint8_t endTransmission() { return 0; }
  size_t write(uint8_t val) { return 1; }
  uint8_t requestFrom(uint8_t address, uint8_t quantity) { return quantity; }
  int available() { return 0; }
  int read() { return 0; }
};

extern TwoWire Wire;
