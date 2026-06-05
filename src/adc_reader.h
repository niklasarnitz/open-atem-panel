#pragma once

#include <Arduino.h>

class AdcReader {
 public:
  AdcReader(uint8_t i2cAddress = 0x34);

  void begin();
  void update();

 private:
  uint8_t address_;
  uint32_t lastReadMs_ = 0;
};
