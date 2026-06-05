#pragma once

#include <Arduino.h>

constexpr uint8_t kAdcChannelCount = 4;

class AdcReader {
 public:
  AdcReader(uint8_t i2cAddress = 0x34);

  void begin();
  void update();

  uint16_t channel(uint8_t ch) const {
    return ch < kAdcChannelCount ? channels_[ch] : 0;
  }

 private:
  uint8_t address_;
  uint16_t channels_[kAdcChannelCount] = {};
  uint32_t lastReadMs_ = 0;
};
