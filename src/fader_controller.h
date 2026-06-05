#pragma once

#include <Arduino.h>
#include "adc_reader.h"
#include "atem_client.h"

class FaderController {
 public:
  FaderController(AdcReader& adc, AtemClient& atem, uint8_t adcChannel = 0);

  void update();

 private:
  static constexpr uint16_t kAdcMax = 4095;
  static constexpr uint16_t kAtemPositionMax = 10000;

  // Deadzone at both extremes of fader travel to ensure clean 0 and 10000
  static constexpr uint16_t kDeadzoneAdc = 20;

  // Minimum ATEM position change before sending a new command (prevents jitter)
  static constexpr uint16_t kMinPositionChange = 30;

  uint16_t mapAdcToAtemPosition(uint16_t adcValue) const;

  AdcReader& adc_;
  AtemClient& atem_;
  uint8_t adcChannel_;
  uint16_t lastSentPosition_ = 0;
  bool hasSentInitial_ = false;
};
