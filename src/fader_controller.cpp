#include "fader_controller.h"

FaderController::FaderController(AdcReader& adc, AtemClient& atem, uint8_t adcChannel)
    : adc_(adc), atem_(atem), adcChannel_(adcChannel) {}

uint16_t FaderController::mapAdcToAtemPosition(uint16_t adcValue) const {
  // ADC 0 = fader at top, ADC 4095 = fader at bottom
  // ATEM CTPs receives the absolute fader-bar position. The returned TrPs
  // transitionPosition is then used as the source of truth for progress LEDs.

  // Apply deadzone at extremes for clean endpoints
  if (adcValue <= kDeadzoneAdc) {
    adcValue = 0;
  } else if (adcValue >= kAdcMax - kDeadzoneAdc) {
    adcValue = kAdcMax;
  }

  // Linear map ADC [0, 4095] → [0, 10000]
  uint16_t mapped;
  if (adcValue == 0) {
    mapped = 0;
  } else if (adcValue == kAdcMax) {
    mapped = kAtemPositionMax;
  } else {
    uint32_t adjusted = adcValue - kDeadzoneAdc;
    uint32_t range = kAdcMax - (2 * kDeadzoneAdc);
    mapped = (uint16_t)((adjusted * kAtemPositionMax) / range);
  }

  return mapped;
}

void FaderController::update() {
  if (!atem_.connected()) {
    hasSentInitial_ = false;
    return;
  }

  uint16_t adcValue = adc_.channel(adcChannel_);
  uint16_t position = mapAdcToAtemPosition(adcValue);

  // On first read after connection, always send to sync state
  if (!hasSentInitial_) {
    hasSentInitial_ = true;
    lastSentPosition_ = position;

    uint8_t payload[4] = {
        0x00,                              // M/E index
        0x00,                              // unused
        (uint8_t)(position >> 8),          // position high byte
        (uint8_t)(position & 0xFF),        // position low byte
    };
    atem_.sendCommand("CTPs", payload, sizeof(payload));
    Serial.printf("FADER: Initial sync -> ATEM absolute position %u (ADC: %u)\r\n",
                  position, adcValue);
    return;
  }

  // Only send if position changed enough to avoid jitter
  int32_t delta = (int32_t)position - (int32_t)lastSentPosition_;
  if (delta < 0) delta = -delta;

  // Always send if we hit the endpoints (important for completing transitions)
  bool atEndpoint = (position == 0 || position == kAtemPositionMax);
  bool lastWasEndpoint = (lastSentPosition_ == 0 || lastSentPosition_ == kAtemPositionMax);

  if (delta >= kMinPositionChange || (atEndpoint && !lastWasEndpoint)) {
    lastSentPosition_ = position;

    uint8_t payload[4] = {
        0x00,
        0x00,
        (uint8_t)(position >> 8),
        (uint8_t)(position & 0xFF),
    };
    atem_.sendCommand("CTPs", payload, sizeof(payload));
  }
}
