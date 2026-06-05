#include "adc_reader.h"
#include <Wire.h>

namespace {
constexpr uint32_t kAdcPollIntervalMs = 10;
}  // namespace

AdcReader::AdcReader(uint8_t i2cAddress) : address_(i2cAddress) {}

void AdcReader::begin() {
  Wire.beginTransmission(address_);
  Wire.write(0x82); // Setup Byte: VDD reference, internal clock, unipolar, no reset
  Wire.write(0x07); // Configuration Byte: SCAN=00 (scan AIN0→AIN3), CS=0011 (AIN3), single-ended
  uint8_t err = Wire.endTransmission();
  if (err == 0) {
    Serial.printf("MAX11612 (4-Channel): Initialized successfully at address 0x%02X (VDD Reference)\r\n", address_);
  } else {
    Serial.printf("MAX11612 (4-Channel): Failed to initialize at address 0x%02X (I2C error %u)\r\n", address_, err);
  }
}

void AdcReader::update() {
  uint32_t now = millis();
  if (now - lastReadMs_ < kAdcPollIntervalMs) {
    return;
  }
  lastReadMs_ = now;

  // Trigger a new conversion scan
  Wire.beginTransmission(address_);
  Wire.write(0x07); // Configuration Byte: SCAN=00 (scan AIN0→AIN3), single-ended
  uint8_t err = Wire.endTransmission();
  if (err != 0) {
    return;
  }

  delayMicroseconds(100);

  uint8_t bytesRead = Wire.requestFrom(address_, (uint8_t)8);
  if (bytesRead == 8) {
    for (int i = 0; i < kAdcChannelCount; ++i) {
      byte msb = Wire.read();
      byte lsb = Wire.read();
      channels_[i] = ((msb & 0x0F) << 8) | lsb;
    }
  } else {
    while (Wire.available() > 0) {
      Wire.read();
    }
  }
}
