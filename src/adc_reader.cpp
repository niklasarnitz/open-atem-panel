#include "adc_reader.h"
#include <Wire.h>

AdcReader::AdcReader(uint8_t i2cAddress) : address_(i2cAddress) {}

void AdcReader::begin() {
  Wire.beginTransmission(address_);
  Wire.write(0x82); // Setup Byte: internal clock, unipolar, VDD reference, no reset
  Wire.write(0x37); // Configuration Byte: scan AIN0 to AIN11, single-ended
  uint8_t err = Wire.endTransmission();
  if (err == 0) {
    Serial.printf("MAX11612: Initialized successfully at address 0x%02X\r\n", address_);
  } else {
    Serial.printf("MAX11612: Failed to initialize at address 0x%02X (I2C error %u)\r\n", address_, err);
  }
}

void AdcReader::update() {
  uint32_t now = millis();
  if (now - lastReadMs_ < 1000) {
    return;
  }
  lastReadMs_ = now;

  uint8_t bytesRead = Wire.requestFrom(address_, (uint8_t)24);
  if (bytesRead == 24) {
    Serial.print("MAX11612 ADC Values: ");
    for (int i = 0; i < 12; ++i) {
      byte msb = Wire.read();
      byte lsb = Wire.read();
      uint16_t value = ((msb & 0x0F) << 8) | lsb;
      Serial.printf("CH%d: %4u", i, value);
      if (i < 11) {
        Serial.print(" | ");
      }
    }
    Serial.println();
  } else {
    // Empty the buffer in case of partial read
    while (Wire.available() > 0) {
      Wire.read();
    }
    Serial.printf("MAX11612: Failed to read from ADC (read %u bytes)\r\n", bytesRead);
  }
}
