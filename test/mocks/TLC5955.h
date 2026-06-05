#pragma once

#include <stdint.h>
#include <string.h>

class TLC5955 {
public:
  static const uint8_t chip_count;
  static const uint8_t LEDS_PER_CHIP = 16;
  static const uint8_t COLOR_CHANNEL_COUNT = 3;

  static float max_current_amps;
  static bool enforce_max_current;
  static uint8_t _dc_data[3][16][3];
  static uint8_t _rgb_order[3][16][3];
  static uint16_t _grayscale_data[3][16][3];

  void init(uint8_t lat, uint8_t sin, uint8_t sclk, uint8_t gsclk, void* spi, uint8_t miso) {}
  void set_sclk_frequency(uint32_t freq) {}
  void set_gsclk_frequency(uint32_t freq) {}
  void set_all_dc_data(uint8_t val) {}
  void set_max_current(uint8_t r, uint8_t g, uint8_t b) {}
  void set_function_data(bool a, bool b, bool c, bool d, bool e) {}
  void set_brightness_current(uint8_t r, uint8_t g, uint8_t b) {}
  void update_control() {}

  void set_all(uint16_t value) {
    for (int c = 0; c < 3; ++c) {
      for (int l = 0; l < 16; ++l) {
        for (int ch = 0; ch < 3; ++ch) {
          _grayscale_data[c][l][ch] = value;
        }
      }
    }
  }

  void update() {}

  void set_single_channel(uint16_t channel, uint16_t value) {
    if (channel < 144) {
      uint8_t chip = channel / 48;
      uint8_t led = (channel % 48) / 3;
      uint8_t color = channel % 3;
      _grayscale_data[chip][led][color] = value;
    }
  }
};
