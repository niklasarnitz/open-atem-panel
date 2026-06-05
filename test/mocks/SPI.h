#pragma once

#include <stdint.h>

#define HSPI 1

class SPIClass {
public:
  SPIClass(int bus) {}
  void begin(int sclk, int miso, int mosi, int cs) {}
};

extern SPIClass SPI;
