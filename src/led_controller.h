#pragma once

#include <Arduino.h>
#include "atem_types.h"

// Forward declarations
class TLC5955;
class AtemClient;
class SPIClass;

// LED Brightness values
constexpr uint16_t kLedBrightness = 0xFFFF;
constexpr uint16_t kDimBrightness = 0x0800;
constexpr uint16_t kFaderLedBrightness = 0x1200;

enum class LedId : uint8_t {
  PGM1, PGM2, PGM3, PGM4, PGM5, PGM6, PGM7, PGM8,
  PRV1, PRV2, PRV3, PRV4, PRV5, PRV6, PRV7, PRV8,
  BKGD, PRV_SHIFT, CUT, KEY1_TIE, PGM_SHIFT, KEY1_CUT, AUTO,
  DSK1_CUT, DSK2_CUT, DSK2_AUTO, DSK2_TIE, DSK1_AUTO, FTB, DSK1_TIE,
  NONE
};

struct RgbLedMapping {
  LedId id;
  const char* name;
  uint16_t redChannel;
  uint16_t greenChannel;
  uint16_t blueChannel;
};

extern const RgbLedMapping kRgbLeds[];
extern const size_t kRgbLedCount;

extern const uint16_t kFaderLedChannels[];
extern const size_t kFaderLedCount;

class LedController {
 public:
  LedController(TLC5955& tlc, AtemClient& atem);

  void begin(SPIClass* spi);
  void update(bool pgmShift, bool prvShift);
  void setRgbLed(LedId id, uint16_t red, uint16_t green, uint16_t blue);
  const RgbLedMapping* findRgbLed(LedId id) const;
  bool isLedSupportedByActiveProfile(const RgbLedMapping& led, uint8_t btnIdx, bool pgmShift, bool prvShift) const;

 private:
  void updateFaderLeds();

  TLC5955& tlc_;
  AtemClient& atem_;
};
