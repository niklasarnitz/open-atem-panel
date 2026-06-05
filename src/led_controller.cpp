#include "led_controller.h"
#include "atem_client.h"
#include <TLC5955.h>
#include <SPI.h>

// Static declarations for TLC5955 library
const uint8_t TLC5955::chip_count = 3;
float TLC5955::max_current_amps = 10;
bool TLC5955::enforce_max_current = false;

uint8_t TLC5955::_dc_data[TLC5955::chip_count][TLC5955::LEDS_PER_CHIP][TLC5955::COLOR_CHANNEL_COUNT];
uint8_t TLC5955::_rgb_order[TLC5955::chip_count][TLC5955::LEDS_PER_CHIP][TLC5955::COLOR_CHANNEL_COUNT];
uint16_t TLC5955::_grayscale_data[TLC5955::chip_count][TLC5955::LEDS_PER_CHIP][TLC5955::COLOR_CHANNEL_COUNT];

namespace {

// TLC5955 Pins (GPIO 33, 34, 36 are reserved for Octal PSRAM on ESP32-S3R8!)
constexpr uint8_t kPinSin = 15;
constexpr uint8_t kPinSclk = 16;
constexpr uint8_t kPinLat = 17;
constexpr uint8_t kPinGsclk = 2;
constexpr uint8_t kPinMiso = 4;
constexpr uint32_t kGsclkFrequencyHz = 20000000;
constexpr uint32_t kSpiFrequencyHz = 25000000;

constexpr uint32_t kLedUpdateIntervalMs = 50;

constexpr uint16_t channelFor(uint8_t chip, uint8_t led, uint8_t colorChannel) {
  return static_cast<uint16_t>(
      chip * TLC5955::LEDS_PER_CHIP * TLC5955::COLOR_CHANNEL_COUNT +
      led * TLC5955::COLOR_CHANNEL_COUNT +
      colorChannel);
}

} // namespace

// Define the global LED mappings
const RgbLedMapping kRgbLeds[] = {
    {LedId::PGM1, "PGM1", channelFor(0, 5, 0), channelFor(0, 5, 1), channelFor(0, 5, 2)},
    {LedId::PGM2, "PGM2", channelFor(0, 0, 0), channelFor(0, 0, 1), channelFor(0, 0, 2)},
    {LedId::PGM3, "PGM3", channelFor(0, 4, 0), channelFor(0, 4, 1), channelFor(0, 4, 2)},
    {LedId::PGM4, "PGM4", channelFor(0, 8, 0), channelFor(0, 8, 1), channelFor(0, 8, 2)},
    {LedId::PGM5, "PGM5", channelFor(0, 12, 0), channelFor(0, 12, 1), channelFor(0, 12, 2)},
    {LedId::PGM6, "PGM6", channelFor(0, 9, 0), channelFor(0, 9, 1), channelFor(0, 9, 2)},
    {LedId::PGM7, "PGM7", channelFor(0, 13, 0), channelFor(0, 13, 1), channelFor(0, 13, 2)},
    {LedId::PGM8, "PGM8", channelFor(0, 14, 0), channelFor(0, 14, 1), channelFor(0, 14, 2)},
    {LedId::PRV1, "Preview 1", channelFor(0, 1, 0), channelFor(0, 1, 1), channelFor(0, 1, 2)},
    {LedId::PRV2, "Preview 2", channelFor(0, 2, 0), channelFor(0, 2, 1), channelFor(0, 2, 2)},
    {LedId::PRV3, "Preview 3", channelFor(0, 6, 0), channelFor(0, 6, 1), channelFor(0, 6, 2)},
    {LedId::PRV4, "Preview 4", channelFor(0, 3, 0), channelFor(0, 3, 1), channelFor(0, 3, 2)},
    {LedId::PRV5, "Preview 5", channelFor(0, 7, 0), channelFor(0, 7, 1), channelFor(0, 7, 2)},
    {LedId::PRV6, "Preview 6", channelFor(0, 11, 0), channelFor(0, 11, 1), channelFor(0, 11, 2)},
    {LedId::PRV7, "Preview 7", channelFor(0, 15, 0), channelFor(0, 15, 1), channelFor(0, 15, 2)},
    {LedId::PRV8, "Preview 8", channelFor(0, 10, 0), channelFor(0, 10, 1), channelFor(0, 10, 2)},
    {LedId::BKGD, "BKGD", channelFor(1, 0, 0), channelFor(1, 0, 1), channelFor(1, 0, 2)},
    {LedId::PRV_SHIFT, "Preview Shift", channelFor(1, 1, 0), channelFor(1, 1, 1), channelFor(1, 1, 2)},
    {LedId::CUT, "CUT", channelFor(1, 2, 0), channelFor(1, 2, 1), channelFor(1, 2, 2)},
    {LedId::KEY1_TIE, "KEY1 TIE", channelFor(1, 4, 0), channelFor(1, 4, 1), channelFor(1, 4, 2)},
    {LedId::PGM_SHIFT, "PGM Shift", channelFor(1, 5, 0), channelFor(1, 5, 1), channelFor(1, 5, 2)},
    {LedId::KEY1_CUT, "KEY1 CUT", channelFor(1, 9, 0), channelFor(1, 9, 1), channelFor(1, 9, 2)},
    {LedId::AUTO, "AUTO", channelFor(1, 11, 0), channelFor(1, 11, 1), channelFor(1, 11, 2)},
    {LedId::DSK1_CUT, "DSK1 CUT", channelFor(2, 0, 0), channelFor(2, 0, 1), channelFor(2, 0, 2)},
    {LedId::DSK2_CUT, "DSK2 CUT", channelFor(2, 1, 0), channelFor(2, 1, 1), channelFor(2, 1, 2)},
    {LedId::DSK2_AUTO, "DSK2 AUTO", channelFor(2, 2, 0), channelFor(2, 2, 1), channelFor(2, 2, 2)},
    {LedId::DSK2_TIE, "DSK2 TIE", channelFor(2, 3, 0), channelFor(2, 3, 1), channelFor(2, 3, 2)},
    {LedId::DSK1_AUTO, "DSK1 AUTO", channelFor(2, 5, 0), channelFor(2, 5, 1), channelFor(2, 5, 2)},
    {LedId::FTB, "FTB", channelFor(2, 6, 0), channelFor(2, 6, 1), channelFor(2, 6, 2)},
    {LedId::DSK1_TIE, "DSK1 TIE", channelFor(2, 7, 0), channelFor(2, 7, 1), channelFor(2, 7, 2)},
};
const size_t kRgbLedCount = sizeof(kRgbLeds) / sizeof(kRgbLeds[0]);

const uint16_t kFaderLedChannels[] = {
    channelFor(1, 8, 2),  // Fader LED 1
    channelFor(1, 8, 0),  // Fader LED 2
    channelFor(1, 8, 1),  // Fader LED 3
    channelFor(1, 12, 2), // Fader LED 4
    channelFor(1, 12, 0), // Fader LED 5
    channelFor(1, 12, 1), // Fader LED 6
    channelFor(1, 13, 2), // Fader LED 7
    channelFor(1, 13, 0), // Fader LED 8
    channelFor(1, 13, 1), // Fader LED 9
    channelFor(1, 14, 2), // Fader LED 10
    channelFor(1, 14, 0), // Fader LED 11
    channelFor(1, 14, 1), // Fader LED 12
    channelFor(1, 10, 2), // Fader LED 13
    channelFor(1, 10, 0), // Fader LED 14
    channelFor(1, 10, 1), // Fader LED 15
    channelFor(1, 15, 2), // Fader LED 16
};
const size_t kFaderLedCount = sizeof(kFaderLedChannels) / sizeof(kFaderLedChannels[0]);

LedController::LedController(TLC5955& tlc, AtemClient& atem)
    : tlc_(tlc), atem_(atem) {}

void LedController::begin(SPIClass* spi) {
  tlc_.init(kPinLat, kPinSin, kPinSclk, kPinGsclk, spi, kPinMiso);
  tlc_.set_sclk_frequency(kSpiFrequencyHz);
  tlc_.set_gsclk_frequency(kGsclkFrequencyHz);

  tlc_.set_all_dc_data(255);
  tlc_.set_max_current(1, 1, 1);
  tlc_.set_function_data(true, true, true, true, true);
  tlc_.set_brightness_current(127, 127, 127);
  tlc_.update_control();

  tlc_.set_all(0);
  tlc_.update();
}

void LedController::setRgbLed(LedId id, uint16_t red, uint16_t green, uint16_t blue) {
  const RgbLedMapping* led = findRgbLed(id);
  if (led == nullptr) return;
  tlc_.set_single_channel(led->redChannel, red);
  tlc_.set_single_channel(led->greenChannel, green);
  tlc_.set_single_channel(led->blueChannel, blue);
}

const RgbLedMapping* LedController::findRgbLed(LedId id) const {
  for (const RgbLedMapping& led : kRgbLeds) {
    if (led.id == id) {
      return &led;
    }
  }
  return nullptr;
}

bool LedController::isLedSupportedByActiveProfile(const RgbLedMapping& led, uint8_t btnIdx, bool pgmShift, bool prvShift) const {
  if (strncmp(led.name, "PGM", 3) == 0 && btnIdx > 0) {
    return atem_.sourceForButton(btnIdx - 1, pgmShift) != kUnknownAtemSource;
  }

  if (strncmp(led.name, "Preview", 7) == 0 && btnIdx > 0) {
    return atem_.sourceForButton(btnIdx - 1, prvShift) != kUnknownAtemSource;
  }

  if (led.id == LedId::DSK1_CUT || led.id == LedId::DSK1_TIE || led.id == LedId::DSK1_AUTO) {
    return atem_.supportsDownstreamKeyer(0);
  }

  if (led.id == LedId::DSK2_CUT || led.id == LedId::DSK2_TIE || led.id == LedId::DSK2_AUTO) {
    return atem_.supportsDownstreamKeyer(1);
  }

  return true;
}

void LedController::update(bool pgmShift, bool prvShift) {
  static uint32_t lastLedUpdateMs = 0;
  const uint32_t now = millis();

  if (now - lastLedUpdateMs < kLedUpdateIntervalMs) return;
  lastLedUpdateMs = now;

  bool blinkState = (now / 250) % 2 == 0;
  const AtemSwitcherState& atem = atem_.state();

  tlc_.set_all(0);

  // Map each RGB LED according to ATEM state
  for (const RgbLedMapping& led : kRgbLeds) {
    uint16_t r = kDimBrightness, g = kDimBrightness, b = kDimBrightness;

    uint8_t btnIdx = 0;
    if (led.name[3] >= '1' && led.name[3] <= '8') {
      btnIdx = led.name[3] - '0';
    } else if (led.name[8] >= '1' && led.name[8] <= '8') {
      btnIdx = led.name[8] - '0';
    }

    if (!isLedSupportedByActiveProfile(led, btnIdx, pgmShift, prvShift)) {
      tlc_.set_single_channel(led.redChannel, 0);
      tlc_.set_single_channel(led.greenChannel, 0);
      tlc_.set_single_channel(led.blueChannel, 0);
      continue;
    }

    // Program inputs row
    if (strncmp(led.name, "PGM", 3) == 0 && btnIdx > 0) {
      const uint16_t normalSrc = atem_.sourceForButton(btnIdx - 1, false);
      const uint16_t shiftedSrc = atem_.sourceForButton(btnIdx - 1, true);
      if (atem.programSource == normalSrc) {
        r = kLedBrightness; g = 0; b = 0; // Solid Red
      } else if (atem.programSource == shiftedSrc) {
        r = blinkState ? kLedBrightness : 0;
        g = 0;
        b = 0;
      }
    }
    // Preview inputs row
    else if (strncmp(led.name, "Preview", 7) == 0 && btnIdx > 0) {
      const uint16_t normalSrc = atem_.sourceForButton(btnIdx - 1, false);
      const uint16_t shiftedSrc = atem_.sourceForButton(btnIdx - 1, true);
      if (atem.previewSource == normalSrc) {
        r = 0; g = kLedBrightness; b = 0; // Solid Green
      } else if (atem.previewSource == shiftedSrc) {
        r = 0;
        g = blinkState ? kLedBrightness : 0;
        b = 0;
      }
    }
    // Shift buttons
    else if (led.id == LedId::PGM_SHIFT) {
      if (pgmShift) {
        r = kLedBrightness; g = kLedBrightness; b = 0; // Shift active
      }
    } else if (led.id == LedId::PRV_SHIFT) {
      if (prvShift) {
        r = kLedBrightness; g = kLedBrightness; b = 0; // Shift active
      }
    }
    // Next Transition
    else if (led.id == LedId::BKGD) {
      if (atem.nextTrBkgd) {
        r = kLedBrightness; g = kLedBrightness; b = 0; // Yellow
      }
    } else if (led.id == LedId::KEY1_TIE) {
      if (atem.nextTrKey1) {
        r = kLedBrightness; g = kLedBrightness; b = 0; // Yellow
      }
    }
    // Upstream Keyer On Air
    else if (led.id == LedId::KEY1_CUT) {
      if (atem.key1OnAir) {
        r = kLedBrightness; g = 0; b = 0; // Red
      }
    }
    // DSK Tie
    else if (led.id == LedId::DSK1_TIE) {
      if (atem.dskTie[0]) {
        r = kLedBrightness; g = kLedBrightness; b = 0; // Yellow
      }
    } else if (led.id == LedId::DSK2_TIE) {
      if (atem.dskTie[1]) {
        r = kLedBrightness; g = kLedBrightness; b = 0; // Yellow
      }
    }
    // DSK Cut / On Air
    else if (led.id == LedId::DSK1_CUT) {
      if (atem.dskOnAir[0]) {
        r = kLedBrightness; g = 0; b = 0; // Red
      }
    } else if (led.id == LedId::DSK2_CUT) {
      if (atem.dskOnAir[1]) {
        r = kLedBrightness; g = 0; b = 0; // Red
      }
    }
    // DSK Auto (Blinks when active)
    else if (led.id == LedId::DSK1_AUTO) {
      if (atem.dskTransitioning[0]) {
        if (blinkState) {
          r = kLedBrightness; g = 0; b = 0; // Blinking Red
        } else {
          r = 0; g = 0; b = 0;
        }
      }
    } else if (led.id == LedId::DSK2_AUTO) {
      if (atem.dskTransitioning[1]) {
        if (blinkState) {
          r = kLedBrightness; g = 0; b = 0; // Blinking Red
        } else {
          r = 0; g = 0; b = 0;
        }
      }
    }
    // Fade to Black
    else if (led.id == LedId::FTB) {
      if (!atem_.connected()) {
        if (blinkState) {
          r = 0; g = 0; b = kLedBrightness; // Blinking Blue
        } else {
          r = 0; g = 0; b = 0;
        }
      } else if (atem.ftbActive) {
        if (blinkState) {
          r = kLedBrightness; g = 0; b = 0; // Blinking Red
        } else {
          r = 0; g = 0; b = 0;
        }
      } else if (atem.ftbDone) {
        r = kLedBrightness; g = 0; b = 0; // Solid Red
      }
    }
    // Auto transition (Blinks when transition is running)
    else if (led.id == LedId::AUTO) {
      if (atem.transitionInProgress) {
        if (blinkState) {
          r = kLedBrightness; g = 0; b = 0;
        } else {
          r = 0; g = 0; b = 0;
        }
      }
    }

    tlc_.set_single_channel(led.redChannel, r);
    tlc_.set_single_channel(led.greenChannel, g);
    tlc_.set_single_channel(led.blueChannel, b);
  }

  updateFaderLeds();
  tlc_.update();
}

void LedController::updateFaderLeds() {
  const AtemSwitcherState& atem = atem_.state();
  
  uint32_t progress = 0;
  if (atem.faderStartPosition == kAtemTransitionPositionMax) {
    if (atem.faderLedPosition <= kAtemTransitionPositionMax) {
      progress = kAtemTransitionPositionMax - atem.faderLedPosition;
    }
  } else {
    progress = atem.faderLedPosition;
  }

  uint8_t litCount = (progress * kFaderLedCount) / kAtemTransitionPositionMax;
  if (litCount < 1) litCount = 1;
  if (litCount > kFaderLedCount) litCount = kFaderLedCount;

  for (size_t i = 0; i < kFaderLedCount; ++i) {
    bool shouldLight = false;
    if (atem.faderStartPosition == kAtemTransitionPositionMax) {
      shouldLight = (i < litCount);
    } else {
      shouldLight = (i >= kFaderLedCount - litCount);
    }
    tlc_.set_single_channel(kFaderLedChannels[i], shouldLight ? kFaderLedBrightness : 0);
  }
}
