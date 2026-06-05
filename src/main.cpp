#include <Arduino.h>
#include <SPI.h>
#include <Ethernet.h>
#include <Wire.h>
#include <MCP23017.h>
#include <TLC5955.h>
#include <utility/w5100.h>

#include "atem_client.h"

// Static declarations for TLC5955 library
const uint8_t TLC5955::chip_count = 3;
float TLC5955::max_current_amps = 10;
bool TLC5955::enforce_max_current = false;

uint8_t TLC5955::_dc_data[TLC5955::chip_count][TLC5955::LEDS_PER_CHIP][TLC5955::COLOR_CHANNEL_COUNT];
uint8_t TLC5955::_rgb_order[TLC5955::chip_count][TLC5955::LEDS_PER_CHIP][TLC5955::COLOR_CHANNEL_COUNT];
uint16_t TLC5955::_grayscale_data[TLC5955::chip_count][TLC5955::LEDS_PER_CHIP][TLC5955::COLOR_CHANNEL_COUNT];

namespace {

// Logging and execution rates
constexpr uint32_t kBaudRate = 115200;
constexpr uint32_t kDebounceMs = 20;
constexpr uint32_t kScanIntervalMs = 2;
constexpr uint32_t kLedUpdateIntervalMs = 50;
constexpr uint32_t kAtemTaskDelayMs = 1;
constexpr uint32_t kEthernetLinkPollMs = 500;
constexpr uint32_t kAtemTaskStackWords = 6144;
constexpr UBaseType_t kAtemTaskPriority = 2;

// Network configuration parameters
const IPAddress kLocalIp(10, 0, 0, 142);
const IPAddress kGateway(10, 0, 0, 254);
const IPAddress kSubnet(255, 255, 255, 0);

const IPAddress kSwitcherIp(10, 0, 0, 146);

constexpr uint16_t kSwitcherPort = 9910;
constexpr uint16_t kLocalPort = 50991;

AtemClient gAtem(kSwitcherIp, kSwitcherPort, kLocalPort);
TaskHandle_t gAtemTaskHandle = nullptr;
EthernetLinkStatus gLastEthernetLinkStatus = Unknown;
uint32_t gLastEthernetLinkPollMs = 0;

// SPI pins for Waveshare ESP32-S3-ETH (W5500 Ethernet)
constexpr uint8_t kEthMosiPin = 11;
constexpr uint8_t kEthMisoPin = 12;
constexpr uint8_t kEthSclkPin = 13;
constexpr uint8_t kEthCsPin = 14;
constexpr uint8_t kEthRstPin = 9;

// I2C pins for MCP23017 keymatrix scanner
#ifndef KEYMATRIX_MCP_SDA_PIN
#define KEYMATRIX_MCP_SDA_PIN 48
#endif
#ifndef KEYMATRIX_MCP_SCL_PIN
#define KEYMATRIX_MCP_SCL_PIN 47
#endif
constexpr uint8_t kI2cSdaPin = KEYMATRIX_MCP_SDA_PIN;
constexpr uint8_t kI2cSclPin = KEYMATRIX_MCP_SCL_PIN;
constexpr uint32_t kI2cFrequencyHz = 400000;
constexpr uint8_t kMcpAddress = 0x20;

// MCP23017 rows & columns configuration
constexpr uint8_t kMcpRowPins[] = {12, 13, 14, 15};
constexpr uint8_t kMcpColPins[] = {0, 1, 2, 3, 4, 5, 6, 8, 9};
constexpr uint8_t kRowBoardPins[] = {47, 48, 49, 50};
constexpr uint8_t kColBoardPins[] = {37, 38, 39, 40, 41, 42, 43, 44, 45};

constexpr size_t kRowCount = sizeof(kMcpRowPins) / sizeof(kMcpRowPins[0]);
constexpr size_t kColCount = sizeof(kMcpColPins) / sizeof(kMcpColPins[0]);

// TLC5955 Pins (GPIO 33, 34, 36 are reserved for Octal PSRAM on ESP32-S3R8!)
constexpr uint8_t kPinSin = 15;
constexpr uint8_t kPinSclk = 16;
constexpr uint8_t kPinLat = 17;
constexpr uint8_t kPinGsclk = 2;
constexpr uint8_t kPinMiso = 4;
constexpr uint32_t kGsclkFrequencyHz = 20000000;
constexpr uint32_t kSpiFrequencyHz = 25000000;

// LED Brightness values
constexpr uint16_t kLedBrightness = 0xFFFF;
constexpr uint16_t kDimBrightness = 0x0800;
constexpr uint16_t kFaderLedBrightness = 0x1800;

constexpr uint16_t bitFor(uint8_t pin) {
  return static_cast<uint16_t>(1U << pin);
}

constexpr uint16_t channelFor(uint8_t chip, uint8_t led, uint8_t colorChannel) {
  return static_cast<uint16_t>(
      chip * TLC5955::LEDS_PER_CHIP * TLC5955::COLOR_CHANNEL_COUNT +
      led * TLC5955::COLOR_CHANNEL_COUNT +
      colorChannel);
}

constexpr uint16_t kRowMask = bitFor(12) | bitFor(13) | bitFor(14) | bitFor(15);
constexpr uint16_t kColMask =
    bitFor(0) | bitFor(1) | bitFor(2) | bitFor(3) | bitFor(4) |
    bitFor(5) | bitFor(6) | bitFor(8) | bitFor(9);

MCP23017 gMcp(kMcpAddress);
SPIClass gTlcSpi(HSPI);
TLC5955 gTlc;
bool gMatrixReady = false;

bool gStableState[kRowCount][kColCount] = {};
bool gLastRawState[kRowCount][kColCount] = {};
uint32_t gLastChangeMs[kRowCount][kColCount] = {};

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

constexpr RgbLedMapping kRgbLeds[] = {
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

// Top-to-bottom fader LED order. LED 1 is the top physical LED, LED 16 is the bottom one.
constexpr uint16_t kFaderLedChannels[] = {
    channelFor(1, 3, 0),
    channelFor(1, 6, 0),
    channelFor(1, 7, 0),
    channelFor(1, 8, 0),
    channelFor(1, 10, 0),
    channelFor(1, 12, 0),
    channelFor(1, 13, 0),
    channelFor(1, 14, 0),
    channelFor(2, 4, 0),
    channelFor(2, 8, 0),
    channelFor(2, 9, 0),
    channelFor(2, 10, 0),
    channelFor(2, 11, 0),
    channelFor(2, 12, 0),
    channelFor(2, 13, 0),
    channelFor(2, 14, 0),
};
constexpr size_t kFaderLedCount = sizeof(kFaderLedChannels) / sizeof(kFaderLedChannels[0]);

struct ButtonMapping {
  uint8_t row;
  uint8_t col;
  const char* name;
  LedId ledId;
};

constexpr ButtonMapping kButtonMappings[] = {
    {50, 45, "PGM1", LedId::PGM1},
    {50, 44, "PGM2", LedId::PGM2},
    {50, 43, "PGM3", LedId::PGM3},
    {50, 42, "PGM4", LedId::PGM4},
    {50, 41, "PGM5", LedId::PGM5},
    {50, 40, "PGM6", LedId::PGM6},
    {50, 39, "PGM7", LedId::PGM7},
    {50, 38, "PGM8", LedId::PGM8},
    {50, 37, "PGM Shift", LedId::PGM_SHIFT},
    {49, 45, "Preview 1", LedId::PRV1},
    {49, 44, "Preview 2", LedId::PRV2},
    {49, 43, "Preview 3", LedId::PRV3},
    {49, 42, "Preview 4", LedId::PRV4},
    {49, 41, "Preview 5", LedId::PRV5},
    {49, 40, "Preview 6", LedId::PRV6},
    {49, 39, "Preview 7", LedId::PRV7},
    {49, 38, "Preview 8", LedId::PRV8},
    {49, 37, "Preview Shift", LedId::PRV_SHIFT},
    {48, 45, "CUT", LedId::CUT},
    {47, 45, "AUTO", LedId::AUTO},
    {47, 44, "KEY1 CUT", LedId::KEY1_CUT},
    {47, 43, "KEY1 TIE", LedId::KEY1_TIE},
    {48, 43, "BKGD", LedId::BKGD},
    {48, 42, "DSK1 AUTO", LedId::DSK1_AUTO},
    {47, 42, "DSK2 AUTO", LedId::DSK2_AUTO},
    {48, 41, "DSK1 CUT", LedId::DSK1_CUT},
    {47, 41, "DSK2 CUT", LedId::DSK2_CUT},
    {47, 39, "FTB", LedId::FTB},
    {48, 40, "DSK1 TIE", LedId::DSK1_TIE},
    {47, 40, "DSK2 TIE", LedId::DSK2_TIE},
};

// Control panel local toggles
bool gPgmShift = false;
bool gPrvShift = false;

const ButtonMapping* findButtonMapping(uint8_t row, uint8_t col) {
  for (const ButtonMapping& mapping : kButtonMappings) {
    if (mapping.row == row && mapping.col == col) {
      return &mapping;
    }
  }
  return nullptr;
}

const RgbLedMapping* findRgbLed(LedId id) {
  for (const RgbLedMapping& led : kRgbLeds) {
    if (led.id == id) {
      return &led;
    }
  }
  return nullptr;
}

void setRgbLed(LedId id, uint16_t red, uint16_t green, uint16_t blue) {
  const RgbLedMapping* led = findRgbLed(id);
  if (led == nullptr) return;
  gTlc.set_single_channel(led->redChannel, red);
  gTlc.set_single_channel(led->greenChannel, green);
  gTlc.set_single_channel(led->blueChannel, blue);
}

void setupTlc() {
  gTlc.init(kPinLat, kPinSin, kPinSclk, kPinGsclk, &gTlcSpi, kPinMiso);
  gTlc.set_sclk_frequency(kSpiFrequencyHz);
  gTlc.set_gsclk_frequency(kGsclkFrequencyHz);

  gTlc.set_all_dc_data(255);
  gTlc.set_max_current(1, 1, 1);
  gTlc.set_function_data(true, true, true, true, true);
  gTlc.set_brightness_current(127, 127, 127);
  gTlc.update_control();

  gTlc.set_all(kDimBrightness);
  gTlc.update();
}

void setupMatrix() {
  Wire.begin(kI2cSdaPin, kI2cSclPin);
  Wire.setClock(kI2cFrequencyHz);

  // Auto-detect MCP23017 I2C Address (range 0x20 to 0x27)
  uint8_t detectedAddr = 0;
  for (uint8_t addr = 0x20; addr <= 0x27; ++addr) {
    Wire.beginTransmission(addr);
    if (Wire.endTransmission() == 0) {
      detectedAddr = addr;
      break;
    }
  }

  if (detectedAddr == 0) {
    Serial.println("ERROR: MCP23017 not found on I2C bus! Scanning entire bus...");
    bool foundDevice = false;
    for (uint8_t addr = 1; addr < 127; ++addr) {
      Wire.beginTransmission(addr);
      if (Wire.endTransmission() == 0) {
        Serial.printf("  I2C device detected at address 0x%02X\r\n", addr);
        foundDevice = true;
      }
    }
    if (!foundDevice) {
      Serial.println("  No I2C devices found at all. Check SDA/SCL lines and pull-up resistors.");
    }
    return;
  }

  Serial.printf("I2C: MCP23017 auto-detected at address 0x%02X\r\n", detectedAddr);
  gMcp = MCP23017(detectedAddr);

  if (!gMcp.begin(false)) {
    Serial.printf("ERROR: Failed to initialize MCP23017 at address 0x%02X\r\n", detectedAddr);
    return;
  }

  gMcp.reverse16ByteOrder(false);

  for (size_t row = 0; row < kRowCount; ++row) {
    gMcp.pinMode1(kMcpRowPins[row], INPUT_PULLUP);
    gMcp.setPullup(kMcpRowPins[row], true);
  }

  for (size_t col = 0; col < kColCount; ++col) {
    gMcp.pinMode1(kMcpColPins[col], OUTPUT);
    gMcp.write1(kMcpColPins[col], HIGH);
  }

  gMatrixReady = true;
}

// Button Matrix scans and client commands dispatcher
void handleButtonPress(const ButtonMapping* button, uint8_t row, uint8_t col) {
  Serial.printf("BUTTON PRESS: %s (row %u, col %u)\r\n", button->name, row, col);
  const AtemSwitcherState& atem = gAtem.state();

  // Parse button name to index (1-8)
  uint8_t btnIdx = 0;
  if (button->name[3] >= '1' && button->name[3] <= '8') {
    btnIdx = button->name[3] - '0';
  } else if (button->name[8] >= '1' && button->name[8] <= '8') {
    btnIdx = button->name[8] - '0';
  }

  if (strncmp(button->name, "PGM", 3) == 0 && btnIdx > 0) {
    uint16_t src = gAtem.sourceForButton(btnIdx - 1, gPgmShift);
    if (src == kUnknownAtemSource) return;
    uint8_t payload[4] = { 0x00, 0x00, (uint8_t)(src >> 8), (uint8_t)(src & 0xFF) };
    if (gAtem.sendCommand("CPgI", payload, 4)) {
      Serial.printf("Sent CPgI program source %d\r\n", src);
    }
  } else if (strncmp(button->name, "Preview", 7) == 0 && btnIdx > 0) {
    uint16_t src = gAtem.sourceForButton(btnIdx - 1, gPrvShift);
    if (src == kUnknownAtemSource) return;
    uint8_t payload[4] = { 0x00, 0x00, (uint8_t)(src >> 8), (uint8_t)(src & 0xFF) };
    if (gAtem.sendCommand("CPvI", payload, 4)) {
      Serial.printf("Sent CPvI preview source %d\r\n", src);
    }
  } else if (strcmp(button->name, "PGM Shift") == 0) {
    gPgmShift = !gPgmShift;
    Serial.printf("PGM Shift toggle -> %s\r\n", gPgmShift ? "ON" : "OFF");
  } else if (strcmp(button->name, "Preview Shift") == 0) {
    gPrvShift = !gPrvShift;
    Serial.printf("Preview Shift toggle -> %s\r\n", gPrvShift ? "ON" : "OFF");
  } else if (strcmp(button->name, "CUT") == 0) {
    uint8_t payload[4] = { 0x00, 0x00, 0x00, 0x00 };
    if (gAtem.sendCommand("DCut", payload, 4)) {
      Serial.println("Sent DCut");
    }
  } else if (strcmp(button->name, "AUTO") == 0) {
    uint8_t payload[4] = { 0x00, 0x00, 0x00, 0x00 };
    if (gAtem.sendCommand("DAut", payload, 4)) {
      Serial.println("Sent DAut");
    }
  } else if (strcmp(button->name, "KEY1 CUT") == 0) {
    bool newState = !atem.key1OnAir;
    uint8_t payload[4] = { 0x00, 0x00, (uint8_t)(newState ? 1 : 0), 0x00 };
    if (gAtem.sendCommand("CKOn", payload, 4)) {
      Serial.printf("Sent CKOn state %d\r\n", newState);
    }
  } else if (strcmp(button->name, "KEY1 TIE") == 0) {
    uint8_t nextTr = (atem.nextTrBkgd ? 1 : 0) | (atem.nextTrKey1 ? 2 : 0);
    nextTr ^= 2; // Toggle Key 1
    uint8_t payload[4] = { 0x02, 0x00, 0x00, nextTr }; // Next transition selection change mask = 0x02
    if (gAtem.sendCommand("CTTp", payload, 4)) {
      Serial.printf("Sent CTTp selection %d\r\n", nextTr);
    }
  } else if (strcmp(button->name, "BKGD") == 0) {
    uint8_t nextTr = (atem.nextTrBkgd ? 1 : 0) | (atem.nextTrKey1 ? 2 : 0);
    nextTr ^= 1; // Toggle BKGD
    uint8_t payload[4] = { 0x02, 0x00, 0x00, nextTr };
    if (gAtem.sendCommand("CTTp", payload, 4)) {
      Serial.printf("Sent CTTp selection %d\r\n", nextTr);
    }
  } else if (strcmp(button->name, "DSK1 CUT") == 0) {
    if (!gAtem.supportsDownstreamKeyer(0)) return;
    bool newState = !atem.dskOnAir[0];
    uint8_t payload[4] = { 0x00, (uint8_t)(newState ? 1 : 0), 0x00, 0x00 };
    if (gAtem.sendCommand("CDsL", payload, 4)) {
      Serial.printf("Sent CDsL DSK1 state %d\r\n", newState);
    }
  } else if (strcmp(button->name, "DSK2 CUT") == 0) {
    if (!gAtem.supportsDownstreamKeyer(1)) return;
    bool newState = !atem.dskOnAir[1];
    uint8_t payload[4] = { 0x01, (uint8_t)(newState ? 1 : 0), 0x00, 0x00 };
    if (gAtem.sendCommand("CDsL", payload, 4)) {
      Serial.printf("Sent CDsL DSK2 state %d\r\n", newState);
    }
  } else if (strcmp(button->name, "DSK1 TIE") == 0) {
    if (!gAtem.supportsDownstreamKeyer(0)) return;
    bool newState = !atem.dskTie[0];
    uint8_t payload[4] = { 0x00, (uint8_t)(newState ? 1 : 0), 0x00, 0x00 };
    if (gAtem.sendCommand("CDsT", payload, 4)) {
      Serial.printf("Sent CDsT DSK1 state %d\r\n", newState);
    }
  } else if (strcmp(button->name, "DSK2 TIE") == 0) {
    if (!gAtem.supportsDownstreamKeyer(1)) return;
    bool newState = !atem.dskTie[1];
    uint8_t payload[4] = { 0x01, (uint8_t)(newState ? 1 : 0), 0x00, 0x00 };
    if (gAtem.sendCommand("CDsT", payload, 4)) {
      Serial.printf("Sent CDsT DSK2 state %d\r\n", newState);
    }
  } else if (strcmp(button->name, "DSK1 AUTO") == 0) {
    if (!gAtem.supportsDownstreamKeyer(0)) return;
    uint8_t payload[4] = { 0x00, 0x00, 0x00, 0x00 };
    if (gAtem.sendCommand("DDsA", payload, 4)) {
      Serial.println("Sent DDsA DSK1 Auto");
    }
  } else if (strcmp(button->name, "DSK2 AUTO") == 0) {
    if (!gAtem.supportsDownstreamKeyer(1)) return;
    uint8_t payload[4] = { 0x01, 0x00, 0x00, 0x00 };
    if (gAtem.sendCommand("DDsA", payload, 4)) {
      Serial.println("Sent DDsA DSK2 Auto");
    }
  } else if (strcmp(button->name, "FTB") == 0) {
    uint8_t payload[4] = { 0x00, 0x00, 0x00, 0x00 };
    if (gAtem.sendCommand("FtbA", payload, 4)) {
      Serial.println("Sent FtbA");
    }
  }
}

void handleButtonRelease(const ButtonMapping* button, uint8_t row, uint8_t col) {
  Serial.printf("BUTTON RELEASE: %s (row %u, col %u)\r\n", button->name, row, col);
}

void writeMcpOutputs(uint16_t outputState) {
  for (size_t col = 0; col < kColCount; ++col) {
    gMcp.write1(kMcpColPins[col], (outputState & bitFor(kMcpColPins[col])) ? HIGH : LOW);
  }
}

void primeMatrixState() {
  const uint32_t now = millis();

  for (size_t col = 0; col < kColCount; ++col) {
    gMcp.write1(kMcpColPins[col], LOW);
    delayMicroseconds(50);

    for (size_t row = 0; row < kRowCount; ++row) {
      const bool rawPressed = (gMcp.read1(kMcpRowPins[row]) == LOW);
      gLastRawState[row][col] = rawPressed;
      gStableState[row][col] = rawPressed;
      gLastChangeMs[row][col] = now;
    }

    gMcp.write1(kMcpColPins[col], HIGH);
  }

  Serial.println("Key matrix baseline captured.");
}

void scanMatrix() {
  const uint32_t now = millis();

  for (size_t col = 0; col < kColCount; ++col) {
    gMcp.write1(kMcpColPins[col], LOW);
    delayMicroseconds(50);

    for (size_t row = 0; row < kRowCount; ++row) {
      const bool rawPressed = (gMcp.read1(kMcpRowPins[row]) == LOW);

      if (rawPressed != gLastRawState[row][col]) {
        gLastRawState[row][col] = rawPressed;
        gLastChangeMs[row][col] = now;
      }

      if (rawPressed != gStableState[row][col] &&
          (now - gLastChangeMs[row][col]) >= kDebounceMs) {
        gStableState[row][col] = rawPressed;
        const uint8_t boardRow = kRowBoardPins[row];
        const uint8_t boardCol = kColBoardPins[col];
        const ButtonMapping* button = findButtonMapping(boardRow, boardCol);
        if (button != nullptr) {
          if (rawPressed) {
            handleButtonPress(button, boardRow, boardCol);
          } else {
            handleButtonRelease(button, boardRow, boardCol);
          }
        } else {
          Serial.printf("BUTTON %s: unmapped (row %u, col %u)\r\n",
                        rawPressed ? "PRESS" : "RELEASE",
                        boardRow,
                        boardCol);
        }
      }
    }

    gMcp.write1(kMcpColPins[col], HIGH);
  }
}

void updateMatrixScanner() {
  if (!gMatrixReady) return;

  static uint32_t lastScanMs = 0;
  const uint32_t now = millis();

  if ((now - lastScanMs) < kScanIntervalMs) return;

  lastScanMs = now;
  scanMatrix();
}

uint8_t faderLedIndexForPosition(uint16_t position) {
  if (position > kAtemTransitionPositionMax) {
    position = kAtemTransitionPositionMax;
  }

  const uint32_t invertedPosition = kAtemTransitionPositionMax - position;
  return static_cast<uint8_t>(
      (invertedPosition * (kFaderLedCount - 1) + (kAtemTransitionPositionMax / 2)) /
      kAtemTransitionPositionMax);
}

bool isLedSupportedByActiveProfile(const RgbLedMapping& led, uint8_t btnIdx) {
  if (strncmp(led.name, "PGM", 3) == 0 && btnIdx > 0) {
    return gAtem.sourceForButton(btnIdx - 1, gPgmShift) != kUnknownAtemSource;
  }

  if (strncmp(led.name, "Preview", 7) == 0 && btnIdx > 0) {
    return gAtem.sourceForButton(btnIdx - 1, gPrvShift) != kUnknownAtemSource;
  }

  if (led.id == LedId::DSK1_CUT || led.id == LedId::DSK1_TIE || led.id == LedId::DSK1_AUTO) {
    return gAtem.supportsDownstreamKeyer(0);
  }

  if (led.id == LedId::DSK2_CUT || led.id == LedId::DSK2_TIE || led.id == LedId::DSK2_AUTO) {
    return gAtem.supportsDownstreamKeyer(1);
  }

  return true;
}

void updateFaderLeds() {
  const AtemSwitcherState& atem = gAtem.state();
  const uint8_t currentLed = faderLedIndexForPosition(atem.faderLedPosition);

  for (size_t i = 0; i < kFaderLedCount; ++i) {
    gTlc.set_single_channel(
        kFaderLedChannels[i],
        i == currentLed ? kFaderLedBrightness : 0);
  }
}

// LED update mapping logic
void updateLeds() {
  static uint32_t lastLedUpdateMs = 0;
  const uint32_t now = millis();

  if (now - lastLedUpdateMs < kLedUpdateIntervalMs) return;
  lastLedUpdateMs = now;

  bool blinkState = (now / 250) % 2 == 0;
  const AtemSwitcherState& atem = gAtem.state();

  gTlc.set_all(0);

  // Map each RGB LED according to ATEM state
  for (const RgbLedMapping& led : kRgbLeds) {
    uint16_t r = kDimBrightness, g = kDimBrightness, b = kDimBrightness;

    uint8_t btnIdx = 0;
    if (led.name[3] >= '1' && led.name[3] <= '8') {
      btnIdx = led.name[3] - '0';
    } else if (led.name[8] >= '1' && led.name[8] <= '8') {
      btnIdx = led.name[8] - '0';
    }

    if (!isLedSupportedByActiveProfile(led, btnIdx)) {
      gTlc.set_single_channel(led.redChannel, 0);
      gTlc.set_single_channel(led.greenChannel, 0);
      gTlc.set_single_channel(led.blueChannel, 0);
      continue;
    }

    // Program inputs row
    if (strncmp(led.name, "PGM", 3) == 0 && btnIdx > 0) {
      const uint16_t normalSrc = gAtem.sourceForButton(btnIdx - 1, false);
      const uint16_t shiftedSrc = gAtem.sourceForButton(btnIdx - 1, true);
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
      const uint16_t normalSrc = gAtem.sourceForButton(btnIdx - 1, false);
      const uint16_t shiftedSrc = gAtem.sourceForButton(btnIdx - 1, true);
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
      if (gPgmShift) {
        r = kLedBrightness; g = kLedBrightness; b = 0; // Shift active
      }
    } else if (led.id == LedId::PRV_SHIFT) {
      if (gPrvShift) {
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
      if (!gAtem.connected()) {
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

    gTlc.set_single_channel(led.redChannel, r);
    gTlc.set_single_channel(led.greenChannel, g);
    gTlc.set_single_channel(led.blueChannel, b);
  }

  updateFaderLeds();
  gTlc.update();
}

void printHeader() {
  Serial.println();
  Serial.println("=========================================================");
  Serial.println("Waveshare ESP32-S3-ETH Custom ATEM Control Panel Loaded");
  Serial.printf("Local Static IP:  %d.%d.%d.%d\r\n", kLocalIp[0], kLocalIp[1], kLocalIp[2], kLocalIp[3]);
  Serial.printf("ATEM Switcher IP: %d.%d.%d.%d\r\n", kSwitcherIp[0], kSwitcherIp[1], kSwitcherIp[2], kSwitcherIp[3]);
  Serial.println("=========================================================");
  Serial.println();
}

EthernetLinkStatus pollEthernetLinkStatus(bool forceReport = false) {
  const uint32_t now = millis();
  if (!forceReport && (now - gLastEthernetLinkPollMs) < kEthernetLinkPollMs) {
    return gLastEthernetLinkStatus;
  }
  gLastEthernetLinkPollMs = now;

  const EthernetLinkStatus currentStatus = Ethernet.linkStatus();
  if (!forceReport && currentStatus == gLastEthernetLinkStatus) {
    return currentStatus;
  }

  const EthernetLinkStatus previousStatus = gLastEthernetLinkStatus;
  gLastEthernetLinkStatus = currentStatus;
  if (currentStatus == LinkON) {
    Serial.println("Ethernet cable connected.");
  } else if (currentStatus == LinkOFF) {
    Serial.println("WARNING: Ethernet cable disconnected.");
    if (previousStatus != LinkOFF) {
      gAtem.resetConnection();
    }
  } else {
    Serial.println("Ethernet cable status unknown.");
  }
  return currentStatus;
}

void atemNetworkTask(void*) {
  for (;;) {
    if (pollEthernetLinkStatus() == LinkON) {
      gAtem.update();
    }
    vTaskDelay(pdMS_TO_TICKS(kAtemTaskDelayMs));
  }
}

void startAtemNetworkTask() {
  if (gAtemTaskHandle != nullptr) return;

  BaseType_t result = xTaskCreatePinnedToCore(
      atemNetworkTask,
      "atem-network",
      kAtemTaskStackWords,
      nullptr,
      kAtemTaskPriority,
      &gAtemTaskHandle,
      0);

  if (result == pdPASS) {
    Serial.println("ATEM network task started on core 0.");
  } else {
    Serial.println("ERROR: Failed to start ATEM network task.");
  }
}

}  // namespace

void setup() {
  Serial.begin(kBaudRate);
  delay(300);

  setupTlc();
  setupMatrix();
  if (gMatrixReady) {
    primeMatrixState();
  }
  printHeader();

  // Reset W5500 SPI Ethernet
  pinMode(kEthRstPin, OUTPUT);
  digitalWrite(kEthRstPin, LOW);
  delay(10);
  digitalWrite(kEthRstPin, HIGH);
  delay(100);

  // Initialize Ethernet SPI - Pass -1 for hardware SS to avoid conflicting with manual CS pin 14
  SPI.begin(kEthSclkPin, kEthMisoPin, kEthMosiPin, -1);
  Ethernet.init(kEthCsPin);

  // Initialize static Ethernet config
  uint8_t mac[] = { 0xDE, 0xAD, 0xBE, 0xEF, 0xFE, 0xED };
  Ethernet.begin(mac, kLocalIp, kGateway, kGateway, kSubnet);
  
  // Print diagnostic network configuration
  Serial.print("Ethernet IP:      ");
  Serial.println(Ethernet.localIP());
  Serial.print("Ethernet Subnet:  ");
  Serial.println(Ethernet.subnetMask());
  Serial.print("Ethernet Gateway: ");
  Serial.println(Ethernet.gatewayIP());

  pollEthernetLinkStatus(true);

  // Configure W5500 to fail fast on ARP/transmission timeouts (avoid freezing main loop)
  W5100.setRetransmissionTime(400); // 40ms retransmission timeout
  W5100.setRetransmissionCount(3);  // 3 retries (total 120ms)

  // Open UDP listener on client port
  if (gAtem.begin()) {
    Serial.printf("UDP Listener opened on local port %u\r\n", kLocalPort);
  } else {
    Serial.printf("ERROR: Failed to open UDP port %u\r\n", kLocalPort);
  }

  startAtemNetworkTask();
}

void loop() {
  updateMatrixScanner();
  updateLeds();
}
