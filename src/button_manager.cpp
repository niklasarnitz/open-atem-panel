#include "button_manager.h"
#include "atem_client.h"
#include <MCP23017.h>
#include <Wire.h>

#ifndef KEYMATRIX_MCP_SDA_PIN
#define KEYMATRIX_MCP_SDA_PIN 48
#endif
#ifndef KEYMATRIX_MCP_SCL_PIN
#define KEYMATRIX_MCP_SCL_PIN 47
#endif

namespace {

constexpr uint8_t kI2cSdaPin = KEYMATRIX_MCP_SDA_PIN;
constexpr uint8_t kI2cSclPin = KEYMATRIX_MCP_SCL_PIN;
constexpr uint32_t kI2cFrequencyHz = 400000;

// MCP23017 rows & columns configuration
constexpr uint8_t kMcpRowPins[] = {12, 13, 14, 15};
constexpr uint8_t kMcpColPins[] = {0, 1, 2, 3, 4, 5, 6, 8, 9};
constexpr uint8_t kRowBoardPins[] = {47, 48, 49, 50};
constexpr uint8_t kColBoardPins[] = {37, 38, 39, 40, 41, 42, 43, 44, 45};

constexpr uint32_t kDebounceMs = 20;
constexpr uint32_t kScanIntervalMs = 2;

} // namespace

const ButtonMapping kButtonMappings[] = {
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
const size_t kButtonMappingCount = sizeof(kButtonMappings) / sizeof(kButtonMappings[0]);

ButtonManager::ButtonManager(MCP23017& mcp, AtemClient& atem)
    : mcp_(mcp), atem_(atem) {}

void ButtonManager::begin() {
  Wire.begin(kI2cSdaPin, kI2cSclPin);
  Wire.setClock(kI2cFrequencyHz);

  // Scan entire I2C bus (excluding address 0) to report all connected devices
  Serial.println("I2C Scanner: Scanning bus...");
  bool foundAny = false;
  uint8_t detectedMcpAddr = 0;

  for (uint8_t addr = 1; addr < 127; ++addr) {
    Wire.beginTransmission(addr);
    if (Wire.endTransmission() == 0) {
      Serial.printf("  I2C device detected at address 0x%02X", addr);
      
      if (addr >= 0x20 && addr <= 0x27) {
        if (detectedMcpAddr == 0) {
          detectedMcpAddr = addr;
        }
        Serial.print(" (MCP23017 Pin Expander)");
      } else if (addr == 0x33) {
        Serial.print(" (MAX11612 ADC)");
      }
      
      Serial.println();
      foundAny = true;
    }
  }

  if (!foundAny) {
    Serial.println("  No I2C devices found at all. Check SDA/SCL lines and pull-up resistors.");
  }

  if (detectedMcpAddr == 0) {
    Serial.println("ERROR: MCP23017 not found on I2C bus!");
    return;
  }

  Serial.printf("I2C: MCP23017 auto-detected at address 0x%02X\r\n", detectedMcpAddr);
  mcp_ = MCP23017(detectedMcpAddr);

  if (!mcp_.begin(false)) {
    Serial.printf("ERROR: Failed to initialize MCP23017 at address 0x%02X\r\n", detectedMcpAddr);
    return;
  }

  mcp_.reverse16ByteOrder(false);

  for (size_t row = 0; row < kRowCount; ++row) {
    mcp_.pinMode1(kMcpRowPins[row], INPUT_PULLUP);
    mcp_.setPullup(kMcpRowPins[row], true);
  }

  for (size_t col = 0; col < kColCount; ++col) {
    mcp_.pinMode1(kMcpColPins[col], OUTPUT);
    mcp_.write1(kMcpColPins[col], HIGH);
  }

  matrixReady_ = true;
}

void ButtonManager::primeMatrixState() {
  const uint32_t now = millis();

  for (size_t col = 0; col < kColCount; ++col) {
    mcp_.write1(kMcpColPins[col], LOW);
    delayMicroseconds(50);

    for (size_t row = 0; row < kRowCount; ++row) {
      const bool rawPressed = (mcp_.read1(kMcpRowPins[row]) == LOW);
      lastRawState_[row][col] = rawPressed;
      stableState_[row][col] = rawPressed;
      lastChangeMs_[row][col] = now;
    }

    mcp_.write1(kMcpColPins[col], HIGH);
  }

  Serial.println("Key matrix baseline captured.");
}

const ButtonMapping* ButtonManager::findButtonMapping(uint8_t row, uint8_t col) const {
  for (const ButtonMapping& mapping : kButtonMappings) {
    if (mapping.row == row && mapping.col == col) {
      return &mapping;
    }
  }
  return nullptr;
}

void ButtonManager::handleButtonPress(const ButtonMapping* button, uint8_t row, uint8_t col) {
  Serial.printf("BUTTON PRESS: %s (row %u, col %u)\r\n", button->name, row, col);
  const AtemSwitcherState& atem = atem_.state();

  // Parse button name to index (1-8)
  uint8_t btnIdx = 0;
  if (button->name[3] >= '1' && button->name[3] <= '8') {
    btnIdx = button->name[3] - '0';
  } else if (button->name[8] >= '1' && button->name[8] <= '8') {
    btnIdx = button->name[8] - '0';
  }

  if (strncmp(button->name, "PGM", 3) == 0 && btnIdx > 0) {
    uint16_t src = atem_.sourceForButton(btnIdx - 1, pgmShift_);
    if (src == kUnknownAtemSource) return;
    uint8_t payload[4] = { 0x00, 0x00, (uint8_t)(src >> 8), (uint8_t)(src & 0xFF) };
    if (atem_.sendCommand("CPgI", payload, 4)) {
      Serial.printf("Sent CPgI program source %d\r\n", src);
    }
  } else if (strncmp(button->name, "Preview", 7) == 0 && btnIdx > 0) {
    uint16_t src = atem_.sourceForButton(btnIdx - 1, prvShift_);
    if (src == kUnknownAtemSource) return;
    uint8_t payload[4] = { 0x00, 0x00, (uint8_t)(src >> 8), (uint8_t)(src & 0xFF) };
    if (atem_.sendCommand("CPvI", payload, 4)) {
      Serial.printf("Sent CPvI preview source %d\r\n", src);
    }
  } else if (strcmp(button->name, "PGM Shift") == 0) {
    pgmShift_ = true;
    Serial.println("PGM Shift hold -> ON");
  } else if (strcmp(button->name, "Preview Shift") == 0) {
    prvShift_ = true;
    Serial.println("Preview Shift hold -> ON");
  } else if (strcmp(button->name, "CUT") == 0) {
    uint8_t payload[4] = { 0x00, 0x00, 0x00, 0x00 };
    if (atem_.sendCommand("DCut", payload, 4)) {
      Serial.println("Sent DCut");
    }
  } else if (strcmp(button->name, "AUTO") == 0) {
    uint8_t payload[4] = { 0x00, 0x00, 0x00, 0x00 };
    if (atem_.sendCommand("DAut", payload, 4)) {
      Serial.println("Sent DAut");
    }
  } else if (strcmp(button->name, "KEY1 CUT") == 0) {
    bool newState = !atem.key1OnAir;
    uint8_t payload[4] = { 0x00, 0x00, (uint8_t)(newState ? 1 : 0), 0x00 };
    if (atem_.sendCommand("CKOn", payload, 4)) {
      Serial.printf("Sent CKOn state %d\r\n", newState);
    }
  } else if (strcmp(button->name, "KEY1 TIE") == 0) {
    uint8_t nextTr = (atem.nextTrBkgd ? 1 : 0) | (atem.nextTrKey1 ? 2 : 0);
    nextTr ^= 2; // Toggle Key 1
    uint8_t payload[4] = { 0x02, 0x00, 0x00, nextTr }; // Next transition selection change mask = 0x02
    if (atem_.sendCommand("CTTp", payload, 4)) {
      Serial.printf("Sent CTTp selection %d\r\n", nextTr);
    }
  } else if (strcmp(button->name, "BKGD") == 0) {
    uint8_t nextTr = (atem.nextTrBkgd ? 1 : 0) | (atem.nextTrKey1 ? 2 : 0);
    nextTr ^= 1; // Toggle BKGD
    uint8_t payload[4] = { 0x02, 0x00, 0x00, nextTr };
    if (atem_.sendCommand("CTTp", payload, 4)) {
      Serial.printf("Sent CTTp selection %d\r\n", nextTr);
    }
  } else if (strcmp(button->name, "DSK1 CUT") == 0) {
    if (!atem_.supportsDownstreamKeyer(0)) return;
    bool newState = !atem.dskOnAir[0];
    uint8_t payload[4] = { 0x00, (uint8_t)(newState ? 1 : 0), 0x00, 0x00 };
    if (atem_.sendCommand("CDsL", payload, 4)) {
      Serial.printf("Sent CDsL DSK1 state %d\r\n", newState);
    }
  } else if (strcmp(button->name, "DSK2 CUT") == 0) {
    if (!atem_.supportsDownstreamKeyer(1)) return;
    bool newState = !atem.dskOnAir[1];
    uint8_t payload[4] = { 0x01, (uint8_t)(newState ? 1 : 0), 0x00, 0x00 };
    if (atem_.sendCommand("CDsL", payload, 4)) {
      Serial.printf("Sent CDsL DSK2 state %d\r\n", newState);
    }
  } else if (strcmp(button->name, "DSK1 TIE") == 0) {
    if (!atem_.supportsDownstreamKeyer(0)) return;
    bool newState = !atem.dskTie[0];
    uint8_t payload[4] = { 0x00, (uint8_t)(newState ? 1 : 0), 0x00, 0x00 };
    if (atem_.sendCommand("CDsT", payload, 4)) {
      Serial.printf("Sent CDsT DSK1 state %d\r\n", newState);
    }
  } else if (strcmp(button->name, "DSK2 TIE") == 0) {
    if (!atem_.supportsDownstreamKeyer(1)) return;
    bool newState = !atem.dskTie[1];
    uint8_t payload[4] = { 0x01, (uint8_t)(newState ? 1 : 0), 0x00, 0x00 };
    if (atem_.sendCommand("CDsT", payload, 4)) {
      Serial.printf("Sent CDsT DSK2 state %d\r\n", newState);
    }
  } else if (strcmp(button->name, "DSK1 AUTO") == 0) {
    if (!atem_.supportsDownstreamKeyer(0)) return;
    uint8_t payload[4] = { 0x00, 0x00, 0x00, 0x00 };
    if (atem_.sendCommand("DDsA", payload, 4)) {
      Serial.println("Sent DDsA DSK1 Auto");
    }
  } else if (strcmp(button->name, "DSK2 AUTO") == 0) {
    if (!atem_.supportsDownstreamKeyer(1)) return;
    uint8_t payload[4] = { 0x01, 0x00, 0x00, 0x00 };
    if (atem_.sendCommand("DDsA", payload, 4)) {
      Serial.println("Sent DDsA DSK2 Auto");
    }
  } else if (strcmp(button->name, "FTB") == 0) {
    uint8_t payload[4] = { 0x00, 0x00, 0x00, 0x00 };
    if (atem_.sendCommand("FtbA", payload, 4)) {
      Serial.println("Sent FtbA");
    }
  }
}

void ButtonManager::handleButtonRelease(const ButtonMapping* button, uint8_t row, uint8_t col) {
  Serial.printf("BUTTON RELEASE: %s (row %u, col %u)\r\n", button->name, row, col);
  if (strcmp(button->name, "PGM Shift") == 0) {
    pgmShift_ = false;
    Serial.println("PGM Shift hold -> OFF");
  } else if (strcmp(button->name, "Preview Shift") == 0) {
    prvShift_ = false;
    Serial.println("Preview Shift hold -> OFF");
  }
}

void ButtonManager::scanMatrix() {
  const uint32_t now = millis();

  for (size_t col = 0; col < kColCount; ++col) {
    mcp_.write1(kMcpColPins[col], LOW);
    delayMicroseconds(50);

    for (size_t row = 0; row < kRowCount; ++row) {
      const bool rawPressed = (mcp_.read1(kMcpRowPins[row]) == LOW);

      if (rawPressed != lastRawState_[row][col]) {
        lastRawState_[row][col] = rawPressed;
        lastChangeMs_[row][col] = now;
      }

      if (rawPressed != stableState_[row][col] &&
          (now - lastChangeMs_[row][col]) >= kDebounceMs) {
        stableState_[row][col] = rawPressed;
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

    mcp_.write1(kMcpColPins[col], HIGH);
  }
}

void ButtonManager::update() {
  if (!matrixReady_) return;

  static uint32_t lastScanMs = 0;
  const uint32_t now = millis();

  if ((now - lastScanMs) < kScanIntervalMs) return;

  lastScanMs = now;
  scanMatrix();
}
