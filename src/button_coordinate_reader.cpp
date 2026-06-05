#include <Arduino.h>
#include <MCP23017.h>
#include <Wire.h>

namespace {

constexpr uint32_t kBaudRate = 115200;
constexpr uint32_t kDebounceMs = 20;
constexpr uint32_t kScanIntervalMs = 2;

#ifndef KEYMATRIX_MCP_SDA_PIN
#define KEYMATRIX_MCP_SDA_PIN 48
#endif
#ifndef KEYMATRIX_MCP_SCL_PIN
#define KEYMATRIX_MCP_SCL_PIN 47
#endif

constexpr uint8_t kI2cSdaPin = KEYMATRIX_MCP_SDA_PIN;
constexpr uint8_t kI2cSclPin = KEYMATRIX_MCP_SCL_PIN;
constexpr uint32_t kI2cFrequencyHz = 400000;
constexpr uint8_t kMcpDefaultAddress = 0x20;

// Same matrix coordinates as src/main.cpp.
constexpr uint8_t kMcpRowPins[] = {12, 13, 14, 15};
constexpr uint8_t kMcpColPins[] = {0, 1, 2, 3, 4, 5, 6, 8, 9};
constexpr uint8_t kRowBoardPins[] = {47, 48, 49, 50};
constexpr uint8_t kColBoardPins[] = {37, 38, 39, 40, 41, 42, 43, 44, 45};

constexpr size_t kRowCount = sizeof(kMcpRowPins) / sizeof(kMcpRowPins[0]);
constexpr size_t kColCount = sizeof(kMcpColPins) / sizeof(kMcpColPins[0]);

static_assert(kRowCount == sizeof(kRowBoardPins) / sizeof(kRowBoardPins[0]),
              "Each row pin needs a board coordinate.");
static_assert(kColCount == sizeof(kColBoardPins) / sizeof(kColBoardPins[0]),
              "Each column pin needs a board coordinate.");

constexpr uint16_t bitFor(uint8_t pin) {
  return static_cast<uint16_t>(1U << pin);
}

MCP23017 gMcp(kMcpDefaultAddress);
bool gMatrixReady = false;

bool gStableState[kRowCount][kColCount] = {};
bool gLastRawState[kRowCount][kColCount] = {};
uint32_t gLastChangeMs[kRowCount][kColCount] = {};

void writeMcpOutputs(uint16_t outputState) {
  for (size_t col = 0; col < kColCount; ++col) {
    gMcp.write1(kMcpColPins[col], (outputState & bitFor(kMcpColPins[col])) ? HIGH : LOW);
  }
}

uint8_t detectMcpAddress() {
  for (uint8_t addr = 0x20; addr <= 0x27; ++addr) {
    Wire.beginTransmission(addr);
    if (Wire.endTransmission() == 0) {
      return addr;
    }
  }
  return 0;
}

void setupMatrix() {
  Wire.begin(kI2cSdaPin, kI2cSclPin);
  Wire.setClock(kI2cFrequencyHz);

  const uint8_t detectedAddr = detectMcpAddress();
  if (detectedAddr == 0) {
    Serial.println("ERROR: MCP23017 not found on I2C bus");
    return;
  }

  gMcp = MCP23017(detectedAddr);
  if (!gMcp.begin(false)) {
    Serial.printf("ERROR: Failed to initialize MCP23017 at address 0x%02X\r\n", detectedAddr);
    return;
  }

  for (size_t row = 0; row < kRowCount; ++row) {
    gMcp.pinMode1(kMcpRowPins[row], INPUT_PULLUP);
    gMcp.setPullup(kMcpRowPins[row], true);
  }

  for (size_t col = 0; col < kColCount; ++col) {
    gMcp.pinMode1(kMcpColPins[col], OUTPUT);
    gMcp.write1(kMcpColPins[col], HIGH);
  }

  gMatrixReady = true;

  Serial.printf("Button coordinate reader ready: MCP23017=0x%02X SDA=%u SCL=%u\r\n",
                detectedAddr,
                kI2cSdaPin,
                kI2cSclPin);
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
}

void printButtonCoordinate(size_t rowIndex, size_t colIndex) {
  Serial.printf("%u:%u\r\n",
                static_cast<unsigned>(kRowBoardPins[rowIndex]),
                static_cast<unsigned>(kColBoardPins[colIndex]));
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
        if (rawPressed) {
          printButtonCoordinate(row, col);
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

}  // namespace

void setup() {
  Serial.begin(kBaudRate);
  delay(300);

  setupMatrix();
  if (gMatrixReady) {
    primeMatrixState();
  }
}

void loop() {
  updateMatrixScanner();
}
