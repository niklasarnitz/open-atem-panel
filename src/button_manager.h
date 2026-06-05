#pragma once

#include <Arduino.h>
#include "led_controller.h" // for LedId

class MCP23017;
class AtemClient;

constexpr size_t kRowCount = 4;
constexpr size_t kColCount = 9;

struct ButtonMapping {
  uint8_t row;
  uint8_t col;
  const char* name;
  LedId ledId;
};

extern const ButtonMapping kButtonMappings[];
extern const size_t kButtonMappingCount;

class ButtonManager {
 public:
  ButtonManager(MCP23017& mcp, AtemClient& atem);

  void begin();
  void update();

  bool pgmShift() const { return pgmShift_; }
  bool prvShift() const { return prvShift_; }
  
  // Expose these for testing/debugging if needed
  void setPgmShift(bool active) { pgmShift_ = active; }
  void setPrvShift(bool active) { prvShift_ = active; }
  
  void primeMatrixState();
  void scanMatrix();
  const ButtonMapping* findButtonMapping(uint8_t row, uint8_t col) const;
  void handleButtonPress(const ButtonMapping* button, uint8_t row, uint8_t col);
  void handleButtonRelease(const ButtonMapping* button, uint8_t row, uint8_t col);

 private:
  MCP23017& mcp_;
  AtemClient& atem_;
  bool matrixReady_ = false;

  bool pgmShift_ = false;
  bool prvShift_ = false;

  bool stableState_[kRowCount][kColCount] = {};
  bool lastRawState_[kRowCount][kColCount] = {};
  uint32_t lastChangeMs_[kRowCount][kColCount] = {};
};
