#include <Arduino.h>
#include <SPI.h>
#include <Ethernet.h>
#include <Wire.h>
#include <MCP23017.h>
#include <TLC5955.h>

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
constexpr uint32_t kAtemTimeoutMs = 5000;
constexpr uint32_t kAtemConnectRetryIntervalMs = 2000;

// Network configuration parameters
const IPAddress kLocalIp(192, 168, 178, 246);
const IPAddress kGateway(192, 168, 178, 1);
const IPAddress kSubnet(255, 255, 255, 0);
const IPAddress kSwitcherIp(192, 168, 178, 240);

constexpr uint16_t kSwitcherPort = 9910;
constexpr uint16_t kLocalPort = 50991;

// SPI pins for Waveshare ESP32-S3-ETH (W5500 Ethernet)
constexpr uint8_t kEthMosiPin = 11;
constexpr uint8_t kEthMisoPin = 13;
constexpr uint8_t kEthSclkPin = 12;
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
constexpr uint32_t kI2cFrequencyHz = 1000000;
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
constexpr uint32_t kGsclkFrequencyHz = 33000000;
constexpr uint32_t kSpiFrequencyHz = 25000000;

// LED Brightness values
constexpr uint16_t kLedBrightness = 0xFFFF;
constexpr uint16_t kDimBrightness = 0x0800;

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
TLC5955 gTlc;
uint16_t gMcpOutputState = 0;
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

// Shift Mappings (SDI 9-10, Media Players, Generators, Color Bars, Black)
constexpr uint16_t kShiftSources[] = {
    9,     // SDI 9
    10,    // SDI 10
    3010,  // Media Player 1
    3020,  // Media Player 2
    2001,  // Color Generator 1
    2002,  // Color Generator 2
    10010, // Color Bars
    0      // Black
};

// ATEM client state variables
enum class AtemState {
  DISCONNECTED,
  CONNECTING,
  CONNECTED
};

AtemState gAtemState = AtemState::DISCONNECTED;
uint16_t gSessionID = 0;
uint16_t gLastRemotePacketID = 0;
uint16_t gLocalPacketIdCounter = 1;
uint32_t gLastPacketReceivedMs = 0;
uint32_t gLastConnectAttemptMs = 0;
bool gWaitingForInitialDump = true;
EthernetUDP gUdp;

// Switcher states tracked
uint16_t gActiveProgramSource = 0;
uint16_t gActivePreviewSource = 0;
bool gKey1OnAir = false;
bool gNextTrBkgd = false;
bool gNextTrKey1 = false;
bool gDskOnAir[2] = {false, false};
bool gDskTie[2] = {false, false};
bool gDskTransitioning[2] = {false, false};
bool gFtbActive = false;
bool gFtbDone = false;
bool gTransitionInProgress = false;

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
  gTlc.init(kPinLat, kPinSin, kPinSclk, kPinGsclk);
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

  if (!gMcp.begin(false)) {
    Serial.println("ERROR: MCP23017 not found at address 0x20");
    return;
  }

  gMcp.reverse16ByteOrder(false);
  gMcp.pinMode16(kColMask);
  gMcp.setPullup16(0);
  gMcp.write16(gMcpOutputState);
  gMatrixReady = true;
}

// Custom ATEM library core functions
void sendAck(uint16_t remotePacketId, bool special = false) {
  uint8_t ack[12];
  memset(ack, 0, 12);
  ack[0] = 0x80; // ACK flag
  ack[1] = 0x0C; // Packet length (12)
  ack[2] = gSessionID >> 8;
  ack[3] = gSessionID & 0xFF;
  ack[4] = remotePacketId >> 8;
  ack[5] = remotePacketId & 0xFF;
  if (special) {
    ack[9] = 0x61; // Remote sequence number set to 0x61 for empty packet ACK
  } else {
    ack[9] = 0x00; // Remote sequence number is 0 for standard ACKs
  }

  gUdp.beginPacket(kSwitcherIp, kSwitcherPort);
  gUdp.write(ack, 12);
  gUdp.endPacket();
}

void sendCommand(const char* cmdName, const uint8_t* payload, uint8_t payloadLen) {
  if (gAtemState != AtemState::CONNECTED) return;

  uint16_t cmdLen = 8 + payloadLen;
  uint16_t packetLen = 12 + cmdLen;

  uint8_t packet[64];
  memset(packet, 0, sizeof(packet));

  // UDP Header
  packet[0] = 0x08 | (packetLen >> 8); // Reliable flag (0x08) + high length
  packet[1] = packetLen & 0xFF;
  packet[2] = gSessionID >> 8;
  packet[3] = gSessionID & 0xFF;
  packet[4] = gLastRemotePacketID >> 8;
  packet[5] = gLastRemotePacketID & 0xFF;
  packet[10] = gLocalPacketIdCounter >> 8;
  packet[11] = gLocalPacketIdCounter & 0xFF;

  // Command Header
  packet[12] = cmdLen >> 8;
  packet[13] = cmdLen & 0xFF;
  packet[16] = cmdName[0];
  packet[17] = cmdName[1];
  packet[18] = cmdName[2];
  packet[19] = cmdName[3];

  // Command Payload
  for (uint8_t i = 0; i < payloadLen; ++i) {
    packet[20 + i] = payload[i];
  }

  gLocalPacketIdCounter++;

  gUdp.beginPacket(kSwitcherIp, kSwitcherPort);
  gUdp.write(packet, packetLen);
  gUdp.endPacket();
}

void connectToAtem() {
  gLocalPacketIdCounter = 1;
  gLastConnectAttemptMs = millis();
  gWaitingForInitialDump = true;
  
  // Handshake SYN packet
  uint8_t connectHello[] = {
      0x10, 0x14, // SYN, Length 20
      0x53, 0xAB, // Client Session ID
      0x00, 0x00,
      0x00, 0x00,
      0x00, 0x3A, // Sequence indicator
      0x00, 0x00,
      0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00
  };

  gUdp.beginPacket(kSwitcherIp, kSwitcherPort);
  gUdp.write(connectHello, sizeof(connectHello));
  gUdp.endPacket();

  gAtemState = AtemState::CONNECTING;
  Serial.println("ATEM Handshake: Sending SYN...");
}

void parseAtemState(const uint8_t* packet, uint16_t packetLen) {
  uint16_t offset = 12; // Start after 12-byte header
  while (offset < packetLen) {
    uint16_t cmdLen = (packet[offset] << 8) | packet[offset + 1];
    if (cmdLen < 8 || offset + cmdLen > packetLen) break;

    char cmd[5] = {
        (char)packet[offset + 4],
        (char)packet[offset + 5],
        (char)packet[offset + 6],
        (char)packet[offset + 7],
        0
    };

    const uint8_t* data = packet + offset + 8;

    if (strcmp(cmd, "PrgI") == 0) {
      uint8_t me = data[0];
      uint16_t src = (data[2] << 8) | data[3];
      if (me == 0 && gActiveProgramSource != src) {
        gActiveProgramSource = src;
        Serial.printf("STATE: Program Input -> %u\r\n", src);
      }
    } else if (strcmp(cmd, "PrvI") == 0) {
      uint8_t me = data[0];
      uint16_t src = (data[2] << 8) | data[3];
      if (me == 0 && gActivePreviewSource != src) {
        gActivePreviewSource = src;
        Serial.printf("STATE: Preview Input -> %u\r\n", src);
      }
    } else if (strcmp(cmd, "DskS") == 0) {
      uint8_t idx = data[0];
      bool onAir = data[1] != 0;
      bool transitioning = data[2] != 0;
      if (idx < 2) {
        if (gDskOnAir[idx] != onAir || gDskTransitioning[idx] != transitioning) {
          gDskOnAir[idx] = onAir;
          gDskTransitioning[idx] = transitioning;
          Serial.printf("STATE: DSK%d -> OnAir: %d, Trans: %d\r\n", idx + 1, onAir, transitioning);
        }
      }
    } else if (strcmp(cmd, "DskP") == 0) {
      uint8_t idx = data[0];
      bool tie = data[1] != 0;
      if (idx < 2) {
        if (gDskTie[idx] != tie) {
          gDskTie[idx] = tie;
          Serial.printf("STATE: DSK%d Tie -> %d\r\n", idx + 1, tie);
        }
      }
    } else if (strcmp(cmd, "KeOn") == 0) {
      uint8_t me = data[0];
      uint8_t idx = data[1];
      bool onAir = data[2] != 0;
      if (me == 0 && idx == 0) {
        if (gKey1OnAir != onAir) {
          gKey1OnAir = onAir;
          Serial.printf("STATE: KEY1 OnAir -> %d\r\n", onAir);
        }
      }
    } else if (strcmp(cmd, "TrSS") == 0) {
      uint8_t me = data[0];
      uint8_t nextTr = data[2];
      if (me == 0) {
        bool bkgd = (nextTr & 0x01) != 0;
        bool key1 = (nextTr & 0x02) != 0;
        if (gNextTrBkgd != bkgd || gNextTrKey1 != key1) {
          gNextTrBkgd = bkgd;
          gNextTrKey1 = key1;
          Serial.printf("STATE: Next Transition -> BKGD: %d, KEY1: %d\r\n", bkgd, key1);
        }
      }
    } else if (strcmp(cmd, "FtbS") == 0) {
      uint8_t me = data[0];
      bool ftbDone = data[1] != 0;
      bool ftbActive = data[2] != 0;
      if (me == 0) {
        if (gFtbDone != ftbDone || gFtbActive != ftbActive) {
          gFtbDone = ftbDone;
          gFtbActive = ftbActive;
          Serial.printf("STATE: FTB -> Done: %d, Active: %d\r\n", ftbDone, ftbActive);
        }
      }
    } else if (strcmp(cmd, "TrIP") == 0) {
      uint8_t me = data[0];
      bool inProgress = data[1] != 0;
      if (me == 0) {
        if (gTransitionInProgress != inProgress) {
          gTransitionInProgress = inProgress;
          Serial.printf("STATE: Transition in Progress -> %d\r\n", inProgress);
        }
      }
    } else if (strcmp(cmd, "InCm") == 0) {
      Serial.println("ATEM: Initial configuration complete.");
    }

    offset += cmdLen;
  }
}

void updateAtemConnection() {
  uint32_t now = millis();

  // Disconnection and connection logic
  if (gAtemState == AtemState::DISCONNECTED) {
    if (now - gLastConnectAttemptMs > kAtemConnectRetryIntervalMs) {
      connectToAtem();
    }
    return;
  }

  if (gAtemState == AtemState::CONNECTING) {
    int packetSize = gUdp.parsePacket();
    if (packetSize == 20) {
      uint8_t buffer[20];
      gUdp.read(buffer, 20);

      // Check if it's SYN response (packetSize 20 and starts with 0x10)
      if ((buffer[0] & 0x10) != 0) {
        uint8_t status = buffer[12];
        if (status == 0x02) {
          gSessionID = (buffer[2] << 8) | buffer[3];
          gLastRemotePacketID = 0;
          sendAck(0); // Acknowledge SYN response

          gAtemState = AtemState::CONNECTED;
          gLastPacketReceivedMs = millis();
          Serial.printf("ATEM Connected: SessionID = 0x%04X\r\n", gSessionID);
        } else if (status == 0x04) {
          Serial.println("ATEM connection rejected (status 0x04). Retrying...");
          gAtemState = AtemState::DISCONNECTED;
        } else {
          Serial.printf("ATEM connection unexpected status 0x%02X. Retrying...\r\n", status);
          gAtemState = AtemState::DISCONNECTED;
        }
      }
    } else if (now - gLastConnectAttemptMs > kAtemConnectRetryIntervalMs) {
      Serial.println("ATEM connection handshake timeout. Retrying...");
      gAtemState = AtemState::DISCONNECTED;
    }
    return;
  }

  // Handle connected communication state
  if (gAtemState == AtemState::CONNECTED) {
    if (now - gLastPacketReceivedMs > kAtemTimeoutMs) {
      Serial.println("ATEM connection timed out. Reconnecting...");
      gAtemState = AtemState::DISCONNECTED;
      return;
    }

    int packetSize = gUdp.parsePacket();
    if (packetSize > 0) {
      gLastPacketReceivedMs = now;
      uint8_t buffer[1500];
      if (packetSize > (int)sizeof(buffer)) packetSize = sizeof(buffer);
      gUdp.read(buffer, packetSize);

      uint8_t flags = buffer[0] & 0xF8;
      uint16_t remotePacketId = (buffer[10] << 8) | buffer[11];
      gLastRemotePacketID = remotePacketId;

      // Check for SYN redirect/restart (0x10 and redirect payload)
      if (flags & 0x10) {
        // Redirection or reset from switcher
        gAtemState = AtemState::DISCONNECTED;
        return;
      }

      // If switcher expects ACK, acknowledge
      if (flags & 0x08) {
        if (gWaitingForInitialDump && packetSize == 12) {
          sendAck(remotePacketId, true); // Special ACK for first empty packet (0x61)
          gWaitingForInitialDump = false;
          Serial.println("ATEM: Initial dump complete. Ready.");
        } else {
          sendAck(remotePacketId, false); // Standard ACK (0x00)
        }
      }

      // Parse payload state fields
      if (packetSize > 12) {
        parseAtemState(buffer, packetSize);
      }
    }
  }
}

// Button Matrix scans and client commands dispatcher
void handleButtonPress(const ButtonMapping* button) {
  Serial.printf("BUTTON PRESS: %s\r\n", button->name);

  // Parse button name to index (1-8)
  uint8_t btnIdx = 0;
  if (button->name[3] >= '1' && button->name[3] <= '8') {
    btnIdx = button->name[3] - '0';
  } else if (button->name[8] >= '1' && button->name[8] <= '8') {
    btnIdx = button->name[8] - '0';
  }

  if (strncmp(button->name, "PGM", 3) == 0 && btnIdx > 0) {
    uint16_t src = gPgmShift ? kShiftSources[btnIdx - 1] : btnIdx;
    uint8_t payload[4] = { 0x00, 0x00, (uint8_t)(src >> 8), (uint8_t)(src & 0xFF) };
    sendCommand("CPgI", payload, 4);
    Serial.printf("Sent CPgI program source %d\r\n", src);
  } else if (strncmp(button->name, "Preview", 7) == 0 && btnIdx > 0) {
    uint16_t src = gPrvShift ? kShiftSources[btnIdx - 1] : btnIdx;
    uint8_t payload[4] = { 0x00, 0x00, (uint8_t)(src >> 8), (uint8_t)(src & 0xFF) };
    sendCommand("CPvI", payload, 4);
    Serial.printf("Sent CPvI preview source %d\r\n", src);
  } else if (strcmp(button->name, "PGM Shift") == 0) {
    gPgmShift = !gPgmShift;
    Serial.printf("PGM Shift toggle -> %s\r\n", gPgmShift ? "ON" : "OFF");
  } else if (strcmp(button->name, "Preview Shift") == 0) {
    gPrvShift = !gPrvShift;
    Serial.printf("Preview Shift toggle -> %s\r\n", gPrvShift ? "ON" : "OFF");
  } else if (strcmp(button->name, "CUT") == 0) {
    uint8_t payload[4] = { 0x00, 0xEF, 0xBF, 0x5F };
    sendCommand("DCut", payload, 4);
    Serial.println("Sent DCut");
  } else if (strcmp(button->name, "AUTO") == 0) {
    uint8_t payload[4] = { 0x00, 0x32, 0x16, 0x02 };
    sendCommand("DAut", payload, 4);
    Serial.println("Sent DAut");
  } else if (strcmp(button->name, "KEY1 CUT") == 0) {
    bool newState = !gKey1OnAir;
    uint8_t payload[4] = { 0x00, 0x00, (uint8_t)(newState ? 1 : 0), 0x00 };
    sendCommand("CKOn", payload, 4);
    Serial.printf("Sent CKOn state %d\r\n", newState);
  } else if (strcmp(button->name, "KEY1 TIE") == 0) {
    uint8_t nextTr = (gNextTrBkgd ? 1 : 0) | (gNextTrKey1 ? 2 : 0);
    nextTr ^= 2; // Toggle Key 1
    uint8_t payload[4] = { 0x02, 0x00, 0x00, nextTr }; // Next transition selection change mask = 0x02
    sendCommand("CTTp", payload, 4);
    Serial.printf("Sent CTTp selection %d\r\n", nextTr);
  } else if (strcmp(button->name, "BKGD") == 0) {
    uint8_t nextTr = (gNextTrBkgd ? 1 : 0) | (gNextTrKey1 ? 2 : 0);
    nextTr ^= 1; // Toggle BKGD
    uint8_t payload[4] = { 0x02, 0x00, 0x00, nextTr };
    sendCommand("CTTp", payload, 4);
    Serial.printf("Sent CTTp selection %d\r\n", nextTr);
  } else if (strcmp(button->name, "DSK1 CUT") == 0) {
    bool newState = !gDskOnAir[0];
    uint8_t payload[4] = { 0x00, (uint8_t)(newState ? 1 : 0), 0x00, 0x00 };
    sendCommand("CDsO", payload, 4);
    Serial.printf("Sent CDsO DSK1 state %d\r\n", newState);
  } else if (strcmp(button->name, "DSK2 CUT") == 0) {
    bool newState = !gDskOnAir[1];
    uint8_t payload[4] = { 0x01, (uint8_t)(newState ? 1 : 0), 0x00, 0x00 };
    sendCommand("CDsO", payload, 4);
    Serial.printf("Sent CDsO DSK2 state %d\r\n", newState);
  } else if (strcmp(button->name, "DSK1 TIE") == 0) {
    bool newState = !gDskTie[0];
    uint8_t payload[4] = { 0x00, (uint8_t)(newState ? 1 : 0), 0x00, 0x00 };
    sendCommand("CDsT", payload, 4);
    Serial.printf("Sent CDsT DSK1 state %d\r\n", newState);
  } else if (strcmp(button->name, "DSK2 TIE") == 0) {
    bool newState = !gDskTie[1];
    uint8_t payload[4] = { 0x01, (uint8_t)(newState ? 1 : 0), 0x00, 0x00 };
    sendCommand("CDsT", payload, 4);
    Serial.printf("Sent CDsT DSK2 state %d\r\n", newState);
  } else if (strcmp(button->name, "DSK1 AUTO") == 0) {
    uint8_t payload[4] = { 0x00, 0x00, 0x00, 0x00 };
    sendCommand("DDsA", payload, 4);
    Serial.println("Sent DDsA DSK1 Auto");
  } else if (strcmp(button->name, "DSK2 AUTO") == 0) {
    uint8_t payload[4] = { 0x01, 0x00, 0x00, 0x00 };
    sendCommand("DDsA", payload, 4);
    Serial.println("Sent DDsA DSK2 Auto");
  } else if (strcmp(button->name, "FTB") == 0) {
    uint8_t payload[4] = { 0x00, 0x02, 0x58, 0x99 };
    sendCommand("FtbA", payload, 4);
    Serial.println("Sent FtbA");
  }
}

void handleButtonRelease(const ButtonMapping* button) {
  Serial.printf("BUTTON RELEASE: %s\r\n", button->name);
}

void writeMcpOutputs(uint16_t outputState) {
  gMcpOutputState = outputState & kRowMask;
  gMcp.write16(gMcpOutputState);
}

void scanMatrix() {
  const uint32_t now = millis();

  for (size_t row = 0; row < kRowCount; ++row) {
    writeMcpOutputs(bitFor(kMcpRowPins[row]));
    delayMicroseconds(50);
    const uint16_t inputs = gMcp.read16();

    for (size_t col = 0; col < kColCount; ++col) {
      const bool rawPressed = (inputs & bitFor(kMcpColPins[col])) != 0;

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
            handleButtonPress(button);
          } else {
            handleButtonRelease(button);
          }
        }
      }
    }

    writeMcpOutputs(0);
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

// LED update mapping logic
void updateLeds() {
  static uint32_t lastLedUpdateMs = 0;
  const uint32_t now = millis();

  if (now - lastLedUpdateMs < kLedUpdateIntervalMs) return;
  lastLedUpdateMs = now;

  bool blinkState = (now / 250) % 2 == 0;

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

    // Program inputs row
    if (strncmp(led.name, "PGM", 3) == 0 && btnIdx > 0) {
      uint16_t src = gPgmShift ? kShiftSources[btnIdx - 1] : btnIdx;
      if (gActiveProgramSource == src) {
        r = kLedBrightness; g = 0; b = 0; // Solid Red
      } else if (gActivePreviewSource == src) {
        r = 0; g = kLedBrightness; b = 0; // Solid Green (showing selected preview on PGM button)
      }
    }
    // Preview inputs row
    else if (strncmp(led.name, "PRV", 3) == 0 && btnIdx > 0) {
      uint16_t src = gPrvShift ? kShiftSources[btnIdx - 1] : btnIdx;
      if (gActivePreviewSource == src) {
        r = 0; g = kLedBrightness; b = 0; // Solid Green
      }
    }
    // Shift buttons
    else if (led.id == LedId::PGM_SHIFT) {
      if (gPgmShift) {
        r = kLedBrightness; g = kLedBrightness; b = 0; // Yellow
      }
    } else if (led.id == LedId::PRV_SHIFT) {
      if (gPrvShift) {
        r = 0; g = kLedBrightness; b = kLedBrightness; // Cyan
      }
    }
    // Next Transition
    else if (led.id == LedId::BKGD) {
      if (gNextTrBkgd) {
        r = 0; g = 0; b = kLedBrightness; // Solid Blue
      }
    } else if (led.id == LedId::KEY1_TIE) {
      if (gNextTrKey1) {
        r = 0; g = 0; b = kLedBrightness; // Solid Blue
      }
    }
    // Upstream Keyer On Air
    else if (led.id == LedId::KEY1_CUT) {
      if (gKey1OnAir) {
        r = kLedBrightness; g = 0; b = 0; // Red
      }
    }
    // DSK Tie
    else if (led.id == LedId::DSK1_TIE) {
      if (gDskTie[0]) {
        r = 0; g = 0; b = kLedBrightness; // Blue
      }
    } else if (led.id == LedId::DSK2_TIE) {
      if (gDskTie[1]) {
        r = 0; g = 0; b = kLedBrightness; // Blue
      }
    }
    // DSK Cut / On Air
    else if (led.id == LedId::DSK1_CUT) {
      if (gDskOnAir[0]) {
        r = kLedBrightness; g = 0; b = 0; // Red
      }
    } else if (led.id == LedId::DSK2_CUT) {
      if (gDskOnAir[1]) {
        r = kLedBrightness; g = 0; b = 0; // Red
      }
    }
    // DSK Auto (Blinks when active)
    else if (led.id == LedId::DSK1_AUTO) {
      if (gDskTransitioning[0]) {
        if (blinkState) {
          r = kLedBrightness; g = 0; b = 0; // Blinking Red
        } else {
          r = 0; g = 0; b = 0;
        }
      }
    } else if (led.id == LedId::DSK2_AUTO) {
      if (gDskTransitioning[1]) {
        if (blinkState) {
          r = kLedBrightness; g = 0; b = 0; // Blinking Red
        } else {
          r = 0; g = 0; b = 0;
        }
      }
    }
    // Fade to Black
    else if (led.id == LedId::FTB) {
      if (gFtbActive) {
        if (blinkState) {
          r = kLedBrightness; g = 0; b = 0; // Blinking Red
        } else {
          r = 0; g = 0; b = 0;
        }
      } else if (gFtbDone) {
        r = kLedBrightness; g = 0; b = 0; // Solid Red
      }
    }
    // Auto transition (Blinks when transition is running)
    else if (led.id == LedId::AUTO) {
      if (gTransitionInProgress) {
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

}  // namespace

void setup() {
  Serial.begin(kBaudRate);
  delay(300);

  setupTlc();
  setupMatrix();
  printHeader();

  // Reset W5500 SPI Ethernet
  pinMode(kEthRstPin, OUTPUT);
  digitalWrite(kEthRstPin, LOW);
  delay(10);
  digitalWrite(kEthRstPin, HIGH);
  delay(100);

  // Initialize Ethernet SPI
  SPI.begin(kEthSclkPin, kEthMisoPin, kEthMosiPin, kEthCsPin);
  Ethernet.init(kEthCsPin);

  // Initialize static Ethernet config
  // Ethernet.begin(mac, ip, dns, gateway, subnet)
  uint8_t mac[] = { 0xDE, 0xAD, 0xBE, 0xEF, 0xFE, 0xED };
  Ethernet.begin(mac, kLocalIp, kGateway, kGateway, kSubnet);
  
  if (Ethernet.linkStatus() == LinkOFF) {
    Serial.println("WARNING: Ethernet cable is disconnected!");
  } else {
    Serial.println("Ethernet Link Connected.");
  }

  // Open UDP listener on client port
  gUdp.begin(kLocalPort);

  // Establish handshake
  connectToAtem();
}

void loop() {
  updateAtemConnection();
  updateMatrixScanner();
  updateLeds();
}
