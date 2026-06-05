#pragma once

#include <Arduino.h>

constexpr uint16_t kUnknownAtemSource = 0xFFFF;
constexpr uint16_t kAtemTransitionPositionMax = 10000;
constexpr size_t kAtemPanelSourceButtonCount = 8;
constexpr size_t kAtemCommandPayloadMax = 32;
constexpr size_t kAtemCommandQueueSize = 16;

enum class AtemModelProfileId : uint8_t {
  Unknown,
  MiniPro,
  ConstellationHD1ME,
};

enum class AtemSource : uint16_t {
  Black = 0,
  Input1 = 1,
  Input2 = 2,
  Input3 = 3,
  Input4 = 4,
  Input5 = 5,
  Input6 = 6,
  Input7 = 7,
  Input8 = 8,
  Input9 = 9,
  Input10 = 10,
  ColorBars = 1000,
  ColorGenerator1 = 2001,
  ColorGenerator2 = 2002,
  MediaPlayer1 = 3010,
  MediaPlayer2 = 3020,
  Program = 10010,
  None = 0xFFFF
};

struct AtemModelProfile {
  AtemModelProfileId id;
  const char* name;
  uint8_t mixEffects;
  uint8_t upstreamKeyers;
  uint8_t downstreamKeyers;
  AtemSource primarySources[kAtemPanelSourceButtonCount];
  AtemSource shiftedSources[kAtemPanelSourceButtonCount];
};

enum class AtemConnectionState : uint8_t {
  DISCONNECTED,
  CONNECTING,
  CONNECTED,
};

struct AtemSwitcherState {
  uint16_t programSource = kUnknownAtemSource;
  uint16_t previewSource = kUnknownAtemSource;
  bool key1OnAir = false;
  bool nextTrBkgd = false;
  bool nextTrKey1 = false;
  bool dskOnAir[2] = {false, false};
  bool dskTie[2] = {false, false};
  bool dskTransitioning[2] = {false, false};
  bool ftbActive = false;
  bool ftbDone = false;
  bool transitionInProgress = false;
  uint16_t transitionPosition = 0;
  uint16_t faderStartPosition = 0;
  uint16_t faderLedPosition = 0;
  bool virtualFaderAtTop = false;
  bool transitionStartedAtTop = false;
  bool faderTransitionActive = false;
};

struct QueuedAtemCommand {
  char name[4];
  uint8_t payload[kAtemCommandPayloadMax];
  uint8_t payloadLen;
};
