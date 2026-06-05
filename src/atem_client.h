#pragma once

#include <Arduino.h>
#include <Ethernet.h>

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

struct AtemModelProfile {
  AtemModelProfileId id;
  const char* name;
  uint8_t mixEffects;
  uint8_t upstreamKeyers;
  uint8_t downstreamKeyers;
  uint16_t primarySources[kAtemPanelSourceButtonCount];
  uint16_t shiftedSources[kAtemPanelSourceButtonCount];
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
  uint16_t faderStartPosition = kAtemTransitionPositionMax;
  uint16_t faderLedPosition = kAtemTransitionPositionMax;
  bool virtualFaderAtTop = true;
  bool transitionStartedAtTop = true;
  bool faderTransitionActive = false;
};

struct QueuedAtemCommand {
  char name[4];
  uint8_t payload[kAtemCommandPayloadMax];
  uint8_t payloadLen;
};

class AtemClient {
 public:
  AtemClient(IPAddress switcherIp, uint16_t switcherPort, uint16_t localPort);

  bool begin();
  void connect();
  void resetConnection();
  void update();
  bool sendCommand(const char* cmdName, const uint8_t* payload, uint8_t payloadLen);

  AtemConnectionState connectionState() const { return connectionState_; }
  bool connected() const { return connectionState_ == AtemConnectionState::CONNECTED; }
  const AtemSwitcherState& state() const { return switcherState_; }
  const AtemModelProfile& profile() const { return *profile_; }
  uint16_t sourceForButton(uint8_t buttonIndex, bool shifted) const;
  bool supportsDownstreamKeyer(uint8_t keyerIndex) const;

 private:
  void resetReceivedRemotePackets();
  bool hasReceivedRemotePacket(uint16_t remotePacketId) const;
  void rememberReceivedRemotePacket(uint16_t remotePacketId);
  void drainUdpReceiveQueue();
  void sendAck(uint16_t remotePacketId, uint16_t remoteSequenceField = 0);
  void sendTimeRequest();
  bool enqueueCommand(const char* cmdName, const uint8_t* payload, uint8_t payloadLen);
  bool sendCommandNow(const char* cmdName, const uint8_t* payload, uint8_t payloadLen);
  void processQueuedCommands();
  bool hasValidCommandPayload(const uint8_t* packet, uint16_t packetLen) const;
  bool hasValidHeaderLength(const uint8_t* packet, uint16_t packetSize) const;
  void parseState(const uint8_t* packet, uint16_t packetLen);
  void setProfileForProductIdentifier(const uint8_t* data, uint16_t dataLen);
  void updateVirtualFaderPosition(uint16_t transitionPosition);

  IPAddress switcherIp_;
  uint16_t switcherPort_;
  uint16_t localPort_;
  EthernetUDP udp_;
  AtemConnectionState connectionState_ = AtemConnectionState::DISCONNECTED;
  AtemSwitcherState switcherState_;
  const AtemModelProfile* profile_;
  uint16_t sessionId_ = 0;
  uint16_t lastRemotePacketId_ = 0;
  uint16_t localPacketIdCounter_ = 1;
  uint16_t receivedRemotePacketIds_[1024] = {};
  size_t receivedRemotePacketCount_ = 0;
  bool awaitingAssignedSessionId_ = false;
  uint32_t lastPacketReceivedMs_ = 0;
  uint32_t lastConnectAttemptMs_ = 0;
  uint32_t lastPacketSentMs_ = 0;
  uint32_t lastIgnoredSessionLogMs_ = 0;
  uint16_t ignoredSessionPacketCount_ = 0;
  bool waitingForInitialDump_ = true;
  bool initialSyncComplete_ = false;
  bool initialStateSeen_ = false;
  bool initialEmptyAckSent_ = false;
  bool initialSessionRecoveryUsed_ = false;
  bool markNextConnected_ = false;
  uint16_t initialSyncCommandCount_ = 0;
  bool initialSyncSawProgram_ = false;
  bool initialSyncSawPreview_ = false;
  portMUX_TYPE commandQueueMux_ = portMUX_INITIALIZER_UNLOCKED;
  QueuedAtemCommand commandQueue_[kAtemCommandQueueSize] = {};
  size_t commandQueueHead_ = 0;
  size_t commandQueueTail_ = 0;
  size_t commandQueueCount_ = 0;
};
