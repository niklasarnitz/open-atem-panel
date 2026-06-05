#include "atem_client.h"

#include <ctype.h>

namespace {

constexpr uint32_t kAtemTimeoutMs = 5000;
constexpr uint32_t kAtemConnectRetryIntervalMs = 2000;
constexpr uint16_t kAtemMaxPacketId = 32768;
constexpr uint8_t kAtemFlagReliable = 0x08;
constexpr uint8_t kAtemFlagSyn = 0x10;
constexpr uint8_t kAtemFlagRetransmission = 0x20;
constexpr uint8_t kAtemFlagRequestRetransmission = 0x40;
constexpr uint8_t kAtemFlagAck = 0x80;
constexpr uint16_t kAtemHandshakeSessionId = 0x53AB;
constexpr size_t kAtemMaxPacketSize = 2048;
constexpr bool kAtemVerbosePackets = false;
constexpr bool kAtemLogAcks = false;

const AtemModelProfile kUnknownProfile = {
    AtemModelProfileId::Unknown,
    "Unknown ATEM",
    1,
    1,
    1,
    {1, 2, 3, 4, 5, 6, 7, 8},
    {9, 10, 3010, 3020, 2001, 2002, 10010, 0},
};

const AtemModelProfile kMiniProProfile = {
    AtemModelProfileId::MiniPro,
    "ATEM Mini Pro",
    1,
    1,
    1,
    {1, 2, 3, 4, 3010, 3020, 10010, 0},
    {1, 2, 3, 4, 2001, 2002, 10010, 0},
};

const AtemModelProfile kConstellationHD1MEProfile = {
    AtemModelProfileId::ConstellationHD1ME,
    "1 M/E Constellation HD",
    1,
    4,
    1,
    {1, 2, 3, 4, 5, 6, 7, 8},
    {9, 10, 3010, 3020, 2001, 2002, 10010, 0},
};

bool containsIgnoreCase(const char* haystack, const char* needle) {
  if (*needle == '\0') return true;

  for (size_t i = 0; haystack[i] != '\0'; ++i) {
    size_t j = 0;
    while (needle[j] != '\0' && haystack[i + j] != '\0' &&
           tolower((unsigned char)haystack[i + j]) == tolower((unsigned char)needle[j])) {
      ++j;
    }
    if (needle[j] == '\0') return true;
  }

  return false;
}

}  // namespace

AtemClient::AtemClient(IPAddress switcherIp, uint16_t switcherPort, uint16_t localPort)
    : switcherIp_(switcherIp), switcherPort_(switcherPort), localPort_(localPort), profile_(&kUnknownProfile) {}

uint16_t AtemClient::sourceForButton(uint8_t buttonIndex, bool shifted) const {
  if (buttonIndex >= kAtemPanelSourceButtonCount) return kUnknownAtemSource;
  return shifted ? profile_->shiftedSources[buttonIndex] : profile_->primarySources[buttonIndex];
}

bool AtemClient::supportsDownstreamKeyer(uint8_t keyerIndex) const {
  return keyerIndex < profile_->downstreamKeyers;
}

bool AtemClient::begin() {
  return udp_.begin(localPort_) != 0;
}

void AtemClient::resetReceivedRemotePackets() {
  memset(receivedRemotePacketIds_, 0, sizeof(receivedRemotePacketIds_));
  receivedRemotePacketCount_ = 0;
}

bool AtemClient::hasReceivedRemotePacket(uint16_t remotePacketId) const {
  for (size_t i = 0; i < receivedRemotePacketCount_; ++i) {
    if (receivedRemotePacketIds_[i] == remotePacketId) {
      return true;
    }
  }
  return false;
}

void AtemClient::rememberReceivedRemotePacket(uint16_t remotePacketId) {
  if (hasReceivedRemotePacket(remotePacketId)) return;

  if (receivedRemotePacketCount_ < 1024) {
    receivedRemotePacketIds_[receivedRemotePacketCount_++] = remotePacketId;
    return;
  }

  memmove(receivedRemotePacketIds_,
          receivedRemotePacketIds_ + 1,
          1023 * sizeof(receivedRemotePacketIds_[0]));
  receivedRemotePacketIds_[1023] = remotePacketId;
}

void AtemClient::drainUdpReceiveQueue() {
  uint8_t discard[64];
  uint16_t drained = 0;

  int packetSize;
  while ((packetSize = udp_.parsePacket()) > 0) {
    while (packetSize > 0) {
      const size_t chunk = min((size_t)packetSize, sizeof(discard));
      udp_.read(discard, chunk);
      packetSize -= chunk;
    }
    ++drained;
  }

  if (drained > 0) {
    Serial.printf("ATEM: Drained %u stale UDP packets before handshake.\r\n", drained);
  }
}

void AtemClient::sendAck(uint16_t remotePacketId, uint16_t remoteSequenceField) {
  uint8_t ack[12];
  memset(ack, 0, 12);
  ack[0] = kAtemFlagAck;
  ack[1] = 0x0C;
  ack[2] = sessionId_ >> 8;
  ack[3] = sessionId_ & 0xFF;
  ack[4] = remotePacketId >> 8;
  ack[5] = remotePacketId & 0xFF;
  ack[8] = remoteSequenceField >> 8;
  ack[9] = remoteSequenceField & 0xFF;

  udp_.beginPacket(switcherIp_, switcherPort_);
  udp_.write(ack, 12);
  udp_.endPacket();
  lastPacketSentMs_ = millis();
  if (kAtemLogAcks) {
    Serial.printf("ATEM: Tx ACK for PacketID %u\r\n", remotePacketId);
  }
}

bool AtemClient::sendCommand(const char* cmdName, const uint8_t* payload, uint8_t payloadLen) {
  if (connectionState_ != AtemConnectionState::CONNECTED) return false;
  return enqueueCommand(cmdName, payload, payloadLen);
}

bool AtemClient::enqueueCommand(const char* cmdName, const uint8_t* payload, uint8_t payloadLen) {
  if (payloadLen > kAtemCommandPayloadMax) return false;

  portENTER_CRITICAL(&commandQueueMux_);
  if (commandQueueCount_ >= kAtemCommandQueueSize) {
    portEXIT_CRITICAL(&commandQueueMux_);
    Serial.printf("ATEM: Command queue full, dropping %.4s\r\n", cmdName);
    return false;
  }

  QueuedAtemCommand& command = commandQueue_[commandQueueTail_];
  memcpy(command.name, cmdName, sizeof(command.name));
  command.payloadLen = payloadLen;
  if (payloadLen > 0 && payload != nullptr) {
    memcpy(command.payload, payload, payloadLen);
  }
  commandQueueTail_ = (commandQueueTail_ + 1) % kAtemCommandQueueSize;
  ++commandQueueCount_;
  portEXIT_CRITICAL(&commandQueueMux_);
  return true;
}

bool AtemClient::sendCommandNow(const char* cmdName, const uint8_t* payload, uint8_t payloadLen) {
  if (connectionState_ != AtemConnectionState::CONNECTED) return false;

  uint16_t cmdLen = 8 + payloadLen;
  uint16_t packetLen = 12 + cmdLen;

  uint8_t packet[64];
  memset(packet, 0, sizeof(packet));

  packet[0] = kAtemFlagReliable | (packetLen >> 8);
  packet[1] = packetLen & 0xFF;
  packet[2] = sessionId_ >> 8;
  packet[3] = sessionId_ & 0xFF;
  packet[10] = localPacketIdCounter_ >> 8;
  packet[11] = localPacketIdCounter_ & 0xFF;

  packet[12] = cmdLen >> 8;
  packet[13] = cmdLen & 0xFF;
  packet[16] = cmdName[0];
  packet[17] = cmdName[1];
  packet[18] = cmdName[2];
  packet[19] = cmdName[3];

  for (uint8_t i = 0; i < payloadLen; ++i) {
    packet[20 + i] = payload[i];
  }

  localPacketIdCounter_ = (localPacketIdCounter_ + 1) % kAtemMaxPacketId;
  if (localPacketIdCounter_ == 0) {
    localPacketIdCounter_ = 1;
  }

  udp_.beginPacket(switcherIp_, switcherPort_);
  udp_.write(packet, packetLen);
  udp_.endPacket();
  lastPacketSentMs_ = millis();
  return true;
}

void AtemClient::processQueuedCommands() {
  while (connectionState_ == AtemConnectionState::CONNECTED) {
    QueuedAtemCommand command;

    portENTER_CRITICAL(&commandQueueMux_);
    if (commandQueueCount_ == 0) {
      portEXIT_CRITICAL(&commandQueueMux_);
      return;
    }
    command = commandQueue_[commandQueueHead_];
    commandQueueHead_ = (commandQueueHead_ + 1) % kAtemCommandQueueSize;
    --commandQueueCount_;
    portEXIT_CRITICAL(&commandQueueMux_);

    sendCommandNow(command.name, command.payload, command.payloadLen);
  }
}

void AtemClient::sendTimeRequest() {
  if (sendCommandNow("TiRq", nullptr, 0) && kAtemVerbosePackets) {
    Serial.println("ATEM: Sent TiRq");
  }
}

bool AtemClient::hasValidCommandPayload(const uint8_t* packet, uint16_t packetLen) const {
  if (packetLen < 20) return false;

  uint16_t offset = 12;
  while (offset < packetLen) {
    if (offset + 8 > packetLen) return false;

    uint16_t cmdLen = (packet[offset] << 8) | packet[offset + 1];
    if (cmdLen < 8 || offset + cmdLen > packetLen) return false;

    for (uint8_t i = 0; i < 4; ++i) {
      char c = (char)packet[offset + 4 + i];
      if (c < 0x20 || c > 0x7E) return false;
    }

    offset += cmdLen;
  }

  return offset == packetLen;
}

bool AtemClient::hasValidHeaderLength(const uint8_t* packet, uint16_t packetSize) const {
  if (packetSize < 12) return false;

  const uint16_t headerWord = (static_cast<uint16_t>(packet[0]) << 8) | packet[1];
  const uint16_t declaredLen = headerWord & 0x07FF;
  return declaredLen == packetSize;
}

void AtemClient::setProfileForProductIdentifier(const uint8_t* data, uint16_t dataLen) {
  char productIdentifier[64];
  size_t copyLen = min((size_t)dataLen, sizeof(productIdentifier) - 1);
  memcpy(productIdentifier, data, copyLen);
  productIdentifier[copyLen] = '\0';

  const AtemModelProfile* newProfile = &kUnknownProfile;
  if (containsIgnoreCase(productIdentifier, "mini pro")) {
    newProfile = &kMiniProProfile;
  } else if (containsIgnoreCase(productIdentifier, "constellation") &&
             containsIgnoreCase(productIdentifier, "1 m/e") &&
             containsIgnoreCase(productIdentifier, "hd")) {
    newProfile = &kConstellationHD1MEProfile;
  }

  if (profile_ != newProfile) {
    profile_ = newProfile;
    Serial.printf("ATEM: Model profile -> %s (reported: %s)\r\n", profile_->name, productIdentifier);
  } else if (profile_->id == AtemModelProfileId::Unknown) {
    Serial.printf("ATEM: Unknown model profile (reported: %s), using generic 1 M/E layout\r\n", productIdentifier);
  }
}

void AtemClient::connect() {
  localPacketIdCounter_ = 1;
  lastRemotePacketId_ = 0;
  sessionId_ = kAtemHandshakeSessionId;
  awaitingAssignedSessionId_ = false;
  resetReceivedRemotePackets();
  lastConnectAttemptMs_ = millis();
  waitingForInitialDump_ = true;
  initialSyncComplete_ = false;
  initialStateSeen_ = false;
  initialEmptyAckSent_ = false;
  initialSessionRecoveryUsed_ = false;
  markNextConnected_ = false;
  initialSyncCommandCount_ = 0;
  initialSyncSawProgram_ = false;
  initialSyncSawPreview_ = false;

  drainUdpReceiveQueue();

  uint8_t connectHello[] = {
      0x10, 0x14,
      (uint8_t)(kAtemHandshakeSessionId >> 8), (uint8_t)(kAtemHandshakeSessionId & 0xFF),
      0x00, 0x00,
      0x00, 0x00,
      0x00, 0x3A,
      0x00, 0x00,
      0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00
  };

  udp_.beginPacket(switcherIp_, switcherPort_);
  udp_.write(connectHello, sizeof(connectHello));
  udp_.endPacket();
  lastPacketSentMs_ = millis();

  connectionState_ = AtemConnectionState::CONNECTING;
  Serial.println("ATEM Handshake: Sending SYN...");
}

void AtemClient::resetConnection() {
  if (connectionState_ != AtemConnectionState::DISCONNECTED) {
    Serial.println("ATEM: Network link down, pausing connection attempts.");
  }
  connectionState_ = AtemConnectionState::DISCONNECTED;
  lastPacketReceivedMs_ = 0;
  lastConnectAttemptMs_ = millis();
  awaitingAssignedSessionId_ = false;
  lastIgnoredSessionLogMs_ = 0;
  ignoredSessionPacketCount_ = 0;
  waitingForInitialDump_ = true;
  initialSyncComplete_ = false;
  initialStateSeen_ = false;
  initialEmptyAckSent_ = false;
  initialSessionRecoveryUsed_ = false;
  markNextConnected_ = false;
  resetReceivedRemotePackets();
}

void AtemClient::updateVirtualFaderPosition(uint16_t transitionPosition) {
  if (!switcherState_.faderTransitionActive) {
    if (transitionPosition == 0 || transitionPosition == kAtemTransitionPositionMax) {
      switcherState_.faderLedPosition = switcherState_.virtualFaderAtTop ? kAtemTransitionPositionMax : 0;
      switcherState_.faderStartPosition = switcherState_.faderLedPosition;
      switcherState_.transitionInProgress = false;
      return;
    }

    switcherState_.faderTransitionActive = true;
    switcherState_.transitionInProgress = true;
    switcherState_.transitionStartedAtTop = switcherState_.virtualFaderAtTop;
    switcherState_.faderStartPosition = switcherState_.virtualFaderAtTop ? kAtemTransitionPositionMax : 0;
  }

  if (transitionPosition == 0) {
    switcherState_.faderTransitionActive = false;
    switcherState_.transitionInProgress = false;
    switcherState_.virtualFaderAtTop = !switcherState_.transitionStartedAtTop;
    switcherState_.faderLedPosition = switcherState_.virtualFaderAtTop ? kAtemTransitionPositionMax : 0;
    switcherState_.faderStartPosition = switcherState_.faderLedPosition;
    return;
  }

  switcherState_.transitionInProgress = true;
  switcherState_.faderLedPosition = switcherState_.transitionStartedAtTop
                                        ? kAtemTransitionPositionMax - transitionPosition
                                        : transitionPosition;
}

void AtemClient::parseState(const uint8_t* packet, uint16_t packetLen) {
  uint16_t offset = 12;

  while (offset < packetLen) {
    uint16_t cmdLen = (packet[offset] << 8) | packet[offset + 1];
    if (cmdLen < 8 || offset + cmdLen > packetLen) {
      Serial.printf("ATEM: Invalid command segment at offset %u, len %u, packet %u\r\n",
                    offset,
                    cmdLen,
                    packetLen);
      break;
    }

    char cmd[5] = {
        (char)packet[offset + 4],
        (char)packet[offset + 5],
        (char)packet[offset + 6],
        (char)packet[offset + 7],
        0
    };

    const uint8_t* data = packet + offset + 8;
    const uint16_t dataLen = cmdLen - 8;
    ++initialSyncCommandCount_;

    if (strcmp(cmd, "_pin") == 0) {
      setProfileForProductIdentifier(data, dataLen);
    } else if (strcmp(cmd, "PrgI") == 0) {
      uint8_t me = data[0];
      uint16_t src = (data[2] << 8) | data[3];
      initialSyncSawProgram_ = true;
      if (me == 0 && switcherState_.programSource != src) {
        switcherState_.programSource = src;
        Serial.printf("STATE: Program Input -> %u\r\n", src);
      }
    } else if (strcmp(cmd, "PrvI") == 0) {
      uint8_t me = data[0];
      uint16_t src = (data[2] << 8) | data[3];
      initialSyncSawPreview_ = true;
      if (me == 0 && switcherState_.previewSource != src) {
        switcherState_.previewSource = src;
        Serial.printf("STATE: Preview Input -> %u\r\n", src);
      }
    } else if (strcmp(cmd, "DskS") == 0) {
      uint8_t idx = data[0];
      bool onAir = data[1] != 0;
      bool transitioning = data[2] != 0;
      if (idx < 2) {
        if (switcherState_.dskOnAir[idx] != onAir || switcherState_.dskTransitioning[idx] != transitioning) {
          switcherState_.dskOnAir[idx] = onAir;
          switcherState_.dskTransitioning[idx] = transitioning;
          Serial.printf("STATE: DSK%d -> OnAir: %d, Trans: %d\r\n", idx + 1, onAir, transitioning);
        }
      }
    } else if (strcmp(cmd, "DskP") == 0) {
      uint8_t idx = data[0];
      bool tie = data[1] != 0;
      if (idx < 2) {
        if (switcherState_.dskTie[idx] != tie) {
          switcherState_.dskTie[idx] = tie;
          Serial.printf("STATE: DSK%d Tie -> %d\r\n", idx + 1, tie);
        }
      }
    } else if (strcmp(cmd, "KeOn") == 0) {
      uint8_t me = data[0];
      uint8_t idx = data[1];
      bool onAir = data[2] != 0;
      if (me == 0 && idx == 0) {
        if (switcherState_.key1OnAir != onAir) {
          switcherState_.key1OnAir = onAir;
          Serial.printf("STATE: KEY1 OnAir -> %d\r\n", onAir);
        }
      }
    } else if (strcmp(cmd, "TrSS") == 0) {
      uint8_t me = data[0];
      uint8_t nextTr = data[2];
      if (me == 0) {
        bool bkgd = (nextTr & 0x01) != 0;
        bool key1 = (nextTr & 0x02) != 0;
        if (switcherState_.nextTrBkgd != bkgd || switcherState_.nextTrKey1 != key1) {
          switcherState_.nextTrBkgd = bkgd;
          switcherState_.nextTrKey1 = key1;
          Serial.printf("STATE: Next Transition -> BKGD: %d, KEY1: %d\r\n", bkgd, key1);
        }
      }
    } else if (strcmp(cmd, "FtbS") == 0) {
      uint8_t me = data[0];
      bool ftbDone = data[1] != 0;
      bool ftbActive = data[2] != 0;
      if (me == 0) {
        if (switcherState_.ftbDone != ftbDone || switcherState_.ftbActive != ftbActive) {
          switcherState_.ftbDone = ftbDone;
          switcherState_.ftbActive = ftbActive;
          Serial.printf("STATE: FTB -> Done: %d, Active: %d\r\n", ftbDone, ftbActive);
        }
      }
    } else if (strcmp(cmd, "TrPs") == 0) {
      uint8_t me = data[0];
      bool inProgress = data[1] != 0;
      uint16_t position = (data[4] << 8) | data[5];
      if (position > kAtemTransitionPositionMax) {
        position = kAtemTransitionPositionMax;
      }
      if (me == 0) {
        if (switcherState_.transitionInProgress != inProgress || switcherState_.transitionPosition != position) {
          switcherState_.transitionPosition = position;
          updateVirtualFaderPosition(position);
          Serial.printf("STATE: Transition -> InProgress: %d, Position: %u, Fader: %s, FaderActive: %d\r\n",
                        inProgress,
                        position,
                        switcherState_.virtualFaderAtTop ? "top" : "bottom",
                        switcherState_.faderTransitionActive);
        }
      }
    } else if (strcmp(cmd, "InCm") == 0) {
      initialSyncComplete_ = true;
      initialStateSeen_ = true;
      markNextConnected_ = true;
      Serial.println("ATEM: Initial configuration complete.");
    }

    offset += cmdLen;
  }
}

void AtemClient::update() {
  uint32_t now = millis();

  if (connectionState_ == AtemConnectionState::DISCONNECTED) {
    if (now - lastConnectAttemptMs_ > kAtemConnectRetryIntervalMs) {
      connect();
    }
    return;
  }

  if (connectionState_ == AtemConnectionState::CONNECTING) {
    int packetSize = udp_.parsePacket();
    if (packetSize > 0) {
      uint8_t buffer[kAtemMaxPacketSize];
      if (packetSize > (int)sizeof(buffer)) packetSize = sizeof(buffer);
      udp_.read(buffer, packetSize);

      if (packetSize < 12 || !hasValidHeaderLength(buffer, packetSize)) {
        Serial.printf("ATEM: Ignoring corrupt handshake packet size %d\r\n", packetSize);
        return;
      }

      if (packetSize >= 13 && (buffer[0] & kAtemFlagSyn) != 0) {
        uint8_t status = buffer[12];
        if (status == 0x02) {
          sessionId_ = (buffer[2] << 8) | buffer[3];
          lastRemotePacketId_ = (buffer[10] << 8) | buffer[11];
          sendAck(lastRemotePacketId_);
          awaitingAssignedSessionId_ = true;

          connectionState_ = AtemConnectionState::CONNECTED;
          lastPacketReceivedMs_ = millis();
          Serial.println("ATEM: Video mixer successfully connected.");
          Serial.println("ATEM: Handshake response accepted. Synchronizing state dump...");
        } else if (status == 0x04) {
          Serial.println("ATEM connection rejected (status 0x04). Retrying...");
          connectionState_ = AtemConnectionState::DISCONNECTED;
        } else {
          Serial.printf("ATEM connection unexpected status 0x%02X. Retrying...\r\n", status);
          connectionState_ = AtemConnectionState::DISCONNECTED;
        }
      }
    } else if (now - lastConnectAttemptMs_ > kAtemConnectRetryIntervalMs) {
      Serial.println("ATEM connection handshake timeout. Retrying...");
      connectionState_ = AtemConnectionState::DISCONNECTED;
    }
    return;
  }

  if (connectionState_ == AtemConnectionState::CONNECTED) {
    if (now - lastPacketReceivedMs_ > kAtemTimeoutMs) {
      Serial.println("ATEM connection timed out. Reconnecting...");
      connectionState_ = AtemConnectionState::DISCONNECTED;
      return;
    }

    processQueuedCommands();

    int packetSize;
    while ((packetSize = udp_.parsePacket()) > 0) {
      uint8_t buffer[kAtemMaxPacketSize];
      if (packetSize > (int)sizeof(buffer)) packetSize = sizeof(buffer);
      udp_.read(buffer, packetSize);
      if (packetSize < 12) {
        Serial.printf("ATEM: Ignoring short packet size %d\r\n", packetSize);
        continue;
      }
      if (!hasValidHeaderLength(buffer, packetSize)) {
        Serial.printf("ATEM: Ignoring corrupt packet size %d\r\n", packetSize);
        continue;
      }

      uint8_t flags = buffer[0] & 0xF8;
      uint16_t packetSessionId = (buffer[2] << 8) | buffer[3];
      uint16_t ackPacketId = (buffer[4] << 8) | buffer[5];
      uint16_t remoteSequenceField = (buffer[8] << 8) | buffer[9];
      uint16_t remotePacketId = (buffer[10] << 8) | buffer[11];
      const bool packetHasCommands = packetSize > 12 && hasValidCommandPayload(buffer, packetSize);

      if (awaitingAssignedSessionId_ && !(flags & kAtemFlagSyn)) {
        sessionId_ = packetSessionId;
        awaitingAssignedSessionId_ = false;
        resetReceivedRemotePackets();
        Serial.printf("ATEM: Assigned SessionID = 0x%04X\r\n", sessionId_);
      }

      if (packetSessionId != sessionId_ && !(flags & kAtemFlagSyn)) {
        if (!initialSyncComplete_ &&
            !initialSessionRecoveryUsed_ &&
            packetHasCommands &&
            ((flags & kAtemFlagReliable) || initialSyncCommandCount_ == 0)) {
          Serial.printf("ATEM: Recovering initial sync on SessionID 0x%04X (was 0x%04X).\r\n",
                        packetSessionId,
                        sessionId_);
          sessionId_ = packetSessionId;
          initialSessionRecoveryUsed_ = true;
          awaitingAssignedSessionId_ = false;
          resetReceivedRemotePackets();
        } else {
        ++ignoredSessionPacketCount_;
        if (now - lastIgnoredSessionLogMs_ > 1000) {
          Serial.printf("ATEM: Ignored %u stale packets for other SessionIDs (last 0x%04X, active 0x%04X)\r\n",
                        ignoredSessionPacketCount_,
                        packetSessionId,
                        sessionId_);
          ignoredSessionPacketCount_ = 0;
          lastIgnoredSessionLogMs_ = now;
        }
        continue;
        }
      }

      if ((flags & kAtemFlagSyn) && packetSessionId != sessionId_) {
        ++ignoredSessionPacketCount_;
        if (now - lastIgnoredSessionLogMs_ > 1000) {
          Serial.printf("ATEM: Ignored %u stale handshake/reset packets (last SessionID 0x%04X, active 0x%04X)\r\n",
                        ignoredSessionPacketCount_,
                        packetSessionId,
                        sessionId_);
          ignoredSessionPacketCount_ = 0;
          lastIgnoredSessionLogMs_ = now;
        }
        continue;
      }

      lastPacketReceivedMs_ = now;
      lastRemotePacketId_ = remotePacketId;
      if (kAtemVerbosePackets) {
        Serial.printf("ATEM: Rx packet size %d, flags 0x%02X, AckID %u, RemoteSeqField %u, PacketID %u\r\n",
                      packetSize,
                      flags,
                      ackPacketId,
                      remoteSequenceField,
                      remotePacketId);
      }

      if (flags & kAtemFlagSyn) {
        uint8_t status = packetSize > 12 ? buffer[12] : 0;
        if (status == 0x04) {
          Serial.println("ATEM connection rejected/reset by switcher (status 0x04). Retrying...");
          connectionState_ = AtemConnectionState::DISCONNECTED;
          return;
        } else if (kAtemVerbosePackets) {
          Serial.printf("ATEM: Ignoring SYN packet outside handshake, status 0x%02X\r\n", status);
        }
        continue;
      }

      if ((flags & kAtemFlagRetransmission) && hasReceivedRemotePacket(remotePacketId)) {
        if (kAtemVerbosePackets) {
          Serial.printf("ATEM: Ignoring duplicate retransmission for PacketID %u\r\n", remotePacketId);
        }
        if (flags & kAtemFlagReliable) {
          sendAck(remotePacketId);
        }
        continue;
      }
      rememberReceivedRemotePacket(remotePacketId);

      if (flags & kAtemFlagRequestRetransmission) {
        if (kAtemVerbosePackets) {
          Serial.println("ATEM: Switcher requested retransmission");
        }
      }

      if (flags & kAtemFlagReliable) {
        if (packetSize == 12 && !initialEmptyAckSent_) {
          sendAck(remotePacketId, 0x0061);
          initialEmptyAckSent_ = true;
          if (!initialSyncComplete_) {
            initialSyncComplete_ = true;
            initialStateSeen_ = true;
            markNextConnected_ = true;
            Serial.println("ATEM: Initial state dump complete.");
            Serial.printf("ATEM: Initial summary - Program: %u, Preview: %u, Commands: %u\r\n",
                          switcherState_.programSource,
                          switcherState_.previewSource,
                          initialSyncCommandCount_);
          }
        } else {
          sendAck(remotePacketId);
        }
        if (waitingForInitialDump_) {
          waitingForInitialDump_ = false;
          Serial.println("ATEM: Control channel ready. Synchronizing initial state...");
        }
      }

      if (packetHasCommands) {
        parseState(buffer, packetSize);
        if (!initialStateSeen_) {
          initialStateSeen_ = true;
          Serial.println("ATEM: Initial state received.");
        }
      } else if (packetSize > 12) {
        Serial.printf("ATEM: Skipping non-command packet size %d, flags 0x%02X, payload %02X %02X %02X %02X\r\n",
                      packetSize,
                      flags,
                      buffer[12],
                      packetSize > 13 ? buffer[13] : 0,
                      packetSize > 14 ? buffer[14] : 0,
                      packetSize > 15 ? buffer[15] : 0);
      }
    }

    if (markNextConnected_) {
      markNextConnected_ = false;
      sendTimeRequest();
    }
  }
}
