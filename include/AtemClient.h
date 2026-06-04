#pragma once

#include <Arduino.h>
#include <WiFiUdp.h>
#include <vector>

// ============================================================================
// ATEM Protocol Configuration & Debug
// ============================================================================

#define ATEM_DEBUG 1              // Debug-Output aktivieren
#define ATEM_DEFAULT_PORT 9910    // ATEM Control Port
#define ATEM_TIMEOUT_MS 5000      // Timeout nach 5 Sekunden ohne Kontakt
#define ATEM_PACKET_BUFFER_SIZE 2048
#define ATEM_MAX_OUTGOING_RELIABLE 32

// ============================================================================
// ATEM Connection States
// ============================================================================
enum class AtemConnectionState : uint8_t {
    DISCONNECTED = 0,
    CONNECTING = 1,
    HANDSHAKE_SENT = 2,
    WAITING_FOR_INITIAL_STATE = 3,
    REQUESTING_MISSING_STATE = 4,
    CONNECTED = 5,
    RECONNECTING = 6,
    ERROR = 7
};

// ============================================================================
// ATEM Packet Header Structure (Big-Endian!)
// Definiert nach OpenSwitcher / SKAARHOJ Reverse-Engineering
// ============================================================================
struct AtemPacketHeader {
    uint8_t flagsAndLengthHigh;  // Flags im höheren Nibble, oberes Byte Länge
    uint8_t lengthLow;           // Unteres Byte Paketlänge
    uint16_t sessionId;          // Session Identifier
    uint16_t ackNumber;          // Remote Packet ID / Acknowledgement
    uint16_t unknown;            // Unknown / Reserved
    uint16_t remoteSequence;     // Remote Sequence Number
    uint16_t localSequence;      // Local Sequence Number
};

// Header-Flags (oberes Nibble von flagsAndLengthHigh)
#define ATEM_FLAG_SYN 0x80          // SYN / Hello Packet
#define ATEM_FLAG_ACK 0x40          // ACK Packet
#define ATEM_FLAG_RETRANSMISSION 0x20  // Retransmission / Resend
#define ATEM_FLAG_REQUEST_RETRANS 0x10  // Request Next After (Retransmit Request)

// Für Flags im unteren Nibble
#define ATEM_FLAG_RELIABLE 0x08     // Reliable / AckRequest
#define ATEM_FLAG_HELLO 0x01        // Hello / InitialData

// ============================================================================
// ATEM Command Segment Header (10 Byte pro Kommando + Payload)
// ============================================================================
struct AtemCommandSegment {
    uint16_t length;             // Länge dieser Segment (Header + Payload)
    uint16_t reserved;           // Reserved / Unknown / Checksum
    char commandName[4];         // 4-Byte ASCII Kommando-Name
    // dann folgt Payload
};

// ============================================================================
// Outgoing Reliable Packet Buffer für Retransmission
// ============================================================================
struct OutgoingReliablePacket {
    uint32_t sentTime;           // Wann das Paket gesendet wurde (millis)
    uint16_t localSequence;      // Local Sequence Number
    uint8_t buffer[ATEM_PACKET_BUFFER_SIZE];
    uint16_t length;             // Länge des Pakets
};

// ============================================================================
// ATEM Device Status
// ============================================================================
struct AtemDeviceStatus {
    uint16_t programInput;       // aktueller Program Input
    uint16_t previewInput;       // aktueller Preview Input
    uint16_t transitionPosition; // T-Bar Position 0-10000
    bool dsk1OnAir;              // DSK1 On Air Status
    bool dsk1Tie;                // DSK1 Tie Status
    uint8_t firmwareVersion[4];  // Firmware Version
    char deviceName[64];         // Device Name / Model
};

// ============================================================================
// Main ATEM Client Class
// ============================================================================
class AtemClient {
public:
    AtemClient();
    ~AtemClient();

    /**
     * Initialisiert den ATEM-Client mit IP und Port
     * IP sollte vom DHCP/Netzwerk bekannt sein
     */
    bool begin(IPAddress atemIp, uint16_t port = ATEM_DEFAULT_PORT);

    /**
     * Muss regelmäßig aufgerufen werden (mind. 10-50x pro Sekunde)
     * Nicht blockierend, kümmert sich um Verbindungsaufbau, Paket-Empfang, Timeouts
     */
    void loop();

    /**
     * Connection State abfragen
     */
    AtemConnectionState state() const { return m_connectionState; }
    bool isConnected() const { return m_connectionState == AtemConnectionState::CONNECTED; }
    bool isInitialized() const { return m_initialized; }

    // ========================================================================
    // Status Query
    // ========================================================================
    uint16_t getProgramInput() const { return m_status.programInput; }
    uint16_t getPreviewInput() const { return m_status.previewInput; }
    uint16_t getTransitionPosition() const { return m_status.transitionPosition; }
    bool isDSK1OnAir() const { return m_status.dsk1OnAir; }
    bool isDSK1Tie() const { return m_status.dsk1Tie; }

    // ========================================================================
    // Control Commands (nur senden, wenn CONNECTED && INITIALIZED)
    // ========================================================================

    /**
     * Program Input setzen (M/E 0)
     * @param inputId Source Index (z.B. 0=Black, 1=Camera 1, etc.)
     * @param me Mix Effect Bus (üblicherweise 0)
     */
    bool setProgramInput(uint16_t inputId, uint8_t me = 0);

    /**
     * Preview Input setzen
     */
    bool setPreviewInput(uint16_t inputId, uint8_t me = 0);

    /**
     * Cut - sofort zu Program wechseln
     */
    bool cut(uint8_t me = 0);

    /**
     * Auto Transition (T-Bar Fade)
     */
    bool autoTransition(uint8_t me = 0);

    /**
     * T-Bar / Transition Position setzen (0-10000)
     * 0 = ganz oben (Preview), 10000 = ganz unten (Program)
     */
    bool setTransitionPosition(uint16_t position, uint8_t me = 0);

    /**
     * DSK1 (Downstream Keyer 1) Settings
     */
    bool setDSK1OnAir(bool onAir);
    bool setDSK1Tie(bool tie);
    bool autoDSK1();

    /**
     * Fade To Black
     */
    bool fadeToBlack(bool enabled, uint8_t me = 0);

    // ========================================================================
    // Debug / Diagnostics
    // ========================================================================
    void printStatus() const;

private:
    // ========================================================================
    // Private State
    // ========================================================================
    WiFiUDP m_udpSocket;
    IPAddress m_atemIp;
    uint16_t m_atemPort;

    AtemConnectionState m_connectionState;
    bool m_initialized;
    uint32_t m_lastPacketTime;
    uint32_t m_lastReconnectAttempt;
    uint8_t m_reconnectBackoffLevel;  // 0=500ms, 1=1s, 2=2s, 3=5s

    // Session Management
    uint16_t m_sessionId;
    uint16_t m_localSequence;
    uint16_t m_remoteSequence;
    uint16_t m_lastAckNumber;

    // Initial State Tracking
    std::vector<uint16_t> m_receivedInitialStateSequences;
    bool m_allInitialStateParsed;

    // Status
    AtemDeviceStatus m_status;

    // Packet Buffers
    uint8_t m_rxBuffer[ATEM_PACKET_BUFFER_SIZE];
    OutgoingReliablePacket m_outgoingReliable[ATEM_MAX_OUTGOING_RELIABLE];
    uint8_t m_outgoingReliableCount;

    // ========================================================================
    // Private Methods: Connection Lifecycle
    // ========================================================================
    void updateEthernetLink();
    void handleConnectionTimeout();
    void initateHandshake();
    void scheduleReconnect();
    uint32_t getReconnectDelay() const;

    // ========================================================================
    // Private Methods: Packet I/O
    // ========================================================================
    void processIncomingPackets();
    bool parseHeader(const uint8_t* buffer, size_t len, AtemPacketHeader& out);
    void processPacketPayload(const AtemPacketHeader& hdr, const uint8_t* payload, size_t payloadLen);
    void parseCommands(const uint8_t* payload, size_t len, bool isInitialState);
    void handleCommand(const char cmd[4], const uint8_t* data, size_t len);

    // ========================================================================
    // Private Methods: Sending
    // ========================================================================
    bool buildAndSendPacket(uint8_t flags, const uint8_t* payload, uint16_t payloadLen, bool reliable);
    void sendAck(uint16_t ackNumber);
    bool sendCommand(const char cmd[4], const uint8_t* payload, uint16_t payloadLen);
    void storeOutgoingReliable(const uint8_t* packet, uint16_t length, uint16_t localSequence);
    void handleRetransmitRequest(uint16_t fromSequence);

    // ========================================================================
    // Private Methods: Helpers
    // ========================================================================
    uint16_t readU16BE(const uint8_t* p) const;
    void writeU16BE(uint8_t* p, uint16_t value);
    void buildHeader(uint8_t* buffer, uint8_t flags, uint16_t length,
                     uint16_t sessionId, uint16_t ackNumber, uint16_t localSequence);
    uint32_t millis32() const { return (uint32_t)millis(); }

    // Debug Helper
    void debugLog(const char* fmt, ...);
    void debugPacketHeader(const AtemPacketHeader& hdr);
};
