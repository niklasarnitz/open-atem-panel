#pragma once

#include <stdint.h>
#include <stddef.h>

class IPAddress {
public:
  IPAddress() {
    ip_[0] = 0; ip_[1] = 0; ip_[2] = 0; ip_[3] = 0;
  }
  IPAddress(uint8_t a, uint8_t b, uint8_t c, uint8_t d) {
    ip_[0] = a; ip_[1] = b; ip_[2] = c; ip_[3] = d;
  }
  uint8_t operator[](int index) const {
    if (index >= 0 && index < 4) return ip_[index];
    return 0;
  }
private:
  uint8_t ip_[4];
};

enum EthernetLinkStatus {
  Unknown,
  LinkON,
  LinkOFF
};

class MockEthernet {
public:
  EthernetLinkStatus linkStatus() { return LinkON; }
  void init(int cs) {}
  void begin(uint8_t* mac, IPAddress ip, IPAddress dns, IPAddress gateway, IPAddress subnet) {}
  IPAddress localIP() { return IPAddress(10, 0, 0, 142); }
  IPAddress subnetMask() { return IPAddress(255, 255, 255, 0); }
  IPAddress gatewayIP() { return IPAddress(10, 0, 0, 254); }
};

extern MockEthernet Ethernet;

class EthernetUDP {
public:
  uint8_t begin(uint16_t port) { return 1; }
  int parsePacket() { return 0; }
  int read(uint8_t* buffer, size_t len) { return 0; }
  void beginPacket(IPAddress ip, uint16_t port) {}
  void write(const uint8_t* buffer, size_t len) {}
  void endPacket() {}
};

class MockW5100 {
public:
  void setRetransmissionTime(uint16_t time) {}
  void setRetransmissionCount(uint8_t count) {}
};

extern MockW5100 W5100;
