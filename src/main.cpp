#include <Arduino.h>
#include <SPI.h>
#include <Ethernet.h>
#include <Wire.h>
#include <MCP23017.h>
#include <TLC5955.h>
#include <utility/w5100.h>

#include "atem_client.h"
#include "led_controller.h"
#include "button_manager.h"
#include "adc_reader.h"
#include "fader_controller.h"

namespace {

// Logging and execution rates
constexpr uint32_t kBaudRate = 115200;
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

// SPI pins for Waveshare ESP32-S3-ETH (W5500 Ethernet)
constexpr uint8_t kEthMosiPin = 11;
constexpr uint8_t kEthMisoPin = 12;
constexpr uint8_t kEthSclkPin = 13;
constexpr uint8_t kEthCsPin = 14;
constexpr uint8_t kEthRstPin = 9;

constexpr uint8_t kMcpAddress = 0x20;

} // namespace

AtemClient gAtem(kSwitcherIp, kSwitcherPort, kLocalPort);
TLC5955 gTlc;
MCP23017 gMcp(kMcpAddress);
AdcReader gAdcReader(0x34);

LedController gLedController(gTlc, gAtem, gAdcReader);
ButtonManager gButtonManager(gMcp, gAtem);
FaderController gFaderController(gAdcReader, gAtem, 0);

namespace {

SPIClass gTlcSpi(HSPI);
TaskHandle_t gAtemTaskHandle = nullptr;
EthernetLinkStatus gLastEthernetLinkStatus = Unknown;
uint32_t gLastEthernetLinkPollMs = 0;

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

  gLedController.begin(&gTlcSpi);
  gButtonManager.begin();
  gButtonManager.primeMatrixState();
  gAdcReader.begin();
  
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
  gButtonManager.update();
  gLedController.update(gButtonManager.pgmShift(), gButtonManager.prvShift());
  gAdcReader.update();
  gFaderController.update();
}
