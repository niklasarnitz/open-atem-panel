#pragma once

// ============================================================================
// Ethernet Configuration für verschiedene ESP32 Boards
// ============================================================================

// Definiere hier welches Board verwendet wird (nur EINES aktivieren!)
// #define BOARD_WT32_ETH01
// #define BOARD_ESP32_POE
// #define BOARD_OLIMEX_ESP32_POE
#define BOARD_W5500_CUSTOM

// ============================================================================
// WT32-ETH01 (LAN8720)
// ============================================================================
#ifdef BOARD_WT32_ETH01
#define ETH_TYPE ETH_PHY_LAN8720
#define ETH_ADDR 1
#define ETH_MDC_PIN 23
#define ETH_MDIO_PIN 18
#define ETH_CLK_MODE ETH_CLOCK_GPIO0_IN
#define ETH_POWER_PIN 16
#define ETH_POWER_EN true
#endif

// ============================================================================
// ESP32-POE (LAN8720)
// ============================================================================
#ifdef BOARD_ESP32_POE
#define ETH_TYPE ETH_PHY_LAN8720
#define ETH_ADDR 0
#define ETH_MDC_PIN 23
#define ETH_MDIO_PIN 18
#define ETH_CLK_MODE ETH_CLOCK_GPIO0_IN
#define ETH_POWER_PIN 12
#define ETH_POWER_EN true
#endif

// ============================================================================
// Olimex ESP32-POE (LAN8720)
// ============================================================================
#ifdef BOARD_OLIMEX_ESP32_POE
#define ETH_TYPE ETH_PHY_LAN8720
#define ETH_ADDR 0
#define ETH_MDC_PIN 23
#define ETH_MDIO_PIN 18
#define ETH_CLK_MODE ETH_CLOCK_GPIO0_IN
#define ETH_POWER_PIN 12
#define ETH_POWER_EN true
#endif

// ============================================================================
// Generic ESP32 + W5500 (SPI)
// ============================================================================
#ifdef BOARD_W5500_CUSTOM
#define USE_W5500_SPI true

// W5500 SPI Pins (anpassen je nach Hardware)
#define W5500_CS_PIN 5
#define W5500_MOSI_PIN 23
#define W5500_MISO_PIN 19
#define W5500_CLK_PIN 18
#define W5500_RST_PIN 4

// W5500 MAC und IP (falls DHCP fehlschlägt)
#define W5500_MAC { 0xDE, 0xAD, 0xBE, 0xEF, 0x00, 0x01 }
#define W5500_USE_DHCP true
#define W5500_STATIC_IP { 192, 168, 1, 100 }
#define W5500_GATEWAY { 192, 168, 1, 1 }
#define W5500_SUBNET { 255, 255, 255, 0 }
#endif

// ============================================================================
// Default Fallback
// ============================================================================
#ifndef ETH_TYPE
#error "Bitte ein Ethernet-Board in EthernetConfig.h definieren!"
#endif
