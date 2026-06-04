#ifndef ETHERNET_CONFIG_H
#define ETHERNET_CONFIG_H

/*
  Ethernet-Konfigurationen für verschiedene ESP32-Boards
  
  Wähle ein Board-Profil oder definiere eigene Parameter.
  Diese Werte müssen mit den Defines in platformio.ini oder main.cpp übereinstimmen.
*/

// ============================================================================
// BOARD-PROFILE
// ============================================================================

// Wähle EINES der folgenden Boards:
#define BOARD_WT32_ETH01
// #define BOARD_OLIMEX_ESP32_POE
// #define BOARD_OLIMEX_ESP32_EVB
// #define BOARD_CUSTOM_LAN8720

// ============================================================================
// Automatische Konfiguration basierend auf Board-Wahl
// ============================================================================

#if defined(BOARD_WT32_ETH01)
  // WT32-ETH01: LAN8720 mit GPIO17-Out Clock
  #define BOARD_NAME "WT32-ETH01 (LAN8720)"
  #define ETH_PHY_TYPE_STR "LAN8720"
  #define ETH_PHY_ADDR 1
  #define ETH_PHY_MDC 23
  #define ETH_PHY_MDIO 18
  #define ETH_CLK_MODE ETH_CLOCK_GPIO17_OUT

#elif defined(BOARD_OLIMEX_ESP32_POE)
  // Olimex ESP32-POE: LAN8720 mit GPIO17-Out Clock
  #define BOARD_NAME "Olimex ESP32-POE (LAN8720)"
  #define ETH_PHY_TYPE_STR "LAN8720"
  #define ETH_PHY_ADDR 0
  #define ETH_PHY_MDC 23
  #define ETH_PHY_MDIO 18
  #define ETH_CLK_MODE ETH_CLOCK_GPIO17_OUT

#elif defined(BOARD_OLIMEX_ESP32_EVB)
  // Olimex ESP32-EVB: LAN8720 mit GPIO17-Out Clock
  #define BOARD_NAME "Olimex ESP32-EVB (LAN8720)"
  #define ETH_PHY_TYPE_STR "LAN8720"
  #define ETH_PHY_ADDR 0
  #define ETH_PHY_MDC 23
  #define ETH_PHY_MDIO 18
  #define ETH_CLK_MODE ETH_CLOCK_GPIO17_OUT

#elif defined(BOARD_CUSTOM_LAN8720)
  // Generischer ESP32 + externes LAN8720 Modul
  // WICHTIG: Diese Werte MÜSSEN für deine Hardware angepasst werden!
  #define BOARD_NAME "Custom ESP32 + LAN8720"
  #define ETH_PHY_TYPE_STR "LAN8720"
  #define ETH_PHY_ADDR 0           // oder 1 - probiere beide
  #define ETH_PHY_MDC 23            // MDC-Pin
  #define ETH_PHY_MDIO 18           // MDIO-Pin
  #define ETH_CLK_MODE ETH_CLOCK_GPIO17_OUT  // Andere Optionen: ETH_CLOCK_GPIO0_IN, ETH_CLOCK_GPIO16_OUT

#else
  #error "Bitte wähle ein Board-Profil (BOARD_WT32_ETH01, etc.) oder definiere ETH_PHY_* manuell!"
#endif

// ============================================================================
// PHY-TYP (automatisch LAN8720 in obigen Profilen, kann aber geändert werden)
// ============================================================================

#ifndef ETH_PHY_TYPE
  #define ETH_PHY_TYPE ETH_PHY_LAN8720
#endif

// ============================================================================
// ALTERNATIVE CLK_MODE OPTIONEN
// ============================================================================

/*
  Clock-Modus hängt davon ab, wie der CLK-Pin verdrahtet ist:
  
  ETH_CLOCK_GPIO17_OUT:    Clock output auf GPIO 17  (häufig bei WT32-ETH01)
  ETH_CLOCK_GPIO0_IN:      Clock input von GPIO 0    (seltener, für externe Clock)
  ETH_CLOCK_GPIO16_OUT:    Clock output auf GPIO 16  (experimentell)
  
  Falls Ethernet nicht funktioniert, probiere andere CLK_MODE Optionen!
  Beispiel:
    #define ETH_CLK_MODE ETH_CLOCK_GPIO0_IN
    #define ETH_CLK_MODE ETH_CLOCK_GPIO16_OUT
*/

// ============================================================================
// PIN-MAPPING-REFERENZ (falls man einen Custom-Board baut)
// ============================================================================

/*
  Ethernet-RMII-Interface für LAN8720:
  
  Erforderliche Pins (Verbindung zum LAN8720):
  ├─ MDC (Management Data Clock)      -> GPIO 23
  ├─ MDIO (Management Data I/O)       -> GPIO 18
  ├─ TX0 (Transmit Data 0)            -> GPIO 19
  ├─ TX1 (Transmit Data 1)            -> GPIO 21
  ├─ TX2 (Transmit Data 2)            -> GPIO 22
  ├─ TX3 (Transmit Data 3)            -> GPIO 26
  ├─ RX0 (Receive Data 0)             -> GPIO 25
  ├─ RX1 (Receive Data 1)             -> GPIO 32
  ├─ RX2 (Receive Data 2)             -> GPIO 33
  ├─ RX3 (Receive Data 3)             -> GPIO 27
  ├─ CLK (Ethernet Clock, 50 MHz)     -> GPIO 17 (oder 0/16)
  ├─ CRS (Carrier Sense)              -> GPIO 13
  └─ COL (Collision Detection)        -> GPIO 4

  Diese Pins sind für ESP32-RMII FEST vorgegeben und können nicht geändert werden!
  
  Nur MDC/MDIO Pin können variabel sein (normalerweise 23/18).
*/

// ============================================================================
// HELPER MAKROS
// ============================================================================

#define STRINGIFY(x) #x
#define TOSTRING(x) STRINGIFY(x)

// Beispiel-Nutzung im Code:
// Serial.println("Board: " BOARD_NAME);
// Serial.println("PHY: " ETH_PHY_TYPE_STR);

#endif // ETHERNET_CONFIG_H