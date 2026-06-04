#ifndef ATEM_DISCOVERY_H
#define ATEM_DISCOVERY_H

/*
  ATEM Discovery Helper Functions & Data Structures
  
  Enthält:
  - AtemDevice Struktur für Geräteverwaltung
  - Hilfsfunktionen für Discovery
  - Caching und Persistierung
*/

#include <Arduino.h>
#include <IPAddress.h>
#include <vector>

// ============================================================================
// ATEM DEVICE STRUKTUR
// ============================================================================

/**
 * Repräsentation eines gefundenen ATEM-Geräts
 */
class AtemDevice {
public:
  String serviceName;      // z.B. "ATEM Broadcast"
  String hostname;         // z.B. "atem-broadcast.local"
  IPAddress ipAddress;     // z.B. 192.168.1.50
  uint16_t port;           // mDNS-Service Port (normalerweise 5353)
  String txtRecords;       // TXT-Records als String (z.B. "vers=2.0,fw=8.6.1")
  unsigned long lastSeen;  // Zeitstempel wann Gerät zuletzt gesehen wurde
  bool isResponsive;       // Optional: True wenn Ping erfolgreich war
  uint32_t rssi;           // Optional: Signal-Stärke bei WiFi (hier 0)

  // Konstruktoren
  AtemDevice() 
    : port(0), lastSeen(0), isResponsive(false), rssi(0) {}

  AtemDevice(const String& sName, const String& hName, 
             const IPAddress& ip, uint16_t p, const String& txt = "")
    : serviceName(sName), hostname(hName), ipAddress(ip), port(p),
      txtRecords(txt), lastSeen(millis()), isResponsive(true), rssi(0) {}

  // Hilfsfunktionen
  
  /**
   * Gibt die Gerätinformation als einzelnen String zurück
   */
  String toString() const {
    String result = serviceName;
    result += " (";
    result += ipAddress.toString();
    result += ":";
    result += port;
    result += ")";
    return result;
  }

  /**
   * Überprüft ob das Gerät älter als maxAge Millisekunden ist
   */
  bool isStale(unsigned long maxAge) const {
    return (millis() - lastSeen) > maxAge;
  }

  /**
   * Extrahiert einen bestimmten TXT-Record-Wert
   * Beispiel: getTxtValue("vers") -> "2.0"
   */
  String getTxtValue(const char* key) const {
    if (txtRecords.length() == 0) return "";
    
    String searchKey = String(key);
    searchKey += "=";
    int startIdx = txtRecords.indexOf(searchKey);
    
    if (startIdx == -1) return "";
    
    startIdx += searchKey.length();
    int endIdx = txtRecords.indexOf(",", startIdx);
    if (endIdx == -1) endIdx = txtRecords.length();
    
    return txtRecords.substring(startIdx, endIdx);
  }

  /**
   * Extrahiert die Firmware-Version aus TXT-Records
   */
  String getFirmwareVersion() const {
    return getTxtValue("fw");
  }

  /**
   * Extrahiert das Modell aus TXT-Records
   */
  String getModel() const {
    return getTxtValue("model");
  }
};

// ============================================================================
// DEVICE COLLECTION HELPER
// ============================================================================

/**
 * Hilfsfunktionen für Device-Collection-Verwaltung
 */
class AtemDeviceManager {
private:
  std::vector<AtemDevice> devices;

public:
  /**
   * Fügt ein Device hinzu oder aktualisiert bestehendes
   */
  void addOrUpdate(const AtemDevice& device) {
    for (size_t i = 0; i < devices.size(); i++) {
      if (devices[i].ipAddress == device.ipAddress) {
        devices[i] = device; // Aktualisieren
        return;
      }
    }
    devices.push_back(device); // Neu hinzufügen
  }

  /**
   * Gibt alle Devices zurück
   */
  const std::vector<AtemDevice>& getAll() const {
    return devices;
  }

  /**
   * Findet Gerät nach IP-Adresse
   */
  AtemDevice* findByIp(const IPAddress& ip) {
    for (size_t i = 0; i < devices.size(); i++) {
      if (devices[i].ipAddress == ip) {
        return &devices[i];
      }
    }
    return nullptr;
  }

  /**
   * Findet Gerät nach Service-Name
   */
  AtemDevice* findByName(const String& name) {
    for (size_t i = 0; i < devices.size(); i++) {
      if (devices[i].serviceName == name) {
        return &devices[i];
      }
    }
    return nullptr;
  }

  /**
   * Entfernt abgelaufene Geräte (älter als maxAge ms)
   */
  void removeStale(unsigned long maxAge) {
    for (int i = devices.size() - 1; i >= 0; i--) {
      if (devices[i].isStale(maxAge)) {
        devices.erase(devices.begin() + i);
      }
    }
  }

  /**
   * Gibt Anzahl verwalteter Devices zurück
   */
  size_t count() const {
    return devices.size();
  }

  /**
   * Löscht alle Devices
   */
  void clear() {
    devices.clear();
  }

  /**
   * Druckt alle Devices auf Serial
   */
  void printAll(bool detailed = false) const {
    if (devices.empty()) {
      Serial.println("Keine Geräte gefunden.");
      return;
    }

    Serial.println("\n╔════════════════════════════════════════════╗");
    Serial.println("║        Gefundene ATEM-Geräte              ║");
    Serial.println("╠════════════════════════════════════════════╣");

    for (size_t i = 0; i < devices.size(); i++) {
      const AtemDevice& dev = devices[i];
      Serial.print("║ [");
      Serial.print(i + 1);
      Serial.print("] ");
      Serial.println(dev.serviceName);

      if (detailed) {
        Serial.print("║     IP: ");
        Serial.println(dev.ipAddress);
        Serial.print("║     Hostname: ");
        Serial.println(dev.hostname);
        Serial.print("║     Port: ");
        Serial.println(dev.port);
        
        String fw = dev.getFirmwareVersion();
        if (fw.length() > 0) {
          Serial.print("║     Firmware: ");
          Serial.println(fw);
        }
        
        String model = dev.getModel();
        if (model.length() > 0) {
          Serial.print("║     Model: ");
          Serial.println(model);
        }
      }
    }

    Serial.println("╚════════════════════════════════════════════╝");
  }
};

// ============================================================================
// CONSTANT DEFINITIONS
// ============================================================================

// mDNS Service Definition
#define ATEM_MDNS_SERVICE_TYPE "_blackmagic"
#define ATEM_MDNS_SERVICE_PROTO "_tcp"
#define ATEM_MDNS_SERVICE_LOCAL "local"

// Typische ATEM Control Ports (für zukünftige Nutzung)
#define ATEM_CONTROL_PORT_TCP 9910   // Control Commands (TCP)
#define ATEM_CONTROL_PORT_UDP 9910   // Control Commands (UDP)
#define ATEM_TALLY_PORT_UDP 9911      // Tally Information (UDP)
#define ATEM_MONITORING_PORT 9912     // Monitoring (UDP)

// Discovery Parameter
#define ATEM_DEFAULT_DISCOVERY_INTERVAL 7000  // ms
#define ATEM_DEFAULT_DEVICE_TIMEOUT 60000     // ms - Geräte als stale markieren nach 60s

// ============================================================================
// UTILITY MACROS
// ============================================================================

/**
 * Hilfsmakro zum Formatieren von IP-Adressen
 */
#define IP_STR(ip) (String(ip[0]) + "." + String(ip[1]) + "." + String(ip[2]) + "." + String(ip[3])).c_str()

#endif // ATEM_DISCOVERY_H
