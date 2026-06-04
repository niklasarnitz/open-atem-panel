#pragma once

#include <Arduino.h>

/**
 * Entprellte Button-Klasse ohne blockierende Delays
 * Verwendet ein einfaches Debouncing mit Schwelle
 */
class Button {
public:
    /**
     * Initialisiert einen Button
     * @param pin GPIO-Pin
     * @param activeLow true wenn Button auf LOW = gedrückt (Default)
     */
    void begin(uint8_t pin, bool activeLow = true);
    
    /**
     * Muss regelmäßig aufgerufen werden (mind. 100-200x pro Sekunde)
     * Nicht blockierend
     */
    void loop();
    
    /**
     * True wenn Button in diesem Loop-Durchgang gereleased wurde
     * (Flanke LOW->HIGH oder HIGH->LOW je nach activeLow)
     */
    bool fell();
    
    /**
     * True wenn Button in diesem Loop-Durchgang gedrückt wurde
     */
    bool rose();
    
    /**
     * Aktueller Button-Status
     */
    bool isPressed() const;

private:
    uint8_t m_pin;
    bool m_activeLow;
    bool m_lastState;
    bool m_currentState;
    bool m_fellFlag;
    bool m_roseFlag;
    uint32_t m_lastChangeTime;
    
    static const uint32_t DEBOUNCE_TIME_MS = 20;
};
