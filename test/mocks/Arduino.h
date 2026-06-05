#pragma once

#include <stdint.h>
#include <stddef.h>
#include <string.h>
#include <stdio.h>
#include <stdarg.h>
#include <algorithm>

typedef uint8_t byte;
typedef bool boolean;

#ifndef min
#define min(a,b) ((a)<(b)?(a):(b))
#endif
#ifndef max
#define max(a,b) ((a)>(b)?(a):(b))
#endif

#define INPUT 0
#define OUTPUT 1
#define INPUT_PULLUP 2
#define HIGH 1
#define LOW 0

// Millis control
extern uint32_t mock_millis_val;
inline uint32_t millis() { return mock_millis_val; }
inline void set_mock_millis(uint32_t ms) { mock_millis_val = ms; }

inline void delay(uint32_t ms) { mock_millis_val += ms; }
inline void delayMicroseconds(uint32_t us) {}

inline void pinMode(uint8_t pin, uint8_t mode) {}
inline void digitalWrite(uint8_t pin, uint8_t val) {}
inline int digitalRead(uint8_t pin) { return LOW; }

// FreeRTOS Mocks
typedef int portMUX_TYPE;
#define portMUX_INITIALIZER_UNLOCKED 0
inline void portENTER_CRITICAL(portMUX_TYPE*) {}
inline void portEXIT_CRITICAL(portMUX_TYPE*) {}

typedef void* TaskHandle_t;
typedef uint32_t UBaseType_t;
typedef int BaseType_t;
#define pdPASS 1
#define pdMS_TO_TICKS(ms) (ms)
inline void vTaskDelay(uint32_t ticks) { mock_millis_val += ticks; }
inline BaseType_t xTaskCreatePinnedToCore(void (*task)(void*), const char* name, uint32_t stack, void* param, UBaseType_t priority, TaskHandle_t* handle, int core) { return pdPASS; }

// Serial Mock
class MockSerial {
public:
  void begin(uint32_t baud) {}
  void printf(const char* format, ...) {
    va_list args;
    va_start(args, format);
    vprintf(format, args);
    va_end(args);
  }
  void print(const char* str) { ::printf("%s", str); }
  void print(int val) { ::printf("%d", val); }
  void println(const char* str = "") { ::printf("%s\n", str); }
  void println(int val) { ::printf("%d\n", val); }
  template <typename T>
  void println(const T& val) {}
};
extern MockSerial Serial;
