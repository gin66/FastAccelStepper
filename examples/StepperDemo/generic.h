#ifndef GENERIC_H
#define GENERIC_H

#include <Arduino.h>

#if defined(ARDUINO_ARCH_AVR)
#define get_char(x) pgm_read_byte(x)
#define MSG_TYPE PGM_P
#elif defined(ARDUINO_ARCH_ESP32) || defined(ARDUINO_ARCH_SAM)
#define get_char(x) *x
#define MSG_TYPE const char*
#else
#error "Unsupported derivate"
#endif

#if defined(CONFIG_IDF_TARGET_ESP32C3) && (ARDUINO_USB_MODE == 1)
#define PRINT_INIT() \
   	USBSerial.begin(115200); \
    while (!Serial) { \
      /* wait for USB serial port to connect */ \
    }
#define PRINTLN USBSerial.println
#define PRINT USBSerial.println
#define POLL_CHAR_IF_ANY(ch) \
    if (USBSerial.available()) { \
      ch = USBSerial.read(); \
    }
#else
#define PRINT_INIT() Serial.begin(115200);
#define PRINTLN Serial.println
#define PRINT Serial.println
#define POLL_CHAR_IF_ANY(ch) \
    if (Serial.available()) { \
      ch = Serial.read(); \
    }
#endif

#endif
