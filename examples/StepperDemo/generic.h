#ifndef GENERIC_H
#define GENERIC_H

#if defined(ARDUINO_ARCH_AVR)
#define get_char(x) pgm_read_byte(x)
#define MSG_TYPE PGM_P
#elif defined(ARDUINO_ARCH_ESP32) || defined(ARDUINO_ARCH_SAM)
#define get_char(x) *x
#define MSG_TYPE const char*
#else
#error "Unsupported derivate"
#endif

#if defined(CONFIG_IDF_TARGET_ESP32C3) && defined(CONFIG_TINYUSB_CDC_ENABLED) && (CONFIG_TINYUSB_CDC_ENABLED == 1)
USBCDC USBSerial;
#define SerialInterface USBSerial
#else
#define SerialInterface Serial
#endif

#endif
