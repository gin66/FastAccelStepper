/*
 * saleae_hal_arduino.cpp — Arduino HAL for the Saleae test apps.
 *
 * Used by the ESP32 (Arduino framework) and RP2040/Pico builds.
 *
 * The discriminator is `ARDUINO`, not `ESP_PLATFORM`. Both HALs are linked into
 * every build, and the Arduino ESP32 core defines ESP_PLATFORM as well, so
 * guarding on ESP_PLATFORM let the ESP-IDF HAL win in the Arduino build: the
 * wrong `saleae_hal_write` and `saleae_hal_delay_ms` were linked there, and
 * they behaved differently enough to matter (the ESP-IDF ones busy-wait and
 * ignore sub-millisecond arguments).
 */

// See the note in saleae_hal_espidf.cpp: both HALs are linked everywhere.
#if defined(ARDUINO)

#include "saleae_hal.h"

#include <Arduino.h>

extern "C" void saleae_hal_pin_output(int pin) { pinMode(pin, OUTPUT); }

extern "C" void saleae_hal_write(int pin, int level) {
  digitalWrite(pin, level ? HIGH : LOW);
}

extern "C" uint32_t saleae_hal_millis(void) { return millis(); }

extern "C" void saleae_hal_delay_ms(uint32_t ms) { delay(ms); }

extern "C" void saleae_hal_serial_begin(uint32_t baud) { Serial.begin(baud); }

extern "C" int saleae_hal_serial_read(void) {
  return Serial.available() ? Serial.read() : -1;
}

extern "C" void saleae_hal_serial_write(const char* text) {
  Serial.print(text);
}

// A flash-resident literal. On AVR every reply string lives in program memory
// (see saleae_str.h), and `Serial.print(const char*)` would dereference it as
// if it were SRAM -- so the __FlashStringHelper overload is the one that must
// be picked, and it is what actually reaches the TX path. It has to be spelled
// with the cast: the two overloads differ only in parameter type.
extern "C" void saleae_hal_serial_write_p(const char* text) {
#if defined(__AVR__)
  Serial.print(reinterpret_cast<const __FlashStringHelper*>(text));
#else
  // No flash/RAM distinction off AVR, and no __FlashStringHelper on plain
  // ESP-IDF either, so this is the same call.
  Serial.print(text);
#endif
}

#endif  // !ESP_PLATFORM
