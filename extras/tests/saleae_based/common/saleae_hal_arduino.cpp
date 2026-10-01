/*
 * saleae_hal_arduino.cpp — Arduino HAL for the Saleae test apps.
 *
 * Used by the ESP32 (Arduino framework) and RP2040/Pico builds.
 */

#include "saleae_hal.h"

#include <Arduino.h>

extern "C" void saleae_hal_pin_output(int pin) { pinMode(pin, OUTPUT); }

extern "C" void saleae_hal_write(int pin, int level) {
  digitalWrite(pin, level ? HIGH : LOW);
}

extern "C" uint32_t saleae_hal_millis(void) { return millis(); }

extern "C" void saleae_hal_delay_ms(uint32_t ms) { delay(ms); }
