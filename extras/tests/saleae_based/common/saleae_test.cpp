/*
 * saleae_test.cpp — SR_00 connection self-test (shared by all platforms).
 *
 * Toggles all 8 identification pins at exactly 1 Hz (1000 ms period), each
 * with a different high time (5 % .. 40 % in 5 % steps). Since no channel is
 * 50 % duty, an electrically inverted channel shows up as the complement
 * duty (95 % .. 60 %) and is immediately recognisable as wrong. This proves
 * that every MCU I/O used by the harness is alive and correctly wired.
 *
 *   CH 0 (D0): 1 Hz,  5 % high ( 50 ms)   GPIO 2
 *   CH 1 (D1): 1 Hz, 10 % high (100 ms)   GPIO 0
 *   CH 2 (D2): 1 Hz, 15 % high (150 ms)   GPIO 4
 *   CH 3 (D3): 1 Hz, 20 % high (200 ms)   GPIO 16
 *   CH 4 (D4): 1 Hz, 25 % high (250 ms)   GPIO 17
 *   CH 5 (D5): 1 Hz, 30 % high (300 ms)   GPIO 5
 *   CH 6 (D6): 1 Hz, 35 % high (350 ms)   GPIO 18
 *   CH 7 (D7): 1 Hz, 40 % high (400 ms)   GPIO 19
 */

#include "saleae_test.h"

#include <stdint.h>

#include "saleae_hal.h"

#define SALEAE_PIN_COUNT 8

// 8 identification pins. On ESP32 these match the white paper §3.3 channel
// map. On other targets use a contiguous, always-valid range that avoids the
// UART pins (0/1 on AVR) so the serial control channel keeps working.
#if defined(ARDUINO_ARCH_ESP32)
static const int saleae_pins[SALEAE_PIN_COUNT] = {2, 0,  4,  16,
                                                  17, 5, 18, 19};
#else
static const int saleae_pins[SALEAE_PIN_COUNT] = {2, 3, 4, 5, 6, 7, 8, 9};
#endif

// High time in milliseconds for a fixed 1000 ms (1 Hz) period.
static const uint16_t saleae_high_ms[SALEAE_PIN_COUNT] = {
    50, 100, 150, 200, 250, 300, 350, 400};

void saleae_test_setup(void) {
  for (int i = 0; i < SALEAE_PIN_COUNT; i++) {
    saleae_hal_pin_output(saleae_pins[i]);
    saleae_hal_write(saleae_pins[i], 0);
  }
}

void saleae_test_stop(void) {
  for (int i = 0; i < SALEAE_PIN_COUNT; i++) {
    saleae_hal_write(saleae_pins[i], 0);
  }
}

void saleae_test_loop(void) {
  uint16_t phase = (uint16_t)(saleae_hal_millis() % 1000);
  for (int i = 0; i < SALEAE_PIN_COUNT; i++) {
    saleae_hal_write(saleae_pins[i], phase < saleae_high_ms[i] ? 1 : 0);
  }
  saleae_hal_delay_ms(1);
}
