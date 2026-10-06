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
#include "saleae_str.h"

#define SALEAE_PIN_COUNT 8
#define PERIOD_MS 1000

// The pattern's edges are all on a 50 ms grid: the high times are 50, 100 ...
// 400 ms, so every 50 ms boundary is an edge on exactly one channel. See
// saleae_test_loop(), which uses it to sleep instead of spinning between them.
#define EDGE_GRID_MS 50

// 8 identification pins. On ESP32 these match the white paper §3.3 channel
// map. On AVR they are the fixed Nano cable, whose Timer1 compare pins (D9/D10)
// sit on channels 3/2. On other targets use a contiguous, always-valid range
// that avoids the UART pins (0/1 on AVR) so the serial control channel keeps
// working.
//
// PROGMEM because `const` is not free on AVR: the linker script copies
// `.rodata` into SRAM to initialise it at reset, so a 22-byte lookup table is
// 22 bytes of SRAM on a 328P and nothing on an ESP32. See saleae_str.h.
//
// Same order and same pins as SAL_CHAN_PINS in saleae_app.cpp, off the same
// SALEAE_TARGET_ESP32 test, so a channel the self-test proves is the channel a
// scenario measures.
#if defined(SALEAE_TARGET_ESP32)
static const int saleae_pins[SALEAE_PIN_COUNT] SAL_PROGMEM = {2,  0, 4,  16,
                                                              17, 5, 18, 19};
#elif defined(ARDUINO_ARCH_AVR)
// The same fixed cable as SAL_CHAN_PINS in saleae_app.cpp, in analyzer-channel
// order: a 328P Nano where D9/D10 (Timer1's compare outputs) are on channels 3
// and 2. SR_00 proves this wiring by toggling channel i with duty i, so a cable
// that does not match the firmware's map fails the pre-check rather than being
// silently mis-measured.
static const int saleae_pins[SALEAE_PIN_COUNT] SAL_PROGMEM = {12, 11, 10, 9,
                                                              5,  4,  3,  2};
#else
static const int saleae_pins[SALEAE_PIN_COUNT] SAL_PROGMEM = {2, 3, 4, 5,
                                                              6, 7, 8, 9};
#endif

// High time in milliseconds for a fixed 1000 ms (1 Hz) period. PROGMEM for the
// same reason as the pin table above.
static const uint16_t saleae_high_ms[SALEAE_PIN_COUNT] SAL_PROGMEM = {
    50, 100, 150, 200, 250, 300, 350, 400};

#define PIN_OF(i) ((int)sal_pgm_read_word(&saleae_pins[i]))
#define HIGH_MS_OF(i) ((uint16_t)sal_pgm_read_word(&saleae_high_ms[i]))

// Phase anchor. `millis() % 1000` starts the pattern at whatever point of the
// millisecond clock the SR00 command happened to land on, so the FIRST cycle is
// a fragment -- a short high, then a long low -- and the evaluator, which
// requires every width to be exactly the commanded one, reads the fragment as a
// spurious edge. Measured on the ESP-IDF build: one ~9 ms width and one ~400 ms
// width per channel where 50..400/600..950 ms were commanded. Anchoring the
// phase to the start of the self-test makes every cycle whole, which is also
// what makes the measured duty unbiased.
static uint32_t phase0_ms = 0;

void saleae_test_setup(void) {
  phase0_ms = saleae_hal_millis();
  for (int i = 0; i < SALEAE_PIN_COUNT; i++) {
    saleae_hal_pin_output(PIN_OF(i));
    saleae_hal_write(PIN_OF(i), 0);
  }
}

void saleae_test_stop(void) {
  for (int i = 0; i < SALEAE_PIN_COUNT; i++) {
    saleae_hal_write(PIN_OF(i), 0);
  }
}

// Sleep until just before the next edge, then spin the last moment.
//
// The edges are at phase 0, 50, 100 ... 400 ms -- one every 50 ms -- so between
// them this loop has nothing to do. Spinning through all of it is what used to
// be here, and it starved IDLE0 into a task-watchdog panic: on ESP-IDF 5.5.3
// `main` runs `saleae_test_loop()` in a `saleae_hal_delay_ms(1)` spin, IDLE0 is
// subscribed to the WDT by default, and it never gets to run. The panic landed
// inside SR_00's capture window and truncated the pattern, which SR_00 then
// reported as eight dead pins.
//
// So the spin is kept for the window where it is load-bearing -- the edge has to
// land within the evaluator's 2 ms tolerance -- and the quiet time between edges
// blocks instead. `SPIN_WINDOW_MS` is comfortably wider than that tolerance and
// narrower than the 50 ms between edges, so an edge is always approached by
// spinning and never by a timer wake-up.
//
// The spin itself is unchanged (`saleae_hal_delay_ms(1)`, the sub-tick branch),
// because this is where an edge has to land; see saleae_hal_espidf.cpp.
#define SPIN_WINDOW_MS 12

void saleae_test_loop(void) {
  uint32_t now = saleae_hal_millis();
  uint16_t phase = (uint16_t)((now - phase0_ms) % PERIOD_MS);
  for (int i = 0; i < SALEAE_PIN_COUNT; i++) {
    saleae_hal_write(PIN_OF(i), phase < HIGH_MS_OF(i) ? 1 : 0);
  }

  // The next edge is the next 50 ms boundary: the high times are 50..400 in
  // steps of 50, so every boundary is an edge on exactly one channel.
  const uint32_t ms_to_edge = EDGE_GRID_MS - (phase % EDGE_GRID_MS);
  if (ms_to_edge > SPIN_WINDOW_MS) {
    saleae_hal_delay_ms(ms_to_edge - SPIN_WINDOW_MS);
    now = saleae_hal_millis();
    phase = (uint16_t)((now - phase0_ms) % PERIOD_MS);
    // Re-drive the pins: the block may have overshot the phase by up to a
    // tick, and a pin left at the previous phase would report a width that is
    // short by that much.
    for (int i = 0; i < SALEAE_PIN_COUNT; i++) {
      saleae_hal_write(PIN_OF(i), phase < HIGH_MS_OF(i) ? 1 : 0);
    }
  }
  saleae_hal_delay_ms(1);
}
