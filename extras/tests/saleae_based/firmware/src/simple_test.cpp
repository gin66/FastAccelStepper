/*
 * simple_test.cpp — SR_00 connection self-test.
 *
 * Toggles all 8 identification pins at exactly 1 Hz (1000 ms period), each
 * with a different high time (5 % .. 40 % in 5 % steps). Since no channel is
 * 50 % duty, an electrically inverted channel shows up as the complement
 * duty (95 % .. 60 %) and is immediately recognisable as wrong. This proves
 * that every ESP32 I/O used by the harness is alive and correctly wired.
 *
 *   CH 0 (D0): 1 Hz,  5 % high ( 50 ms)
 *   CH 1 (D1): 1 Hz, 10 % high (100 ms)
 *   CH 2 (D2): 1 Hz, 15 % high (150 ms)
 *   CH 3 (D3): 1 Hz, 20 % high (200 ms)
 *   CH 4 (D4): 1 Hz, 25 % high (250 ms)
 *   CH 5 (D5): 1 Hz, 30 % high (300 ms)
 *   CH 6 (D6): 1 Hz, 35 % high (350 ms)
 *   CH 7 (D7): 1 Hz, 40 % high (400 ms)
 *
 * The file can be flashed standalone (Arduino sketch entry points below) or
 * linked into the main test app as a module. Define SIMPLE_TEST_INTEGRATED to
 * suppress the standalone setup()/loop() and avoid clashing with main.cpp.
 */

#include <Arduino.h>
#include <stdint.h>

#define SIMPLE_TEST_PIN_COUNT 8

// 8 pins matching white paper Section 3.3 — ESP32-DevKitC
static const uint8_t simple_pins[SIMPLE_TEST_PIN_COUNT] = {2, 0,  4,  16,
                                                           17, 5, 18, 19};

// High time in milliseconds for a fixed 1000 ms (1 Hz) period.
static const uint16_t simple_high_ms[SIMPLE_TEST_PIN_COUNT] = {
    50, 100, 150, 200, 250, 300, 350, 400};

static bool simple_active = false;

void simple_test_start(void) {
  for (int i = 0; i < SIMPLE_TEST_PIN_COUNT; i++) {
    pinMode(simple_pins[i], OUTPUT);
    digitalWrite(simple_pins[i], LOW);
  }
  simple_active = true;
}

void simple_test_stop(void) {
  for (int i = 0; i < SIMPLE_TEST_PIN_COUNT; i++) {
    digitalWrite(simple_pins[i], LOW);
  }
  simple_active = false;
}

bool simple_test_is_active(void) { return simple_active; }

void simple_test_loop(void) {
  if (!simple_active) {
    return;
  }
  uint16_t phase = (uint16_t)(millis() % 1000);
  for (int i = 0; i < SIMPLE_TEST_PIN_COUNT; i++) {
    digitalWrite(simple_pins[i], phase < simple_high_ms[i] ? HIGH : LOW);
  }
}

#ifndef SIMPLE_TEST_INTEGRATED
void setup() { simple_test_start(); }

void loop() {
  simple_test_loop();
  delay(1);
}
#endif
