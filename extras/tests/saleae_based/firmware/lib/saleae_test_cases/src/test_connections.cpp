/*
 * test_connections.cpp — SR_00: Port toggle (connection verification).
 *
 * Runs BEFORE any stepper-motion test (SR_01–SR_40).
 * Verifies that every Saleae channel is electrically connected
 * to the correct GPIO and that the firmware can toggle pins
 * without loading the stepper driver.
 *
 * Protocol:
 *   HOST → FIRMWARE: PORT_TOGGLE <pin> <hz>
 *   FIRMWARE → HOST: OK: pin=<pin> freq=<hz>Hz
 *   HOST → FIRMWARE: PORT_TOGGLE_STOP
 *   FIRMWARE → HOST: OK: stopped
 *
 * Firmware toggles each configured pin at the specified frequency
 * (50% duty square wave) indefinitely until STOP is received.
 * The host captures a few seconds on Saleae, verifies the waveform
 * on every channel, then sends STOP.
 *
 * If SR_00 fails (missing channel, no signal, wrong frequency),
 * the remaining tests are skipped — the harness does not proceed
 * with stepper motion until the wiring is corrected.
 */

#include <Arduino.h>
#include <stdint.h>
#include <string.h>

// Maximum number of pins that can be toggled simultaneously
#define MAX_TOGGLE_PINS 16

// Toggle state for each pin
typedef struct {
  uint8_t pin;
  uint32_t hz;
  uint32_t half_period_us;
  uint32_t last_toggle_ms;
  bool active;
} toggle_pin_t;

static toggle_pin_t g_toggle_pins[MAX_TOGGLE_PINS];
static uint8_t g_toggle_count = 0;

void test_sr00_port_toggle(uint8_t pin, uint32_t hz) {
  if (g_toggle_count >= MAX_TOGGLE_PINS) {
    // Already at maximum — return error
    return;
  }

  // Find free slot or reuse existing pin
  int slot = -1;
  for (int i = 0; i < MAX_TOGGLE_PINS; i++) {
    if (!g_toggle_pins[i].active || g_toggle_pins[i].pin == pin) {
      slot = i;
      break;
    }
  }

  if (slot < 0) return;

  // Configure pin as output
  pinMode(pin, OUTPUT);

  // Calculate half-period in microseconds
  uint32_t period_us = 1000000 / hz;
  uint32_t half_period_us = period_us / 2;

  g_toggle_pins[slot].pin = pin;
  g_toggle_pins[slot].hz = hz;
  g_toggle_pins[slot].half_period_us = half_period_us;
  g_toggle_pins[slot].last_toggle_ms = millis();
  g_toggle_pins[slot].active = true;

  // Send OK response
  char response[64];
  snprintf(response, sizeof(response), "OK: pin=%u freq=%luHz", pin, (unsigned long)hz);
  send_ok_response(response);
}

void test_sr00_port_toggle_stop(void) {
  for (int i = 0; i < g_toggle_count; i++) {
    if (g_toggle_pins[i].active) {
      digitalWrite(g_toggle_pins[i].pin, LOW);
      g_toggle_pins[i].active = false;
    }
  }

  g_toggle_count = 0;
  send_ok_response("OK: stopped");
}

// Called from loop() to toggle pins at specified frequencies
void toggle_pins_loop(void) {
  uint32_t now = millis();

  for (int i = 0; i < MAX_TOGGLE_PINS; i++) {
    if (g_toggle_pins[i].active) {
      uint32_t elapsed = now - g_toggle_pins[i].last_toggle_ms;

      if (elapsed >= g_toggle_pins[i].half_period_us / 1000) {
        // Toggle pin
        static bool pin_states[MAX_TOGGLE_PINS];
        pin_states[i] = !pin_states[i];
        digitalWrite(g_toggle_pins[i].pin, pin_states[i] ? HIGH : LOW);
        g_toggle_pins[i].last_toggle_ms = now;
      }
    }
  }
}

// Forward declaration from serial_protocol.cpp
extern void send_ok_response(const char *msg);