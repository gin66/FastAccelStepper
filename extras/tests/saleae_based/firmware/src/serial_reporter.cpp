/*
 * serial_reporter.cpp — Sends metrics back to host via serial.
 *
 * Provides per-stepper metrics:
 *   - step_count: Total step pulses counted
 *   - dir_to_step_us: Direction-to-first-step delay
 *   - glitch_count: Spurious edges detected
 *   - avg_inter_step_us: Average inter-step period
 */

#include <Arduino.h>
#include <stdint.h>

// Metrics structure for a single stepper
typedef struct {
  uint32_t step_count;
  float dir_to_step_us;
  uint32_t glitch_count;
  float avg_inter_step_us;
  float max_pulse_width_us;
} stepper_metrics_t;

// Per-stepper metric storage (platform-specific counters)
#define MAX_STEPPERS 8
static stepper_metrics_t g_metrics[MAX_STEPPERS];
static bool g_metrics_valid[MAX_STEPPERS];

static void update_metrics(uint8_t stepper_idx) {
  // Platform-specific: read hardware counters
  // This is implemented in the platform adapter
  // For now, return placeholder values
  g_metrics[stepper_idx].step_count = 0;
  g_metrics[stepper_idx].dir_to_step_us = 0.0f;
  g_metrics[stepper_idx].glitch_count = 0;
  g_metrics[stepper_idx].avg_inter_step_us = 0.0f;
  g_metrics[stepper_idx].max_pulse_width_us = 0.0f;
  g_metrics_valid[stepper_idx] = false;
}

static void get_metrics_response(uint8_t stepper_idx, char *response, uint16_t response_len) {
  if (stepper_idx >= MAX_STEPPERS || !g_metrics_valid[stepper_idx]) {
    snprintf(response, response_len, "ERR: stepper %u not initialized", stepper_idx);
    send_error_response(0x05);
    return;
  }

  snprintf(response, response_len,
           "steps=%lu, dt=%.1fus, glitches=%lu, avg_dt=%.1fus, max_pw=%.1fus",
           (unsigned long)g_metrics[stepper_idx].step_count,
           g_metrics[stepper_idx].dir_to_step_us,
           (unsigned long)g_metrics[stepper_idx].glitch_count,
           g_metrics[stepper_idx].avg_inter_step_us,
           g_metrics[stepper_idx].max_pulse_width_us);
  send_ok_response(response);
}

// ISR-based metric collection (platform-specific)
// Called from RMT/MCPWM/I2S interrupt handlers
static void isr_record_step(uint8_t stepper_idx, uint32_t timestamp_ns) {
  if (stepper_idx >= MAX_STEPPERS) return;

  g_metrics[stepper_idx].step_count++;

  // Track edge timestamps for inter-step period calculation
  // (circular buffer of timestamps)

  // Detect glitches (pulse width < MIN_CMD_TICKS / 2)
  // (compare with previous edge timestamp)
}

static void isr_record_direction_change(uint8_t stepper_idx, uint32_t timestamp_ns) {
  if (stepper_idx >= MAX_STEPPERS) return;

  // Record direction change timestamp for dir→step delay calculation
  // (first step after this timestamp = dir_to_step delay)
}

// Forward declaration from serial_protocol.cpp
extern void send_ok_response(const char *msg);
extern void send_error_response(uint8_t error_code);