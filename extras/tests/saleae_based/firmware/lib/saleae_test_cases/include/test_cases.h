/*
 * test_cases.h — Generic test case definitions for Saleae test firmware.
 *
 * All platforms share this library — platform-specific behavior
 * is handled by the adapter layer.
 *
 * Test case IDs (SR_XX):
 *   SR_00:  port_toggle (connection verification)
 *   SR_01:  basic_move_forward
 *   SR_02:  basic_move_reverse
 *   SR_03:  mixed_direction_ramp
 *   SR_04:  acceleration_ramp
 *   SR_05:  speed_profile_multi_phase
 *   SR_06:  direction_to_step_delay
 *   SR_07:  step_pulse_width_at_speed
 *   SR_08:  duty_cycle_symmetry
 *   SR_09:  abrupt_speed_change
 *   SR_10:  min_tick_boundary
 *   SR_11:  max_speed_boundary
 *   SR_12:  queue_fill_latency
 *   SR_13:  sync_start_same_tick
 *   SR_14:  sync_start_delayed_start
 *   SR_15:  sync_start_different_speeds
 *   SR_16:  sync_start_n_axis
 *   SR_17:  sync_start_cross_driver
 *   SR_18:  queue_full_behavior
 *   SR_19:  queue_empty_prevention
 *   SR_20:  moveTimed_accuracy
 *   SR_21:  moveTimed_direction_change
 *   SR_22:  pause_command_insertion
 *   SR_23:  queue_overflow_cycling
 *   SR_24:  moveTimed_drift_check
 *   SR_25:  rmt_buffer_split
 *   SR_26:  rmt_v2_fill_encoder
 *   SR_27:  i2s_direct_timing
 *   SR_28:  i2s_mux_timing
 *   SR_29:  mcpwm_pcnt_sync
 *   SR_30:  rmt_sync_manager
 *   SR_31:  8ch_step_only_max_speed
 *   SR_32:  7ch_shared_dir_consistency
 *   SR_33:  mixed_driver_interference
 *   SR_34:  i2s_extender_scaling
 *   SR_35:  channel_reassignment
 *   SR_36:  gpio_pin_reuse
 *   SR_37:  interrupt_load
 *   SR_38:  power_sag_recovery
 *   SR_39:  emergency_stop
 *   SR_40:  overflow_wraparound
 */

#ifndef TEST_CASES_H
#define TEST_CASES_H

#include <stdint.h>

// Test case identifier
typedef struct {
  uint8_t id;           // SR_XX number (0–40)
  const char *name;     // Human-readable name
  const char *category; // Category (e.g., "connection", "ramp", "timing")
  uint8_t steppers;     // Number of steppers involved
  uint8_t flags;        // Test flags (e.g., requires hardware, ESP32-only)
} test_case_t;

// Available test cases (defined in test_connections.cpp, test_basic.cpp, etc.)
extern const test_case_t test_cases[];
extern const uint8_t test_case_count;

// SR_00: Port toggle (connection verification)
void test_sr00_port_toggle(uint8_t pin, uint32_t hz);
void test_sr00_port_toggle_stop(void);

// SR_01–SR_05: Basic ramp tests
void test_sr01_basic_move_forward(uint32_t steps, uint32_t speed_us);
void test_sr02_basic_move_reverse(uint32_t steps, uint32_t speed_us);
void test_sr03_mixed_direction_ramp(uint32_t *steps_array, uint8_t count, uint32_t speed_us);
void test_sr04_acceleration_ramp(uint32_t steps, uint32_t start_speed_us, uint32_t end_speed_us, uint32_t acceleration);
void test_sr05_speed_profile_multi_phase(uint32_t steps, uint32_t *speeds_array, uint8_t phases);

// SR_06–SR_12: Timing precision tests
void test_sr06_direction_to_step_delay(uint32_t speed_us, float *result_us);
void test_sr07_step_pulse_width_at_speed(uint32_t speed_us, float *width_us);
void test_sr08_duty_cycle_symmetry(uint32_t speed_us, float *deviation_percent);
void test_sr09_abrupt_speed_change(uint32_t high_speed_us, uint32_t low_speed_us);
void test_sr10_min_tick_boundary(void);
void test_sr11_max_speed_boundary(uint32_t steps);
void test_sr12_queue_fill_latency(uint32_t steps, float *latency_us);

// SR_13–SR_17: Synchronized start tests
void test_sr13_sync_start_same_tick(uint32_t steps);
void test_sr14_sync_start_delayed_start(uint32_t steps, uint32_t *delays_us);
void test_sr15_sync_start_different_speeds(uint32_t steps, uint32_t *speeds_array);
void test_sr16_sync_start_n_axis(uint32_t steps, int32_t *targets);
void test_sr17_sync_start_cross_driver(uint32_t steps);

// SR_18–SR_24: Queue management tests
void test_sr18_queue_full_behavior(uint32_t steps);
void test_sr19_queue_empty_prevention(uint32_t steps);
void test_sr20_moveTimed_accuracy(uint32_t steps, uint32_t duration_ms, int32_t *target_pos);
void test_sr21_moveTimed_direction_change(uint32_t steps, int32_t mid_point);
void test_sr22_pause_command_insertion(uint32_t steps, uint32_t pause_ticks);
void test_sr23_queue_overflow_cycling(uint32_t cycles);
void test_sr24_moveTimed_drift_check(uint32_t cycles, uint32_t steps);

// SR_25–SR_30: Driver-specific tests
void test_sr25_rmt_buffer_split(void);
void test_sr26_rmt_v2_fill_encoder(void);
void test_sr27_i2s_direct_timing(uint32_t steps);
void test_sr28_i2s_mux_timing(uint32_t steps);
void test_sr29_mcpwm_pcnt_sync(void);
void test_sr30_rmt_sync_manager(void);

// SR_31–SR_35: Channel configuration stress tests
void test_sr31_8ch_step_only_max_speed(void);
void test_sr32_7ch_shared_dir_consistency(void);
void test_sr33_mixed_driver_interference(void);
void test_sr34_i2s_extender_scaling(uint8_t stepper_count);
void test_sr35_channel_reassignment(void);

// SR_36–SR_40: Edge cases and error conditions
void test_sr36_gpio_pin_reuse(uint8_t pin);
void test_sr37_interrupt_load(void);
void test_sr38_power_sag_recovery(void);
void test_sr39_emergency_stop(uint32_t steps);
void test_sr40_overflow_wraparound(uint32_t steps);

#endif  // TEST_CASES_H