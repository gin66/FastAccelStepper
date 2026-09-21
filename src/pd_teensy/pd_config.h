// pd_teensy/pd_config.h - Teensy 4.x (i.MX RT1062) platform configuration
//
// !! EXPERIMENTAL / NOT YET VERIFIED ON REAL HARDWARE !!
// This backend was written without access to a Teensy 4.x board, a
// compiler for it, or an oscilloscope. It is adapted from two sources
// that WERE verified on real hardware:
//  - the register-level QuadTimer (TMR) sequence in luni64/TeensyStep4
//    (MIT licensed), a proven stepper library for Teensy 4.x
//  - the queue/ramp protocol already shipped here for SAMD51
//    (src/pd_samd), which this backend's structure mirrors
// Before relying on this in a real project: build it, and check step
// timing/pulse width on a scope or logic analyzer at the speeds you need.
//
// This file defines Teensy-4.x-specific constants for the
// FastAccelStepper library:
// - Queue topology (MAX_STEPPER, NUM_QUEUES, QUEUE_LEN)
// - Timing constants (TICKS_PER_S, MIN_CMD_TICKS, delays)
// - Feature flags for Teensy-4.x-specific behavior
//
// Included by fas_arch/common.h during platform dispatch.

#ifndef PD_TEENSY_CONFIG_H
#define PD_TEENSY_CONFIG_H

// i.MX RT1062 has 4 QuadTimer (TMR) modules, 4 channels each. Unlike
// SAMD/SAM, the step pin is NOT muxed to the timer's own output - the ISR
// toggles it directly via digitalWriteFast() (see pd_teensy/teensy_queue.cpp),
// so any digital pin can be used, same as ESP32.
#define MAX_STEPPER 16
#define NUM_QUEUES 16
#define QUEUE_LEN 32

// The QuadTimer modules are clocked from the IPG peripheral clock, which
// Teensyduino configures to 150 MHz regardless of F_CPU/overclocking
// (confirmed via TeensyStep4's TMR.h). Prescaler divides by 16 (2^4),
// giving a 9.375 MHz tick rate: up to 65535/9.375e6 =~ 7 ms per single
// queue entry, comfortably above the 4 ms ramp tick period.
//
// This does not match the two precomputed fast-path constants in
// RampCalculator.h (16 MHz / 21 MHz), so the generic runtime log2 timer
// frequency path is used automatically (see SUPPORT_LOG2_TIMER_FREQ_VARIABLES
// in RampCalculator.h/RampControl.cpp) - no extra setup needed here.
#define TICKS_PER_S 9375000L
#define MIN_CMD_TICKS (TICKS_PER_S / 5000)
#define MIN_DIR_DELAY_US (MIN_CMD_TICKS / (TICKS_PER_S / 1000000))
#define MAX_DIR_DELAY_US (65535 / (TICKS_PER_S / 1000000))
#define DELAY_MS_BASE 4

#define DEBUG_LED_HALF_PERIOD 50

#define noop_or_wait

#define SUPPORT_QUEUE_ENTRY_END_POS_U16

#define NEED_GENERIC_GET_CURRENT_POSITION

// Lets the application raise/lower the per-stepper max speed ceiling set
// below via setAbsoluteSpeedLimit() - useful here since the default ceiling
// is an untested guess (see teensy_queue.cpp _pd_initVars()).
#define SUPPORT_UNSAFE_ABS_SPEED_LIMIT_SETTING

// Step pulse high time in microseconds. Kept above DRV8825's 1.9 us
// minimum with margin for the tick-rounding below (2 us truncates to
// under 1.9 us at 9.375 MHz ticks).
#ifndef FAS_TEENSY_PULSE_WIDTH_US
#define FAS_TEENSY_PULSE_WIDTH_US 3
#endif

#endif /* PD_TEENSY_CONFIG_H */
