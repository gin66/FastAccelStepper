// pd_teensy/pd_config.h - Teensy 4.x (i.MX RT1062) platform configuration
//
// !! EXPERIMENTAL !!
// This backend is adapted from two sources that were verified on real
// hardware:
//  - the register-level QuadTimer (TMR) sequence in luni64/TeensyStep4
//    (MIT licensed), a proven stepper library for Teensy 4.x
//  - the queue/ramp protocol already shipped here for SAMD51
//    (src/pd_samd), which this backend's structure mirrors
// It has since been tested on a real Teensy 4.0 with a DM556 industrial
// stepper driver: a single axis was speed-swept up to 200 kHz with no
// missed steps (verified by marking the shaft - there is no pulse
// counter/encoder feedback to check this in software), and all 16
// steppers were run simultaneously across all 4 QuadTimer modules with
// no cross-talk between channels. Pulse width/edge timing has not been
// checked on a scope or logic analyzer yet - if you have one, please
// verify and report back.
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
// (confirmed via TeensyStep4's TMR.h). FAS_TEENSY_TMR_PRESCALE (0..7)
// divides that by 2^prescale (see teensy_queue.cpp) to get the tick rate:
//   0 -> 150 MHz    (65535 ticks =~ 0.44 ms per queue entry)
//   4 -> 9.375 MHz  (default: =~ 7 ms per queue entry)
//   7 -> 1.17 MHz   (=~ 56 ms per queue entry)
// A smaller prescale gives finer timing resolution (matters most for very
// high step rates and for how smoothly the ramp can approximate its curve),
// at the cost of a shorter max duration per single queue entry - still
// comfortably above the 4 ms ramp tick down to prescale 2 (37.5 MHz,
// =~1.75 ms/entry). Untested: worth sweeping this while measuring actual
// achievable step rate/jitter on a scope, see pd_teensy warning above.
#ifndef FAS_TEENSY_TMR_PRESCALE
#define FAS_TEENSY_TMR_PRESCALE 4
#endif
#if (FAS_TEENSY_TMR_PRESCALE < 0) || (FAS_TEENSY_TMR_PRESCALE > 7)
#error "FAS_TEENSY_TMR_PRESCALE must be 0..7"
#endif
//
// This does not match the two precomputed fast-path constants in
// RampCalculator.h (16 MHz / 21 MHz), so the generic runtime log2 timer
// frequency path is used automatically (see SUPPORT_LOG2_TIMER_FREQ_VARIABLES
// in RampCalculator.h/RampControl.cpp) - no extra setup needed here.
#define TICKS_PER_S (150000000L >> FAS_TEENSY_TMR_PRESCALE)
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
