// pd_samd/pd_config.h - SAMD51 platform configuration
//
// This file defines SAMD51-specific constants for the FastAccelStepper
// library:
// - Queue topology (MAX_STEPPER, NUM_QUEUES, QUEUE_LEN)
// - Timing constants (TICKS_PER_S, MIN_CMD_TICKS, delays)
// - Feature flags for SAMD51-specific behavior
//
// Included by fas_arch/common.h during platform dispatch.

#ifndef PD_SAMD_CONFIG_H
#define PD_SAMD_CONFIG_H

// One stepper per TCC instance: the period register (PER) is shared by all
// channels of a TCC, so a TCC can only generate one step interval stream.
// TCC_INST_NUM comes from the CMSIS device header (3 on SAMD51G, 5 on J/N/P).
#define MAX_STEPPER (TCC_INST_NUM)
#define NUM_QUEUES (TCC_INST_NUM)
#define QUEUE_LEN 32

// Queue TCCs run at exactly 16 MHz from a dedicated GCLK generator (DFLL48M
// divided by 3), matching the tick rate of the AVR/ESP32/Pico ports so the
// well-tested precomputed ramp math constants apply (see ADR 0002).
// 16-bit entry ticks => max 4.096 ms per queue entry; slower step rates are
// represented by chained pause entries (upstream generic mechanism).
#define TICKS_PER_S 16000000L
// A command must span >= 200 us, so a full queue (32 entries) holds >= 6.4 ms
// of runway against the 4 ms ramp tick.
#define MIN_CMD_TICKS (TICKS_PER_S / 5000)
#define MIN_DIR_DELAY_US (MIN_CMD_TICKS / (TICKS_PER_S / 1000000))
#define MAX_DIR_DELAY_US (65535 / (TICKS_PER_S / 1000000))
#define DELAY_MS_BASE 4

#define DEBUG_LED_HALF_PERIOD 50

#define noop_or_wait

#define SUPPORT_QUEUE_ENTRY_END_POS_U16

#define NEED_GENERIC_GET_CURRENT_POSITION

#define SUPPORT_UNSAFE_ABS_SPEED_LIMIT_SETTING

// The generic clock generator claimed for the 16 MHz step timebase.
// The Adafruit core startup uses generators 0-5 (CPU, 48M, 100M, XOSC32K,
// 12M, 1M-for-DPLL); 6-11 are free. Override if another library in your
// project claims GEN6. Never point this at a generator the core owns.
#ifndef FAS_SAMD_GCLK_GEN
#define FAS_SAMD_GCLK_GEN 6
#endif

// The plain TC instance driving the ~4 ms ramp tick (manageSteppers()).
// Override with e.g. -DFAS_SAMD_RAMP_TC=2 if your project claims TC3.
// TC0 is used by the Arduino core's tone().
#ifndef FAS_SAMD_RAMP_TC
#define FAS_SAMD_RAMP_TC 3
#endif

// Step pulse high time in microseconds. 2 us satisfies all mainstream step/dir
// drivers (DRV8825 needs 1.9 us; A4988 needs ~1 us; Trinamic less).
#ifndef FAS_SAMD_PULSE_WIDTH_US
#define FAS_SAMD_PULSE_WIDTH_US 2
#endif

#endif /* PD_SAMD_CONFIG_H */
