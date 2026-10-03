// pd_test/pd_config.h - Test platform configuration
//
// This file defines test-specific constants for the FastAccelStepper library:
// - Queue topology (MAX_STEPPER, NUM_QUEUES, QUEUE_LEN)
// - Timing constants (TICKS_PER_S, MIN_CMD_TICKS, delays)
// - Feature flags for PC-based testing
//
// Included by fas_arch/common.h during platform dispatch.

#ifndef PD_TEST_CONFIG_H
#define PD_TEST_CONFIG_H

#define MAX_STEPPER 2
#define NUM_QUEUES 2
#define QUEUE_LEN 16
#ifndef PART_SIZE
#define PART_SIZE debug_part_size
#endif

// RMT V2 fill model constants (test fallback).
// A symbol holds TWO sub-entries, so one RMT half (PART_SIZE symbols) spans at
// most 2*PART_SIZE*RMT_MAX_SYMBOL_TICKS. The divisor is 2*PART_SIZE: with
// PART_SIZE the half covered 2*RMT_BLOCK_TICKS, doubling the buffer and
// shortening the direction-change drain below the buffer's playback content.
#ifndef RMT_BLOCK_COUNT
#define RMT_BLOCK_COUNT 2
#endif
#ifndef RMT_BLOCK_TICKS
#define RMT_BLOCK_TICKS 8000
#endif
#ifndef RMT_MAX_INFLIGHT_TICKS
#define RMT_MAX_INFLIGHT_TICKS (RMT_BLOCK_COUNT * RMT_BLOCK_TICKS)
#endif
#ifndef RMT_MAX_SYMBOL_TICKS
#define RMT_MAX_SYMBOL_TICKS (RMT_BLOCK_TICKS / (2 * PART_SIZE))
#endif
#ifndef RMT_BUFFER_TICKS
#define RMT_BUFFER_TICKS (2 * PART_SIZE * 2 * RMT_MAX_SYMBOL_TICKS)
// Pause the driver injects before a direction change. It must EXCEED
// RMT_BUFFER_TICKS: the toggle runs at encode time, so a pause that fits
// in the buffer delays it not at all. One RMT_BLOCK_TICKS of margin.
#define RMT_DIR_DRAIN_TICKS (RMT_BUFFER_TICKS + RMT_BLOCK_TICKS)
#endif

#define TICKS_PER_S 16000000L
#define MIN_CMD_TICKS (TICKS_PER_S / 5000)
#define MIN_DIR_DELAY_US (MIN_CMD_TICKS / (TICKS_PER_S / 1000000))
#define MAX_DIR_DELAY_US (65535 / (TICKS_PER_S / 1000000))
#define DELAY_MS_BASE 1
#define SUPPORT_UNSAFE_ABS_SPEED_LIMIT_SETTING

#define noop_or_wait

#define SUPPORT_QUEUE_ENTRY_END_POS_U16

#define SUPPORT_PAUSE_CMD_COUNTING

#define NEED_GENERIC_GET_CURRENT_POSITION

#endif /* PD_TEST_CONFIG_H */
