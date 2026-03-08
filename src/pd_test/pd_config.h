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

#define TICKS_PER_S 16000000L
#define MIN_CMD_TICKS (TICKS_PER_S / 5000)
#define MIN_DIR_DELAY_US (MIN_CMD_TICKS / (TICKS_PER_S / 1000000))
#define MAX_DIR_DELAY_US (65535 / (TICKS_PER_S / 1000000))
#define DELAY_MS_BASE 1
#define SUPPORT_UNSAFE_ABS_SPEED_LIMIT_SETTING

#define noop_or_wait

#define SUPPORT_QUEUE_ENTRY_END_POS_U16

#endif /* PD_TEST_CONFIG_H */
