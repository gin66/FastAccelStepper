// pd_sam/pd_config.h - SAM Due platform configuration
//
// This file defines SAM-specific constants for the FastAccelStepper library:
// - Queue topology (MAX_STEPPER, NUM_QUEUES, QUEUE_LEN)
// - Timing constants (TICKS_PER_S, MIN_CMD_TICKS, delays)
// - Feature flags for SAM-specific behavior
//
// Included by fas_arch/common.h during platform dispatch.

#ifndef PD_SAM_CONFIG_H
#define PD_SAM_CONFIG_H

#define MAX_STEPPER 6
#define NUM_QUEUES 6
#define QUEUE_LEN 32

#define TICKS_PER_S 21000000L
#define MIN_CMD_TICKS (TICKS_PER_S / 5000)
#define MIN_DIR_DELAY_US (MIN_CMD_TICKS / (TICKS_PER_S / 1000000))
#define MAX_DIR_DELAY_US (65535 / (TICKS_PER_S / 1000000))
#define DELAY_MS_BASE 2

#define DEBUG_LED_HALF_PERIOD 50

#define noop_or_wait

#define SUPPORT_DIR_PIN_MASK uint32_t

// TO BE CHECKED
#define SUPPORT_QUEUE_ENTRY_END_POS_U16

#endif /* PD_SAM_CONFIG_H */
