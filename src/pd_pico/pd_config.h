// pd_pico/pd_config.h - RP2040/RP2350 platform configuration
//
// This file defines Pico-specific constants for the FastAccelStepper library:
// - Queue topology (MAX_STEPPER, NUM_QUEUES, QUEUE_LEN)
// - Timing constants (TICKS_PER_S, MIN_CMD_TICKS, delays)
// - Feature flags for Pico-specific behavior
//
// Included by fas_arch/common.h during platform dispatch.

#ifndef PD_PICO_CONFIG_H
#define PD_PICO_CONFIG_H

#include <FreeRTOS.h>
#include <task.h>

#define SUPPORT_UNSAFE_ABS_SPEED_LIMIT_SETTING

#define NUM_QUEUES 4 * NUM_PIOS
#define MAX_STEPPER (NUM_QUEUES)
#define QUEUE_LEN 32

#define TICKS_PER_S 16000000L
#define MIN_CMD_TICKS (TICKS_PER_S / 5000)
#define MIN_DIR_DELAY_US (MIN_CMD_TICKS / (TICKS_PER_S / 1000000))
#define MAX_DIR_DELAY_US (65535 / (TICKS_PER_S / 1000000))
#define DELAY_MS_BASE 4

#define DEBUG_LED_HALF_PERIOD 50

#define noop_or_wait vTaskDelay(1)

#define SUPPORT_TASK_RATE_CHANGE

#endif /* PD_PICO_CONFIG_H */
