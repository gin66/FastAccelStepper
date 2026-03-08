// pd_avr/pd_config.h - AVR platform configuration
//
// This file defines AVR-specific constants for the FastAccelStepper library:
// - Queue topology (MAX_STEPPER, NUM_QUEUES, QUEUE_LEN)
// - Timing constants (TICKS_PER_S, MIN_CMD_TICKS, delays)
// - Feature flags for AVR-specific behavior
//
// Included by fas_arch/common.h during platform dispatch.

#ifndef PD_AVR_CONFIG_H
#define PD_AVR_CONFIG_H

#define QUEUE_LEN 16
#define TICKS_PER_S F_CPU
#define MIN_CMD_TICKS (TICKS_PER_S / 25000)
#define MIN_DIR_DELAY_US 40
#define MAX_DIR_DELAY_US (65535 / (TICKS_PER_S / 1000000))
#define DELAY_MS_BASE (65536000 / TICKS_PER_S)

#define DEBUG_LED_HALF_PERIOD (TICKS_PER_S / 65536 / 2)

#define noop_or_wait

#define SUPPORT_DIR_PIN_MASK uint8_t

#define SUPPORT_QUEUE_ENTRY_END_POS_U16

#define SUPPORT_UNSAFE_ABS_SPEED_LIMIT_SETTING

#define NEED_ADJUSTABLE_MAX_SPEED_DEPENDING_ON_STEPPER_COUNT
#define NEED_FIXED_QUEUE_TO_PIN_MAPPING

//==========================================================================
//
// AVR derivate ATmega 168/328/P
//
//==========================================================================
#if (defined(__AVR_ATmega168__) || defined(__AVR_ATmega168P__) || \
     defined(__AVR_ATmega328__) || defined(__AVR_ATmega328P__))
#define MAX_STEPPER 2
#define NUM_QUEUES 2
//==========================================================================
//
// AVR derivate ATmega 2560
//
//==========================================================================
#elif defined(__AVR_ATmega2560__)
#define MAX_STEPPER 3
#define NUM_QUEUES 3
//==========================================================================
//
// AVR derivate ATmega 32U4
//
//==========================================================================
#elif defined(__AVR_ATmega32U4__)
#define MAX_STEPPER 3
#define NUM_QUEUES 3
//==========================================================================
//
// For all unsupported AVR derivates
//
//==========================================================================
#else
#error "Unsupported AVR derivate"
#endif

#endif /* PD_AVR_CONFIG_H */
