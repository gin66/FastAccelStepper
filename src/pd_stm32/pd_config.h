#ifndef PD_STM32_CONFIG_H
#define PD_STM32_CONFIG_H

#include <stdint.h>

// ====================================================================
// Compile-time TICKS_PER_S
//
// User MUST define TICKS_PER_S matching their board's TIM2 counter clock.
// TIM2 counter clock = PCLK1 * (APB1_prescaler==1 ? 1 : 2)
//
// Examples:
//   STM32F103 @72MHz:  TICKS_PER_S = 72000000  (APB1=36MHz ×2)
//   STM32F407 @168MHz: TICKS_PER_S = 84000000  (APB1=42MHz ×2)
//   STM32G0   @64MHz:  TICKS_PER_S = 64000000  (APB1=64MHz ×1)
//   STM32H743 @480MHz: TICKS_PER_S = 240000000 (APB1=120MHz ×2)
// ====================================================================
#ifndef TICKS_PER_S
#define TICKS_PER_S 72000000UL
#endif

// ---- Queue topology ----
#define MAX_STEPPER 4
#define NUM_QUEUES  4
#define QUEUE_LEN   32

// ---- Pulse width (configurable) ----
#ifndef STEP_PULSE_WIDTH_US
#define STEP_PULSE_WIDTH_US 6
#endif
#define STEP_PULSE_WIDTH_TICKS ((uint32_t)(STEP_PULSE_WIDTH_US * (TICKS_PER_S / 1000000L)))

// ---- Timing constants ----
#define MIN_CMD_TICKS          (TICKS_PER_S / 5000)
#define MIN_DIR_DELAY_US       200
#define MAX_DIR_DELAY_US       (65535 / (TICKS_PER_S / 1000000))
#define DELAY_MS_BASE          2
#define CYCLIC_INTERVAL_MS     3

// ====================================================================
// NOTE: STM32F1 TIM2 is 16-bit only.
// ARR = 0xFFFFFFFF is masked to 0xFFFF by F1 hardware.
// Minimum speed = TICKS_PER_S / 65536 ≈ 1098 steps/s @72MHz.
// ====================================================================

// ---- Feature flags ----
#define SUPPORT_QUEUE_ENTRY_END_POS_U16
#define NEED_GENERIC_GET_CURRENT_POSITION
#define noop_or_wait __NOP()
#define DEBUG_LED_HALF_PERIOD 50

#endif /* PD_STM32_CONFIG_H */