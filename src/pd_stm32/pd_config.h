#ifndef PD_STM32_CONFIG_H
#define PD_STM32_CONFIG_H

#include <stdint.h>

// ====================================================================
// Compile-time TICKS_PER_S
//
// Default is 16MHz. Each board's TIM2 prescaler (stm32_queue.cpp) brings the
// actual timer clock close to this value, ensuring all tick values fit in uint16_t
// (no overflow for 1ms/2ms pipeline operations).
//
// Timer clock formula: TIM_CLK = PCLK1 * (APB1_prescaler > 1 ? 2 : 1)
// Prescaler formula: PSC = (actual_timer_clk / TICKS_PER_S) - 1
//
// CI boards with override (build_flags_extra in build_matrix.yaml):
//   F103 (bluepill):  -DTICKS_PER_S=18000000  (72M÷4, PSC=3)  → 18MHz
//   F401 (blackpill): -DTICKS_PER_S=16800000  (84M÷5, PSC=4)  → 16.8MHz
//   H743 (nucleo):    -DTICKS_PER_S=20000000  (200M÷10, PSC=9) → 20MHz
//
// CI boards using default (no override):
//   G070 (nucleo):    64M÷4 (PSC=3) → 16MHz — exact
//   L476 (nucleo):    80M÷5 (PSC=4) → 16MHz — exact
//
// If TICKS_PER_S is not a supported predefined value, RampCalculator.h will
// emit a clear #error. Always use timer prescaler to match a supported value.
// ====================================================================
#ifndef TICKS_PER_S
#define TICKS_PER_S 16000000UL
#endif

// ---- Queue topology ----
#define MAX_STEPPER 4
#define NUM_QUEUES  4
#define QUEUE_LEN   32

// ---- Pulse width (configurable) ----
#ifndef STEP_PULSE_WIDTH_US
#define STEP_PULSE_WIDTH_US 6
#endif
#define STEP_PULSE_WIDTH_TICKS ((uint32_t)(STEP_PULSE_WIDTH_US * (TICKS_PER_S / 1000000UL)))

// ---- Timing constants ----
#define MIN_CMD_TICKS          (TICKS_PER_S / 5000)
#define MIN_DIR_DELAY_US       200
#define MAX_DIR_DELAY_US       (65535 / (TICKS_PER_S / 1000000UL))
#define DELAY_MS_BASE          2
#define CYCLIC_INTERVAL_MS     3

// ====================================================================
// NOTE: STM32F1 TIM2 is 16-bit only.
// C0/G0 TIM3 is 16-bit (ARR=0xFFFF). With PSC=2 => timer=16MHz.
// L0 TIM2 is also 16-bit (RM0367 §24).
// Min speed = 16MHz / 65536 ≈ 244 steps/s.
// ARR = 0xFFFFFFFF is masked to 0xFFFF by F1 hardware.
// Minimum speed = TICKS_PER_S / 65536 ≈ 1098 steps/s @72MHz.
// ====================================================================

// ---- Feature flags ----
#define SUPPORT_QUEUE_ENTRY_END_POS_U16
#define NEED_GENERIC_GET_CURRENT_POSITION
#define noop_or_wait __NOP()
#define DEBUG_LED_HALF_PERIOD 50

// ====================================================================
// Timer Grouping — FAS_STM32_TIMER_16BIT / FAS_STM32_TIMER_32BIT
//
// Helper macro for fas_tim_set_ccr(). Mỗi family được liệt kê explicit.
// #else chỉ dùng cho #error — báo family không được hỗ trợ.
//
// CMSIS/RM-verified assignments:
//   16-bit timers (Group A+B):
//     Group A: TIM3 used (C0 không có TIM2, G0 TIM3 common denominator)
//     Group B: TIM2 HW-limited to 16-bit (F1, L0, L1)
//
//   32-bit timers (Group C):
//     TIM2 có 32-bit counter. Gồm F0, F2, F3, F4, F7, G4, H5, H7,
//     L4, L5, MP1, U0, U3, U5, WBA, WB, WL
//
//   Excluded:
//     WB0x — no general-purpose timer (BLE-optimized)
//     WL3x — no RCC_CFGR_PPRE register (non-standard clock tree)
// ====================================================================
#if defined(STM32C0xx) || defined(STM32G0xx)
  // Group A: TIM3 16-bit
  #define FAS_STM32_TIMER_16BIT

#elif defined(STM32F1xx) || defined(STM32L0xx) || defined(STM32L1xx)
  // Group B: TIM2 16-bit
  #define FAS_STM32_TIMER_16BIT

#elif defined(STM32F0xx) || defined(STM32F2xx) || defined(STM32F3xx) || \
      defined(STM32F4xx) || defined(STM32F7xx) || defined(STM32G4xx) || \
      defined(STM32H5xx) || defined(STM32H7xx) || defined(STM32L4xx) || \
      defined(STM32L5xx) || defined(STM32MP1xx) || defined(STM32U0xx) || \
      defined(STM32U3xx) || defined(STM32U5xx) || defined(STM32WBAxx) || \
      defined(STM32WBxx) || defined(STM32WLxx)
  // Group C: TIM2 32-bit (17 families)
  #define FAS_STM32_TIMER_32BIT

#elif defined(STM32WL3x)
  // WL3x has TIM2 (32-bit) but no RCC_CFGR_PPRE register
  #error "FAS: STM32WL3x unsupported — non-standard APB clock (no PPRE register)."

#elif defined(STM32WB0x)
  // WB0x has no general-purpose timer
  #error "FAS: STM32WB0x unsupported — no TIM2/TIM3 peripheral. \
Use STM32WBxx (Cortex-M4) for FAS support."

#else
  #error "FAS: Unsupported STM32 family. \
See src/pd_stm32/stm32_queue.cpp (product-line detection at line ~14). \
Add your board's product_line macro there, then add family \
(e.g. STM32XXxx) to this block and timer selection block."
#endif

// ====================================================================
// STM32 Family Guard
//
// Chỉ cho phép compile với các dòng STM32 đã được xác nhận.
// Nếu dòng của bạn chưa có trong danh sách, thêm macro tương ứng
// và kiểm tra các files cần sửa (xem README STM32 section).
//
// ⚠️ STM32WB0x and STM32WL3x are deliberately excluded:
//    WB0x — no general-purpose timer
//    WL3x — no RCC_CFGR_PPRE register (non-standard clock tree)
// ====================================================================
#if !defined(STM32C0xx) && !defined(STM32F0xx) && !defined(STM32F1xx) && \
    !defined(STM32F2xx) && !defined(STM32F3xx) && !defined(STM32F4xx) && \
    !defined(STM32F7xx) && !defined(STM32G0xx) && !defined(STM32G4xx) && \
    !defined(STM32H5xx) && !defined(STM32H7xx) && !defined(STM32L0xx) && \
    !defined(STM32L1xx) && !defined(STM32L4xx) && !defined(STM32L5xx) && \
    !defined(STM32MP1xx) && !defined(STM32U0xx) && !defined(STM32U3xx) && \
    !defined(STM32U5xx) && !defined(STM32WBAxx) && !defined(STM32WBxx) && \
    !defined(STM32WLxx)
#error "FAS: STM32 family not detected. \
See src/pd_stm32/stm32_queue.cpp (product-line detection at line ~14). \
Add your board's product_line macro there, then add family \
(e.g. STM32XXxx) to timer selection block (line ~110) and this guard."
#endif

#endif /* PD_STM32_CONFIG_H */