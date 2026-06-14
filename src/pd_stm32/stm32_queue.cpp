#include "fas_queue/stepper_queue.h"
#include "log2/Log2Representation.h"
#include "fas_ramp/RampControl.h"

#if defined(ARDUINO_ARCH_STM32)

// ====================================================================
// STM32 Family Detection (3-Layer Architecture)
//
// Layer 1: Product-Line Detection
// stm32duino core passes per-device macros via build.product_line
// (-DSTM32G070xx, -DSTM32F103xB). These are ALWAYS available from
// compiler flags. We map them to family macros (STM32G0xx, STM32F1xx).
//
// Layer 2: Legacy Aliases
// CMSIS device headers (stm32f1xx.h) define short names (STM32F1).
// These aliases catch non-Arduino frameworks (CubeMX, Mbed, Zephyr).
//
// Layer 3: Timer Grouping
// pd_config.h defines FAS_STM32_TIMER_16BIT / FAS_STM32_TIMER_32BIT
// based on the detected family macro.
// ====================================================================

// ===== LAYER 1: Product-Line to Family Macro Map =====
// Source: official stm32_def_build.h from framework-arduinoststm32

// STM32C0xx
#if defined(STM32C011xx) || defined(STM32C031xx) || defined(STM32C051xx) || \
    defined(STM32C071xx) || defined(STM32C091xx) || defined(STM32C092xx)
  #ifndef STM32C0xx
    #define STM32C0xx
  #endif
#endif

// STM32F0xx
#if defined(STM32F030x6) || defined(STM32F030x8) || defined(STM32F030xC) || \
    defined(STM32F031x6) || defined(STM32F038xx) || defined(STM32F042x6) || \
    defined(STM32F048xx) || defined(STM32F051x8) || defined(STM32F058xx) || \
    defined(STM32F070x6) || defined(STM32F070xB) || defined(STM32F071xB) || \
    defined(STM32F072xB) || defined(STM32F078xx) || defined(STM32F091xC) || \
    defined(STM32F098xx)
  #ifndef STM32F0xx
    #define STM32F0xx
  #endif
#endif

// STM32F1xx
#if defined(STM32F100xB) || defined(STM32F100xE) || defined(STM32F101x6) || \
    defined(STM32F101xB) || defined(STM32F101xE) || defined(STM32F101xG) || \
    defined(STM32F102x6) || defined(STM32F102xB) || defined(STM32F103x6) || \
    defined(STM32F103xB) || defined(STM32F103xE) || defined(STM32F103xG) || \
    defined(STM32F105xC) || defined(STM32F107xC)
  #ifndef STM32F1xx
    #define STM32F1xx
  #endif
#endif

// STM32F2xx
#if defined(STM32F205xx) || defined(STM32F207xx) || defined(STM32F215xx) || \
    defined(STM32F217xx)
  #ifndef STM32F2xx
    #define STM32F2xx
  #endif
#endif

// STM32F3xx
#if defined(STM32F301x8) || defined(STM32F302x8) || defined(STM32F302xC) || \
    defined(STM32F302xE) || defined(STM32F303x8) || defined(STM32F303xC) || \
    defined(STM32F303xE) || defined(STM32F318xx) || defined(STM32F328xx) || \
    defined(STM32F334x8) || defined(STM32F358xx) || defined(STM32F373xC) || \
    defined(STM32F378xx) || defined(STM32F398xx)
  #ifndef STM32F3xx
    #define STM32F3xx
  #endif
#endif

// STM32F4xx
#if defined(STM32F401xC) || defined(STM32F401xE) || defined(STM32F405xx) || \
    defined(STM32F407xx) || defined(STM32F410Cx) || defined(STM32F410Rx) || \
    defined(STM32F410Tx) || defined(STM32F411xE) || defined(STM32F412Cx) || \
    defined(STM32F412Rx) || defined(STM32F412Vx) || defined(STM32F412Zx) || \
    defined(STM32F413xx) || defined(STM32F415xx) || defined(STM32F417xx) || \
    defined(STM32F423xx) || defined(STM32F427xx) || defined(STM32F429xx) || \
    defined(STM32F437xx) || defined(STM32F439xx) || defined(STM32F446xx) || \
    defined(STM32F469xx) || defined(STM32F479xx)
  #ifndef STM32F4xx
    #define STM32F4xx
  #endif
#endif

// STM32F7xx
#if defined(STM32F722xx) || defined(STM32F723xx) || defined(STM32F730xx) || \
    defined(STM32F732xx) || defined(STM32F733xx) || defined(STM32F745xx) || \
    defined(STM32F746xx) || defined(STM32F750xx) || defined(STM32F756xx) || \
    defined(STM32F765xx) || defined(STM32F767xx) || defined(STM32F769xx) || \
    defined(STM32F777xx) || defined(STM32F779xx)
  #ifndef STM32F7xx
    #define STM32F7xx
  #endif
#endif

// STM32G0xx (bao gồm STM32G070xx)
#if defined(STM32G030xx) || defined(STM32G031xx) || defined(STM32G041xx) || \
    defined(STM32G050xx) || defined(STM32G051xx) || defined(STM32G061xx) || \
    defined(STM32G070xx) || defined(STM32G071xx) || defined(STM32G081xx) || \
    defined(STM32G0B0xx) || defined(STM32G0B1xx) || defined(STM32G0C1xx) || \
    defined(STM32GBK1CB)
  #ifndef STM32G0xx
    #define STM32G0xx
  #endif
#endif

// STM32G4xx
#if defined(STM32G411xB) || defined(STM32G411xC) || defined(STM32G414xx) || \
    defined(STM32G431xx) || defined(STM32G441xx) || defined(STM32G471xx) || \
    defined(STM32G473xx) || defined(STM32G474xx) || defined(STM32G483xx) || \
    defined(STM32G484xx) || defined(STM32G491xx) || defined(STM32G4A1xx)
  #ifndef STM32G4xx
    #define STM32G4xx
  #endif
#endif

// STM32H5xx
#if defined(STM32H503xx) || defined(STM32H523xx) || defined(STM32H533xx) || \
    defined(STM32H562xx) || defined(STM32H563xx) || defined(STM32H573xx)
  #ifndef STM32H5xx
    #define STM32H5xx
  #endif
#endif

// STM32H7xx
#if defined(STM32H723xx) || defined(STM32H725xx) || defined(STM32H730xx) || \
    defined(STM32H730xxQ) || defined(STM32H733xx) || defined(STM32H735xx) || \
    defined(STM32H742xx) || defined(STM32H743xx) || defined(STM32H745xG) || \
    defined(STM32H745xx) || defined(STM32H747xG) || defined(STM32H747xx) || \
    defined(STM32H750xx) || defined(STM32H753xx) || defined(STM32H755xx) || \
    defined(STM32H757xx) || defined(STM32H7A3xx) || defined(STM32H7A3xxQ) || \
    defined(STM32H7B0xx) || defined(STM32H7B0xxQ) || defined(STM32H7B3xx) || \
    defined(STM32H7B3xxQ)
  #ifndef STM32H7xx
    #define STM32H7xx
  #endif
#endif

// STM32L0xx
#if defined(STM32L010x4) || defined(STM32L010x6) || defined(STM32L010x8) || \
    defined(STM32L010xB) || defined(STM32L011xx) || defined(STM32L021xx) || \
    defined(STM32L031xx) || defined(STM32L041xx) || defined(STM32L051xx) || \
    defined(STM32L052xx) || defined(STM32L053xx) || defined(STM32L062xx) || \
    defined(STM32L063xx) || defined(STM32L071xx) || defined(STM32L072xx) || \
    defined(STM32L073xx) || defined(STM32L081xx) || defined(STM32L082xx) || \
    defined(STM32L083xx)
  #ifndef STM32L0xx
    #define STM32L0xx
  #endif
#endif

// STM32L1xx
#if defined(STM32L100xB) || defined(STM32L100xBA) || defined(STM32L100xC) || \
    defined(STM32L151xB) || defined(STM32L151xBA) || defined(STM32L151xC) || \
    defined(STM32L151xCA) || defined(STM32L151xD) || defined(STM32L151xDx) || \
    defined(STM32L151xE) || defined(STM32L152xB) || defined(STM32L152xBA) || \
    defined(STM32L152xC) || defined(STM32L152xCA) || defined(STM32L152xD) || \
    defined(STM32L152xDx) || defined(STM32L152xE) || defined(STM32L162xC) || \
    defined(STM32L162xCA) || defined(STM32L162xD) || defined(STM32L162xDx) || \
    defined(STM32L162xE)
  #ifndef STM32L1xx
    #define STM32L1xx
  #endif
#endif

// STM32L4xx
#if defined(STM32L412xx) || defined(STM32L422xx) || defined(STM32L431xx) || \
    defined(STM32L432xx) || defined(STM32L433xx) || defined(STM32L442xx) || \
    defined(STM32L443xx) || defined(STM32L451xx) || defined(STM32L452xx) || \
    defined(STM32L462xx) || defined(STM32L471xx) || defined(STM32L475xx) || \
    defined(STM32L476xx) || defined(STM32L485xx) || defined(STM32L486xx) || \
    defined(STM32L496xx) || defined(STM32L4A6xx) || defined(STM32L4P5xx) || \
    defined(STM32L4Q5xx) || defined(STM32L4R5xx) || defined(STM32L4R7xx) || \
    defined(STM32L4R9xx) || defined(STM32L4S5xx) || defined(STM32L4S7xx) || \
    defined(STM32L4S9xx)
  #ifndef STM32L4xx
    #define STM32L4xx
  #endif
#endif

// STM32L5xx
#if defined(STM32L552xx) || defined(STM32L562xx)
  #ifndef STM32L5xx
    #define STM32L5xx
  #endif
#endif

// STM32MP1xx
#if defined(STM32MP151Axx) || defined(STM32MP151Cxx) || defined(STM32MP153Axx) || \
    defined(STM32MP153Cxx) || defined(STM32MP157Axx) || defined(STM32MP157Cxx) || \
    defined(STM32MP15xx)
  #ifndef STM32MP1xx
    #define STM32MP1xx
  #endif
#endif

// STM32U0xx
#if defined(STM32U031xx) || defined(STM32U073xx) || defined(STM32U083xx)
  #ifndef STM32U0xx
    #define STM32U0xx
  #endif
#endif

// STM32U3xx
#if defined(STM32U375xx) || defined(STM32U385xx)
  #ifndef STM32U3xx
    #define STM32U3xx
  #endif
#endif

// STM32U5xx
#if defined(STM32U535xx) || defined(STM32U545xx) || defined(STM32U575xx) || \
    defined(STM32U585xx) || defined(STM32U595xx) || defined(STM32U599xx) || \
    defined(STM32U5A5xx) || defined(STM32U5A9xx) || defined(STM32U5F7xx) || \
    defined(STM32U5F9xx) || defined(STM32U5G7xx) || defined(STM32U5G9xx)
  #ifndef STM32U5xx
    #define STM32U5xx
  #endif
#endif

// STM32WB0x (⚠️ product line KHÔNG có 'xx' suffix)
#if defined(STM32WB05) || defined(STM32WB06) || defined(STM32WB07) || \
    defined(STM32WB09)
  #ifndef STM32WB0x
    #define STM32WB0x
  #endif
#endif

// STM32WBAxx
#if defined(STM32WBA50xx) || defined(STM32WBA52xx) || defined(STM32WBA54xx) || \
    defined(STM32WBA55xx) || defined(STM32WBA5Mxx) || defined(STM32WBA62xx) || \
    defined(STM32WBA63xx) || defined(STM32WBA64xx) || defined(STM32WBA65xx) || \
    defined(STM32WBA6Mxx)
  #ifndef STM32WBAxx
    #define STM32WBAxx
  #endif
#endif

// STM32WBxx
#if defined(STM32WB10xx) || defined(STM32WB15xx) || defined(STM32WB1Mxx) || \
    defined(STM32WB30xx) || defined(STM32WB35xx) || defined(STM32WB50xx) || \
    defined(STM32WB55xx) || defined(STM32WB5Mxx)
  #ifndef STM32WBxx
    #define STM32WBxx
  #endif
#endif

// STM32WL3x (⚠️ Pattern: STM32WL3XX, STM32WL3RX)
#if defined(STM32WL3RX) || defined(STM32WL3XX)
  #ifndef STM32WL3x
    #define STM32WL3x
  #endif
#endif

// STM32WLxx
#if defined(STM32WL54xx) || defined(STM32WL55xx) || defined(STM32WL5Mxx) || \
    defined(STM32WLE4xx) || defined(STM32WLE5xx)
  #ifndef STM32WLxx
    #define STM32WLxx
  #endif
#endif

// ===== LAYER 2: Legacy Short-Name Aliases =====
// Cho non-Arduino frameworks (CubeMX, Mbed, Zephyr)
// CMSIS device headers define STM32F1, STM32G0 etc.
#if defined(STM32C0) && !defined(STM32C0xx)
#define STM32C0xx
#endif
#if defined(STM32F0) && !defined(STM32F0xx)
#define STM32F0xx
#endif
#if defined(STM32F1) && !defined(STM32F1xx)
#define STM32F1xx
#endif
#if defined(STM32F2) && !defined(STM32F2xx)
#define STM32F2xx
#endif
#if defined(STM32F3) && !defined(STM32F3xx)
#define STM32F3xx
#endif
#if defined(STM32F4) && !defined(STM32F4xx)
#define STM32F4xx
#endif
#if defined(STM32F7) && !defined(STM32F7xx)
#define STM32F7xx
#endif
#if defined(STM32G0) && !defined(STM32G0xx)
#define STM32G0xx
#endif
#if defined(STM32G4) && !defined(STM32G4xx)
#define STM32G4xx
#endif
#if defined(STM32H5) && !defined(STM32H5xx)
#define STM32H5xx
#endif
#if defined(STM32H7) && !defined(STM32H7xx)
#define STM32H7xx
#endif
#if defined(STM32L0) && !defined(STM32L0xx)
#define STM32L0xx
#endif
#if defined(STM32L1) && !defined(STM32L1xx)
#define STM32L1xx
#endif
#if defined(STM32L4) && !defined(STM32L4xx)
#define STM32L4xx
#endif
#if defined(STM32L5) && !defined(STM32L5xx)
#define STM32L5xx
#endif
#if defined(STM32MP1) && !defined(STM32MP1xx)
#define STM32MP1xx
#endif
#if defined(STM32U0) && !defined(STM32U0xx)
#define STM32U0xx
#endif
#if defined(STM32U3) && !defined(STM32U3xx)
#define STM32U3xx
#endif
#if defined(STM32U5) && !defined(STM32U5xx)
#define STM32U5xx
#endif
#if defined(STM32WB0) && !defined(STM32WB0x)
#define STM32WB0x
#endif
#if defined(STM32WBA) && !defined(STM32WBAxx)
#define STM32WBAxx
#endif
#if defined(STM32WB) && !defined(STM32WBxx)
#define STM32WBxx
#endif
#if defined(STM32WL3) && !defined(STM32WL3x)
#define STM32WL3x
#endif
#if defined(STM32WL) && !defined(STM32WLxx)
#define STM32WLxx
#endif

// ====================================================================
// FAS_DMB — Data Memory Barrier wrapper
//
// ARMv6-M (M0/M0+) does not have __DMB(). Use __DSB() instead.
// ARMv7-M (M3/M4/M7) and ARMv8-M.main (M33) have __DMB().
// ARMv8-M.base (M23) does not have __DMB() — reserved, use __DSB().
//
// Affected STM32 families:
//   __DMB() OK:   F1, F4, F7, H7, G4, L4, WB, WL, L5, U5, H5
//   __DSB() needed: G0, F0, L0, C0 (ARMv6-M, __ARM_ARCH_6M__)
//   Reserved: M23 (ARMv8-M.base, __ARM_ARCH_8M_BASE__)
//
// GCC and ARMCC define __ARM_ARCH_6M__ automatically when compiling
// with -mcpu=cortex-m0 or -mcpu=cortex-m0plus.
// ====================================================================

// ====================================================================
// FAS_SPURIOUS_MAX — Spurious Interrupt Guard (Phase 2A)
//
// If a timer channel fires an interrupt when no queue is active (e.g. due
// to EMI or configuration error), the ISR would loop indefinitely clearing
// the flag. This guard counts consecutive spurious interrupts per channel
// and disables the channel after FAS_SPURIOUS_MAX occurrences.
// ====================================================================
#define FAS_SPURIOUS_MAX 10
static uint8_t fas_spurious_count[4] = {0, 0, 0, 0};
#if defined(__ARM_ARCH_6M__)
    // M0/M0+ (G0, F0, L0, C0): không có __DMB()
    #define FAS_DMB() __DSB()
#elif defined(__ARM_ARCH_7M__) || defined(__ARM_ARCH_7EM__) || \
      defined(__ARM_ARCH_8M_MAIN__)
    // M3/M4/M7/M33: có __DMB()
    #define FAS_DMB() __DMB()
    // reserved: __ARM_ARCH_8M_BASE__ (M23) → dùng __DSB() nếu cần
#else
    #define FAS_DMB() __DSB()  // fallback an toàn
#endif

// ====================================================================
// Timer selection — explicit per-family (24 families)
//
// Mỗi STM32 family có #elif riêng. Nếu board của bạn chưa có trong
// danh sách, thêm #elif tương ứng. Xem README để biết thông số cần.
//
// Families được nhóm theo timer type (CMSIS/RM-verified):
//   Group A (TIM3 16-bit): C0, G0
//   Group B (TIM2 16-bit): F1, L0, L1
//   Group C (TIM2 32-bit): F0, F2, F3, F4, F7, G4, H5, H7, L4, L5,
//                          MP1, U0, U3, U5, WBA, WB, WL
//   Excluded:              WB0x (no general-purpose timer)
//                          WL3x (no RCC_CFGR_PPRE — non-standard clock tree)
//
// CMSIS evidence for "single APB PPRE":
//   F0: stm32f091xc.h — RCC_CFGR_PPRE (bit 8, 3-bit mask 0x7)
//   U0: stm32u031xx.h — RCC_CFGR_PPRE (bit 12, 3-bit mask 0x7)
//   C0/G0/L0: same macro, same bit position
//   WL3x: NONE — excluded (no PPRE register)
//
// CMSIS evidence for "dual APB PPRE1":
//   All families: confirmed via findstr /M "RCC_CFGR_PPRE1" in hal_rcc.h
// ====================================================================
#if defined(STM32C0xx) || defined(STM32G0xx)
    // Group A: TIM3 16-bit
    // G0: TIM2 availability varies (G030/G031 = no TIM2, G0B1/G0C1 = has TIM2).
    // TIM3 exists on ALL G0 parts — common denominator for reliable support.
    #define FAS_TIMER            TIM3
    #define FAS_TIMER_IRQn       TIM3_IRQn
    #define FAS_TIMER_RCC_ENABLE() __HAL_RCC_TIM3_CLK_ENABLE()
    #define FAS_TIMER_ARR_MAX    0xFFFF

#elif defined(STM32F1xx) || defined(STM32L0xx) || defined(STM32L1xx)
    // Group B: TIM2 16-bit (F1 RM0008, L0 RM0367 §24, L1 RM0038)
    #define FAS_TIMER            TIM2
    #define FAS_TIMER_IRQn       TIM2_IRQn
    #define FAS_TIMER_RCC_ENABLE() __HAL_RCC_TIM2_CLK_ENABLE()
    #define FAS_TIMER_ARR_MAX    0xFFFF

#elif defined(STM32F0xx) || defined(STM32F2xx) || defined(STM32F3xx) || \
      defined(STM32F4xx) || defined(STM32F7xx) || defined(STM32G4xx) || \
      defined(STM32H5xx) || defined(STM32H7xx) || defined(STM32L4xx) || \
      defined(STM32L5xx) || defined(STM32MP1xx) || defined(STM32U0xx) || \
      defined(STM32U3xx) || defined(STM32U5xx) || defined(STM32WBAxx) || \
      defined(STM32WBxx) || defined(STM32WLxx)
    // Group C: TIM2 32-bit — 17 families
    // F0: RM0091 §18 "32-bit counter" (previously in Group B in v12).
    // U0: stm32u031xx.h — TIM2_TypeDef with __IO uint32_t CNT → 32-bit
    //     PPRE at bit 12 (differs from other M0+ at bit 8). 3-bit field (mask 0x7).
    //     Macros handle position — code is transportable.
    // F2/F3/F4/F7/G4/H5/H7/L4/L5/MP1/U3/U5/WBA/WB/WL: confirmed 32-bit
    #define FAS_TIMER            TIM2
    #define FAS_TIMER_IRQn       TIM2_IRQn
    #define FAS_TIMER_RCC_ENABLE() __HAL_RCC_TIM2_CLK_ENABLE()
    #define FAS_TIMER_ARR_MAX    0xFFFFFFFF

#elif defined(STM32WL3x)
    // WL3x has no RCC_CFGR_PPRE register — stm32wl3xx.h uses CLKSYSDIV instead.
    // TIM2 exists and is 32-bit (uint32_t CNT) but clock detection is unsupported.
    // No CI board available — pending hardware verification.
    #error "FAS: STM32WL3x unsupported — no RCC_CFGR_PPRE register (non-standard clock tree). \
Pending hardware verification — contact maintainer to add support."

#elif defined(STM32WB0x)
    // WB0x (WB05/WB06/WB07/WB09) has NO general-purpose timer
    // TIM2_IRQn exists but no TIM2_TypeDef/CNT/ARR/CCR registers
    #error "FAS: STM32WB0x unsupported — no general-purpose timer with CCR registers. \
Use STM32WBxx (Cortex-M4) for FAS support."

#else
    #error "FAS: Unsupported STM32 family. \
Add product-line macro to detection block (line ~14), then add family \
to timer selection block (line ~110). See README STM32 section."
#endif

// ====================================================================
// Static data
// ====================================================================
static FastAccelStepperEngine* fas_engine = NULL;
static uint8_t stepper_allocated_mask = 0;
static volatile bool _cyclic_pending = false;
static uint32_t _last_cyclic_tick = 0;
uint8_t fas_stm32_clock_error = 0;
uint32_t fas_stm32_clock_tim_clk = 0;   // Cached timer clock for warning output
StepperQueue* StepperQueue::_ch_to_queue[4] = {NULL, NULL, NULL, NULL};

// ====================================================================
// Timer clock detection
//
// Timer counter clock = PCLK1 * (APB1_prescaler==1 ? 1 : 2)
//
// The APB1 prescaler is in:
//   - H7: RCC->D2CFGR.D2PPRE1 (D2 domain, encoding: 0-3=÷1, 4=÷2, 5=÷4, 6=÷8, 7=÷16)
//   - G0/C0: RCC->CFGR.PPRE (single APB bus, same encoding)
//   - Others F1/F4/F7/L1/L4/G4/WB: RCC->CFGR.PPRE1 (same encoding)
//
// NOTE: APB prescaler field encoding (all STM32 families):
//   0-3 = ÷1 (0,1 valid; 2,3 reserved)
//   4   = ÷2
//   5   = ÷4
//   6   = ÷8
//   7   = ÷16
// Condition for clock ×2: prescaler > 1 ↔ field >= 4
// ====================================================================
static uint32_t getTimClock(void) {
    uint32_t pclk1 = HAL_RCC_GetPCLK1Freq();
#if defined(STM32H7xx)
    // H7 series: D2 domain, D2CFGR register
    uint32_t dppre1 = (RCC->D2CFGR & RCC_D2CFGR_D2PPRE1) >> RCC_D2CFGR_D2PPRE1_Pos;
    if (dppre1 >= 4) pclk1 *= 2;
#elif defined(STM32C0xx) || defined(STM32F0xx) || defined(STM32G0xx) || \
      defined(STM32L0xx) || defined(STM32U0xx)
    // Single APB bus (Cortex-M0+): uses RCC_CFGR_PPRE (no '1')
    // C0, F0, G0, L0, U0
    // U0: PPRE at bit 12 (macro handles difference automatically)
    uint32_t pp = (RCC->CFGR & RCC_CFGR_PPRE) >> RCC_CFGR_PPRE_Pos;
    if (pp >= 4) pclk1 *= 2;
#elif defined(STM32WL3x)
    // WL3x has no RCC_CFGR_PPRE — non-standard clock tree
    #error "FAS: STM32WL3x unsupported — requires WL3x-specific APB clock detection. \
No CI board available — contact maintainer to add support."
#elif defined(STM32H5xx)
    // H5 series: APB1 prescaler is in RCC->CFGR2 (not CFGR), per RM0481
    uint32_t pp = (RCC->CFGR2 & RCC_CFGR2_PPRE1) >> RCC_CFGR2_PPRE1_Pos;
    if (pp >= 4) pclk1 *= 2;

#elif defined(STM32F1xx) || defined(STM32F2xx) || defined(STM32F3xx) || \
      defined(STM32F4xx) || defined(STM32F7xx) || defined(STM32G4xx) || \
      defined(STM32L1xx) || defined(STM32L4xx) || \
      defined(STM32L5xx) || defined(STM32MP1xx) || defined(STM32U3xx) || \
      defined(STM32U5xx) || defined(STM32WBAxx) || defined(STM32WBxx) || \
      defined(STM32WLxx)
    // Multi-APB: uses RCC_CFGR_PPRE1
    uint32_t pp = (RCC->CFGR & RCC_CFGR_PPRE1) >> RCC_CFGR_PPRE1_Pos;
    if (pp >= 4) pclk1 *= 2;
#else
    #error "FAS: Unsupported STM32 family in getTimClock(). \
Add APB prescaler detection for your family."
#endif
    return pclk1;
}

// ====================================================================
// fas_tim_set_ccr — Write CCR with 16-bit wrap handling (F1, C0)
//
// On 16-bit timers (F1 TIM2, C0 TIM3), (cnt + delay) may exceed 0xFFFF.
// The & 0xFFFF mask correctly handles the wrap:
//   target = (cnt + delay) & 0xFFFF = cnt + delay - 65536 (if overflow)
//   Actual ticks = (0xFFFF - cnt + 1) + target
//                = (65536 - cnt) + (cnt + delay - 65536) = delay ✓
//
// Example: cnt=65520, delay=432
//   target = (65520+432) & 0xFFFF = 416
//   ticks = (65536-65520) + 416 = 16 + 416 = 432 = delay ✓
//
// On 32-bit timers, simple addition is safe (no overflow in practice).
// ====================================================================
static inline void fas_tim_set_ccr(volatile uint32_t* ccr, uint32_t delay) {
#if defined(FAS_STM32_TIMER_16BIT)
    uint32_t cnt = FAS_TIMER->CNT;
    // 16-bit timer: (cnt + delay) & 0xFFFF handles wrap correctly
    *ccr = (cnt + delay) & 0xFFFF;
#elif defined(FAS_STM32_TIMER_32BIT)
    *ccr = FAS_TIMER->CNT + delay;
#else
    #error "FAS: FAS_STM32_TIMER_16BIT or _32BIT not defined. \
Check pd_config.h timer grouping."
#endif
}

// ====================================================================
// Step timer initialization
// Called once when first stepper is initialized.
// Uses FAS_TIMER macros to support TIM2 (all families) or TIM3 (C0).
// ====================================================================
static void initStepTimer(void) {
    static bool initialized = false;
    if (initialized) return;
    initialized = true;

    // Enable timer clock (TIM2 on most, TIM3 on C0)
    FAS_TIMER_RCC_ENABLE();

    // Cache the actual timer clock for later warning output
    fas_stm32_clock_tim_clk = getTimClock();

    // ---- Clock validation ----
    if (TICKS_PER_S == 0 || TICKS_PER_S > fas_stm32_clock_tim_clk) {
        // Cannot achieve TICKS_PER_S at this clock frequency.
        // Run at maximum rate (PSC=0). All timing will be incorrect.
        fas_stm32_clock_error = 1;
    } else if (fas_stm32_clock_tim_clk % TICKS_PER_S != 0) {
        // Non-integer prescaler — timer tick rate will deviate.
        // Example: 84MHz TIM2 with TICKS_PER_S=72MHz → psc=(84/72)-1=0 → timer at 84MHz, +16.7% error.
        // (Addition in v8: error code 2 was not present in original code.)
        fas_stm32_clock_error = 2;
    }

    // Compute prescaler
    uint32_t psc;
    if (fas_stm32_clock_error == 1) {
        psc = 0;  // Run at maximum rate (timing will be wrong)
    } else {
        psc = (fas_stm32_clock_tim_clk / TICKS_PER_S) - 1;
        if (psc > 65535) {
            psc = 65535;
            fas_stm32_clock_error = 1;  // Prescaler clamped — timing will be wrong
        }
    }

    // Configure timer
    FAS_TIMER->CR1 = 0;
    FAS_TIMER->PSC = psc;
    FAS_TIMER->ARR = FAS_TIMER_ARR_MAX;
    FAS_TIMER->EGR |= TIM_EGR_UG;  // Reload shadow registers (PSC, ARR)
    FAS_TIMER->DIER = 0;           // All interrupts disabled initially

    // Force LOW all channels (OCxM = 100 = Force Inactive)
    // Using HAL constants ensures correct 4-bit OCxM on all STM32 families
    // (F1/F4: 3-bit, others: 4-bit with bit[3]=0 in CCMR bit 16).
    FAS_TIMER->CCMR1 = TIM_CCMR1_OC1M_2 | TIM_CCMR1_OC2M_2;  // OC1M=4, OC2M=4
    FAS_TIMER->CCMR2 = TIM_CCMR2_OC3M_2 | TIM_CCMR2_OC4M_2;  // OC3M=4, OC4M=4

    // NVIC configuration
    NVIC_SetPriority(FAS_TIMER_IRQn, 0);   // Highest priority for step timing
    NVIC_EnableIRQ(FAS_TIMER_IRQn);

    // Start timer
    FAS_TIMER->CR1 |= TIM_CR1_CEN;
}

// ====================================================================
// Dynamic slot allocation — supports ANY GPIO pin for step
// ====================================================================
static int8_t findFreeSlot(void) {
    for (int i = 0; i < MAX_STEPPER; i++) {
        if (!(stepper_allocated_mask & (1 << i))) {
            return i;
        }
    }
    return -1;  // All slots used
}

// ====================================================================
// Queue initialization
// ====================================================================
void StepperQueue::init(uint8_t queue_num, uint8_t step_pin) {
    static const uint8_t ch_map[4] = {0, 1, 2, 3};
    _timer_ch = ch_map[queue_num];

    // Ensure step timer is initialized (TIM2 / TIM3 on C0)
    initStepTimer();

    // Step pin GPIO configuration — validate port first
    _step_pin = step_pin;
    _step_port = digitalPinToPort(step_pin);
    if (!_step_port) return;  // Invalid pin — init fails silently, ISR skips

    uint32_t mask = digitalPinToBitMask(step_pin);
    _step_set_mask = mask;

    // Clear mask: BSRR high half = reset (mask << 16 works on ALL families)
    _step_clr_mask = mask << 16;

    // Configure pin as OUTPUT, initial LOW
    pinMode(step_pin, OUTPUT);
    digitalWrite(step_pin, LOW);

    // Store CCR register pointer for fast ISR access
    // Must match the timer type: TIM2 on most families, TIM3 on C0
    // This is the only place where the concrete timer register is referenced.
    // All other CCR writes go through _ccr_reg (fast pointer) or fas_tim_set_ccr().
#if defined(STM32C0xx) || defined(STM32G0xx)
    // Group A: TIM3 CCR1-CCR4
    volatile uint32_t* ccr[] = {&TIM3->CCR1, &TIM3->CCR2, &TIM3->CCR3, &TIM3->CCR4};
#elif defined(STM32F0xx) || defined(STM32F1xx) || defined(STM32F2xx) || \
      defined(STM32F3xx) || defined(STM32F4xx) || defined(STM32F7xx) || \
      defined(STM32G4xx) || defined(STM32H5xx) || defined(STM32H7xx) || \
      defined(STM32L0xx) || defined(STM32L1xx) || defined(STM32L4xx) || \
      defined(STM32L5xx) || defined(STM32MP1xx) || defined(STM32U0xx) || \
      defined(STM32U3xx) || defined(STM32U5xx) || defined(STM32WBAxx) || \
      defined(STM32WBxx) || defined(STM32WLxx)
    // Groups B + C: TIM2 CCR1-CCR4
    volatile uint32_t* ccr[] = {&TIM2->CCR1, &TIM2->CCR2, &TIM2->CCR3, &TIM2->CCR4};
#elif defined(STM32WL3x)
    #error "FAS: STM32WL3x unsupported — no timer CCR registers (no PPRE clock)."
#elif defined(STM32WB0x)
    #error "FAS: STM32WB0x unsupported — no timer CCR registers."
#else
    #error "FAS: Unsupported STM32 family in CCR init."
#endif
    _ccr_reg = ccr[_timer_ch];

    // Register channel-to-queue mapping
    _ch_to_queue[_timer_ch] = this;
    // _initialized = true was removed (fix_plan_v3 FIX #7)
    _isRunning = false;
}

// ====================================================================
// Start / Stop
// ====================================================================
void StepperQueue::startQueue(void) {
    _isRunning = true;
    _pulse_high = false;
    _dir_delay_active = false;

    // Write CCR first, then barrier, then enable interrupt
    // Use fas_tim_set_ccr for 16-bit safe CCR write (handles F1/C0 wrap)
    fas_tim_set_ccr(_ccr_reg, TICKS_PER_S / 1000000);  // ~1µs offset
    FAS_DMB();

    // Save/restore PRIMASK for reentrant-safe IRQ disable
    uint32_t prim = __get_PRIMASK();
    __disable_irq();
    FAS_TIMER->DIER |= CCXIE_BIT(_timer_ch);
    if (!prim) __enable_irq();
}

void StepperQueue::forceStop(void) {
    // Save/restore PRIMASK for reentrant-safe IRQ disable
    uint32_t prim = __get_PRIMASK();
    __disable_irq();
    FAS_TIMER->DIER &= ~CCXIE_BIT(_timer_ch);
    _isRunning = false;
    read_idx = next_write_idx;  // Discard remaining queue entries
    if (!prim) __enable_irq();

    // Ensure step pin is LOW (BSRR high-half clear works on ALL families)
    if (_step_port) {
        _step_port->BSRR = _step_clr_mask;
    }
}

void StepperQueue::connect(void) {}
void StepperQueue::disconnect(void) {}

// ====================================================================
// Speed adjustment based on number of active steppers
// ====================================================================
void StepperQueue::adjustSpeedToStepperCount(uint8_t steppers) {
    if (steppers == 1)
        max_speed_in_ticks = STEP_PULSE_WIDTH_TICKS * 2;
    else if (steppers == 2)
        max_speed_in_ticks = STEP_PULSE_WIDTH_TICKS * 3;
    else
        max_speed_in_ticks = STEP_PULSE_WIDTH_TICKS * 4;

    if (max_speed_in_ticks < MIN_CMD_TICKS)
        max_speed_in_ticks = MIN_CMD_TICKS;
}

// ====================================================================
// getActualTicksWithDirection — retrieve current step rate
// ====================================================================
bool StepperQueue::getActualTicksWithDirection(
    struct actual_ticks_s* speed) const {
    fasDisableInterrupts();
    speed->count_up = queue_end.count_up;
    speed->ticks = _last_command_ticks;
    fasEnableInterrupts();
    inject_fill_interrupt(0);
    return true;
}

// ====================================================================
// Cyclic PendSV trigger
// Called at end of FAS_TIMER_IRQHandler every ~3ms.
// Triggers PendSV exception to fill queues without consuming ISR time.
// ====================================================================
static void cyclic_check_and_pend(void) {
    uint32_t now = HAL_GetTick();
    if ((now - _last_cyclic_tick) >= CYCLIC_INTERVAL_MS) {
        _last_cyclic_tick = now;
        if (!_cyclic_pending) {
            _cyclic_pending = true;
            SCB->ICSR = SCB_ICSR_PENDSVSET_Msk;
        }
    }
}

// ====================================================================
// Clock error reporting — prints warning to Serial if clock mismatch
//
// Call site: engine.init() → FastAccelStepperEngine::init() → fas_init_engine()
//            → fas_stm32_report_clock_error()
//
// ⚠️ User MUST call Serial.begin() BEFORE engine.init().
//     If Serial is not initialized, the warning is silently dropped
//     (no crash, but user won't see the error).
//
// Error codes:
//   0 = OK (no error)
//   1 = TICKS_PER_S > actual timer clock, OR prescaler clamped at 65535
//       (timing will be wrong — define correct TICKS_PER_S in build_flags)
//   2 = Non-integer prescaler (tim2_clk % TICKS_PER_S != 0)
//       (timing will be slightly off — define correct TICKS_PER_S in build_flags)
// ====================================================================
static void fas_stm32_report_clock_error(void) {
    if (fas_stm32_clock_error == 0) return;
    if (!Serial) return;  // Serial not initialized → silent, no crash

    Serial.print("[FAS] WARNING: Step timer clock error (code=");
    Serial.print(fas_stm32_clock_error);
    Serial.println(")");

    switch (fas_stm32_clock_error) {
        case 1:
            Serial.println("  Cause: TICKS_PER_S > actual timer clock, or prescaler > 65535.");
            break;
        case 2:
            Serial.println("  Cause: Non-integer prescaler (timer clock not divisible by TICKS_PER_S).");
            break;
    }
    Serial.print("  TICKS_PER_S=");
    Serial.print(TICKS_PER_S);
    Serial.print("  Timer_CLK=");
    Serial.println(fas_stm32_clock_tim_clk);
    Serial.println("  Fix: add -DTICKS_PER_S=xxx in platformio.ini (see debug_plan_v8.md appendix B).");
}

// ====================================================================
// FAS_TIMER_IRQHandler — Step pulse generation (all 4 channels)
//
// The ISR name depends on which timer is used:
//   - C0:   TIM3_IRQHandler (handles TIM3->SR, TIM3->DIER, TIM3->CNT)
//   - Others: TIM2_IRQHandler (handles TIM2->SR, TIM2->DIER, TIM2->CNT)
//
// State machine:
//   Phase 1 (_pulse_high=false): Set step pin HIGH, schedule LOW
//   Phase 2 (_pulse_high=true):  Set step pin LOW, process queue
//   DirSettle (_dir_delay_active): Direction settling complete → start pulse
//
// SR handling:
//   Snapshot SR at entry. Build ch_processed mask of all handled flags.
//   Clear all at end by writing ~ch_processed (rc_w0 behavior).
//
// IMPORTANT: All CCR writes use fas_tim_set_ccr() for 16-bit wrap safety.
// ====================================================================
#if defined(STM32C0xx) || defined(STM32G0xx)
void TIM3_IRQHandler(void) {
#elif defined(STM32F0xx) || defined(STM32F1xx) || defined(STM32F2xx) || \
      defined(STM32F3xx) || defined(STM32F4xx) || defined(STM32F7xx) || \
      defined(STM32G4xx) || defined(STM32H5xx) || defined(STM32H7xx) || \
      defined(STM32L0xx) || defined(STM32L1xx) || defined(STM32L4xx) || \
      defined(STM32L5xx) || defined(STM32MP1xx) || defined(STM32U0xx) || \
      defined(STM32U3xx) || defined(STM32U5xx) || defined(STM32WBAxx) || \
      defined(STM32WBxx) || defined(STM32WLxx)
void TIM2_IRQHandler(void) {
#elif defined(STM32WL3x)
    #error "FAS: STM32WL3x unsupported — no timer ISR available (no PPRE clock)."
#elif defined(STM32WB0x)
    #error "FAS: STM32WB0x unsupported — no timer ISR available."
#else
    #error "FAS: Unsupported STM32 family in ISR name. \
Add ISR handler name for your family's timer."
#endif
    uint32_t sr = FAS_TIMER->SR;
    uint32_t ch_processed = 0;

    for (uint8_t ch = 0; ch < 4; ch++) {
        uint32_t ccif = CCXIF_BIT(ch);
        if (!(sr & ccif)) continue;

        // Always mark for clear — prevents infinite loop from spurious IRQs
        ch_processed |= ccif;

        StepperQueue* q = StepperQueue::_ch_to_queue[ch];
        // Phase 2A: Spurious interrupt guard — count consecutive inactive
        // channel interrupts. If EMI or config error triggers repeated
        // flags, disable channel after FAS_SPURIOUS_MAX occurrences.
        if (!q || !q->_isRunning) {
            fas_spurious_count[ch]++;
            if (fas_spurious_count[ch] >= FAS_SPURIOUS_MAX) {
                FAS_TIMER->DIER &= ~CCXIE_BIT(ch);
                fas_spurious_count[ch] = 0;
            }
            continue;
        }

        if (q->_pulse_high) {
            // ====== Phase 2: pulse end ======
            // The step pin was HIGH; bring it LOW (BSRR high-half clear).
            q->_pulse_high = false;
            q->_step_port->BSRR = q->_step_clr_mask;

            // Read queue entry
            uint8_t rp = q->read_idx;
            uint8_t wp = q->next_write_idx;

            if (rp == wp) {
                // Queue empty — stop this channel
                FAS_TIMER->DIER &= ~CCXIE_BIT(ch);
                q->_isRunning = false;
                continue;
            }

            struct queue_entry* e = &q->entry[rp & QUEUE_LEN_MASK];
            q->_last_command_ticks = e->ticks;

            if (e->steps > 1) {
                // Multi-step command: reduce step count, continue with same period
                e->steps--;
                fas_tim_set_ccr(q->_ccr_reg, e->ticks);
            } else {
                // Single step complete — advance to next entry
                rp++;
                q->read_idx = rp;

                if (rp == wp) {
                    FAS_TIMER->DIER &= ~CCXIE_BIT(ch);
                    q->_isRunning = false;
                    continue;
                }

                e = &q->entry[rp & QUEUE_LEN_MASK];

                // Handle direction change (BSRR atomic — NOT ODR XOR)
                if (e->toggle_dir && q->_dir_bsrr) {
                    if (e->dirPinState) {
                        *q->_dir_bsrr = q->_dir_set_mask;
                    } else {
                        *q->_dir_bsrr = q->_dir_clr_mask;
                    }
                    e->toggle_dir = 0;  // Clear flag — prevents double-toggle

                    // Insert direction settling delay
                    uint32_t dd = AFTER_SET_DIR_PIN_DELAY_US * (TICKS_PER_S / 1000000UL);
                    if (dd < MIN_CMD_TICKS) dd = MIN_CMD_TICKS;
                    fas_tim_set_ccr(q->_ccr_reg, dd);
                    q->_dir_delay_active = true;
                    continue;
                }

                // No direction change — schedule next step pulse
                fas_tim_set_ccr(q->_ccr_reg, e->ticks);
            }
        } else if (q->_dir_delay_active) {
            // ====== Direction settling complete ======
            // The settling delay has elapsed. Check if the current entry
            // has steps>0 before emitting a pulse.
            q->_dir_delay_active = false;

            uint8_t rp = q->read_idx;
            struct queue_entry* e = &q->entry[rp & QUEUE_LEN_MASK];
            if (e->steps > 0) {
                // Real step: start pulse (set pin HIGH)
                q->_pulse_high = true;
                q->_step_port->BSRR = q->_step_set_mask;
                fas_tim_set_ccr(q->_ccr_reg, STEP_PULSE_WIDTH_TICKS);
            } else {
                // steps=0 after dir settle: advance read_idx, schedule pause
                q->read_idx = rp + 1;     // advance entry
                q->_pulse_high = false;
                fas_tim_set_ccr(q->_ccr_reg, e->ticks);
                // NEXT ISR: Phase 1, reads entry at the new read_idx
            }
        } else {
            // ====== Phase 1: pulse start ======
            uint8_t rp = q->read_idx;
            uint8_t wp = q->next_write_idx;
            // Queue-empty guard: check before reading entry (Phase 2 has this, Phase 1 was missing)
            if (rp == wp) {
                FAS_TIMER->DIER &= ~CCXIE_BIT(ch);
                q->_isRunning = false;
                continue;
            }
            struct queue_entry* e = &q->entry[rp & QUEUE_LEN_MASK];
            if (e->steps > 0) {
                // Start pulse: set pin HIGH, schedule LOW after step pulse width
                q->_pulse_high = true;
                q->_step_port->BSRR = q->_step_set_mask;
                fas_tim_set_ccr(q->_ccr_reg, STEP_PULSE_WIDTH_TICKS);
            } else {
                // steps=0 pause: advance read_idx, schedule pause duration
                q->read_idx = rp + 1;
                q->_pulse_high = false;
                fas_tim_set_ccr(q->_ccr_reg, e->ticks);
                // After pause ticks, ISR re-enters Phase 1 with the NEXT entry
            }
        }
    }

    // Clear all processed flags at once (rc_w0: bits set to 1 are ignored)
    FAS_TIMER->SR = ~ch_processed;

    // Trigger cyclic queue fill
    cyclic_check_and_pend();
}

// ====================================================================
// PendSV_Handler — Deferred queue fill
//
// Weak attribute allows FreeRTOS to override this handler.
// Define DISABLE_FAS_PENDSV to skip installation entirely.
//
// ══════════════════════════════════════════════════════════════════════
// Phase 2B: FreeRTOS Compatibility Warning
//
// FastAccelStepper uses PendSV_Handler (weak attribute) for deferred
// queue filling. FreeRTOS may also claim PendSV for context switching.
// If both are active, they will conflict at runtime.
//
// If you use FreeRTOS:
//   1. Add -DDISABLE_FAS_PENDSV to your build flags
//   2. Call engine->manageSteppers() from a low-priority task/timer
//
// Phase 2C: NVIC Priority (Jitter Protection)
//
// TIMER must have the HIGHEST priority (0) to ensure tick-exact step
// pulse timing. PendSV must have the LOWEST priority so it only runs
// when CPU is idle, never blocking step generation (set in initStepTimer
// and fas_init_engine respectively).
// ══════════════════════════════════════════════════════════════════════
// ====================================================================
#if !defined(DISABLE_FAS_PENDSV)
#if defined(configUSE_PORT_OPTIMISED_TASK_SELECTION) || \
    defined(configUSE_TICKLESS_IDLE) || \
    defined(INC_FREERTOS_H) || \
    defined(FREERTOS_CONFIG_H)
#pragma message "FAS: PendSV_Handler may conflict if FreeRTOS uses PendSV. Define DISABLE_FAS_PENDSV to skip."
#endif
__attribute__((weak)) void PendSV_Handler(void) {
    _cyclic_pending = false;
    FAS_DMB();
    if (fas_engine) {
        fas_engine->manageSteppers();
    }
}
#endif

// ====================================================================
// Allocation — dynamic slot assignment for any GPIO step pin
// ====================================================================
StepperQueue* StepperQueue::tryAllocateQueue(
    FastAccelStepperEngine* engine, uint8_t step_pin) {
    (void)engine;

    // Validate step pin before any hardware access
    if (step_pin == PIN_UNDEFINED) return nullptr;
    if ((step_pin & PIN_EXTERNAL_FLAG)) return nullptr;  // External pins not supported
    if (!digitalPinToPort(step_pin)) return nullptr;      // Invalid pin → NULL port

    int8_t idx = findFreeSlot();
    if (idx < 0) return nullptr;

    fas_queue[idx]._initVars();
    fas_queue[idx].init((uint8_t)idx, step_pin);
    stepper_allocated_mask |= (1 << idx);
    return &fas_queue[idx];
}

// ====================================================================
// freeQueue — Release stepper slot for reallocation
//
// Stops the queue (if running), clears the allocated bit, and resets
// channel mapping so the slot can be reused by another step pin.
// ====================================================================
void StepperQueue::freeQueue(void) {
    uint32_t prim = __get_PRIMASK();
    __disable_irq();

    if (_isRunning) {
        FAS_TIMER->DIER &= ~CCXIE_BIT(_timer_ch);
        _isRunning = false;
    }

    uint8_t ch = _timer_ch;
    stepper_allocated_mask &= ~(1 << ch);
    _ch_to_queue[ch] = NULL;

    // Clear all state to avoid stale values on reallocation
    _step_pin = PIN_UNDEFINED;
    _step_port = NULL;
    _ccr_reg = NULL;
    _dir_bsrr = NULL;
    _last_command_ticks = 0;    // prevent getActualTicksWithDirection() returning stale speed
    _pulse_high = false;        // reset pulse state
    _dir_delay_active = false;  // reset dir settle state

    if (!prim) __enable_irq();
}

// ====================================================================
// Engine initialization
// ====================================================================
void fas_init_engine(FastAccelStepperEngine* engine) {
    fas_engine = engine;

    // Initialize Log2 timer frequency variables if using runtime fallback
    // (SUPPORT_LOG2_TIMER_FREQ_VARIABLES path in RampCalculator.h)
    init_ramp_module();

    // Print clock error warning (if any) — user must have called Serial.begin()
    // before engine.init() for this to appear.
    fas_stm32_report_clock_error();

    // PendSV at lowest priority — avoids blocking higher-priority interrupts
    NVIC_SetPriority(PendSV_IRQn, 0xFF);
}

#endif /* ARDUINO_ARCH_STM32 */