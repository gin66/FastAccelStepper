#include "fas_queue/stepper_queue.h"
#include "log2/Log2Representation.h"
#include "fas_ramp/RampControl.h"

#if defined(ARDUINO_ARCH_STM32)

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
// Timer selection — STM32C0 does NOT have TIM2
//
// STM32C0 series (e.g. STM32C031) only has TIM1, TIM3, TIM14, TIM16, TIM17.
// We use TIM3 on C0. TIM3 is a 16-bit timer (ARR=0xFFFF).
// All other STM32 families use TIM2 (32-bit on most, 16-bit on F1).
//
// TIM3 on C0 supports up to 4 channels (CCR1-CCR4) via TIM3->CCR1-4,
// which matches the 4 stepper channels expected by the code.
//
// Macros:
//   FAS_TIMER           — timer peripheral (TIM2 or TIM3)
//   FAS_TIMER_IRQn      — NVIC IRQ number
//   FAS_TIMER_RCC_ENABLE — HAL macro to enable timer clock
//   FAS_TIMER_ARR_MAX   — auto-reload max (0xFFFF for 16-bit, 0xFFFFFFFF for 32-bit)
//   FAS_TIM_IS_16BIT    — defined if timer is 16-bit (needs wrap handling)
// ====================================================================
#if defined(STM32C0xx)
    #define FAS_TIMER            TIM3
    #define FAS_TIMER_IRQn       TIM3_IRQn
    #define FAS_TIMER_RCC_ENABLE() __HAL_RCC_TIM3_CLK_ENABLE()
    #define FAS_TIM_IS_16BIT
    #define FAS_TIMER_ARR_MAX    0xFFFF
#else
    #define FAS_TIMER            TIM2
    #define FAS_TIMER_IRQn       TIM2_IRQn
    #define FAS_TIMER_RCC_ENABLE() __HAL_RCC_TIM2_CLK_ENABLE()
    #if defined(STM32F1xx)
        #define FAS_TIM_IS_16BIT
        #define FAS_TIMER_ARR_MAX 0xFFFF
    #else
        #define FAS_TIMER_ARR_MAX 0xFFFFFFFF
    #endif
#endif

// ====================================================================
// Static data
// ====================================================================
static FastAccelStepperEngine* fas_engine = NULL;
static uint8_t stepper_allocated_mask = 0;
static volatile bool _cyclic_pending = false;
static uint32_t _last_cyclic_uwtick = 0;
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
#elif defined(STM32G0xx) || defined(STM32C0xx)
    // G0/C0: single APB bus, uses RCC_CFGR_PPRE
    uint32_t pp = (RCC->CFGR & RCC_CFGR_PPRE) >> RCC_CFGR_PPRE_Pos;
    if (pp >= 4) pclk1 *= 2;
#else
    // F1/F4/F7/L1/L4/G4/WB...: uses RCC_CFGR_PPRE1
    uint32_t pp = (RCC->CFGR & RCC_CFGR_PPRE1) >> RCC_CFGR_PPRE1_Pos;
    if (pp >= 4) pclk1 *= 2;
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
#if defined(STM32F1xx) || defined(STM32C0xx)
    uint32_t cnt = FAS_TIMER->CNT;
    // 16-bit timer: (cnt + delay) & 0xFFFF xử lý wrap chính xác.
    // Xem toán học ở comment function.
    *ccr = (cnt + delay) & 0xFFFF;
#else
    *ccr = FAS_TIMER->CNT + delay;
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
    // This prevents spurious pulses during initialization.
    FAS_TIMER->CCMR1 = (4 << 4) | (4 << 12);   // OC1M=100, OC2M=100
    FAS_TIMER->CCMR2 = (4 << 4) | (4 << 12);   // OC3M=100, OC4M=100

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
#if defined(STM32C0xx)
    volatile uint32_t* ccr[] = {&FAS_TIMER->CCR1, &FAS_TIMER->CCR2, &FAS_TIMER->CCR3, &FAS_TIMER->CCR4};
#else
    // non-C0 branch: FAS_TIMER resolves to TIM2, so hardcoding TIM2 is intentional.
    volatile uint32_t* ccr[] = {&TIM2->CCR1, &TIM2->CCR2, &TIM2->CCR3, &TIM2->CCR4};
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
    uint32_t now = uwTick;
    if ((now - _last_cyclic_uwtick) >= CYCLIC_INTERVAL_MS) {
        _last_cyclic_uwtick = now;
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
#if defined(STM32C0xx)
void TIM3_IRQHandler(void) {
#else
void TIM2_IRQHandler(void) {
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
                // Pure pause (steps=0): skip pulse, schedule entry ticks
                q->_pulse_high = false;
                fas_tim_set_ccr(q->_ccr_reg, e->ticks);
            }
        } else {
            // ====== Phase 1: pulse start ======
            uint8_t rp = q->read_idx;
            struct queue_entry* e = &q->entry[rp & QUEUE_LEN_MASK];
            if (e->steps > 0) {
                // Set step pin HIGH, schedule LOW after STEP_PULSE_WIDTH_TICKS.
                q->_pulse_high = true;
                q->_step_port->BSRR = q->_step_set_mask;
                fas_tim_set_ccr(q->_ccr_reg, STEP_PULSE_WIDTH_TICKS);
            } else {
                // steps=0: skip pulse, schedule entry ticks directly.
                // On next ISR, _pulse_high=false and _dir_delay_active=false,
                // so it will re-enter Phase 1 and advance to next entry.
                q->_pulse_high = false;
                fas_tim_set_ccr(q->_ccr_reg, e->ticks);
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
#if defined(configUSE_PORT_OPTIMISED_TASK_SELECTION)
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
