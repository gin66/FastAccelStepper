#include "fas_queue/stepper_queue.h"
#include "log2/Log2Representation.h"
#include "fas_ramp/RampControl.h"

#if defined(ARDUINO_ARCH_STM32)

// ====================================================================
// Static data
// ====================================================================
static FastAccelStepperEngine* fas_engine = NULL;
static uint8_t stepper_allocated_mask = 0;
static volatile bool _cyclic_pending = false;
static uint32_t _last_cyclic_uwtick = 0;
uint8_t fas_stm32_clock_error = 0;
StepperQueue* StepperQueue::_ch_to_queue[4] = {NULL, NULL, NULL, NULL};

// ====================================================================
// Log2 timer frequency variables (for SUPPORT_LOG2_TIMER_FREQ_VARIABLES)
// Defined here when RampCalculator.h selects runtime fallback path.
// ====================================================================
#ifdef SUPPORT_LOG2_TIMER_FREQ_VARIABLES
static log2_value_t log2_timer_freq;
static log2_value_t log2_timer_freq_div_sqrt_of_2;
static log2_value_t log2_timer_freq_square_div_2;
#endif

// ====================================================================
// TIM2 clock detection
//
// TIM2 counter clock = PCLK1 * (APB1_prescaler==1 ? 1 : 2)
//
// The APB1 prescaler is in RCC->CFGR.PPRE on most STM32 families.
// On H7 it is in RCC->D2CFGR.D2PPRE1 (D2 domain).
// ====================================================================
static uint32_t getTim2Clock(void) {
    uint32_t pclk1 = HAL_RCC_GetPCLK1Freq();
#if defined(STM32H7xx)
    // H7 series: D2 domain, D2CFGR register
    uint32_t dppre1 = (RCC->D2CFGR & RCC_D2CFGR_D2PPRE1) >> RCC_D2CFGR_D2PPRE1_Pos;
    if (dppre1 > 1) pclk1 *= 2;
#else
    // All other STM32 families
    uint32_t apb1_pre = (RCC->CFGR & RCC_CFGR_PPRE) >> RCC_CFGR_PPRE_Pos;
    if (apb1_pre > 1) pclk1 *= 2;
#endif
    return pclk1;
}

// ====================================================================
// TIM2 initialization (step generation timer)
// Called once when first stepper is initialized.
// ====================================================================
static void initStepTimer(void) {
    static bool initialized = false;
    if (initialized) return;
    initialized = true;

    // Enable TIM2 clock (portable across all STM32 families)
    __HAL_RCC_TIM2_CLK_ENABLE();

    // Compute prescaler from actual TIM2 clock and desired TICKS_PER_S
    uint32_t tim2_clk = getTim2Clock();
    uint32_t psc;

    if (TICKS_PER_S > tim2_clk) {
        // Cannot achieve TICKS_PER_S at this clock frequency.
        // Run at maximum rate (PSC=0). All timing will be incorrect.
        fas_stm32_clock_error = 1;
        psc = 0;
    } else {
        psc = (tim2_clk / TICKS_PER_S) - 1;
        if (psc > 65535) psc = 65535;
    }

    // Configure TIM2
    TIM2->CR1 = 0;
    TIM2->PSC = psc;
    TIM2->ARR = 0xFFFFFFFF;   // 32-bit auto-reload
    TIM2->EGR |= TIM_EGR_UG;  // Reload shadow registers (PSC, ARR)
    TIM2->DIER = 0;           // All interrupts disabled initially

    // Force LOW all channels (OCxM = 100 = Force Inactive)
    // This prevents spurious pulses during initialization.
    TIM2->CCMR1 = (4 << 4) | (4 << 12);   // OC1M=100, OC2M=100
    TIM2->CCMR2 = (4 << 4) | (4 << 12);   // OC3M=100, OC4M=100

    // NVIC configuration
    NVIC_SetPriority(TIM2_IRQn, 0);   // Highest priority for step timing
    NVIC_EnableIRQ(TIM2_IRQn);

    // Start timer
    TIM2->CR1 |= TIM_CR1_CEN;
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

    // Ensure TIM2 is initialized
    initStepTimer();

    // Step pin GPIO configuration
    _step_pin = step_pin;
    _step_port = digitalPinToPort(step_pin);
    uint32_t mask = digitalPinToBitMask(step_pin);
    _step_set_mask = mask;

    // Configure clear mask based on BSRR/BRR architecture
#if STM32_BSRR_CLEAR_SHIFT
    _step_clr_mask = mask << 16;  // BSRR high half = reset bits
#else
    _step_clr_mask = mask;        // BRR register (direct)
#endif

    // Configure pin as OUTPUT, initial LOW
    pinMode(step_pin, OUTPUT);
    digitalWrite(step_pin, LOW);

    // Store CCR register pointer for fast ISR access
    volatile uint32_t* ccr[] = {&TIM2->CCR1, &TIM2->CCR2, &TIM2->CCR3, &TIM2->CCR4};
    _ccr_reg = ccr[_timer_ch];

    // Register channel-to-queue mapping
    _ch_to_queue[_timer_ch] = this;
    _initialized = true;
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
    // This ensures the compare value is visible before the IRQ can fire.
    *_ccr_reg = TIM2->CNT + (TICKS_PER_S / 1000000);  // ~1µs offset
    __DMB();

    // DIER RMW: disable interrupts to prevent race with ISR on other channels
    __disable_irq();
    TIM2->DIER |= CCXIE_BIT(_timer_ch);
    __enable_irq();
}

void StepperQueue::forceStop(void) {
    // Disable CC interrupt (atomic)
    __disable_irq();
    TIM2->DIER &= ~CCXIE_BIT(_timer_ch);
    _isRunning = false;
    read_idx = next_write_idx;  // Discard remaining queue entries
    __enable_irq();

    // Ensure step pin is LOW (use BSRR/BRR appropriately)
    if (_step_port) {
#if STM32_BSRR_CLEAR_SHIFT
        _step_port->BSRR = _step_clr_mask;
#else
        _step_port->BRR = _step_clr_mask;
#endif
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
// Called at end of TIM2_IRQHandler every ~3ms.
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
// TIM2_IRQHandler — Step pulse generation (all 4 channels)
//
// State machine:
//   Phase 1 (_pulse_high=false): Set step pin HIGH, schedule LOW
//   Phase 2 (_pulse_high=true):  Set step pin LOW, process queue
//   DirSettle (_dir_delay_active): Direction settling complete → start pulse
//
// SR handling:
//   Snapshot SR at entry. Build ch_processed mask of all handled flags.
//   Clear all at end by writing ~ch_processed (rc_w0 behavior).
// ====================================================================
void TIM2_IRQHandler(void) {
    uint32_t sr = TIM2->SR;
    uint32_t ch_processed = 0;

    for (uint8_t ch = 0; ch < 4; ch++) {
        uint32_t ccif = CCXIF_BIT(ch);
        if (!(sr & ccif)) continue;

        // Always mark for clear — prevents infinite loop from spurious IRQs
        ch_processed |= ccif;

        StepperQueue* q = StepperQueue::_ch_to_queue[ch];
        if (!q || !q->_isRunning) continue;

        if (q->_pulse_high) {
            // ====== Phase 2: pulse end ======
            // The step pin was HIGH; bring it LOW.
            q->_pulse_high = false;

#if STM32_BSRR_CLEAR_SHIFT
            q->_step_port->BSRR = q->_step_clr_mask;
#else
            q->_step_port->BRR = q->_step_clr_mask;
#endif

            // Read queue entry
            uint8_t rp = q->read_idx;
            uint8_t wp = q->next_write_idx;

            if (rp == wp) {
                // Queue empty — stop this channel
                TIM2->DIER &= ~CCXIE_BIT(ch);
                q->_isRunning = false;
                continue;
            }

            struct queue_entry* e = &q->entry[rp & QUEUE_LEN_MASK];
            q->_last_command_ticks = e->ticks;

            if (e->steps > 1) {
                // Multi-step command: reduce step count, continue with same period
                e->steps--;
                *q->_ccr_reg = TIM2->CNT + (uint32_t)e->ticks;
            } else {
                // Single step complete — advance to next entry
                rp++;
                q->read_idx = rp;

                if (rp == wp) {
                    TIM2->DIER &= ~CCXIE_BIT(ch);
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
                    *q->_ccr_reg = TIM2->CNT + dd;
                    q->_dir_delay_active = true;
                    continue;
                }

                // No direction change — schedule next step pulse
                *q->_ccr_reg = TIM2->CNT + (uint32_t)e->ticks;
            }
        } else if (q->_dir_delay_active) {
            // ====== Direction settling complete ======
            // The settling delay has elapsed. Now start the first step pulse
            // in the new direction (set pin HIGH).
            q->_dir_delay_active = false;
            q->_pulse_high = true;
            q->_step_port->BSRR = q->_step_set_mask;
            *q->_ccr_reg = TIM2->CNT + STEP_PULSE_WIDTH_TICKS;
        } else {
            // ====== Phase 1: pulse start ======
            // Set step pin HIGH, schedule LOW after STEP_PULSE_WIDTH_TICKS.
            q->_pulse_high = true;
            q->_step_port->BSRR = q->_step_set_mask;
            *q->_ccr_reg = TIM2->CNT + STEP_PULSE_WIDTH_TICKS;
        }
    }

    // Clear all processed flags at once (rc_w0: bits set to 1 are ignored)
    TIM2->SR = ~ch_processed;

    // Trigger cyclic queue fill
    cyclic_check_and_pend();
}

// ====================================================================
// PendSV_Handler — Deferred queue fill
//
// Weak attribute allows FreeRTOS to override this handler.
// Define DISABLE_FAS_PENDSV to skip installation entirely.
// ====================================================================
#if !defined(DISABLE_FAS_PENDSV)
__attribute__((weak)) void PendSV_Handler(void) {
    _cyclic_pending = false;
    __DMB();
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

    // PendSV at lowest priority — avoids blocking higher-priority interrupts
    NVIC_SetPriority(PendSV_IRQn, 0xFF);
}

#endif /* ARDUINO_ARCH_STM32 */