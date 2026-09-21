#include "fas_queue/stepper_queue.h"

#if defined(SUPPORT_TEENSY4)

// ============================================================================
// Teensy 4.x (i.MX RT1062) pulse driver. EXPERIMENTAL - NOT YET VERIFIED ON
// REAL HARDWARE. See pd_teensy/pd_config.h for context.
//
// Register sequence (CTRL/CSCTRL/COMP1/CMPLD1 setup, the "two-phase" toggle
// technique of alternating COMP1 between the pulse width and the remaining
// period) is adapted from luni64/TeensyStep4's TMR.h (MIT licensed), the
// only publicly available, hardware-verified reference found for driving
// these timers this way:
//   https://github.com/luni64/TeensyStep4/blob/master/src/TS4/timers/Teensy4/TMR/TMR.h
// The step/dir pin toggling via digitalWriteFast() with a runtime pin
// number is likewise taken from TeensyStep4's stepperbase.h/.cpp, which
// uses that exact pattern from real ISRs.
//
// Queue/ramp protocol integration (grace period before stopping, dead
// period so a late-arriving command is picked up seamlessly, one
// FastAccelStepperEngine::manageSteppers() tick every ~4 ms) mirrors this
// library's own pd_samd backend, adapted from SAMD51's fully-hardware
// double-buffered PWM to this driver's ISR-per-edge model (closer to how
// the AVR backend uses a hardware compare match to time each edge).
// ============================================================================

#define TMR_PRESCALE 4  // divide by 2^4 = 16 -> 150MHz / 16 = 9.375 MHz

#define PULSE_TICKS \
  ((uint16_t)(((uint32_t)(FAS_TEENSY_PULSE_WIDTH_US) * TICKS_PER_S) / 1000000L))

static FastAccelStepperEngine* fas_engine = NULL;
static IntervalTimer fas_ramp_timer;

static void fas_ramp_tick_isr() {
  if (fas_engine != NULL) {
    fas_engine->manageSteppers();
  }
}

void fas_init_engine(FastAccelStepperEngine* engine) {
  fas_engine = engine;
  fas_ramp_timer.begin(fas_ramp_tick_isr, (unsigned int)(DELAY_MS_BASE * 1000));
}

// ----------------------------------------------------------------------------
// Step generation
// ----------------------------------------------------------------------------

// ISR dispatch: [module][channel] -> owning queue
static StepperQueue* queue_for_channel[4][4] = {{NULL}};

static IMXRT_TMR_t* const tmr_module_regs[4] = {
    (IMXRT_TMR_t*)IMXRT_TMR1_ADDRESS, (IMXRT_TMR_t*)IMXRT_TMR2_ADDRESS,
    (IMXRT_TMR_t*)IMXRT_TMR3_ADDRESS, (IMXRT_TMR_t*)IMXRT_TMR4_ADDRESS};

static const IRQ_NUMBER_t tmr_module_irq[4] = {IRQ_QTIMER1, IRQ_QTIMER2,
                                               IRQ_QTIMER3, IRQ_QTIMER4};

// Consume the queue entry at read_idx `rp` (queue must be non-empty):
// apply a pending direction change, then schedule the immediate next
// compare event - either the rising edge of a step pulse (scheduled
// _pulse_ticks out) or the full duration of a pause entry.
void StepperQueue::primeFromQueue(uint8_t rp) {
  struct queue_entry* e = &entry[rp & QUEUE_LEN_MASK];
  if (e->toggle_dir) {
    e->toggle_dir = 0;
    digitalWriteFast(dirPin, e->dirPinState);
  }
  if (e->hasSteps) {
    _remaining_ticks = (e->ticks > _pulse_ticks)
                          ? (uint16_t)(e->ticks - _pulse_ticks)
                          : (uint16_t)1;
    digitalWriteFast(_step_pin, HIGH);
    _pulse_phase = true;
    _regs->COMP1 = _pulse_ticks;
    _regs->CMPLD1 = _pulse_ticks;
  } else {
    _pulse_phase = false;
    _regs->COMP1 = e->ticks;
    _regs->CMPLD1 = e->ticks;
  }
  if (e->steps > 1) {
    e->steps--;
  } else {
    read_idx = rp + 1;
  }
}

// Called once per QuadTimer compare-match event for this channel (the
// TCF1 flag itself is cleared by the module-level dispatcher below).
//
// State machine (mirrors TeensyStep4's TmrTimer::ISR, adapted for this
// library's repeat-count queue entries):
//  _pulse_phase == true:  pin is HIGH. This event is exactly _pulse_ticks
//    after the rising edge: lower the pin and schedule the remainder of
//    the step period (computed by primeFromQueue() when the pulse started).
//  _pulse_phase == false: pin is LOW. This event marks the end of the
//    previous inter-step gap (or the very start of a run): look at the
//    queue and schedule what's next.
void StepperQueue::handleCompareMatch() {
  if (_pulse_phase) {
    digitalWriteFast(_step_pin, LOW);
    _pulse_phase = false;
    _regs->COMP1 = _remaining_ticks;
    _regs->CMPLD1 = _remaining_ticks;
    return;
  }

  uint8_t rp = read_idx;
  if (rp == next_write_idx) {
    if (_noMoreCommands) {
      // Grace period elapsed without new commands: stop.
      _regs->CTRL = 0;
      _isRunning = false;
      _noMoreCommands = false;
      return;
    }
    // Queue drained: stage one dead period (no pulse) so a command
    // arriving within it continues the stream seamlessly.
    _noMoreCommands = true;
    _regs->COMP1 = MIN_CMD_TICKS;
    _regs->CMPLD1 = MIN_CMD_TICKS;
    return;
  }
  _noMoreCommands = false;
  primeFromQueue(rp);
}

#define TEENSY_TMR_ISR(N)                                             \
  static void teensyTmrIsr##N() {                                     \
    for (uint8_t ch = 0; ch < 4; ch++) {                              \
      StepperQueue* q = queue_for_channel[N][ch];                     \
      if (q == NULL) {                                                \
        continue;                                                     \
      }                                                                \
      IMXRT_TMR_CH_t* regs = &tmr_module_regs[N]->CH[ch];             \
      if (regs->CSCTRL & TMR_CSCTRL_TCF1) {                           \
        regs->CSCTRL &= ~TMR_CSCTRL_TCF1;                             \
        q->handleCompareMatch();                                      \
      }                                                                \
    }                                                                  \
  }

TEENSY_TMR_ISR(0)
TEENSY_TMR_ISR(1)
TEENSY_TMR_ISR(2)
TEENSY_TMR_ISR(3)

static void (*const tmr_module_isr[4])() = {teensyTmrIsr0, teensyTmrIsr1,
                                            teensyTmrIsr2, teensyTmrIsr3};

// ----------------------------------------------------------------------------
// Queue protocol
// ----------------------------------------------------------------------------

bool StepperQueue::isValidStepPin(uint8_t step_pin) {
  if (step_pin == PIN_UNDEFINED) {
    return false;
  }
  for (uint8_t i = 0; i < NUM_QUEUES; i++) {
    if (fas_queue[i]._step_pin == step_pin) {
      return false;  // already claimed by another queue
    }
  }
  return true;
}

void StepperQueue::init(uint8_t queue_num, uint8_t step_pin) {
  digitalWrite(step_pin, LOW);
  pinMode(step_pin, OUTPUT);

  _queue_num = queue_num;
  _step_pin = step_pin;
  _pulse_ticks = PULSE_TICKS;
  _pulse_phase = false;

  uint8_t mod = queue_num >> 2;   // 0..3 -> TMR1..TMR4
  uint8_t ch = queue_num & 0x03;  // 0..3
  _tmr_module = mod;
  _tmr_channel = ch;
  _regs = &tmr_module_regs[mod]->CH[ch];

  _regs->CTRL = 0;
  _regs->CNTR = 0;
  _regs->LOAD = 0;
  _regs->COMP1 = _pulse_ticks;
  _regs->CMPLD1 = _pulse_ticks;
  _regs->CSCTRL = 0;
  _regs->SCTRL = 0;

  queue_for_channel[mod][ch] = this;

  static bool module_isr_attached[4] = {false, false, false, false};
  if (!module_isr_attached[mod]) {
    attachInterruptVector(tmr_module_irq[mod], tmr_module_isr[mod]);
    NVIC_ENABLE_IRQ(tmr_module_irq[mod]);
    module_isr_attached[mod] = true;
  }

  connect();
}

void StepperQueue::connect() {
  pinMode(_step_pin, OUTPUT);
  digitalWriteFast(_step_pin, LOW);
  _connected = true;
}

void StepperQueue::disconnect() {
  digitalWriteFast(_step_pin, LOW);
  _connected = false;
}

void StepperQueue::startQueue() {
  if (_isRunning) {
    return;
  }
  uint8_t rp = read_idx;
  if (rp == next_write_idx) {
    return;  // nothing to do (addQueueEntry guards this already)
  }

  _isRunning = true;
  _noMoreCommands = false;

  _regs->CTRL = 0;
  _regs->CNTR = 0;
  _regs->LOAD = 0;
  _regs->CSCTRL = 0;
  _regs->SCTRL = 0;

  primeFromQueue(rp);

  _regs->CSCTRL |= TMR_CSCTRL_TCF1EN;
  _regs->CTRL =
      TMR_CTRL_CM(1) | TMR_CTRL_PCS(0b1000 | TMR_PRESCALE) | TMR_CTRL_LENGTH;
}

void StepperQueue::forceStop() {
  noInterrupts();
  _regs->CTRL = 0;
  _isRunning = false;
  _noMoreCommands = false;
  _pulse_phase = false;
  digitalWriteFast(_step_pin, LOW);
  read_idx = next_write_idx;
  interrupts();
}

static uint8_t stepper_allocated_count = 0;

StepperQueue* StepperQueue::tryAllocateQueue(FastAccelStepperEngine* engine,
                                             uint8_t step_pin) {
  (void)engine;
  if (!isValidStepPin(step_pin)) {
    return nullptr;
  }
  if (stepper_allocated_count >= MAX_STEPPER) {
    return nullptr;
  }
  for (uint8_t i = 0; i < MAX_STEPPER; i++) {
    if (fas_queue[i]._step_pin == PIN_UNDEFINED) {
      fas_queue[i]._initVars();
      fas_queue[i].init(i, step_pin);
      stepper_allocated_count++;
      return &fas_queue[i];
    }
  }
  return nullptr;
}

#endif  // SUPPORT_TEENSY4
