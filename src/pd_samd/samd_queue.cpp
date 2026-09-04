#include "fas_queue/stepper_queue.h"

#if defined(SUPPORT_SAMD51)

#include <wiring_private.h>  // pinPeripheral()

// ============================================================================
// SAMD51 pulse driver.
//
// Pipeline model (see samd_queue.h): PERBUF/CCBUF written during period k are
// latched by hardware at the k -> k+1 boundary. startQueue() primes period 1
// in the direct registers and stages period 2 in the buffered ones; every
// overflow interrupt thereafter stages exactly one further period.
// ============================================================================

static FastAccelStepperEngine* fas_engine = NULL;

// ISR dispatch: TCC instance index -> owning queue
static StepperQueue* queue_for_tcc[TCC_INST_NUM] = {NULL};

static Tcc* const tcc_instances[TCC_INST_NUM] = TCC_INSTS;

#define PULSE_TICKS \
  ((uint16_t)((FAS_SAMD_PULSE_WIDTH_US) * (TICKS_PER_S / 1000000L)))

// ----------------------------------------------------------------------------
// Ramp tick: a plain TC in 16-bit MFRQ mode interrupts every ~4 ms and runs
// the (platform-independent) ramp generator to refill the command queues.
// ----------------------------------------------------------------------------

#define FAS_TC_CONCAT2_(n) TC##n
#define FAS_TC_CONCAT2(n) FAS_TC_CONCAT2_(n)
#define FAS_TC_HANDLER_(n) void TC##n##_Handler(void)
#define FAS_TC_HANDLER(n) FAS_TC_HANDLER_(n)
#define FAS_TC_IRQN_(n) TC##n##_IRQn
#define FAS_TC_IRQN(n) FAS_TC_IRQN_(n)
#define FAS_TC_GCLK_ID_(n) TC##n##_GCLK_ID
#define FAS_TC_GCLK_ID(n) FAS_TC_GCLK_ID_(n)

#define RAMP_TC FAS_TC_CONCAT2(FAS_SAMD_RAMP_TC)
#define RAMP_TC_IRQN FAS_TC_IRQN(FAS_SAMD_RAMP_TC)
#define RAMP_TC_GCLK_ID FAS_TC_GCLK_ID(FAS_SAMD_RAMP_TC)

// 120 MHz / 1024 prescaler = 117187.5 Hz; 4 ms => 469 counts
#define RAMP_TC_TOP ((uint16_t)((F_CPU / 1024) * DELAY_MS_BASE / 1000))

// NVIC priorities (SAMD51: 0 = highest, 7 = lowest). Step staging must
// preempt the (long-running) ramp calculation.
#define STEP_ISR_PRIORITY 1
#define RAMP_ISR_PRIORITY 6

FAS_TC_HANDLER(FAS_SAMD_RAMP_TC) {
  RAMP_TC->COUNT16.INTFLAG.reg = TC_INTFLAG_OVF;
  if (fas_engine != NULL) {
    fas_engine->manageSteppers();
  }
}

static void ramp_tc_init() {
  GCLK->PCHCTRL[RAMP_TC_GCLK_ID].reg =
      GCLK_PCHCTRL_GEN_GCLK0_Val | (1 << GCLK_PCHCTRL_CHEN_Pos);

  Tc* tc = RAMP_TC;
  tc->COUNT16.CTRLA.bit.ENABLE = 0;
  while (tc->COUNT16.SYNCBUSY.bit.ENABLE) {
  }
  tc->COUNT16.CTRLA.bit.SWRST = 1;
  while (tc->COUNT16.SYNCBUSY.bit.SWRST) {
  }
  tc->COUNT16.CTRLA.reg = TC_CTRLA_MODE_COUNT16 | TC_CTRLA_PRESCALER_DIV1024 |
                          TC_CTRLA_PRESCSYNC_PRESC;
  // Match frequency mode: CC0 is the period
  tc->COUNT16.WAVE.reg = TC_WAVE_WAVEGEN_MFRQ;
  tc->COUNT16.CC[0].reg = RAMP_TC_TOP;
  while (tc->COUNT16.SYNCBUSY.bit.CC0) {
  }
  tc->COUNT16.INTENSET.reg = TC_INTENSET_OVF;
  NVIC_SetPriority(RAMP_TC_IRQN, RAMP_ISR_PRIORITY);
  NVIC_EnableIRQ(RAMP_TC_IRQN);
  tc->COUNT16.CTRLA.bit.ENABLE = 1;
  while (tc->COUNT16.SYNCBUSY.bit.ENABLE) {
  }
}

void fas_init_engine(FastAccelStepperEngine* engine) {
  fas_engine = engine;
  ramp_tc_init();
}

// ----------------------------------------------------------------------------
// Step generation
// ----------------------------------------------------------------------------

// Consume one step (or one pause period) from the queue and stage it into the
// buffered registers. Returns false if the queue is empty.
// Called from the OVF ISR and (for period 2) from startQueue().
bool StepperQueue::stageNextPeriod() {
  uint8_t rp = read_idx;
  if (rp == next_write_idx) {
    return false;
  }
  struct queue_entry* e = &entry[rp & QUEUE_LEN_MASK];
  if (e->toggle_dir) {
    // First period of this entry: apply the direction change. The staged
    // period starts one full period later, so the driver's DIR setup time
    // is amply satisfied. (The step edge of the *current* period fired just
    // before this ISR; DIR hold time after an edge is ~20 ns on common
    // drivers, far below our interrupt latency.)
    e->toggle_dir = 0;
    if (_dirPinPortGrp != NULL) {
      _dirPinPortGrp->OUTTGL.reg = _dirPinMask;
    }
  }
  _tcc->PERBUF.reg = e->ticks;
  _tcc->CCBUF[_tcc_channel].reg = e->hasSteps ? _pulse_ticks : 0;
  if (e->steps > 1) {
    e->steps--;
  } else {
    read_idx = rp + 1;
  }
  return true;
}

void StepperQueue::handleOverflow() {
  if ((_tcc->INTFLAG.reg & TCC_INTFLAG_OVF) == 0) {
    // Spurious: startQueue() cleared the flag after the NVIC had already
    // pended this interrupt. The new run's priming must not be consumed.
    return;
  }
  _tcc->INTFLAG.reg = TCC_INTFLAG_OVF;
  if (stageNextPeriod()) {
    _noMoreCommands = false;
    return;
  }
  if (_noMoreCommands) {
    // Grace period elapsed without new commands: stop.
    // STOP drives the output to the DRVCTRL non-recoverable state (low).
    // Do not overlap a CTRLB command still syncing (e.g. the RETRIGGER of a
    // startQueue() whose first period was very short).
    while (_tcc->SYNCBUSY.bit.CTRLB) {
    }
    _tcc->CTRLBSET.reg = TCC_CTRLBSET_CMD_STOP;
    _noMoreCommands = false;
    _isRunning = false;
  } else {
    // Queue drained. Stage one dead period (no pulse, current interval) so a
    // command arriving within it continues the stream seamlessly.
    _noMoreCommands = true;
    _tcc->CCBUF[_tcc_channel].reg = 0;
  }
}

#define SAMD_TCC_ISR(n)                       \
  void TCC##n##_0_Handler(void) {             \
    StepperQueue* q = queue_for_tcc[n];       \
    if (q != NULL) {                          \
      q->handleOverflow();                    \
    } else {                                  \
      TCC##n->INTFLAG.reg = TCC_INTFLAG_MASK; \
    }                                         \
  }

#ifdef TCC0
SAMD_TCC_ISR(0)
#endif
#ifdef TCC1
SAMD_TCC_ISR(1)
#endif
#ifdef TCC2
SAMD_TCC_ISR(2)
#endif
#ifdef TCC3
SAMD_TCC_ISR(3)
#endif
#ifdef TCC4
SAMD_TCC_ISR(4)
#endif

// ----------------------------------------------------------------------------
// Queue protocol
// ----------------------------------------------------------------------------

// Map a step pin to its TCC instance/channel via the variant's pin table.
// Returns false if the pin has no TCC mux.
static bool step_pin_to_tcc(uint8_t step_pin, uint8_t* tcc_num,
                            uint8_t* channel) {
  if (step_pin >= PINS_COUNT) {
    return false;
  }
  const PinDescription& pd = g_APinDescription[step_pin];
  if ((pd.ulPinAttribute &
       (PIN_ATTR_PWM_E | PIN_ATTR_PWM_F | PIN_ATTR_PWM_G)) == 0) {
    return false;
  }
  uint32_t num = GetTCNumber(pd.ulPWMChannel);
  if (num >= TCC_INST_NUM) {
    return false;  // pin is on a TC, not a TCC
  }
  *tcc_num = (uint8_t)num;
  *channel = GetTCChannelNumber(pd.ulPWMChannel);
  return true;
}

bool StepperQueue::isValidStepPin(uint8_t step_pin) {
  uint8_t tcc_num, channel;
  if (!step_pin_to_tcc(step_pin, &tcc_num, &channel)) {
    return false;
  }
  return queue_for_tcc[tcc_num] == NULL;
}

void StepperQueue::init(uint8_t queue_num, uint8_t step_pin) {
  uint8_t tcc_num = 0, channel = 0;
  step_pin_to_tcc(step_pin, &tcc_num, &channel);  // validated by caller
  _queue_num = queue_num;
  _step_pin = step_pin;
  _tcc_num = tcc_num;
  _tcc_channel = channel;
  _tcc = tcc_instances[tcc_num];
  _pulse_ticks = PULSE_TICKS;

  // Step pin low as plain GPIO first; connect() muxes it to the TCC.
  digitalWrite(step_pin, LOW);
  pinMode(step_pin, OUTPUT);

  // 16 MHz step timebase: dedicated GCLK generator from DFLL48M / 3
  // (see ADR 0002). Idempotent, shared by all queue TCCs.
  GCLK->GENCTRL[FAS_SAMD_GCLK_GEN].reg =
      GCLK_GENCTRL_SRC_DFLL | GCLK_GENCTRL_DIV(3) | GCLK_GENCTRL_GENEN;
  while (GCLK->SYNCBUSY.reg & GCLK_SYNCBUSY_GENCTRL(FAS_SAMD_GCLK_GEN)) {
  }

  // GCLK_CLKCTRL_IDs is the core's instance -> PCHCTRL index table
  // (the channel is shared between sibling instances, e.g. TCC0/TCC1).
  GCLK->PCHCTRL[GCLK_CLKCTRL_IDs[tcc_num]].reg =
      GCLK_PCHCTRL_GEN(FAS_SAMD_GCLK_GEN) | (1 << GCLK_PCHCTRL_CHEN_Pos);

  Tcc* tcc = _tcc;
  tcc->CTRLA.bit.ENABLE = 0;
  while (tcc->SYNCBUSY.bit.ENABLE) {
  }
  tcc->CTRLA.bit.SWRST = 1;
  while (tcc->SYNCBUSY.bit.SWRST) {
  }
  tcc->CTRLA.reg = TCC_CTRLA_PRESCALER_DIV1 | TCC_CTRLA_PRESCSYNC_PRESC;
  tcc->WAVE.reg = TCC_WAVE_WAVEGEN_NPWM;
  while (tcc->SYNCBUSY.bit.WAVE) {
  }
  // When stopped, actively drive the step output low (NRE=1, NRV=0).
  tcc->DRVCTRL.reg |= TCC_DRVCTRL_NRE0 << _tcc_channel;
  // Prime CC=0 (no pulse) before enabling, so enabling cannot emit a glitch.
  tcc->PER.reg = 0xffff;
  while (tcc->SYNCBUSY.bit.PER) {
  }
  tcc->CC[_tcc_channel].reg = 0;
  while (tcc->SYNCBUSY.reg & (TCC_SYNCBUSY_CC0 << _tcc_channel)) {
  }
  tcc->CTRLA.bit.ENABLE = 1;
  while (tcc->SYNCBUSY.bit.ENABLE) {
  }
  // Enabling starts the counter: stop it immediately. Output goes low (NRE).
  tcc->CTRLBSET.reg = TCC_CTRLBSET_CMD_STOP;
  while (tcc->SYNCBUSY.bit.CTRLB) {
  }

  queue_for_tcc[tcc_num] = this;

  IRQn_Type irqn;
  switch (tcc_num) {
#ifdef TCC0
    case 0:
      irqn = TCC0_0_IRQn;
      break;
#endif
#ifdef TCC1
    case 1:
      irqn = TCC1_0_IRQn;
      break;
#endif
#ifdef TCC2
    case 2:
      irqn = TCC2_0_IRQn;
      break;
#endif
#ifdef TCC3
    case 3:
      irqn = TCC3_0_IRQn;
      break;
#endif
#ifdef TCC4
    case 4:
      irqn = TCC4_0_IRQn;
      break;
#endif
    default:
      return;
  }
  tcc->INTENSET.reg = TCC_INTENSET_OVF;
  NVIC_SetPriority(irqn, STEP_ISR_PRIORITY);
  NVIC_EnableIRQ(irqn);

  connect();
}

void StepperQueue::connect() {
  const PinDescription& pd = g_APinDescription[_step_pin];
  if (pd.ulPinAttribute & PIN_ATTR_PWM_E) {
    pinPeripheral(_step_pin, PIO_TIMER);
  } else if (pd.ulPinAttribute & PIN_ATTR_PWM_F) {
    pinPeripheral(_step_pin, PIO_TIMER_ALT);
  } else if (pd.ulPinAttribute & PIN_ATTR_PWM_G) {
    pinPeripheral(_step_pin, PIO_TCC_PDEC);
  }
  _connected = true;
}

void StepperQueue::disconnect() {
  // pinMode() clears PMUXEN: the pin reverts to a plain low GPIO output.
  digitalWrite(_step_pin, LOW);
  pinMode(_step_pin, OUTPUT);
  _connected = false;
}

void StepperQueue::startQueue() {
  if (_isRunning) {
    return;
  }
  Tcc* tcc = _tcc;

  uint8_t rp = read_idx;
  if (rp == next_write_idx) {
    return;  // nothing to do (addQueueEntry guards this already)
  }

  _isRunning = true;
  _noMoreCommands = false;

  // The TCC is stopped here (init(), forceStop() or the OVF ISR stopped it),
  // but three remnants of the previous run must be cleared before priming:
  //
  // 1. A latched-but-unserviced overflow (possible when a stop lands just
  //    after TOP). Left pending, the ISR would fire mid-priming and consume
  //    the entries staged below. Clear it first; handleOverflow() also
  //    guards against a spurious NVIC-pended call.
  tcc->INTFLAG.reg = TCC_INTFLAG_OVF;
  // 2. The STOP command itself may still be crossing into the 16 MHz TCC
  //    clock domain. Register write-syncs must not overlap it.
  while (tcc->SYNCBUSY.bit.CTRLB) {
  }
  // 3. Stale buffered values (an abrupt stop strands a staged PERBUF/CCBUF
  //    that never latched). Silicon erratum DS80000748 2.20.1: clearing
  //    STATUS.PERBUFV/CCBUFVx releases SYNCBUSY *before* the buffer register
  //    is actually restored, so a PER/CC write issued then collides with the
  //    still-running restore and SYNCBUSY.PER locks permanently (observed as
  //    a random hang below). Workaround per errata: clear the flags twice.
  //    The restore itself reports through SYNCBUSY.PER/CCx (per the erratum's
  //    TC wording), so drain those channels too before the direct writes.
  uint32_t bufv_mask =
      TCC_STATUS_PERBUFV | (TCC_STATUS_CCBUFV0 << _tcc_channel);
  uint32_t sync_mask = TCC_SYNCBUSY_STATUS | TCC_SYNCBUSY_PER |
                       (TCC_SYNCBUSY_CC0 << _tcc_channel);
  tcc->STATUS.reg = bufv_mask;
  while (tcc->SYNCBUSY.reg & sync_mask) {
  }
  tcc->STATUS.reg = bufv_mask;
  while (tcc->SYNCBUSY.reg & sync_mask) {
  }

  // Prime period 1 in the direct registers.
  struct queue_entry* e = &entry[rp & QUEUE_LEN_MASK];
  if (e->toggle_dir) {
    e->toggle_dir = 0;
    if (_dirPinPortGrp != NULL) {
      _dirPinPortGrp->OUTTGL.reg = _dirPinMask;
      delayMicroseconds(AFTER_SET_DIR_PIN_DELAY_US);
    }
  }
  tcc->PER.reg = e->ticks;
  while (tcc->SYNCBUSY.bit.PER) {
  }
  tcc->CC[_tcc_channel].reg = e->hasSteps ? _pulse_ticks : 0;
  while (tcc->SYNCBUSY.reg & (TCC_SYNCBUSY_CC0 << _tcc_channel)) {
  }
  if (e->steps > 1) {
    e->steps--;
  } else {
    read_idx = rp + 1;
  }

  // Stage period 2 in the buffered registers (latched at the first period
  // boundary). If the queue only held one period, insert the dead period.
  if (!stageNextPeriod()) {
    _noMoreCommands = true;
    tcc->CCBUF[_tcc_channel].reg = 0;
  }

  // (INTFLAG.OVF was cleared at entry; the counter has been stopped since,
  // so no overflow can have latched in between.)
  // Retrigger: counter restarts at zero, the step edge (if CC > 0) fires now.
  // Scope-verified on SAMD51: RETRIGGER is not an update condition, so the
  // buffered period-2 values staged above are NOT latched here; they take
  // effect at the first period boundary as the pipeline model requires
  // (an acceleration ramp from standstill shows the long first interval,
  // not a duplicated second interval).
  tcc->CTRLBSET.reg = TCC_CTRLBSET_CMD_RETRIGGER;
}

void StepperQueue::forceStop() {
  // Called from application context while the TCC may be running at speed.
  // The OVF ISR stages PERBUF/CCBUF at every period boundary, and those
  // writes synchronize through the shared SYNCBUSY.PER/CCx channels. A STOP
  // command issued while one of those syncs is in flight can wedge the sync
  // bridge (same hardware as erratum DS80000748 2.20.1), leaving SYNCBUSY.PER
  // stuck and hanging the next startQueue(). So: block the ISR, drain every
  // in-flight sync, then stop.
  noInterrupts();
  while (_tcc->SYNCBUSY.reg &
         (TCC_SYNCBUSY_CTRLB | TCC_SYNCBUSY_STATUS | TCC_SYNCBUSY_PER |
          (TCC_SYNCBUSY_CC0 << _tcc_channel))) {
  }
  _tcc->CTRLBSET.reg = TCC_CTRLBSET_CMD_STOP;
  while (_tcc->SYNCBUSY.bit.CTRLB) {
  }
  // A stop landing just after TOP may leave a latched overflow. Drop it so
  // the stale NVIC pend hits handleOverflow()'s spurious-interrupt guard.
  _tcc->INTFLAG.reg = TCC_INTFLAG_OVF;
  _isRunning = false;
  _noMoreCommands = false;
  // empty the queue
  read_idx = next_write_idx;
  interrupts();
}

static uint8_t stepper_allocated_count = 0;

StepperQueue* StepperQueue::tryAllocateQueue(FastAccelStepperEngine* engine,
                                             uint8_t step_pin) {
  (void)engine;
  // isValidStepPin() also rejects pins whose TCC is already claimed
  // (queue_for_tcc occupancy), so queues are simply handed out in order.
  if (!isValidStepPin(step_pin)) {
    return nullptr;
  }
  if (stepper_allocated_count >= NUM_QUEUES) {
    return nullptr;
  }
  StepperQueue* q = &fas_queue[stepper_allocated_count];
  q->_initVars();
  q->init(stepper_allocated_count, step_pin);
  stepper_allocated_count++;
  return q;
}

#endif  // SUPPORT_SAMD51
