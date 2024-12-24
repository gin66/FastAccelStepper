#if defined(ARDUINO_ARCH_AVR)
#include "AVRStepperPins.h"
#include "StepperISR.h"

// T is the timer module number 0,1,2,3...
// X is the Channel name A or B
//
//               BV1 Bv0
// Output one  is 1   1
// Output zero is 1   0
// Toggle      is 0   1
// Disconnect  is 0   0
//
#define Stepper_Zero(T, X) \
  TCCR##T##A = (TCCR##T##A | _BV(COM##T##X##1)) & ~_BV(COM##T##X##0)
// Force compare of Stepper_Toggle appears to be broken in simavr
// In other words, use of Stepper_Toggle yields errors in simavr, for which root
// cause is unclear
#ifdef DISABLE
#define Stepper_Toggle(T, X) \
  TCCR##T##A = (TCCR##T##A | _BV(COM##T##X##0)) & ~_BV(COM##T##X##1)
#endif
#define Stepper_One(T, X) TCCR##T##A |= _BV(COM##T##X##1) | _BV(COM##T##X##0)
#define Stepper_Disconnect(T, X) \
  TCCR##T##A &= ~(_BV(COM##T##X##1) | _BV(COM##T##X##0))
#define Stepper_IsOne(T, X)                                  \
  ((TCCR##T##A & (_BV(COM##T##X##0) | _BV(COM##T##X##1))) == \
   (_BV(COM##T##X##0) | _BV(COM##T##X##1)))
#define Stepper_IsDisconnected(T, X) \
  ((TCCR##T##A & (_BV(COM##T##X##0) | _BV(COM##T##X##1))) == 0)
#define Stepper_IsOneIfOutput(T, X) ((TCCR##T##A & _BV(COM##T##X##0)) != 0)
#define Stepper_ToggleDirection(CHANNEL) \
  *fas_queue_##CHANNEL._dirTogglePinPort = fas_queue_##CHANNEL._dirTogglePinMask
#define PREPARE_DIRECTION_PIN(CHANNEL) \
  if (e->toggle_dir) {                 \
    Stepper_ToggleDirection(CHANNEL);  \
  }

#ifdef SIMAVR_TIME_MEASUREMENT
#define prepareISRtimeMeasurement() DDRB |= 0x18
#define enterStepperISR() PORTB |= 0x08
#define exitStepperISR() PORTB ^= 0x08
#define enterFillQueueISR() PORTB |= 0x10
#define exitFillQueueISR() PORTB ^= 0x10
#elif defined(SIMAVR_TIME_MEASUREMENT_QUEUE)
#define prepareISRtimeMeasurement() DDRB |= 0x10
#define enterStepperISR() \
  {                       \
  }
#define exitStepperISR() \
  {                      \
  }
#define enterFillQueueISR() PORTB |= 0x10
#define exitFillQueueISR() PORTB ^= 0x10
#else
#define prepareISRtimeMeasurement() \
  {                                 \
  }
#define enterStepperISR() \
  {                       \
  }
#define exitStepperISR() \
  {                      \
  }
#define enterFillQueueISR() \
  {                         \
  }
#define exitFillQueueISR() \
  {                        \
  }
#endif

#ifdef SUPPORT_EXTERNAL_DIRECTION_PIN
#define TEST_NOT_REPEATING_ENTRY (e->repeat_entry == 0)
#else
#define TEST_NOT_REPEATING_ENTRY (0 == 0)
#endif

#define ForceCompare(T, X) TCCR##T##C = _BV(FOC##T##X)
#define DisableCompareInterrupt(T, X) TIMSK##T &= ~_BV(OCIE##T##X)
#define EnableCompareInterrupt(T, X) TIMSK##T |= _BV(OCIE##T##X)
#define ClearInterruptFlag(T, X) TIFR##T = _BV(OCF##T##X)
#define SetTimerCompareRelative(T, X, D) OCR##T##X = TCNT##T + D
#define InterruptFlagIsSet(T, X) ((TIFR##T & _BV(OCF##T##X)) != 0)

#define ConfigureTimer(T)                                                 \
  {                                                                       \
    /* Set WGMn3:0 to all zero => Normal mode */                          \
    TCCR##T##A &= ~(_BV(WGM##T##1) | _BV(WGM##T##0));                     \
    TCCR##T##B &= ~(_BV(WGM##T##3) | _BV(WGM##T##2));                     \
    /* Set prescaler to 1 */                                              \
    TCCR##T##B =                                                          \
        (TCCR##T##B & ~(_BV(CS##T##2) | _BV(CS##T##1) | _BV(CS##T##0))) | \
        _BV(CS##T##0);                                                    \
  }
#define EnableOverflowInterrupt(T) TIMSK##T |= _BV(TOIE##T)
#define DisableOverflowInterrupt(T) TIMSK##T &= ~_BV(TOIE##T)

// this is needed to give the background task isr access to engine
static FastAccelStepperEngine* fas_engine = NULL;

// Here are the global variables to interface with the interrupts
StepperQueue fas_queue[NUM_QUEUES];

#define AVR_INIT(T, CHANNEL)                       \
  {                                                \
    /* Disconnect stepper on next compare event */ \
    Stepper_Disconnect(T, CHANNEL);                \
    /* disable compare A interrupt */              \
    DisableCompareInterrupt(T, CHANNEL);           \
    /* force compare to ensure disconnect */       \
    ForceCompare(T, CHANNEL);                      \
    /* Initialize timer for correct time base */   \
    ConfigureTimer(T);                             \
    /* ensure cyclic interrupt is running */       \
    EnableOverflowInterrupt(T);                    \
  }
void StepperQueue::init(uint8_t queue_num, uint8_t step_pin) {
  prepareISRtimeMeasurement();
  _initVars();
  digitalWrite(step_pin, LOW);
  pinMode(step_pin, OUTPUT);
  if (step_pin == stepPinStepperA) {
    channel = channelA;
    AVR_INIT(FAS_TIMER_MODULE, A)
  }
  if (step_pin == stepPinStepperB) {
    channel = channelB;
    AVR_INIT(FAS_TIMER_MODULE, B)
  }
#ifdef stepPinStepperC
  if (step_pin == stepPinStepperC) {
    channel = channelC;
    AVR_INIT(FAS_TIMER_MODULE, C)
  }
#endif
}

// The interrupt is called on compare event, which eventually
// generates a L->H transition. In any case, the current command's
// wait time has still to be executed for the next command, if any.
//
// Remark: Interrupt Flag is automatically cleared on ISR execution
//
// If reaching here without further commands, then the queue is done
#define AVR_STEPPER_ISR(T, CHANNEL)                                            \
  ISR(TIMER##T##_COMP##CHANNEL##_vect) {                                       \
    enterStepperISR();                                                         \
    uint8_t rp = fas_queue_##CHANNEL.read_idx;                                 \
    uint8_t wp = fas_queue_##CHANNEL.next_write_idx;                           \
    if (rp == wp) {                                                            \
      /* queue is empty => set to disconnect */                                \
      /* disable compare interrupt */                                          \
      DisableCompareInterrupt(T, CHANNEL);                                     \
      Stepper_Disconnect(T, CHANNEL);                                          \
      /* force compare to ensure disconnect */                                 \
      ForceCompare(T, CHANNEL);                                                \
      fas_queue_##CHANNEL._isRunning = false;                                  \
      fas_queue_##CHANNEL._noMoreCommands = false;                             \
      exitStepperISR();                                                        \
      return;                                                                  \
    }                                                                          \
    struct queue_entry* e = &fas_queue_##CHANNEL.entry[rp & QUEUE_LEN_MASK];   \
    /* There is a risk, that this new compare time is delayed by one cycle */  \
    uint16_t ticks = e->ticks;                                                 \
    /* Set output to zero, this works in any case. In case of pause: no-op */  \
    Stepper_Zero(T, CHANNEL);                                                  \
    ForceCompare(T, CHANNEL);                                                  \
    if (e->steps > 1) {                                                        \
      /* perform another step with this queue entry */                         \
      e->steps--;                                                              \
      Stepper_One(T, CHANNEL);                                                 \
    } else {                                                                   \
      /* either pause command or no more steps */                              \
      if (fas_queue_##CHANNEL._noMoreCommands) {                               \
        /* new command received after running out of commands */               \
        /* if this new command requires a step, then this step would be lost   \
         */                                                                    \
        fas_queue_##CHANNEL._noMoreCommands = false;                           \
        if (e->steps != 0) {                                                   \
          /* New command needs steps, so do it immediately */                  \
          ticks = 10;                                                          \
        }                                                                      \
      } else if (TEST_NOT_REPEATING_ENTRY) {                                   \
        rp++;                                                                  \
        fas_queue_##CHANNEL.read_idx = rp;                                     \
        if (rp == wp) {                                                        \
          /* queue is empty, wait this command to complete, then disconnect */ \
          fas_queue_##CHANNEL._noMoreCommands = true;                          \
          OCR##T##CHANNEL += ticks;                                            \
          exitStepperISR();                                                    \
          return;                                                              \
        }                                                                      \
        e = &fas_queue_##CHANNEL.entry[rp & QUEUE_LEN_MASK];                   \
      }                                                                        \
      if (e->toggle_dir) {                                                     \
        Stepper_ToggleDirection(CHANNEL);                                      \
      }                                                                        \
      if (e->steps != 0) {                                                     \
        Stepper_One(T, CHANNEL);                                               \
      }                                                                        \
    }                                                                          \
    OCR##T##CHANNEL += ticks;                                                  \
    exitStepperISR();                                                          \
  }

#define AVR_STEPPER_ISR_GEN(T, CHANNEL) AVR_STEPPER_ISR(T, CHANNEL)
AVR_STEPPER_ISR_GEN(FAS_TIMER_MODULE, A)
AVR_STEPPER_ISR_GEN(FAS_TIMER_MODULE, B)
#ifdef stepPinStepperC
AVR_STEPPER_ISR_GEN(FAS_TIMER_MODULE, C)
#endif

// this is for cyclic task
#define AVR_CYCLIC_ISR(T)                          \
  ISR(TIMER##T##_OVF_vect) {                       \
    enterFillQueueISR();                           \
                                                   \
    /* disable OVF interrupt to avoid nesting */   \
    DisableOverflowInterrupt(T);                   \
                                                   \
    /* enable interrupts for nesting */            \
    sei();                                         \
                                                   \
    /* manage steppers */                          \
    fas_engine->manageSteppers();                  \
                                                   \
    /* disable interrupts for exist ISR routine */ \
    cli();                                         \
                                                   \
    /* enable OVF interrupt again */               \
    EnableOverflowInterrupt(T);                    \
                                                   \
    exitFillQueueISR();                            \
  }
#define AVR_CYCLIC_ISR_GEN(T) AVR_CYCLIC_ISR(T)
AVR_CYCLIC_ISR_GEN(FAS_TIMER_MODULE)

#define GET_ENTRY_PTR(T, CHANNEL)    \
  rp = fas_queue_##CHANNEL.read_idx; \
  e = &fas_queue_##CHANNEL.entry[rp & QUEUE_LEN_MASK];

#define AVR_START_QUEUE(T, CHANNEL)              \
  /* ensure no compare event */                  \
  SetTimerCompareRelative(T, CHANNEL, 32768);    \
  /* set output one, if steps to be generated */ \
  if (e->steps != 0) {                           \
    Stepper_One(T, CHANNEL);                     \
  } else {                                       \
    Stepper_Zero(T, CHANNEL);                    \
  }                                              \
  /* clear interrupt flag */                     \
  ClearInterruptFlag(T, CHANNEL);                \
  /* enable compare interrupt */                 \
  EnableCompareInterrupt(T, CHANNEL);            \
  /* start */                                    \
  noInterrupts();                                \
  SetTimerCompareRelative(T, CHANNEL, 10);       \
  interrupts();

void StepperQueue::startQueue() {
  uint8_t rp;
  struct queue_entry* e;

  _isRunning = true;
  switch (channel) {
    case channelA:
      GET_ENTRY_PTR(FAS_TIMER_MODULE, A)
      PREPARE_DIRECTION_PIN(A)
      AVR_START_QUEUE(FAS_TIMER_MODULE, A)
      break;
    case channelB:
      GET_ENTRY_PTR(FAS_TIMER_MODULE, B)
      PREPARE_DIRECTION_PIN(B)
      AVR_START_QUEUE(FAS_TIMER_MODULE, B)
      break;
#ifdef stepPinStepperC
    case channelC:
      GET_ENTRY_PTR(FAS_TIMER_MODULE, C)
      PREPARE_DIRECTION_PIN(C)
      AVR_START_QUEUE(FAS_TIMER_MODULE, C)
      break;
#endif
  }
}

#define FORCE_STOP(T, CHANNEL)               \
  {                                          \
    /* disable compare interrupt */          \
    DisableCompareInterrupt(T, CHANNEL);     \
    /* set to disconnect */                  \
    Stepper_Disconnect(T, CHANNEL);          \
    /* force compare to ensure disconnect */ \
    ForceCompare(T, CHANNEL);                \
  }
void StepperQueue::forceStop() {
  switch (channel) {
    case channelA:
      FORCE_STOP(FAS_TIMER_MODULE, A)
      break;
    case channelB:
      FORCE_STOP(FAS_TIMER_MODULE, B)
      break;
#ifdef stepPinStepperC
    case channelC:
      FORCE_STOP(FAS_TIMER_MODULE, C)
      break;
#endif
  }
  _isRunning = false;

  // empty the queue
  read_idx = next_write_idx;
}
void StepperQueue::connect() {}
void StepperQueue::disconnect() {}
bool StepperQueue::isValidStepPin(uint8_t step_pin) {
  if (step_pin == stepPinStepperA) {
    return true;
  }
  if (step_pin == stepPinStepperB) {
    return true;
  }
#ifdef stepPinStepperC
  if (step_pin == stepPinStepperC) {
    return true;
  }
#endif
  return false;
}

int8_t StepperQueue::queueNumForStepPin(uint8_t step_pin) {
  if (step_pin == stepPinStepperA) {
    return 0;
  }
  if (step_pin == stepPinStepperB) {
    return 1;
  }
#ifdef stepPinStepperC
  if (step_pin == stepPinStepperC) {
    return 2;
  }
#endif
  return -1;
}
void StepperQueue::adjustSpeedToStepperCount(uint8_t steppers) {
  //   commit 9577e9bfd4b9a6cf1ad830901c00c8b129a62aee fails
  //   test_sd_04_timing_2560 as timer 3 reaches 40us.
  //   This includes port set/clear for timer measurement.
  //   So choose 20kHz
  //
  // check if Issue_152.ino, the interrupt need 14us.
  // So 70000 Steps/s is too high.
  if (steppers == 1) {
    max_speed_in_ticks = TICKS_PER_S / 50000;
  } else if (steppers == 2) {
    max_speed_in_ticks = 426;
  } else {
    max_speed_in_ticks = TICKS_PER_S / 20000;
  }
}

void fas_init_engine(FastAccelStepperEngine* engine) { fas_engine = engine; }
#endif
