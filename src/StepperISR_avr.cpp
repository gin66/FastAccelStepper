#include "FastAccelStepper.h"
#include "StepperISR.h"
#include "AVRStepperPins.h"

#if defined(ARDUINO_ARCH_AVR)
#if defined(__AVR_ATmega328P__)
#define FAS_TIMER_MODULE 1
#elif defined(__AVR_ATmega2560__)
#ifndef FAS_TIMER_MODULE
#define FAS_TIMER_MODULE 4
#endif
#endif
#endif

#if (FAS_TIMER_MODULE == 1)
#define stepPinStepperA stepPinStepper1A
#define stepPinStepperB stepPinStepper1B
#if defined(__AVR_ATmega2560__)
#define stepPinStepperC stepPinStepper1C
#endif
#elif (FAS_TIMER_MODULE == 3)
#define stepPinStepperA stepPinStepper3A
#define stepPinStepperB stepPinStepper3B
#if defined(__AVR_ATmega2560__)
#define stepPinStepperC stepPinStepper3C
#endif
#elif (FAS_TIMER_MODULE == 4)
#define stepPinStepperA stepPinStepper4A
#define stepPinStepperB stepPinStepper4B
#if defined(__AVR_ATmega2560__)
#define stepPinStepperC stepPinStepper4C
#endif
#elif (FAS_TIMER_MODULE == 5)
#define stepPinStepperA stepPinStepper5A
#define stepPinStepperB stepPinStepper5B
#if defined(__AVR_ATmega2560__)
#define stepPinStepperC stepPinStepper5C
#endif
#endif

// T is the timer module number 0,1,2,3...
// X is the Channel name A or B or C
#define Stepper_Toggle(T, X) \
  TCCR##T##A = (TCCR##T##A | _BV(COM##T##X##0)) & ~_BV(COM##T##X##1)
#define Stepper_Zero(T, X) \
  TCCR##T##A = (TCCR##T##A | _BV(COM##T##X##1)) & ~_BV(COM##T##X##0)
#define Stepper_Disconnect(T, X) \
  TCCR##T##A = (TCCR##T##A & ~(_BV(COM##T##X##1) | _BV(COM##T##X##0)))
#define Stepper_IsToggling(T, X) \
  ((TCCR##T##A & (_BV(COM##T##X##0) | _BV(COM##T##X##1))) == _BV(COM##T##X##0))
#define Stepper_IsDisconnected(T, X) \
  ((TCCR##T##A & (_BV(COM##T##X##0) | _BV(COM##T##X##1))) == 0)

#define ForceCompare(T, X) TCCR##T##C = _BV(FOC##T##X)
#define DisableCompareInterrupt(T, X) TIMSK##T &= ~_BV(OCIE##T##X)
#define EnableCompareInterrupt(T, X) TIMSK##T |= _BV(OCIE##T##X)
#define ClearInterruptFlag(T, X) TIFR##T = _BV(OCF##T##X)
#define SetTimerCompareRelative(T, X, D) OCR##T##X = TCNT##T + D

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

#define AVR_STEPPER_ISR(T, CHANNEL)                                          \
  ISR(TIMER##T##_COMP##CHANNEL##_vect) {                                     \
    uint8_t rp = fas_queue_##CHANNEL.read_idx;                               \
    if (rp == fas_queue_##CHANNEL.next_write_idx) {                          \
      /* queue is empty => set to disconnect */                              \
      Stepper_Disconnect(T, CHANNEL);                                        \
      /* force compare to ensure disconnect */                               \
      ForceCompare(T, CHANNEL);                                              \
      /* disable compare interrupt */                                        \
      DisableCompareInterrupt(T, CHANNEL);                                   \
      fas_queue_##CHANNEL._isRunning = false;                                \
      fas_queue_##CHANNEL.queue_end.ticks = TICKS_FOR_STOPPED_MOTOR;         \
      return;                                                                \
    }                                                                        \
    struct queue_entry* e = &fas_queue_##CHANNEL.entry[rp & QUEUE_LEN_MASK]; \
    OCR##T##CHANNEL += e->ticks;                                             \
    if (Stepper_IsToggling(T, CHANNEL)) {                                    \
      /* Clear output bit by another toggle */                               \
      ForceCompare(T, CHANNEL);                                              \
      if (e->steps-- > 1) {                                                  \
        /* perform another step with this queue entry */                     \
        return;                                                              \
      }                                                                      \
    }                                                                        \
    rp++;                                                                    \
    fas_queue_##CHANNEL.read_idx = rp;                                       \
    if (rp != fas_queue_##CHANNEL.next_write_idx) {                          \
      /* command in queue */                                                 \
      e = &fas_queue_##CHANNEL.entry[rp & QUEUE_LEN_MASK];                   \
      if (e->steps == 0) {                                                   \
        Stepper_Zero(T, CHANNEL);                                            \
      } else {                                                               \
        Stepper_Toggle(T, CHANNEL);                                          \
      }                                                                      \
      if (e->toggle_dir) {                                                   \
        uint8_t dirPin = fas_queue_##CHANNEL.dirPin;                         \
        digitalWrite(dirPin, digitalRead(dirPin) == HIGH ? LOW : HIGH);      \
      }                                                                      \
    } else {                                                                 \
      Stepper_Zero(T, CHANNEL);                                              \
    }                                                                        \
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
    /* disable OVF interrupt to avoid nesting */   \
    DisableOverflowInterrupt(T);                   \
                                                   \
    /* enable interrupts for nesting */            \
    interrupts();                                  \
                                                   \
    /* manage steppers */                          \
    fas_engine->manageSteppers();                  \
                                                   \
    /* disable interrupts for exist ISR routine */ \
    noInterrupts();                                \
                                                   \
    /* enable OVF interrupt again */               \
    EnableOverflowInterrupt(T);                    \
  }
#define AVR_CYCLIC_ISR_GEN(T) AVR_CYCLIC_ISR(T)
AVR_CYCLIC_ISR_GEN(FAS_TIMER_MODULE)

#define AVR_START_QUEUE(T, CHANNEL)                                          \
  {                                                                          \
    /* force compare to ensure zero */                                       \
    Stepper_Zero(T, CHANNEL);                                                \
    ForceCompare(T, CHANNEL);                                                \
    /* enter critical section */                                             \
    noInterrupts();                                                          \
    /* ensure no compare event */                                            \
    SetTimerCompareRelative(T, CHANNEL, 32768);                              \
    /* set toggle mode, if steps to be generated */                          \
    uint8_t rp = fas_queue_##CHANNEL.read_idx;                               \
    struct queue_entry* e = &fas_queue_##CHANNEL.entry[rp & QUEUE_LEN_MASK]; \
    if (e->steps > 0) {                                                      \
      Stepper_Toggle(T, CHANNEL);                                            \
    } else {                                                                 \
      Stepper_Zero(T, CHANNEL);                                              \
    }                                                                        \
    if (e->toggle_dir) {                                                     \
      uint8_t dirPin = fas_queue_##CHANNEL.dirPin;                           \
      digitalWrite(dirPin, digitalRead(dirPin) == HIGH ? LOW : HIGH);        \
    }                                                                        \
    /* clear interrupt flag */                                               \
    ClearInterruptFlag(T, CHANNEL);                                          \
    /* enable compare interrupt */                                           \
    EnableCompareInterrupt(T, CHANNEL);                                      \
    /* definite start point */                                               \
    SetTimerCompareRelative(T, CHANNEL, 10);                                 \
    interrupts();                                                            \
  }

void StepperQueue::startQueue() {
  if (_isRunning) {
    return;
  }
  _isRunning = true;
  switch (channel) {
    case channelA:
      AVR_START_QUEUE(FAS_TIMER_MODULE, A)
      break;
    case channelB:
      AVR_START_QUEUE(FAS_TIMER_MODULE, B)
      break;
#ifdef stepPinStepperC
    case channelC:
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
  queue_end.ticks = TICKS_FOR_STOPPED_MOTOR;

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
