#include "FastAccelStepper.h"
#include "StepperISR.h"

#if defined(ARDUINO_ARCH_AVR)
#if !(defined(ARDUINO_AVR_NANO) || defined(ARDUINO_AVR_ATmega2560))
#error "Unsupported board"
#endif
#if !(defined(__AVR_ATmega328P__) || defined(__AVR_ATmega2560__))
#error "Unsupported AVR derivate"
#endif
#endif

// The ATmega328P has one 16 bit timer: Timer 1
// The ATmega2560 has four 16 bit timers: Timer 1, 3, 4 and 5

#if defined(ARDUINO_ARCH_AVR)
// T is the timer module number 0,1,2,3...
// X is the Channel name A or B
#define Stepper_Toggle(T, X) \
  TCCR##T##A = (TCCR##T##A | _BV(COM1##X##0)) & ~_BV(COM1##X##1)
#define Stepper_Zero(T, X) \
  TCCR##T##A = (TCCR##T##A | _BV(COM1##X##1)) & ~_BV(COM1##X##0)
#define Stepper_Disconnect(T, X) \
  TCCR##T##A = (TCCR##T##A & ~(_BV(COM1##X##1) | _BV(COM1##X##0)))
#define Stepper_IsToggling(T, X) \
  ((TCCR##T##A & (_BV(COM1##X##0) | _BV(COM1##X##1))) == _BV(COM1##X##0))
#define Stepper_IsDisconnected(T, X) \
  ((TCCR##T##A & (_BV(COM1##X##0) | _BV(COM1##X##1))) == 0)

#define ForceCompare(T, X) TCCR##T##C = _BV(FOC1##X)
#define DisableCompareInterrupt(T, X) TIMSK##T &= ~_BV(OCIE##T####X)
#define EnableCompareInterrupt(T, X) TIMSK##T |= _BV(OCIE##T####X)
#define ClearInterruptFlag(T, X) TIFR##T = _BV(OCF##T####X)
#define SetTimerCompareRelative(T, X, D) OCR##T####X = TCNT##T + D

#define ConfigureTimer(T)                                                 \
  {                                                                       \
    /* Set WGM13:0 to all zero => Normal mode */                          \
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

void StepperQueue::init(uint8_t queue_num, uint8_t step_pin) {
  _initVars();
  digitalWrite(step_pin, LOW);
  pinMode(step_pin, OUTPUT);
  if (step_pin == stepPinStepperA) {
    isChannelA = true;
    // Disconnect stepper on next compare event
    Stepper_Disconnect(1, A);
    // disable compare A interrupt
    DisableCompareInterrupt(1, A);
    // force compare to ensure disconnect
    ForceCompare(1, A);
  }
  if (step_pin == stepPinStepperB) {
    isChannelA = false;
    // Disconnect stepper on next compare event
    Stepper_Disconnect(1, B);
    // disable compare B interrupt
    DisableCompareInterrupt(1, B);
    // force compare to ensure disconnect
    ForceCompare(1, B);
  }
}

#define AVR_STEPPER_ISR(T, CHANNEL, queue, ocr, foc)                  \
  ISR(TIMER##T##_COMP##CHANNEL##_vect) {                              \
    uint8_t rp = queue.read_idx;                                      \
    if (Stepper_IsToggling(T, CHANNEL)) {                             \
      /* Clear output bit by another toggle */                        \
      ForceCompare(T, CHANNEL);                                       \
      struct queue_entry* e = &queue.entry[rp & QUEUE_LEN_MASK];      \
      if (e->steps-- > 0) {                                           \
        /* perform another steps_dir with this queue entry */         \
        ocr += queue.ticks;                                           \
        return;                                                       \
      }                                                               \
      rp++;                                                           \
      queue.read_idx = rp;                                            \
    } else if (!Stepper_IsDisconnected(T, CHANNEL)) {                 \
      rp++;                                                           \
      queue.read_idx = rp;                                            \
    }                                                                 \
    if (rp == queue.next_write_idx) {                                 \
      /* queue is empty => set to disconnect */                       \
      Stepper_Disconnect(T, CHANNEL);                                 \
      /* disable compare interrupt */                                 \
      DisableCompareInterrupt(T, CHANNEL);                            \
      /* force compare to ensure disconnect */                        \
      ForceCompare(T, CHANNEL);                                       \
      queue.isRunning = false;                                        \
      queue.queue_end.ticks = TICKS_FOR_STOPPED_MOTOR;                \
      return;                                                         \
    }                                                                 \
    /* command in queue */                                            \
    struct queue_entry* e = &queue.entry[rp & QUEUE_LEN_MASK];        \
    ocr += (queue.ticks = e->ticks);                                  \
    /* assign to skip and test for not zero */                        \
    if (e->steps == 0) {                                              \
      Stepper_Zero(T, CHANNEL);                                       \
    } else {                                                          \
      Stepper_Toggle(T, CHANNEL);                                     \
    }                                                                 \
    if (e->toggle_dir) {                                              \
      uint8_t dirPin = queue.dirPin;                                  \
      digitalWrite(dirPin, digitalRead(dirPin) == HIGH ? LOW : HIGH); \
    }                                                                 \
  }
AVR_STEPPER_ISR(1, A, fas_queue_A, OCR1A, FOC1A)
AVR_STEPPER_ISR(1, B, fas_queue_B, OCR1B, FOC1B)

// this is for cyclic task
#define AVR_STEPPER_ISR(T)                         \
  ISR(TIMER##T##_OVF_vect) {                       \
    /* disable OVF interrupt to avoid nesting */   \
    DisableCompareInterrupt(T);                    \
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
AVR_CYCLIC_ISR(1)

void StepperQueue::startQueue() {
  isRunning = true;
  if (isChannelA) {
    noInterrupts();
    // Initialize timer for correct time base
    ConfigureTimer(1);
    // clear interrupt flag
    ClearInterruptFlag(1, A);
    // enable compare A interrupt
    EnableCompareInterrupt(1, A);
    // definite start point
    SetTimerCompareRelative(1, A, 40);
    interrupts();
  } else {
    noInterrupts();
    // Initialize timer for correct time base
    ConfigureTimer(1);
    // clear interrupt flag
    ClearInterruptFlag(1, B);
    // enable compare B interrupt
    EnableCompareInterrupt(1, B);
    // definite start point
    SetTimerCompareRelative(1, B, 40);
    interrupts();
  }
}
void StepperQueue::forceStop() {
  if (isChannelA) {
    /* disable compare interrupt */
    DisableCompareInterrupt(1, A);
    /* set to disconnect */
    Stepper_Disconnect(1, A);
    /* force compare to ensure disconnect */
    ForceCompare(1, A);
  } else {
    /* disable compare interrupt */
    DisableCompareInterrupt(1, B);
    /* set to disconnect */
    Stepper_Disconnect(1, B);
    /* force compare to ensure disconnect */
    ForceCompare(1, B);
  }
  isRunning = false;
  queue_end.ticks = TICKS_FOR_STOPPED_MOTOR;

  // empty the queue
  read_idx = next_write_idx;
}
void StepperQueue::connect() {}
void StepperQueue::disconnect() {}

#endif
