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
#define DisableCompareInterrupt(T, X) TIMSK##T &= ~_BV(OCIE1##X)
#define EnableCompareInterrupt(T, X) TIMSK##T |= _BV(OCIE1##X)
#define ClearInterruptFlag(T, X) TIFR##T = _BV(OCF1##X)
#define SetTimerCompareRelative(T, X, D) OCR##T####X = TCNT##T + D
// Here are the global variables to interface with the interrupts
StepperQueue fas_queue[NUM_QUEUES];

void StepperQueue::init(uint8_t queue_num, uint8_t step_pin) {
  _initVars();
  digitalWrite(step_pin, LOW);
  pinMode(step_pin, OUTPUT);
  if (step_pin == stepPinStepperA) {
    isChannelA = true;
    noInterrupts();
    Stepper_Disconnect(1, A);
    // force compare to ensure disconnect
    ForceCompare(1, A);
    // disable compare A interrupt
    DisableCompareInterrupt(1, A);
    interrupts();
  }
  if (step_pin == stepPinStepperB) {
    isChannelA = false;
    noInterrupts();
    Stepper_Disconnect(1, B);
    // force compare to ensure disconnect
    ForceCompare(1, B);
    // disable compare B interrupt
    DisableCompareInterrupt(1, B);
    interrupts();
  }
}

#define AVR_STEPPER_ISR(CHANNEL, queue, ocr, foc)                     \
  ISR(TIMER1_COMP##CHANNEL##_vect) {                                  \
    uint8_t rp = queue.read_idx;                                      \
    if (Stepper_IsToggling(1, CHANNEL)) {                             \
      /* Clear output bit by another toggle */                        \
      ForceCompare(1, CHANNEL);                                       \
      struct queue_entry* e = &queue.entry[rp & QUEUE_LEN_MASK];      \
      if (e->steps-- > 0) {                                           \
        /* perform another steps_dir with this queue entry */         \
        ocr += queue.ticks;                                           \
        return;                                                       \
      }                                                               \
      rp++;                                                           \
      queue.read_idx = rp;                                            \
    } else if (!Stepper_IsDisconnected(1, CHANNEL)) {                 \
      rp++;                                                           \
      queue.read_idx = rp;                                            \
    }                                                                 \
    if (rp == queue.next_write_idx) {                                 \
      /* queue is empty => set to disconnect */                       \
      Stepper_Disconnect(1, CHANNEL);                                 \
      /* disable compare interrupt */                                 \
      DisableCompareInterrupt(1, CHANNEL);                            \
      /* force compare to ensure disconnect */                        \
      ForceCompare(1, CHANNEL);                                       \
      queue.isRunning = false;                                        \
      queue.queue_end.ticks = TICKS_FOR_STOPPED_MOTOR;                \
      return;                                                         \
    }                                                                 \
    /* command in queue */                                            \
    struct queue_entry* e = &queue.entry[rp & QUEUE_LEN_MASK];        \
    ocr += (queue.ticks = e->ticks);                                  \
    /* assign to skip and test for not zero */                        \
    if (e->steps == 0) {                                              \
      Stepper_Zero(1, CHANNEL);                                       \
    } else {                                                          \
      Stepper_Toggle(1, CHANNEL);                                     \
    }                                                                 \
    if (e->toggle_dir) {                                              \
      uint8_t dirPin = queue.dirPin;                                  \
      digitalWrite(dirPin, digitalRead(dirPin) == HIGH ? LOW : HIGH); \
    }                                                                 \
  }
AVR_STEPPER_ISR(A, fas_queue_A, OCR1A, FOC1A)
AVR_STEPPER_ISR(B, fas_queue_B, OCR1B, FOC1B)

void StepperQueue::startQueue() {
  isRunning = true;
  if (isChannelA) {
    noInterrupts();
    TIFR1 = _BV(OCF1A);
    // clear interrupt flag
    ClearInterruptFlag(1, A);
    // enable compare A interrupt
    EnableCompareInterrupt(1, A);
    OCR1A = TCNT1 + 40;
    // definite start point
    SetTimerCompareRelative(1, A, 40);
    interrupts();
  } else {
    noInterrupts();
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
