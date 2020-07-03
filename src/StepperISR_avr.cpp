#include "StepperISR.h"

#if defined(ARDUINO_ARCH_AVR)
#define Stepper_Toggle(X) \
  TCCR1##X = (TCCR1##X | _BV(COM1##X##0)) & ~_BV(COM1##X##1)
#define Stepper_Zero(X) \
  TCCR1##X = (TCCR1##X | _BV(COM1##X##1)) & ~_BV(COM1##X##0)
#define Stepper_Disconnect(X) \
  TCCR1##X = (TCCR1##X & ~(_BV(COM1##X##1) | _BV(COM1##X##0)))
#define Stepper_IsToggling(X) \
  ((TCCR1##X & (_BV(COM1##X##0) | _BV(COM1##X##1))) == _BV(COM1##X##0))
#endif

// Here are the global variables to interface with the interrupts
StepperQueue fas_queue[NUM_QUEUES];

void StepperQueue::init(uint8_t step_pin) {
  dirPin = 255;
  autoEnablePin = 255;
  read_ptr = 0;
  next_write_ptr = 0;
#if defined(ARDUINO_ARCH_AVR)
  skip = 0;
  digitalWrite(step_pin, LOW);
  pinMode(step_pin, OUTPUT);
  if (step_pin == stepPinStepperA) {
    noInterrupts();
    OCR1A = 32768;  // definite start point
    Stepper_Disconnect(A);
    TCCR1C = _BV(FOC1A);    // force compare to ensure disconnect
    TIFR1 = _BV(OCF1A);     // clear interrupt flag
    TIMSK1 |= _BV(OCIE1A);  // enable compare A interrupt
    interrupts();
  }
  if (step_pin == stepPinStepperB) {
    noInterrupts();
    OCR1B = 32768;  // definite start point
    Stepper_Disconnect(B);
    TCCR1C = _BV(FOC1B);    // force compare to ensure disconnect
    TIFR1 = _BV(OCF1B);     // clear interrupt flag
    TIMSK1 |= _BV(OCIE1B);  // enable compare B interrupt
    interrupts();
  }
#endif
}

#if defined(ARDUINO_ARCH_AVR)
#define AVR_STEPPER_ISR(CHANNEL, queue, ocr, foc)                          \
  ISR(TIMER1_COMP##CHANNEL##_vect) {                                       \
    if (queue.skip) {                                                      \
      if ((--queue.skip) == 0) {                                           \
        Stepper_Toggle(CHANNEL);                                           \
      }                                                                    \
      ocr += 16384;                                                        \
      return;                                                              \
    } else if (Stepper_IsToggling(CHANNEL)) {                              \
      TCCR1C = _BV(foc); /* clear bit */                                   \
      uint8_t rp = queue.read_ptr;                                         \
      struct queue_entry* e = &queue.entry[rp];                            \
      if ((e->steps -= 2) > 1) {                                           \
        /* perform another step with this queue entry */                   \
        ocr += (e->delta_lsw += e->delta_change);                          \
        if (queue.skip =                                                   \
                e->delta_msb) { /* assign to skip and test for not zero */ \
          Stepper_Zero(CHANNEL);                                           \
        }                                                                  \
        return;                                                            \
      }                                                                    \
      rp = (rp + 1) & QUEUE_LEN_MASK;                                      \
      queue.read_ptr = rp;                                                 \
      if (rp == queue.next_write_ptr) {                                    \
        /* queue is empty => set to disconnect */                          \
        Stepper_Disconnect(CHANNEL);                                       \
        if (queue.autoEnablePin != 255) {                                  \
          digitalWrite(queue.autoEnablePin, HIGH);                         \
        }                                                                  \
        /* Next Interrupt takes place at next timer cycle => ~4ms */       \
        return;                                                            \
      }                                                                    \
    } else {                                                               \
      /* If reach here, then stepper is idle and waiting for a command */  \
      uint8_t rp = queue.read_ptr;                                         \
      if (rp == queue.next_write_ptr) {                                    \
        /* Next Interrupt takes place at next timer cycle => ~4ms */       \
        return;                                                            \
      }                                                                    \
    }                                                                      \
    /* command in queue */                                                 \
    struct queue_entry* e = &queue.entry[queue.read_ptr];                  \
    ocr += e->delta_lsw;                                                   \
    if (queue.skip =                                                       \
            e->delta_msb) { /* assign to skip and test for not zero */     \
      Stepper_Zero(CHANNEL);                                               \
    } else {                                                               \
      Stepper_Toggle(CHANNEL);                                             \
    }                                                                      \
    uint8_t steps = e->steps;                                              \
    if ((steps & 0x01) != 0) {                                             \
      digitalWrite(queue.dirPin,                                           \
                   digitalRead(queue.dirPin) == HIGH ? LOW : HIGH);        \
    }                                                                      \
    if (queue.autoEnablePin != 255) {                                      \
      digitalWrite(queue.autoEnablePin, LOW);                              \
    }                                                                      \
  }
AVR_STEPPER_ISR(A, fas_queue_A, OCR1A, FOC1A)
AVR_STEPPER_ISR(B, fas_queue_B, OCR1B, FOC1B)
#endif
