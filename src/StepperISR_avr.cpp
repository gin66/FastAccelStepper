#include "StepperISR.h"
#include "FastAccelStepper.h"

#if defined(ARDUINO_ARCH_AVR)

#define Stepper_Toggle(X) \
  TCCR1##X = (TCCR1##X | _BV(COM1##X##0)) & ~_BV(COM1##X##1)
#define Stepper_Zero(X) \
  TCCR1##X = (TCCR1##X | _BV(COM1##X##1)) & ~_BV(COM1##X##0)
#define Stepper_Disconnect(X) \
  TCCR1##X = (TCCR1##X & ~(_BV(COM1##X##1) | _BV(COM1##X##0)))
#define Stepper_IsToggling(X) \
  ((TCCR1##X & (_BV(COM1##X##0) | _BV(COM1##X##1))) == _BV(COM1##X##0))

// Here are the global variables to interface with the interrupts
StepperQueue fas_queue[NUM_QUEUES];

void StepperQueue::init(uint8_t step_pin) {
	_initVars();
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
}

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

int StepperQueue::addQueueEntry(uint32_t start_delta_ticks, uint8_t steps,
                                    bool dir_high, int16_t change_ticks) {
  int32_t c_sum = 0;
  if (steps >= 128) {
    return AQE_STEPS_ERROR;
  }
  if (start_delta_ticks > ABSOLUTE_MAX_TICKS) {
    return AQE_TOO_HIGH;
  }
  if ((change_ticks != 0) && (steps > 1)) {
    c_sum = change_ticks * (steps - 1);
  }
  if (change_ticks > 0) {
    if (c_sum > 32768) {
      return AQE_CHANGE_TOO_HIGH;
    }
  } else if (change_ticks < 0) {
    if (c_sum < -32768) {
      return AQE_CHANGE_TOO_LOW;
    }
    if (start_delta_ticks + c_sum < MIN_DELTA_TICKS) {
      return AQE_CUMULATED_CHANGE_TOO_LOW;
    }
  }

  uint16_t msb = start_delta_ticks >> 14;
  uint16_t lsw;
  if (msb > 1) {
    msb--;
    lsw = start_delta_ticks & 0x3fff;
    lsw |= 0x4000;
  } else {
    msb = 0;
    lsw = start_delta_ticks;
  }

  uint8_t wp = next_write_ptr;
  uint8_t rp = read_ptr;
  struct queue_entry* e = &entry[wp];

  uint8_t next_wp = (wp + 1) & QUEUE_LEN_MASK;
  if (next_wp != rp) {
    pos_at_queue_end += dir_high ? steps : -steps;
    ticks_at_queue_end = change_ticks * (steps - 1) + start_delta_ticks;
    steps <<= 1;
    e->delta_msb = msb;
    e->delta_lsw = lsw;
    e->delta_change = change_ticks;
    e->steps = (dir_high != dir_high_at_queue_end) ? steps | 0x01 : steps;
    dir_high_at_queue_end = dir_high;
#if (TEST_CREATE_QUEUE_CHECKSUM == 1)
    {
      unsigned char* x = (unsigned char*)e;
      for (uint8_t i = 0; i < sizeof(struct queue_entry); i++) {
        if (checksum & 0x80) {
          checksum <<= 1;
          checksum ^= 0xde;
        } else {
          checksum <<= 1;
        }
        checksum ^= *x++;
      }
    }
#endif
    next_write_ptr = next_wp;
    return AQE_OK;
  }
  return AQE_FULL;
}
#endif
