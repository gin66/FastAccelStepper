#include "StepperISR.h"

#ifdef SUPPORT_ESP32_RMT

#ifdef __ESP32_IDF_V44__
#include <driver/periph_ctrl.h>
#include <soc/periph_defs.h>
#endif /* __ESP32_IDF_V44__ */

//#define TEST_PROBE 18

static void IRAM_ATTR prepare_for_next_command(
    StepperQueue *queue, const struct queue_entry *e_next) {
}

static void IRAM_ATTR apply_command(StepperQueue *queue,
                                    const struct queue_entry *e) {
}

static void IRAM_ATTR init_stop(StepperQueue *q) {
  q->_isRunning = false;
}

static void IRAM_ATTR what_is_next(StepperQueue *q) {
  bool isPrepared = q->_nextCommandIsPrepared;
  q->_nextCommandIsPrepared = false;
  uint8_t rp = q->read_idx;
  if (rp != q->next_write_idx) {
    rp++;
    q->read_idx = rp;
    if (rp != q->next_write_idx) {
      struct queue_entry *e_curr = &q->entry[rp & QUEUE_LEN_MASK];
      if (!isPrepared) {
        prepare_for_next_command(q, e_curr);
      }
      apply_command(q, e_curr);
      rp++;
      if (rp != q->next_write_idx) {
        struct queue_entry *e_next = &q->entry[rp & QUEUE_LEN_MASK];
        q->_nextCommandIsPrepared = true;
        prepare_for_next_command(q, e_next);
      }
      return;
    }
  }
  // no more commands: stop timer at period end
  init_stop(q);
}

void StepperQueue::init_rmt(uint8_t queue_num, uint8_t step_pin) {
#ifdef TEST_PROBE
  pinMode(TEST_PROBE, OUTPUT);
#endif

  _initVars();
  _step_pin = step_pin;

    // 160 MHz/5 = 32 MHz => 16 MHz in up/down-mode

  digitalWrite(step_pin, LOW);
  pinMode(step_pin, OUTPUT);

  connect();
}

void StepperQueue::connect_rmt() {
}

void StepperQueue::disconnect_rmt() {
}

void StepperQueue::startQueue_rmt() {
#ifdef TEST_PROBE
  // The time used by this command can have an impact
  digitalWrite(TEST_PROBE, digitalRead(TEST_PROBE) == HIGH ? LOW : HIGH);
#endif

  _isRunning = true;
  _nextCommandIsPrepared = false;
  struct queue_entry *e = &entry[read_idx & QUEUE_LEN_MASK];
  apply_command(this, e);

}
void StepperQueue::forceStop_rmt() {
  init_stop(this);
  read_idx = next_write_idx;
}
uint16_t StepperQueue::_getPerformedPulses_rmt() {
  return 0;
}

#endif
