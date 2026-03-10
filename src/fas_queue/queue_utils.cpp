#include <stdint.h>

#include "fas_queue/stepper_queue.h"

uint32_t StepperQueue::ticksInQueue() const {
  fasDisableInterrupts();
  uint8_t rp = read_idx;
  uint8_t wp = next_write_idx;
  fasEnableInterrupts();
  if (wp == rp) {
    return 0;
  }
  uint32_t ticks = 0;
  rp++;  // ignore currently processed entry
  while (wp != rp) {
    const struct queue_entry* e = &entry[rp++ & QUEUE_LEN_MASK];
    ticks += e->ticks;
    uint8_t steps = e->steps;
    if (steps > 1) {
      uint32_t tmp = e->ticks;
      tmp *= steps - 1;
      ticks += tmp;
    }
  }
  return ticks;
}

bool StepperQueue::hasTicksInQueue(uint32_t min_ticks) const {
  fasDisableInterrupts();
  uint8_t rp = read_idx;
  uint8_t wp = next_write_idx;
  fasEnableInterrupts();
  if (wp == rp) {
    return false;
  }
  rp++;  // ignore currently processed entry
  while (wp != rp) {
    const struct queue_entry* e = &entry[rp & QUEUE_LEN_MASK];
    uint32_t tmp = e->ticks;
    uint8_t steps = fas_max(e->steps, (uint8_t)1);
    tmp *= steps;
    if (tmp >= min_ticks) {
      return true;
    }
    min_ticks -= tmp;
    rp++;
  }
  return false;
}

bool StepperQueue::getActualTicksWithDirection(struct actual_ticks_s* speed) const {
  // Retrieve current step rate from the current command.
  // This is valid only, if the command describes more than one step,
  // or if the next command contains one step, too.
  fasDisableInterrupts();
  uint8_t rp = read_idx;
  uint8_t wp = next_write_idx;
  fasEnableInterrupts();
  if (wp == rp) {
    speed->ticks = 0;
    return true;
  }
  const struct queue_entry* e = &entry[rp & QUEUE_LEN_MASK];
  if (e->hasSteps) {
    speed->count_up = e->countUp;
    speed->ticks = e->ticks;
    if (e->moreThanOneStep) {
      return true;
    }
    if (wp != ++rp) {
      if (entry[rp & QUEUE_LEN_MASK].hasSteps) {
        return true;
      }
    }
  }
  return false;
}
