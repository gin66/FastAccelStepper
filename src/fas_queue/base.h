#ifndef FAS_QUEUE_BASE_H
#define FAS_QUEUE_BASE_H

#include "fas_arch/common.h"

// QUEUE_LEN_MASK is used in inline methods below
#define QUEUE_LEN_MASK (QUEUE_LEN - 1)

// Queue entry: a single low-level stepper command.
// steps == 0 means pure delay (no step pulses).
struct queue_entry {
  uint8_t steps;  // if 0, then the command only adds a delay
  uint8_t toggle_dir : 1;
  uint8_t countUp : 1;
  uint8_t moreThanOneStep : 1;
  uint8_t hasSteps : 1;
  uint8_t dirPinState : 1;
  uint16_t ticks;
#if defined(SUPPORT_QUEUE_ENTRY_END_POS_U16)
  uint16_t end_pos_last16;
#endif
#if defined(SUPPORT_QUEUE_ENTRY_START_POS_U16)
  uint16_t start_pos_last16;
#endif
};

// Architecture-independent stepper queue state.
//
// Each architecture defines `class StepperQueue : public StepperQueueBase`
// in its own pd_*/ directory, adding hardware-specific fields and methods.
// Common methods (addQueueEntry, getCurrentPosition, etc.) are defined in
// StepperISR.cpp and declared here so the compiler sees them on all arches.
class StepperQueueBase {
 public:
  struct queue_entry entry[QUEUE_LEN];

  // Commands are suspended during forceStopAndNewPosition()
  volatile bool ignore_commands;
  volatile uint8_t read_idx;  // ISR stops if read_idx == next_write_idx
  volatile uint8_t next_write_idx;
  bool dirHighCountsUp;
  uint8_t dirPin;

#if defined(SUPPORT_DIR_PIN_MASK)
  volatile SUPPORT_DIR_PIN_MASK* _dirPinPort;
  SUPPORT_DIR_PIN_MASK _dirPinMask;
#endif

  struct queue_end_s queue_end;

#ifdef TEST
  uint16_t max_speed_in_ticks = TICKS_PER_S / 50000;  // default: 50_000 steps/s
#else
  uint16_t max_speed_in_ticks = TICKS_PER_S / 1000;  // default: 1_000 steps/s
#endif
  uint16_t _last_command_ticks;

  inline uint8_t queueEntries() const {
    fasDisableInterrupts();
    uint8_t rp = read_idx;
    uint8_t wp = next_write_idx;
    fasEnableInterrupts();
    inject_fill_interrupt(0);
    return (uint8_t)(wp - rp);
  }
  inline bool isQueueFull() const { return queueEntries() == QUEUE_LEN; }
  inline bool isQueueEmpty() const { return queueEntries() == 0; }
  inline bool hasStepsInQueue() const {
    fasDisableInterrupts();
    uint8_t rp = read_idx;
    uint8_t wp = next_write_idx;
    fasEnableInterrupts();
    while (rp != wp) {
      if (entry[rp & QUEUE_LEN_MASK].steps > 0) {
        return true;
      }
      rp++;
    }
    return false;
  }

  inline uint16_t getMaxSpeedInTicks() const { return max_speed_in_ticks; }

#if defined(SUPPORT_UNSAFE_ABS_SPEED_LIMIT_SETTING)
  void setAbsoluteSpeedLimit(uint16_t ticks) { max_speed_in_ticks = ticks; }
#endif
};

#endif  // FAS_QUEUE_BASE_H
