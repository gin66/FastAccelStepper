#ifndef PD_PICO_QUEUE_H
#define PD_PICO_QUEUE_H

#include "FastAccelStepper.h"
#include "fas_queue/base.h"

class StepperQueue : public StepperQueueBase {
 public:
#include "../fas_queue/protocol.h"

  bool _isActive;
  uint8_t _step_pin;
  uint16_t adjust_80MHz;
  PIO pio;
  uint sm;
  int32_t pos_offset;

  inline void _pd_initVars() {
    _step_pin = PIN_UNDEFINED;
    _isActive = false;
    pos_offset = 0;
    max_speed_in_ticks = 80;
    prog_offset = 0;
    dir_gpio_valid = false;
    dir_gpio_high = false;
  }

  struct PioResources {
    PIO pio;
    uint sm;
    uint8_t offset;
  };
  static bool claim_pio_resources(FastAccelStepperEngine* engine,
                                  uint8_t step_pin, PioResources* out);

  bool isRunning() const;
  bool isReadyForCommands() const;
  void setupSM();
  int32_t getCurrentStepCount() const;
  void attachDirPinToStatemachine();
  void setDirPinState(bool high);
  // Program load address. Idle PC is this word (the blocking pull).
  uint8_t prog_offset;
  // Last level written to a GPIO direction pin, so the filler can tell a
  // change from a repeat without reading the pin from the ISR.
  bool dir_gpio_valid;
  bool dir_gpio_high;

  void setDirPin(uint8_t dir_pin, bool _dirHighCountsUp) {
    dirPin = dir_pin;
    dirHighCountsUp = _dirHighCountsUp;
    attachDirPinToStatemachine();
  }

 private:
  static bool isValidStepPin(uint8_t step_pin);
};

#define SET_DIRECTION_PIN_STATE(q, high) (q)->setDirPinState(high)

#define SET_ENABLE_PIN_STATE(q, pin, high) \
  digitalWrite((pin), (high) ? HIGH : LOW)

// Four old-direction pauses, then the caller enqueues the direction change.
// The pauses keep the old level. One command is in flight besides the four
// FIFO words, so the pin must not change when a pause is pushed: the command
// in flight can still be a step. The filler puts the following command into
// the FIFO and only then changes the pin. A full FIFO leaves the pin alone.
// Returns AQE_DIR_CHANGE_PAUSE_INJECTED when any pause was queued. The
// caller's command is not queued; moveTimed() returns that code and reports
// the pause ticks. The retry queues the direction change. AQE_QUEUE_FULL
// inserts nothing. A user dir-change delay is a further pause at the new
// level, in the same inject, ahead of the caller's later command.
inline AqeResultCode StepperQueue::addDirChangePauseToQueue(
    const struct stepper_command_s* cmd, bool start,
    uint16_t dir_change_delay_ticks) {
  uint8_t missing = 0;
  if (_nr_of_pauses < PICO_DIR_DRAIN_WORDS) {
    missing = (uint8_t)(PICO_DIR_DRAIN_WORDS - _nr_of_pauses);
  }
  bool add_delay = false;
  uint16_t delay_ticks = dir_change_delay_ticks;
  if (dir_change_delay_ticks != 0 &&
      !((cmd->steps == 0) && (cmd->ticks >= dir_change_delay_ticks))) {
    add_delay = true;
    if (cmd->steps == 0) {
      delay_ticks = (uint16_t)(dir_change_delay_ticks - cmd->ticks);
    }
    delay_ticks = (uint16_t)fas_max(delay_ticks, (uint16_t)MIN_CMD_TICKS);
  }
  uint8_t extra = (uint8_t)(missing + (add_delay ? 1 : 0));
  // Pauses plus the caller's command. A short queue inserts nothing.
  if (extra != 0 && (uint16_t)queueEntries() + extra + 1 > QUEUE_LEN) {
    return AQE_QUEUE_FULL;
  }
  uint32_t injected = 0;
  for (uint8_t i = 0; i < missing; i++) {
    struct stepper_command_s pause_cmd = {.ticks = (uint16_t)MIN_CMD_TICKS,
                                           .steps = 0,
                                           .count_up = queue_end.count_up};
    AqeResultCode res = addQueueEntry(&pause_cmd, false);
    if (res != AQE_OK) {
      _injected_pause_ticks = injected;
      return res;
    }
    injected += pause_cmd.ticks;
  }
  if (add_delay) {
    struct stepper_command_s pause_cmd = {
        .ticks = delay_ticks, .steps = 0, .count_up = cmd->count_up};
    AqeResultCode res = addQueueEntry(&pause_cmd, start);
    if (res != AQE_OK) {
      _injected_pause_ticks = injected;
      return res;
    }
    injected += pause_cmd.ticks;
  }
  if (injected == 0) {
    return AQE_OK;
  }
  _injected_pause_ticks = injected;
  return AQE_DIR_CHANGE_PAUSE_INJECTED;
}

#endif  // PD_PICO_QUEUE_H
