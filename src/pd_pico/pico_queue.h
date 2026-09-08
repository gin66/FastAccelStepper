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
  }

  struct PioResources {
    PIO pio;
    uint sm;
  };
  static bool claim_pio_resources(FastAccelStepperEngine* engine,
                                  uint8_t step_pin, PioResources* out);
  static uint8_t s_claimed_pios;
  static PIO s_pio[NUM_PIOS];

  bool isRunning() const;
  bool isReadyForCommands() const;
  void setupSM();
  int32_t getCurrentStepCount() const;
  void attachDirPinToStatemachine();
  void setDirPinState(bool high);

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

inline AqeResultCode StepperQueue::addDirChangePauseToQueue(
    const struct stepper_command_s* cmd, bool start,
    uint16_t dir_change_delay_ticks) {
  if (dir_change_delay_ticks == 0) {
    return AQE_OK;
  }
  if ((cmd->steps == 0) && (cmd->ticks >= dir_change_delay_ticks)) {
    return AQE_OK;
  }
  uint16_t pause_ticks = dir_change_delay_ticks;
  if (cmd->steps == 0) {
    pause_ticks = dir_change_delay_ticks - cmd->ticks;
  }
  struct stepper_command_s pause_cmd = {
      .ticks = (uint16_t)fas_max(pause_ticks, MIN_CMD_TICKS),
      .steps = 0,
      .count_up = cmd->count_up};
  AqeResultCode res = addQueueEntry(&pause_cmd, start);
  if (res != AQE_OK) {
    return res;
  }
  return AQE_DIR_CHANGE_PAUSE_INJECTED;
}

#endif  // PD_PICO_QUEUE_H
