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

  inline void _pd_initVars() { adjust_80MHz = 0; }

  bool isRunning();
  bool isReadyForCommands();
  bool claim_pio_sm(FastAccelStepperEngine* engine);
  void setupSM();
  int32_t getCurrentStepCount();
  void attachDirPinToStatemachine();
  void setDirPinState(bool high);

  void setDirPin(uint8_t dir_pin, bool _dirHighCountsUp) {
    dirPin = dir_pin;
    dirHighCountsUp = _dirHighCountsUp;
    attachDirPinToStatemachine();
  }
};

#define SET_DIRECTION_PIN_STATE(q, high) (q)->setDirPinState(high)

#define SET_ENABLE_PIN_STATE(q, pin, high) \
  digitalWrite((pin), (high) ? HIGH : LOW)

#endif
