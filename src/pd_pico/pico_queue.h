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

  bool isRunning();
  bool isReadyForCommands();
  bool claim_pio_sm(FastAccelStepperEngine* engine);
  void setupSM();
  int32_t getCurrentStepCount();
  void attachDirPinToStatemachine();
  void setDirPinState(bool high);

  AqeResultCode addQueueEntry(const struct stepper_command_s* cmd, bool start);
  int32_t getCurrentPosition();
  uint32_t ticksInQueue();
  bool hasTicksInQueue(uint32_t min_ticks);
  bool getActualTicksWithDirection(struct actual_ticks_s* speed);

  bool init(FastAccelStepperEngine* engine, uint8_t queue_num,
            uint8_t step_pin);
  void startQueue();
  void forceStop();
  void _initVars();
  void connect();
  void disconnect();

  void setDirPin(uint8_t dir_pin, bool _dirHighCountsUp) {
    dirPin = dir_pin;
    dirHighCountsUp = _dirHighCountsUp;
    attachDirPinToStatemachine();
  }
};

#endif
