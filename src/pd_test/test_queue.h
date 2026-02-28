#ifndef PD_TEST_QUEUE_H
#define PD_TEST_QUEUE_H

#include "FastAccelStepper.h"
#include "fas_queue/base.h"

class StepperQueue : public StepperQueueBase {
 public:
  volatile bool _isRunning;
  inline bool isReadyForCommands() { return true; }
  inline bool isRunning() { return _isRunning; }
#ifdef SUPPORT_ESP32_RMT
  RMT_CHANNEL_T channel;
  bool _rmtStopped;
  bool lastChunkContainsSteps;
#endif

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
  }

  static bool isValidStepPin(uint8_t step_pin);
};

#endif  // PD_TEST_QUEUE_H
