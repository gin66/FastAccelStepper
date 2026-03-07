#ifndef PD_TEST_QUEUE_H
#define PD_TEST_QUEUE_H

#include "FastAccelStepper.h"
#include "fas_queue/base.h"

class StepperQueue : public StepperQueueBase {
 public:
#include "../fas_queue/protocol.h"

  volatile bool _isRunning;

  inline void _pd_initVars() { _isRunning = false; }

  inline bool isReadyForCommands() { return true; }
  inline bool isRunning() { return _isRunning; }
#ifdef SUPPORT_ESP32_RMT
  RMT_CHANNEL_T channel;
  bool _rmtStopped;
  bool lastChunkContainsSteps;
#endif

  void setDirPin(uint8_t dir_pin, bool _dirHighCountsUp) {
    dirPin = dir_pin;
    dirHighCountsUp = _dirHighCountsUp;
  }
};

#define SET_DIRECTION_PIN_STATE(q, high) ((void)0)

#define SET_ENABLE_PIN_STATE(q, pin, high) ((void)0)

#endif  // PD_TEST_QUEUE_H
