#ifndef PD_TEST_QUEUE_H
#define PD_TEST_QUEUE_H

#include "FastAccelStepper.h"
#include "fas_queue/base.h"

#define fas_queue_A fas_queue[0]
#define fas_queue_B fas_queue[1]

class StepperQueue : public StepperQueueBase {
 public:
#include "../fas_queue/protocol.h"

  volatile bool _isRunning;
  uint16_t _before_dir_change_delay_ticks;
  uint16_t _after_dir_change_delay_ticks;

  inline void _pd_initVars() {
    max_speed_in_ticks = 80;
#ifdef SUPPORT_ESP32_RMT
    channel = RMT_CHANNEL_T();
    _rmtStopped = true;
    lastChunkContainsSteps = false;
#endif
  }

  inline bool isReadyForCommands() const { return true; }
  inline bool isRunning() const { return _isRunning; }
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

#define BEFORE_DIR_CHANGE_DELAY_TICKS(q) ((q)->_before_dir_change_delay_ticks)
#define AFTER_DIR_CHANGE_DELAY_TICKS(q) ((q)->_after_dir_change_delay_ticks)

#endif  // PD_TEST_QUEUE_H
