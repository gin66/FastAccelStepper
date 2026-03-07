#ifndef PD_AVR_QUEUE_H
#define PD_AVR_QUEUE_H

#include "FastAccelStepper.h"
#include "fas_queue/base.h"

class StepperQueue : public StepperQueueBase {
 public:
#include "../fas_queue/protocol.h"

  volatile bool _noMoreCommands;
  volatile bool _isRunning;
  inline bool isRunning() { return _isRunning; }
  inline bool isReadyForCommands() { return true; }
  enum channels channel;

  volatile uint8_t* _dirTogglePinPort;
  uint8_t _dirTogglePinMask;

  inline void _pd_initVars() {
    _isRunning = false;
    _noMoreCommands = false;
  }

  void setDirPin(uint8_t dir_pin, bool _dirHighCountsUp) {
    dirPin = dir_pin;
    dirHighCountsUp = _dirHighCountsUp;
    if ((dir_pin != PIN_UNDEFINED) && ((dir_pin & PIN_EXTERNAL_FLAG) == 0)) {
      _dirPinPort = portOutputRegister(digitalPinToPort(dir_pin));
      _dirPinMask = digitalPinToBitMask(dir_pin);
      _dirTogglePinPort = portInputRegister(digitalPinToPort(dir_pin));
      _dirTogglePinMask = digitalPinToBitMask(dir_pin);
    }
  }

#if defined(NEED_ADJUSTABLE_MAX_SPEED_DEPENDING_ON_STEPPER_COUNT)
  void adjustSpeedToStepperCount(uint8_t steppers);
#endif
#if defined(NEED_FIXED_QUEUE_TO_PIN_MAPPING)
  static int8_t queueNumForStepPin(uint8_t step_pin);
#endif
};

#define SET_DIRECTION_PIN_STATE(q, high)        \
  do {                                          \
    if (high) {                                 \
      *((q)->_dirPinPort) |= (q)->_dirPinMask;  \
    } else {                                    \
      *((q)->_dirPinPort) &= ~(q)->_dirPinMask; \
    }                                           \
  } while (0)

#endif  // PD_AVR_QUEUE_H
