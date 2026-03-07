#ifndef PD_SAM_QUEUE_H
#define PD_SAM_QUEUE_H

#include "FastAccelStepper.h"
#include "fas_queue/base.h"

class StepperQueue : public StepperQueueBase {
 public:
#include "../fas_queue/protocol.h"

  uint8_t _step_pin;
  uint8_t _queue_num;
  void* driver_data;
  volatile bool _hasISRactive;
  bool _connected;
  volatile bool _pauseCommanded;
  volatile uint32_t timePWMInterruptEnabled;

  inline bool isRunning() { return _hasISRactive; }
  inline bool isReadyForCommands() { return true; }

  void setDirPin(uint8_t dir_pin, bool _dirHighCountsUp) {
    dirPin = dir_pin;
    dirHighCountsUp = _dirHighCountsUp;
#if defined(SUPPORT_DIR_PIN_MASK)
    if ((dir_pin != PIN_UNDEFINED) && ((dir_pin & PIN_EXTERNAL_FLAG) == 0)) {
      _dirPinPort = portOutputRegister(digitalPinToPort(dir_pin));
      _dirPinMask = digitalPinToBitMask(dir_pin);
    }
#endif
  }
};

#define SET_DIRECTION_PIN_STATE(q, high)          \
  do {                                            \
    if ((q)->_dirPinPort != NULL) {               \
      if (high) {                                 \
        *((q)->_dirPinPort) |= (q)->_dirPinMask;  \
      } else {                                    \
        *((q)->_dirPinPort) &= ~(q)->_dirPinMask; \
      }                                           \
    }                                             \
  } while (0)

#define AFTER_SET_DIR_PIN_DELAY_US 30

#define BEFORE_DIR_CHANGE_DELAY(q) ((uint16_t)0)
#define AFTER_DIR_CHANGE_DELAY(q) ((uint16_t)0)

#endif
