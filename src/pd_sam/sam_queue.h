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
  volatile uint32_t* _dirPinPort;
  uint32_t _dirPinMask;

  inline void _pd_initVars() {
    _step_pin = PIN_UNDEFINED;
    max_speed_in_ticks = 420;
  }

  inline bool isRunning() const { return _hasISRactive; }
  inline bool isReadyForCommands() const { return true; }

  void setDirPin(uint8_t dir_pin, bool _dirHighCountsUp) {
    dirPin = dir_pin;
    dirHighCountsUp = _dirHighCountsUp;
    if ((dir_pin != PIN_UNDEFINED) && ((dir_pin & PIN_EXTERNAL_FLAG) == 0)) {
      _dirPinPort = portOutputRegister(digitalPinToPort(dir_pin));
      _dirPinMask = digitalPinToBitMask(dir_pin);
    }
  }

 private:
  static bool isValidStepPin(uint8_t step_pin);
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

#define SET_ENABLE_PIN_STATE(q, pin, high) \
  digitalWrite((pin), (high) ? HIGH : LOW)

#define AFTER_SET_DIR_PIN_DELAY_US 30

#endif
