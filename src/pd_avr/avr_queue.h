#ifndef PD_AVR_QUEUE_H
#define PD_AVR_QUEUE_H

#include "FastAccelStepper.h"
#include "fas_queue/base.h"

//==========================================================================
//
// AVR derivate ATmega 168/328/P
//
//==========================================================================
#if (defined(__AVR_ATmega168__) || defined(__AVR_ATmega168P__) || \
     defined(__AVR_ATmega328__) || defined(__AVR_ATmega328P__))
#define fas_queue_A fas_queue[0]
#define fas_queue_B fas_queue[1]
enum class channels : uint8_t { channelA, channelB };
//==========================================================================
//
// AVR derivate ATmega 2560
//
//==========================================================================
#elif defined(__AVR_ATmega2560__)
#define fas_queue_A fas_queue[0]
#define fas_queue_B fas_queue[1]
#define fas_queue_C fas_queue[2]
enum class channels : uint8_t { channelA, channelB, channelC };
//==========================================================================
//
// AVR derivate ATmega 32U4
//
//==========================================================================
#elif defined(__AVR_ATmega32U4__)
#define fas_queue_A fas_queue[0]
#define fas_queue_B fas_queue[1]
#define fas_queue_C fas_queue[2]
enum class channels : uint8_t { channelA, channelB, channelC };
#endif

class StepperQueue : public StepperQueueBase {
 public:
#include "../fas_queue/protocol.h"

  volatile bool _noMoreCommands;
  volatile bool _isRunning;
  inline bool isRunning() const { return _isRunning; }
  inline bool isReadyForCommands() const { return true; }
  channels channel;

  volatile uint8_t* _dirPinPort;
  uint8_t _dirPinMask;
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

  void adjustSpeedToStepperCount(uint8_t steppers);

 private:
  static int8_t queueNumForStepPin(uint8_t step_pin);
};

#define SET_DIRECTION_PIN_STATE(q, high)        \
  do {                                          \
    if (high) {                                 \
      *((q)->_dirPinPort) |= (q)->_dirPinMask;  \
    } else {                                    \
      *((q)->_dirPinPort) &= ~(q)->_dirPinMask; \
    }                                           \
  } while (0)

#define SET_ENABLE_PIN_STATE(q, pin, high) \
  digitalWrite((pin), (high) ? HIGH : LOW)

#endif  // PD_AVR_QUEUE_H
