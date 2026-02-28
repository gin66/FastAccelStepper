#ifndef PD_AVR_QUEUE_H
#define PD_AVR_QUEUE_H

#include "FastAccelStepper.h"
#include "fas_queue/base.h"

class StepperQueue : public StepperQueueBase {
 public:
  volatile bool _noMoreCommands;
  volatile bool _isRunning;
  inline bool isRunning() { return _isRunning; }
  inline bool isReadyForCommands() { return true; }
  enum channels channel;

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
#if defined(SUPPORT_DIR_PIN_MASK)
    if ((dir_pin != PIN_UNDEFINED) && ((dir_pin & PIN_EXTERNAL_FLAG) == 0)) {
      _dirPinPort = portOutputRegister(digitalPinToPort(dir_pin));
      _dirPinMask = digitalPinToBitMask(dir_pin);
    }
#endif
#if defined(SUPPORT_DIR_TOGGLE_PIN_MASK)
    if ((dir_pin != PIN_UNDEFINED) && ((dir_pin & PIN_EXTERNAL_FLAG) == 0)) {
      _dirTogglePinPort = portInputRegister(digitalPinToPort(dir_pin));
      _dirTogglePinMask = digitalPinToBitMask(dir_pin);
    }
#endif
  }

#if defined(NEED_ADJUSTABLE_MAX_SPEED_DEPENDING_ON_STEPPER_COUNT)
  void adjustSpeedToStepperCount(uint8_t steppers);
#endif
  static bool isValidStepPin(uint8_t step_pin);
#if defined(NEED_FIXED_QUEUE_TO_PIN_MAPPING)
  static int8_t queueNumForStepPin(uint8_t step_pin);
#endif
};

#endif  // PD_AVR_QUEUE_H
