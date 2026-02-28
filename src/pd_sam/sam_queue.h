#ifndef PD_SAM_QUEUE_H
#define PD_SAM_QUEUE_H

#include "FastAccelStepper.h"
#include "fas_queue/base.h"

typedef struct _PWMCHANNELMAPPING {
  uint8_t pin;
  uint32_t channel;
  Pio* port;
  uint32_t channelMask;
} PWMCHANNELMAPPING;

class StepperQueue : public StepperQueueBase {
 public:
  uint8_t _step_pin;
  uint8_t _queue_num;
  void* driver_data;
  volatile bool _hasISRactive;
  bool _connected;
  volatile bool _pauseCommanded;
  volatile uint32_t timePWMInterruptEnabled;

  inline bool isRunning() { return _hasISRactive; }
  inline bool isReadyForCommands() { return true; }

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
  }

  static bool isValidStepPin(uint8_t step_pin);
};

#endif
