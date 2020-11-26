#ifndef FASTACCELSTEPPER_H
#define FASTACCELSTEPPER_H
#if defined(ARDUINO_ARCH_ESP32) || defined(ARDUINO_ARCH_AVR)
#include <Arduino.h>
#else
#include <math.h>
#include <stdio.h>
#include <stdlib.h>

#include "../tests/stubs.h"
#endif
#include <stdint.h>

#include "PoorManFloat.h"
#include "RampGenerator.h"

#ifndef TEST_MEASURE_ISR_SINGLE_FILL
#define TEST_MEASURE_ISR_SINGLE_FILL 0
#endif
#ifndef TEST_CREATE_QUEUE_CHECKSUM
#define TEST_CREATE_QUEUE_CHECKSUM 0
#endif

#if defined(TEST)
#define MAX_STEPPER 2
#define TICKS_PER_S 16000000L
#elif defined(ARDUINO_ARCH_AVR)
#define MAX_STEPPER 2
#define TICKS_PER_S F_CPU
#elif defined(ARDUINO_ARCH_ESP32)
#define MAX_STEPPER 6
#define TICKS_PER_S 16000000L
#else
#define MAX_STEPPER 6
#define TICKS_PER_S 16000000L
#endif

#if !defined(ARDUINO_ARCH_ESP32) && !defined(ARDUINO_ARCH_AVR)
#ifndef F_CPU
#define F_CPU 16000000L
#endif
#endif

#define MIN_DELTA_TICKS (F_CPU / 50000)
#define MIN_REMAINING_PERIOD_TICKS (2 * MIN_DELTA_TICKS)
#define PERIOD_TICKS (65535 - MIN_REMAINING_PERIOD_TICKS)
#define ABSOLUTE_MAX_TICKS (255L * PERIOD_TICKS)

#define PIN_UNDEFINED 255

class FastAccelStepper {
 public:
  // This should be only called by FastAccelStepperEngine !
  void init(uint8_t num, uint8_t step_pin);

  // step pin is defined at creation. Here can retrieve the pin
  uint8_t getStepPin();

  // if direction pin is connected, call this function
  void setDirectionPin(uint8_t dirPin, bool dirHighCountsUp = true);

  // if enable pin is connected, then use this function.
  //
  // In case there are two enable pins: one low and one high active, then
  // these calls are valid and both pins will be operated:
  //	setEnablePin(pin1, true);
  //	setEnablePin(pin2, false);
  // If pin1 and pin2 are same, then the last call will be used.
  void setEnablePin(uint8_t enablePin, bool low_active_enables_stepper = true);

  // using enableOutputs/disableOutputs the stepper can be enabled and disabled
  void enableOutputs();
  void disableOutputs();

  // In auto enable mode, the stepper is enabled before stepping and disabled
  // afterwards. The delay from stepper enabled till first step and from
  // last step to stepper disabled can be separately adjusted.
  // The delay from enable to first step is done in ticks and as such is limited
  // to ABSOLUTE_MAX_TICKS, which translates approximately to 1s (for esp32 and
  // avr at 16 MHz). The delay till disable is done in period interrupt/task
  // with 4 or 10 ms repetition rate and as such is with several ms jitter.
  void setAutoEnable(bool auto_enable);
  int setDelayToEnable(uint32_t delay_us);
  void setDelayToDisable(uint16_t delay_ms);
#define DELAY_OK 0
#define DELAY_TOO_LOW -1
#define DELAY_TOO_HIGH -2

  // Retrieve the current position of the stepper - either in standstill or
  // while moving
  //    for esp32: the position while moving may deviate by the currently
  //    executed queue command's steps
  int32_t getCurrentPosition();

  // Set the current position of the stepper - either in standstill or while
  // moving.
  //    for esp32: the implementation uses getCurrentPosition(), which does not
  //               consider the steps of the current command
  //               => recommend to use only in standstill
  void setCurrentPosition(int32_t new_pos);

  // is true while the stepper is running
  bool isRunning();

  // For stepper movement control by FastAccelStepper
  //
  // setSpeed expects as parameter the minimum time between two steps.
  // If for example 5 steps/s shall be the maximum speed of the stepper,
  // then this call will be
  //      t = 0.2 s/steps = 200000 us/step
  //      setSpeed(200000);
  //
  // New value will be used after call to move/moveTo/stopMove e.g. move(0)
  //
  void setSpeed(uint32_t min_step_us);

  //  set Acceleration expects as parameter the change of speed
  //  as step/s².
  //  If for example the speed should ramp up from 0 to 10000 steps/s within
  //  10s, then the acceleration is 10000 steps/s / 10s = 1000 steps/s²
  //
  // New value will be used after call to move/moveTo/stopMove e.g. move(0)
  //
  void setAcceleration(uint32_t step_s_s);

  // start/move the stepper for (move) steps or to an absolute position.
  //
  // If the stepper is already running, then the current running move will be
  // updated together with any updated values of acceleration/speed. The move is
  // relative to the target position of any ongoing move ! If the new
  // move/moveTo for an ongoing command would reverse the direction, then the
  // command is silently ignored.
  int move(int32_t move);
  int moveTo(int32_t position);
#define MOVE_OK 0
#define MOVE_ERR_OVERFLOW -1  // relative move has caused an overflow
#define MOVE_ERR_NO_DIRECTION_PIN \
  -2  // negative direction requested, but no direction pin defined
#define MOVE_ERR_SPEED_IS_UNDEFINED -3
#define MOVE_ERR_ACCELERATION_IS_UNDEFINED -4
#define MOVE_ERR_STOP_ONGOING -5

  // stop the running stepper as fast as possible with deceleration
  // This only sets a flag and can be called from an interrupt !
  // Another move/moveTo must wait, till the motor has stopped
  void stopMove();

  // get the target position for the current move
  inline int32_t targetPos() { return rg.targetPosition(); }

  // Low level acccess via command queue
  // stepper queue management (low level access)
  //	delta_ticks is multiplied by (1/TICKS_PER_S) s
  //	steps must be less than 128 aka 7 bits
  int addQueueEntry(uint32_t delta_ticks, uint8_t steps, bool dir_high);

  // Return codes for addQueueEntry
#define AQE_OK 0
#define AQE_FULL -1
#define AQE_TOO_HIGH -2
#define AQE_TOO_LOW -3
#define AQE_STEPS_ERROR -4

  // check function s for command queue being empty or full
  bool isQueueEmpty();
  bool isQueueFull();

  // Get the future position of the stepper after all commands in queue are
  // completed
  int32_t getPositionAfterCommandsCompleted();

  // Set the future position of the stepper after all commands in queue are
  // completed. This has immediate effect to getCurrentPosition().
  void setPositionAfterCommandsCompleted(int32_t new_pos);

  // This function provides info, in which state the high level stepper control
  // is operating
#define RAMP_STATE_IDLE 0
#define RAMP_STATE_ACCELERATE 1
#define RAMP_STATE_DECELERATE_TO_STOP 2
#define RAMP_STATE_DECELERATE 3
#define RAMP_STATE_COAST 4
#define RAMP_STATE_MASK 0x0f
#define RAMP_MOVE_UP 0x80
#define RAMP_MOVE_DOWN 0x40
#define RAMP_MOVE_MASK 0xc0
  inline uint8_t rampState() { return rg.rampState(); }

  // returns true, if the ramp generation is active
  inline bool isrSpeedControlEnabled() {
    return rg.rampState() != RAMP_STATE_IDLE;
  };

  // This variable/these functions should NEVER be modified/called by the
  // application
  inline void manage() {
    isr_fill_queue();
    check_for_auto_disable();
  }

#if (TEST_MEASURE_ISR_SINGLE_FILL == 1)
  uint32_t max_micros;
#endif
#if (TEST_CREATE_QUEUE_CHECKSUM == 1)
  uint32_t checksum();
#endif

 private:
  RampGenerator rg;
  uint8_t _stepPin;
  uint8_t _dirPin;
  bool _dirHighCountsUp;
  bool _autoEnable;
  uint8_t _enablePinLowActive;
  uint8_t _enablePinHighActive;
  uint8_t _queue_num;

  uint16_t _off_delay_count;
  uint16_t _auto_disable_delay_counter;
  void isr_fill_queue();
  void isr_single_fill_queue();
  void check_for_auto_disable();
};

class FastAccelStepperEngine {
 public:
  // stable API functions
  void init();

  // ESP32:
  // The first three steppers use mcpwm0, the next three steppers use mcpwm1
  //
  // AVR:
  // Only the pins connected to OC1A and OC1B are allowed
  //
  // If no stepper resources available or pin is wrong, then NULL is returned
  FastAccelStepper* stepperConnectToPin(uint8_t step_pin);

  // unstable API functions
  //
  // If this is called, then the periodic task will let the associated LED
  // blink with 1 Hz
  void setDebugLed(uint8_t ledPin);

  // This should be only called from ISR or stepper task
  void manageSteppers();

 private:
  uint8_t _next_stepper_num;
  FastAccelStepper* _stepper[MAX_STEPPER];

  bool _isValidStepPin(uint8_t step_pin);
};
#endif
