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

#if defined(ARDUINO_ARCH_ESP32)
#define MIN_DELTA_TICKS (TICKS_PER_S / 200000)
#elif defined(ARDUINO_ARCH_AVR)
// AVR:
// tests on arduino nano indicate, that at 40ksteps/s in dual stepper mode,
// the main task is freezing (StepperDemo).
// Thus the limitation set here is set to 25kSteps/s as stated in the README.
#define MIN_DELTA_TICKS (TICKS_PER_S / 25000)
#else
#define MIN_DELTA_TICKS (TICKS_PER_S / 50000)
#endif

#define MIN_CMD_TICKS (10 * MIN_DELTA_TICKS)

#define MAX_ON_DELAY_TICKS ((uint32_t)(65535 * (QUEUE_LEN - 1)))

#define PIN_UNDEFINED 255

class FastAccelStepperEngine;

class FastAccelStepper {
 public:
  // This should be only called by FastAccelStepperEngine !
  void init(FastAccelStepperEngine* engine, uint8_t num, uint8_t step_pin);

  // step pin is defined at creation. Here can retrieve the pin
  uint8_t getStepPin();

  // if direction pin is connected, call this function
  void setDirectionPin(uint8_t dirPin, bool dirHighCountsUp = true);
  uint8_t getDirectionPin() { return _dirPin; }
  bool directionPinHighCountsUp() { return _dirHighCountsUp; }

  // if enable pin is connected, then use this function.
  //
  // In case there are two enable pins: one low and one high active, then
  // these calls are valid and both pins will be operated:
  //	setEnablePin(pin1, true);
  //	setEnablePin(pin2, false);
  // If pin1 and pin2 are same, then the last call will be used.
  void setEnablePin(uint8_t enablePin, bool low_active_enables_stepper = true);
  uint8_t getEnablePinHighActive() { return _enablePinHighActive; }
  uint8_t getEnablePinLowActive() { return _enablePinLowActive; }

  // If the enable pins are e.g. connected via external HW (shift registers),
  // then an external callback function can be supplied.
  // This will be called for defined low active and high active enable pins.
  // If both pins are defined, the function will be called
  // twice. The supplied value is either LOW or HIGH. The return value shall be
  // the status of the pin (either LOW or HIGH).
  //
  // In auto enable mode, this function is called from cyclic task/interrupt
  // with 4ms rate, which creates the commands to put into the command queue.
  // Thus the supplied function should take much less time than 4ms.
  // Otherwise there is risk, that other running steppers are running out of
  // commands in the queue. If this takes longer, then the function should be
  // offloaded and return the new status, after the enable/disable function has
  // been successfully completed.
  void setExternalEnableCall(bool (*func)(uint8_t enablePin, uint8_t value));

  // using enableOutputs/disableOutputs the stepper can be enabled and disabled
  bool enableOutputs();   // returns true, if enabled
  bool disableOutputs();  // returns true, if disabled

  // In auto enable mode, the stepper is enabled before stepping and disabled
  // afterwards. The delay from stepper enabled till first step and from
  // last step to stepper disabled can be separately adjusted.
  // The delay from enable to first step is done in ticks and as such is limited
  // to MAX_ON_DELAY_TICKS, which translates approximately to 120ms for
  // esp32 and 60ms for avr at 16 MHz). The delay till disable is done in period
  // interrupt/task with 4 or 10 ms repetition rate and as such is with several
  // ms jitter.
  void setAutoEnable(bool auto_enable);
  int8_t setDelayToEnable(uint32_t delay_us);
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

  // is true while the stepper is running or ramp generation is active
  bool isRunning();

  // is true while the stepper is running
  bool isMotorRunning();

  // For stepper movement control by FastAccelStepper
  //
  // setSpeed expects as parameter the minimum time between two steps.
  // If for example 5 steps/s shall be the maximum speed of the stepper,
  // then t = 0.2 s/steps = 200000 us/step, so call
  //      setSpeed(200000);
  //
  // New value will be used after call to
  // move/moveTo/runForward/runBackward/applySpeedAcceleration/moveByAcceleration
  //
  // note: no update on stopMove()
  //
  // Returns 0 on success, or -1 on invalid value
  // Invalid is <MIN_DELTA_TICKS in us or >~250 Mio.
  int8_t setSpeed(uint32_t min_step_us);

  //  set Acceleration expects as parameter the change of speed
  //  as step/s².
  //  If for example the speed should ramp up from 0 to 10000 steps/s within
  //  10s, then the acceleration is 10000 steps/s / 10s = 1000 steps/s²
  //
  // New value will be used after call to
  // move/moveTo/stopMove/applySpeedAcceleration
  //
  // Returns 0 on success, or -1 on invalid value (=0)
  int8_t setAcceleration(uint32_t step_s_s);

  // This function applies new values for speed/acceleration.
  // This is convenient especially, if we stepper is set to continuous running.
  void applySpeedAcceleration();

  // start/move the stepper for (move) steps or to an absolute position.
  //
  // If the stepper is already running, then the current running move will be
  // updated together with any updated values of acceleration/speed. The move is
  // relative to the target position of any ongoing move ! If the new
  // move/moveTo for an ongoing command would reverse the direction, then the
  // command is silently ignored.
  int8_t move(int32_t move);
  int8_t moveTo(int32_t position);
#define MOVE_OK 0
#define MOVE_ERR_NO_DIRECTION_PIN \
  -1 /* negative direction requested, but no direction pin defined */
#define MOVE_ERR_SPEED_IS_UNDEFINED -2
#define MOVE_ERR_ACCELERATION_IS_UNDEFINED -3

  // This command flags the stepper to keep run continuously into current
  // direction. It can be stopped by stopMove.
  // Be aware, if the motor is currently decelerating towards reversed
  // direction, then keepRunning() will speed up again and not finish direction
  // reversal first.
  void keepRunning();
  bool isRunningContinuously() { return _rg.isRunningContinuously(); }

  // This command just let the motor run continuously in one direction.
  // If the motor is running in the opposite direction, it will reverse
  // return value as with move/moveTo
  int8_t runForward() { return _rg.startRun(true); }
  int8_t runBackward() { return _rg.startRun(false); }

  // forwardStep()/backwardstep() can be called, while stepper is not moving
  // If stepper is moving, this is a no-op.
  // backwardStep() is a no-op, if no direction pin defined
  // It will immediately let the stepper perform one single step.
  // If blocking = true, then the routine will wait till isRunning() is false
  void forwardStep(bool blocking = false);
  void backwardStep(bool blocking = false);

  // moveByAcceleration() can be called, if only the speed of the stepper
  // is of interest and that speed to be controlled by acceleration.
  // The maximum speed (in both directions) to be set by setSpeed() before.
  // The behaviour will be:
  //	acceleration > 0  => accelerate towards positive maximum speed
  //	acceleration = 0  => keep current speed
  //	acceleration < 0
  //		=> accelerate towards negative maximum speed if allow_reverse
  //		=> decelerate towards motor stop if allow_reverse = false
  // return value as with move/moveTo
  int8_t moveByAcceleration(int32_t acceleration, bool allow_reverse = true);

  // stop the running stepper as fast as possible with deceleration
  // This only sets a flag and can be called from an interrupt !
  void stopMove();
  bool isStopping() { return _rg.isStopping(); }

  // stop the running stepper immediately and set new_pos as new position
  // This can be called from an interrupt !
  void forceStopAndNewPosition(uint32_t new_pos);

  // get the target position for the current move
  inline int32_t targetPos() { return _rg.targetPosition(); }

  // stepper queue management (low level access)
  //
  // If the queue is already running, then the start parameter is obsolote.
  // But the queue may run out of commands while executing addQueueEntry,
  // so it is better to set start=true to automatically restart/continue
  // a running queue.
  //
  // If the queue is not running, then the start parameter defines starting it
  // or not. The latter case is of interest to first fill the queue and then
  // start it.
  //
  // The call addQueueEntry(NULL, true) just starts the queue. This is intended
  // to achieve a near synchronous start of several steppers. Consequently it
  // should be called with interrupts disabled and return very fast.
  int8_t addQueueEntry(const struct stepper_command_s* cmd, bool start = true);

  // Return codes for addQueueEntry
  //    positive values mean, that caller should retry later
#define AQE_OK 0
#define AQE_QUEUE_FULL 1
#define AQE_DIR_PIN_IS_BUSY 2
#define AQE_WAIT_FOR_ENABLE_PIN_ACTIVE 3
#define AQE_ERROR_TICKS_TOO_LOW -1
#define AQE_ERROR_EMPTY_QUEUE_TO_START -2

  // check function s for command queue being empty or full
  bool isQueueEmpty();
  bool isQueueFull();

  // Get the future position of the stepper after all commands in queue are
  // completed
  int32_t getPositionAfterCommandsCompleted();

  // Get the future speed of the stepper after all commands in queue are
  // completed. This is in µs. Returns 0 for stopped motor
  //
  // This value comes from the ramp generator and is not valid for raw command
  // queue
  // ==> Will be renamed in future release
  uint32_t getPeriodAfterCommandsCompleted();

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
#define RAMP_STATE_REVERSE 5
#define RAMP_STATE_MASK 0x0f
  inline uint8_t rampState() { return _rg.rampState(); }

  // returns true, if the ramp generation is active
  inline bool isRampGeneratorActive() { return _rg.isRampGeneratorActive(); }

  // These functions allow to detach and reAttach a step pin for other use.
  // Pretty low level, use with care or not at all
  void detachFromPin();
  void reAttachToPin();

#if defined(ARDUINO_ARCH_ESP32)
  // These two functions are only available on esp32.
  // The first can attach any of the eight pulse counters to this stepper.
  // The second then will read the current pulse counter value
  //
  // The user is responsible to not use a pulse counter, which is occupied by a
  // stepper and by this render the stepper or even blow up the uC.
  //
  // Pulse counter 6 and 7 are not used by the stepper library and are judged as
  // available. If only five steppers are defined, then 5 gets available. If
  // four steppers are defined, then 4 is usable,too.
  //
  // The function are intended for testing, because the library should always
  // output the correct amount of pulses. Possible application usage would be an
  // immediate and interrupt friendly version for getCurrentPosition()
  bool attachToPulseCounter(uint8_t pcnt_unit);
  int16_t readPulseCounter();
  bool pulseCounterAttached() { return _attached_pulse_cnt_unit >= 0; }
#endif

 private:
  void fill_queue();
  void updateAutoDisable();
  bool needAutoDisable();
  bool agreeWithAutoDisable();
  bool usesAutoEnablePin(uint8_t pin);

  FastAccelStepperEngine* _engine;
  bool (*_externalEnableCall)(uint8_t enablePin, uint8_t value);
  RampGenerator _rg;
  uint8_t _stepPin;
  uint8_t _dirPin;
  bool _dirHighCountsUp;
  bool _autoEnable;
  uint8_t _enablePinLowActive;
  uint8_t _enablePinHighActive;
  uint8_t _queue_num;

  uint32_t _on_delay_ticks;
  uint16_t _off_delay_count;
  uint16_t _auto_disable_delay_counter;

#if defined(ARDUINO_ARCH_ESP32)
  int16_t _attached_pulse_cnt_unit;
#endif
#if (TEST_MEASURE_ISR_SINGLE_FILL == 1)
  uint32_t max_micros;
#endif
#if (TEST_CREATE_QUEUE_CHECKSUM == 1)
  uint32_t checksum();
#endif

  friend class FastAccelStepperEngine;
  friend class FastAccelStepperTest;
};

class FastAccelStepperEngine {
 public:
  // stable API functions
  void init();

  // ESP32:
  // The first three steppers use mcpwm0, the next three steppers use mcpwm1
  //
  // Atmega328p:
  // Only the pins connected to OC1A and OC1B are allowed
  //
  // Atmega2560:
  // Only the pins connected to OC4A, OC4B and OC4C are allowed.
  //
  // If no stepper resources available or pin is wrong, then NULL is returned
  FastAccelStepper* stepperConnectToPin(uint8_t step_pin);

  // If this is called, then the periodic task will let the associated LED
  // blink with 1 Hz
  void setDebugLed(uint8_t ledPin);

  // This should be only called from ISR or stepper task
  void manageSteppers();

 private:
  bool isDirPinBusy(uint8_t dirPin, uint8_t except_stepper);

  uint8_t _next_stepper_num;
  FastAccelStepper* _stepper[MAX_STEPPER];

  bool _isValidStepPin(uint8_t step_pin);

  friend class FastAccelStepper;
};

extern FastAccelStepperEngine* fas_engine;
#endif
