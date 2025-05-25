#ifndef FASTACCELSTEPPER_H
#define FASTACCELSTEPPER_H
#include <stdint.h>
#include "Log2Representation.h"
#include "fas_arch/common.h"

// # FastAccelStepper
//
// FastAccelStepper is a high speed alternative for the
// [AccelStepper library](http://www.airspayce.com/mikem/arduino/AccelStepper/).
// Supported are avr (ATmega 168/328/P, ATmega2560), esp32 and atmelsam due.
//
// Here is a basic example to run a stepper from position 0 to 1000 and back
// again to 0.
// ```
// #include <FastAccelStepper.h>
//
// FastAccelStepperEngine engine = FastAccelStepperEngine();
// FastAccelStepper *stepper = NULL;
//
// #define dirPinStepper    5
// #define enablePinStepper 6
// #define stepPinStepper   9
// void setup() {
//    engine.init();
//    stepper = engine.stepperConnectToPin(stepPinStepper);
//    if (stepper) {
//       stepper->setDirectionPin(dirPinStepper);
//       stepper->setEnablePin(enablePinStepper);
//       stepper->setAutoEnable(true);
//
//       stepper->setSpeedInHz(500);
//       stepper->setAcceleration(100);
//       stepper->moveTo(1000, true);
//       stepper->moveTo(0, true);
//    }
// }
//
// void loop() {}
// ```

class FastAccelStepper;

class FastAccelStepperEngine {
  //
  // ## FastAccelStepperEngine
  //
  // This engine - actually a factory - provides you with instances of steppers.

 public:
  // ### Initialization
  //
  // The FastAccelStepperEngine is declared with FastAccelStepperEngine().
  // This is to occupy the needed memory.
  // ```cpp
  // FastAccelStepperEngine engine = FastAccelStepperEngine();
  // ```
  // But it still needs to be initialized.
  // For this init shall be used:
  // ```cpp
  // void setup() {
  //    engine.init();
  // }
  // ```

#if defined(SUPPORT_CPU_AFFINITY)
  // In a multitasking and multicore system like ESP32, the steppers are
  // controlled by a continuously running task. This task can be fixed to one
  // CPU core with this modified init()-call. ESP32 implementation detail: For
  // values 0 and 1, xTaskCreatePinnedToCore() is used, or else xTaskCreate()
  void init(uint8_t cpu_core = 255);
#else
  void init();
#endif

  // ### Creation of FastAccelStepper
  //
  // Using a call to `stepperConnectToPin()` a FastAccelStepper instance is
  // created. This call tells the stepper, which step pin to use. As the
  // hardware may have limitations - e.g. no stepper resources anymore, or the
  // step pin cannot be used, then NULL is returned. So it is advised to check
  // the return value of this call.
#if !defined(SUPPORT_SELECT_DRIVER_TYPE)
  FastAccelStepper* stepperConnectToPin(uint8_t step_pin);
#endif
  // For e.g. esp32, there are two types of driver.
  // One using mcpwm and pcnt module. And another using rmt module.
  // This call allows to select the respective driver
#if defined(SUPPORT_SELECT_DRIVER_TYPE)
#define DRIVER_MCPWM_PCNT FasDriver::MCPWM_PCNT
#define DRIVER_RMT FasDriver::RMT
#define DRIVER_DONT_CARE FasDriver::DONT_CARE
  FastAccelStepper* stepperConnectToPin(
      uint8_t step_pin, FasDriver driver_type = DRIVER_DONT_CARE);
#endif

#if defined(SUPPORT_TASK_RATE_CHANGE)
  // For e.g. esp32 the repetition rate of the stepper task can be changed.
  // The default delay is 4ms.
  //
  // The steppertask is looping with:
  //       manageSteppers()
  //       wdt_reset()
  //       delay()
  //
  // The actual repetition rate of the stepper task is delay + execution time of
  // manageSteppers()
  //
  // This function is primary of interest in conjunction with
  // setForwardPlanningTimeInMs(). If the delay is larger then forward planning
  // time, then the stepper queue will always run out of commands, which lead to
  // a sudden stop of the motor. If the delay is 0, then the stepper task will
  // constantly looping, which may lead to the task blocking other tasks.
  // Consequently, this function is intended for advanced users.
  //
  // There is not planned to test this functionality, because automatic testing
  // is only available for avr devices and those continue to use fixed 4ms rate.
  //
  // Please be aware, that the configured tick rate aka portTICK_PERIOD_MS is
  // relevant. Apparently, arduino-esp32 has FreeRTOS configured to have a
  // tick-rate of 1000Hz
  inline void task_rate(uint8_t delay_ms) { _delay_ms = delay_ms; };
  uint8_t _delay_ms;
#endif

  // Comments to valid pins:
  //
  // clang-format off
  // | Device          | Comment                                                                                           |
  // |:----------------|:--------------------------------------------------------------------------------------------------|
  // | ESP32           | Every output capable GPIO can be used                                                             |
  // | ESP32S2         | Every output capable GPIO can be used                                                             |
  // | Atmega168/328/p | Only the pins connected to OC1A and OC1B are allowed                                              |
  // | Atmega2560      | Only the pins connected to OC4A, OC4B and OC4C are allowed.                                       |
  // | Atmega32u4      | Only the pins connected to OC1A, OC1B and OC1C are allowed                                        |
  // | Atmel SAM       | This can be one of each group of pins: 34/67/74/35, 17/36/72/37/42, 40/64/69/41, 9, 8/44, 7/45, 6 |
  // clang-format on

  // ## External Pins
  //
  // If the direction/enable pins are e.g. connected via external HW (shift
  // registers), then an external callback function can be supplied. The
  // supplied value is either LOW or HIGH. The return value shall be the status
  // of the pin (false for LOW or true for HIGH). If returned value and supplied
  // value do not match, the stepper does not continue, but calls this function
  // again.
  //
  // This function is called from cyclic task/interrupt with 4ms rate, which
  // creates the commands to put into the command queue. Thus the supplied
  // function should take much less time than 4ms. Otherwise there is risk, that
  // other running steppers are running out of commands in the queue. If this
  // takes longer, then the function should be offloaded and return the new
  // status, after the pin change has been successfully completed.
  //
  // The callback has to be called on the FastAccelStepperEngine.
  // See examples/ExternalCall
  //
  // Stepperpins (enable or direction), which should use this external callback,
  // need to be or'ed with PIN_EXTERNAL_FLAG ! FastAccelStepper uses this flag
  // to determine, if a pin is external or internal.
  void setExternalCallForPin(bool (*func)(uint8_t pin, uint8_t value));

  // ### Debug LED
  //
  // If blinking of a LED is required to indicate, the stepper controller is
  // still running, then the port. to which the LED is connected, can be told to
  // the engine. The periodic task will let the associated LED blink with 1 Hz
  void setDebugLed(uint8_t ledPin);

  /* This should be only called from ISR or stepper task. So do not call it */
  void manageSteppers();

 private:
  bool isDirPinBusy(uint8_t dirPin, uint8_t except_stepper);

  uint8_t _stepper_cnt;
  FastAccelStepper* _stepper[MAX_STEPPER];

  bool _isValidStepPin(uint8_t step_pin);
  bool (*_externalCallForPin)(uint8_t pin, uint8_t value);

#if defined(SUPPORT_RP_PICO)
  uint8_t claimed_pios;
  PIO pio[NUM_PIOS];

 public:
  void pushCommands();
#endif

  friend class FastAccelStepper;
  friend class StepperQueue;
};

// ### Return codes of calls to `move()` and `moveTo()`
//
// The defined preprocessor macros are MOVE_xxx:
// MOVE_OK: All is OK:
// MOVE_ERR_NO_DIRECTION_PIN: Negative direction requested, but no direction pin
// MOVE_ERR_SPEED_IS_UNDEFINED: The maximum speed has not been set yet
// MOVE_ERR_ACCELERATION_IS_UNDEFINED: The acceleration to use has not been set
// yet

// ### Return codes of `rampState()`
//
// The return value is an uint8_t, which consist of two fields:
//
// | Bit 7   | Bits 6-5  | Bits 4-0 |
// |:--------|:----------|:---------|
// |Always 0 | Direction | State    |
//
// The bit mask for direction and state:
#define RAMP_DIRECTION_MASK (32 + 64)
#define RAMP_STATE_MASK (1 + 2 + 4 + 8 + 16)

// The defined ramp states are:
#define RAMP_STATE_IDLE 0
#define RAMP_STATE_COAST 1
#define RAMP_STATE_ACCELERATE 2
#define RAMP_STATE_DECELERATE 4
#define RAMP_STATE_REVERSE (4 + 8)
#define RAMP_STATE_ACCELERATING_FLAG 2
#define RAMP_STATE_DECELERATING_FLAG 4

// And the two directions of a move
#define RAMP_DIRECTION_COUNT_UP 32
#define RAMP_DIRECTION_COUNT_DOWN 64

// A ramp state value of 2 is set after any move call on a stopped motor
// and until the stepper task is serviced. The stepper task will then
// control the direction flags

#include "RampGenerator.h"

//
// ## Timing values - Architecture dependent
//
// ### AVR
// |VARIABLE         | Value       | Unit                    |
// |:----------------|------------:|:------------------------|
// |TICKS_PER_S      | 16_000_000  | [ticks/s]               |
// |MIN_CMD_TICKS    |  640        | [1/TICKS_PER_S seconds] |
// |MIN_DIR_DELAY_US |   40        | [µs]                    |
// |MAX_DIR_DELAY_US | 4095        | [µs]                    |
//
// ### ESP32
// |VARIABLE         | Value       | Unit                    |
// |:----------------|------------:|:------------------------|
// |TICKS_PER_S      | 16_000_000  | [ticks/s]               |
// |MIN_CMD_TICKS    | 3200        | [1/TICKS_PER_S seconds] |
// |MIN_DIR_DELAY_US |  200        | [µs]                    |
// |MAX_DIR_DELAY_US | 4095        | [µs]                    |
//
// ### SAM DUE
// |VARIABLE         | Value       | Unit                    |
// |:----------------|------------:|:------------------------|
// |TICKS_PER_S      | 21_000_000  | [ticks/s]               |
// |MIN_CMD_TICKS    | 4200        | [1/TICKS_PER_S seconds] |
// |MIN_DIR_DELAY_US |  200        | [µs]                    |
// |MAX_DIR_DELAY_US | 3120        | [µs]                    |
//
// # FastAccelStepper

#define MAX_ON_DELAY_TICKS ((uint32_t)(65535 * (QUEUE_LEN - 1)))

#define PIN_UNDEFINED 255
#define PIN_EXTERNAL_FLAG 128

class FastAccelStepper {
#ifdef TEST
 public:
#else
 private:
#endif
  bool init(FastAccelStepperEngine* engine, uint8_t num, uint8_t step_pin);

 public:
  // ## Step Pin
  // step pin is defined at creation. Here can retrieve the pin
  uint8_t getStepPin();

  // ## Direction Pin
  // if direction pin is connected, call this function.
  //
  // If the pin number is >= 128, then the direction pin is assumed to be
  // external and the external callback function (set by
  // `setExternalCallForPin()`) is used to set the pin. For direction pin, this
  // is implemented for esp32 and its supported derivates, and avr and its
  // derivates except atmega32u4
  //
  // For slow driver hardware the first step after any polarity change of the
  // direction pin can be delayed by the value dir_change_delay_us. The allowed
  // range is MIN_DIR_DELAY_US and MAX_DIR_DELAY_US. The special value of 0
  // means, that no delay is added. Values 1 up to MIN_DIR_DELAY_US will be
  // clamped to MIN_DIR_DELAY_US. Values above MAX_DIR_DELAY_US will be clamped
  // to MAX_DIR_DELAY_US. For external pins, dir_change_delay_us is ignored,
  // because the mechanism applied for external pins provides already pause
  // in the range of ms or more.
  void setDirectionPin(uint8_t dirPin, bool dirHighCountsUp = true,
                       uint16_t dir_change_delay_us = 0);
  inline uint8_t getDirectionPin() { return _dirPin; }
  inline bool directionPinHighCountsUp() { return _dirHighCountsUp; }

  // ## Enable Pin
  // if enable pin is connected, then use this function.
  //
  // If the pin number is >= 128, then the enable pin is assumed to be
  // external and the external callback function (set by
  // `setExternalCallForPin()`) is used to set the pin.
  //
  // In case there are two enable pins: one low and one high active, then
  // these calls are valid and both pins will be operated:
  //    setEnablePin(pin1, true);
  //    setEnablePin(pin2, false);
  // If pin1 and pin2 are same, then the last call will be used.
  void setEnablePin(uint8_t enablePin, bool low_active_enables_stepper = true);
  inline uint8_t getEnablePinHighActive() { return _enablePinHighActive; }
  inline uint8_t getEnablePinLowActive() { return _enablePinLowActive; }

  // using enableOutputs/disableOutputs the stepper can be enabled and disabled
  // For a running motor with autoEnable set, disableOutputs() will return false
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
  DelayResultCode setDelayToEnable(uint32_t delay_us);
  void setDelayToDisable(uint16_t delay_ms);

  // ## Stepper Position
  // Retrieve the current position of the stepper
  //
  // Comment for esp32 with rmt module:
  // The actual position may be off by the number of steps in the ongoing
  // command. If precise real time position is needed, attaching a pulse counter
  // may be of help.
  int32_t getCurrentPosition();

  // Set the current position of the stepper - either in standstill or while
  // moving.
  //    for esp32: the implementation uses getCurrentPosition(), which does not
  //               consider the steps of the current command
  //               => recommend to use only in standstill
  void setCurrentPosition(int32_t new_pos);

  // ## Stepper running status
  // is true while the stepper is running or ramp generation is active
  bool isRunning();

  // ## Speed
  // For stepper movement control by FastAccelStepper's ramp generator
  //
  // Speed can be defined in four different units:
  // - In Hz: This means steps/s
  // - In millHz: This means in steps/1000s
  // - In us: This means in us/step
  //
  // For the device's maximum allowed speed, the following calls can be used.
  uint16_t getMaxSpeedInUs();
  uint16_t getMaxSpeedInTicks();
  uint32_t getMaxSpeedInHz();
  uint32_t getMaxSpeedInMilliHz();

  // For esp32 and avr, the device's maximum allowed speed can be overridden.
  // Allocating a new stepper will override any absolute speed limit.
  // This is absolutely untested, no error checking implemented.
  // Use at your own risk !
#if SUPPORT_UNSAFE_ABS_SPEED_LIMIT_SETTING == 1
  void setAbsoluteSpeedLimit(uint16_t max_speed_in_ticks);
#endif

  // Setting the speed can be done with the four `setSpeed...()` calls.
  // The new value will be used only after call of these functions:
  //
  // - `move()`
  // - `moveTo()`
  // - `runForward()`
  // - `runBackward()`
  // - `applySpeedAcceleration()`
  // - `moveByAcceleration()`
  //
  // Note: no update on `stopMove()`
  //
  // Returns 0 on success, or -1 on invalid value.
  // Invalid is faster than MaxSpeed or slower than ~250 Mio ticks/step.
  int8_t setSpeedInUs(uint32_t min_step_us);
  int8_t setSpeedInTicks(uint32_t min_step_ticks);
  int8_t setSpeedInHz(uint32_t speed_hz);
  int8_t setSpeedInMilliHz(uint32_t speed_mhz);

  // To retrieve current set speed. This means, while accelerating and/or
  // decelerating, this is NOT the actual speed !
  inline uint32_t getSpeedInUs() { return _rg.getSpeedInUs(); }
  inline uint32_t getSpeedInTicks() { return _rg.getSpeedInTicks(); }
  inline uint32_t getSpeedInMilliHz() { return _rg.getSpeedInMilliHz(); }

  // If the current speed is needed, then use `getCurrentSpeed...()`. This
  // retrieves the actual speed.
  //
  // | value | description                  |
  // |:-----:|:-----------------------------|
  // |   = 0 | while not moving             |
  // |   > 0 | while position counting up   |
  // |   < 0 | while position counting down |
  //
  // If the parameter realtime is true, then the most actual speed
  // from the stepper queue is derived. This works only, if the queue
  // does not contain pauses, which is normally the case for slow speeds.
  // Otherwise the speed from the ramp generator is reported, which is
  // done always in case of `realtime == false`. That speed is typically
  // the value of the speed a couple of milliseconds later.
  //
  // The drawback of `realtime == true` is, that the reported speed
  // may either come from the queue or from the ramp generator.
  // This means the returned speed may have jumps during
  // acceleration/deceleration.
  //
  // For backward compatibility, the default is true.
  int32_t getCurrentSpeedInUs(bool realtime = true);
  int32_t getCurrentSpeedInMilliHz(bool realtime = true);

  // ## Acceleration
  //  setAcceleration() expects as parameter the change of speed
  //  as step/s².
  //  If for example the speed should ramp up from 0 to 10000 steps/s within
  //  10s, then the acceleration is 10000 steps/s / 10s = 1000 steps/s²
  //
  // New value will be used after call to
  // move/moveTo/runForward/runBackward/applySpeedAcceleration/moveByAcceleration
  //
  // note: no update on stopMove()
  //
  // Returns 0 on success, or -1 on invalid value (<=0)
  inline int8_t setAcceleration(int32_t step_s_s) {
    return _rg.setAcceleration(step_s_s);
  }
  inline uint32_t getAcceleration() { return _rg.getAcceleration(); }

  // getCurrentAcceleration() retrieves the actual acceleration.
  //    = 0 while idle or coasting
  //    > 0 while speed is changing towards positive values
  //    < 0 while speed is changeing towards negative values
  inline int32_t getCurrentAcceleration() {
    return _rg.getCurrentAcceleration();
  }

  // ## Linear Acceleration
  //  setLinearAcceleration expects as parameter the number of steps,
  //  where the acceleration is increased linearly from standstill up to the
  //  configured acceleration value. If this parameter is 0, then there will be
  //  no linear acceleration phase
  //
  //  If for example the acceleration should ramp up from 0 to 10000 steps/s^2
  //  within 100 steps, then call setLinearAcceleration(100)
  //
  //  The speed at which linear acceleration turns into constant acceleration
  //  can be calculated from the parameter linear_acceleration_steps.
  //  Let's call this parameter `s_h` for handover steps.
  //  Then the speed is:
  //       `v_h = sqrt(1.5 * a * s_h)`
  //
  // New value will be used after call to
  // move/moveTo/runForward/runBackward/applySpeedAcceleration/moveByAcceleration
  //
  // note: no update on stopMove()
  inline void setLinearAcceleration(uint32_t linear_acceleration_steps) {
    _rg.setLinearAcceleration(linear_acceleration_steps);
  }

  // ## Jump Start
  // setJumpStart expects as parameter the ramp step to start from standstill.
  //
  // The speed at which the stepper will start can be calculated like this:
  // - If linear acceleration is not in use:
  //       start speed `v = sqrt(2 * a * jump_step)`
  // - If linear acceleration is in use and `jump_step <= s_h`:
  //       start speed `v = sqrt(1.5*a)/s_h^(1/6) * jump_step^(2/3)`
  // - If linear acceleration is in use and `jump_step > s_h`:
  //       start speed `v = sqrt(2 * a * (jump_step - s_h/4))`
  //
  //
  // New value will be used after call to
  // move/moveTo/runForward/runBackward
  inline void setJumpStart(uint32_t jump_step) { _rg.setJumpStart(jump_step); }

  // ## Apply new speed/acceleration value
  // This function applies new values for speed/acceleration.
  // This is convenient especially, if the stepper is set to continuous running.
  void applySpeedAcceleration();

  // ## Move commands
  // ### move() and moveTo()
  // start/move the stepper for (move) steps or to an absolute position.
  //
  // If the stepper is already running, then the current running move will be
  // updated together with any updated values of acceleration/speed. The move is
  // relative to the target position of any ongoing move ! If the new
  // move/moveTo for an ongoing command would reverse the direction, then the
  // command is silently ignored.
  // return values are the MOVE_... constants
  MoveResultCode move(int32_t move, bool blocking = false);
  MoveResultCode moveTo(int32_t position, bool blocking = false);

  // ### keepRunning()
  // This command flags the stepper to keep run continuously into current
  // direction. It can be stopped by stopMove.
  // Be aware, if the motor is currently decelerating towards reversed
  // direction, then keepRunning() will speed up again and not finish direction
  // reversal first.
  void keepRunning();
  bool isRunningContinuously() { return _rg.isRunningContinuously(); }

  // ### runForward() and runBackwards()
  // These commands just let the motor run continuously in one direction.
  // If the motor is running in the opposite direction, it will reverse
  // return value as with move/moveTo
  MoveResultCode runForward();
  MoveResultCode runBackward();

  // ### forwardStep() and backwardStep()
  // forwardStep()/backwardstep() can be called, while stepper is not moving
  // If stepper is moving, this is a no-op.
  // backwardStep() is a no-op, if no direction pin defined
  // It will immediately let the stepper perform one single step.
  // If blocking = true, then the routine will wait till isRunning() is false
  void forwardStep(bool blocking = false);
  void backwardStep(bool blocking = false);

  // ### moveByAcceleration()
  // moveByAcceleration() can be called, if only the speed of the stepper
  // is of interest and that speed to be controlled by acceleration.
  // The maximum speed (in both directions) to be set by setSpeedInUs() before.
  // The behaviour will be:
  //    acceleration > 0  => accelerate towards positive maximum speed
  //    acceleration = 0  => keep current speed
  //    acceleration < 0
  //        => accelerate towards negative maximum speed if allow_reverse
  //        => decelerate towards motor stop if allow_reverse = false
  // return value as with move/moveTo
  MoveResultCode moveByAcceleration(int32_t acceleration,
                                    bool allow_reverse = true);

  // ### stopMove()
  // Stop the running stepper with normal deceleration.
  // This only sets a flag and can be called from an interrupt !
  void stopMove();
  inline bool isStopping() { return _rg.isStopping(); }

  // ### stepsToStop()
  // This returns the current step value of the ramp.
  // This equals the number of steps for a motor to
  // reach the current position and speed from standstill
  // and to come to standstill with deceleration if stopped
  // immediately.
  // This value is valid with or without linear acceleration
  // being used.
  // Primary use is to forecast possible stop position.
  // The stop position is:
  //    getCurrentPosition() + stepsToStop()
  // in case of a motor running in positive direction.
  uint32_t stepsToStop() { return _rg.stepsToStop(); }

  // ### forceStop()
  // Abruptly stop the running stepper without deceleration.
  // This can be called from an interrupt !
  //
  // The stepper command queue will be processed, but no further commands are
  // added. This means, that the stepper can be expected to stop within approx.
  // 20ms.
  void forceStop();

  // abruptly stop the running stepper without deceleration.
  // This can be called from an interrupt !
  //
  // No further step will be issued. As this is aborting all commands in the
  // queue, the actual stop position is lost (recovering this position cannot be
  // done within an interrupt). So the new position after stop has to be
  // provided and will be set as current position after stop.
  void forceStopAndNewPosition(int32_t new_pos);

  // get the target position for the current move.
  // As of now, this position is the view of the stepper task.
  // This means, the value will stay unchanged after a move/moveTo until the
  // stepper task is executed.
  // In keep running mode, the targetPos() is not updated
  inline int32_t targetPos() { return _rg.targetPosition(); }

  // ### Task planning
  // The stepper task adds commands to the stepper queue until
  // either at least two commands are planned, or the commands
  // cover sufficient time into the future. Default value for that time is 20ms.
  //
  // The stepper task is cyclically executed every ~4ms.
  // Especially for avr, the step interrupts puts a significant load on the uC,
  // so the cyclical stepper task can even run for 2-3 ms. On top of that,
  // other interrupts caused by the application could increase the load even
  // further.
  //
  // Consequently, the forward planning should fill the queue for ideally two
  // cycles, this means 8ms. This means, the default 20ms provide a sufficient
  // margin and even a missed cycle is not an issue.
  //
  // The drawback of the 20ms is, that any change in speed/acceleration are
  // added after those 20ms and for an application, requiring fast reaction
  // times, this may impact the expected performance.
  //
  // Due to this the forward planning time can be adjusted with the following
  // API call for each stepper individually.
  //
  // Attention:
  // - This is only for advanced users: no error checking is implemented.
  // - Only change the forward planning time, if the stepper is not running.
  // - Too small values bear the risk of a stepper running at full speed
  // suddenly stopping
  //   due to lack of commands in the queue.
  inline void setForwardPlanningTimeInMs(uint8_t ms) {
    _forward_planning_in_ticks = ms;
    _forward_planning_in_ticks *= TICKS_PER_S / 1000;  // ticks per ms
  }

  // ## Intermediate Level Stepper Control for Advanced Users
  //
  // The main purpose is to bypass the ramp generator as mentioned in
  // [#299](https://github.com/gin66/FastAccelStepper/issues/299).
  // This shall allow to run consecutive small moves with fixed speed.
  // The parameters are steps (which can be 0) and duration in ticks.
  // steps=0 makes sense in order to keep the time running and not
  // getting out of sync.
  // Due to integer arithmetics the actual duration may be off by a small value.
  // That's why the actual_duration in TICKS is returned.
  // The application should consider this for the next runTimed move.
  //
  // The optional parameter is a boolean called start. This allows for the first
  // invocation to not start the queue yet. This is for managing steppers in
  // parallel. It allows to fill all steppers' queues and then kick it off by a
  // call to `moveTimed(0,0,NULL,true)`. Successive invocations can keep true.
  //
  // In order to not have another lightweight ramp generator running in
  // background interrupt, the expecation to the application is, that this
  // function is frequently enough called without the queue being emptied.
  //
  // The current implementation immediately starts with a step, if there should
  // be one. Perhaps performing the step in the middle of the duration is more
  // appropriate ?
  //
  // Meaning of the return values - which are in addtion to AQE from below
  // - OK:        Move has been successfully appended to the queue
  // - BUSY:      Queue does not have sufficient entries to append this timed
  // move.
  // - EMPTY:     The queue has run out of commands, but the move has been
  // appended.
  // - TOO_LARGE: The move request does not fit into the queue.
  //              Reasons: The queue depth is (32/16) for SAM+ESP32/AVR.
  //                       Each queue entry can emit 255 steps => (8160/4080)
  //                       steps If the time between steps is >65535 ticks, then
  //                       pauses have to be generated. In this case only (16/8)
  //                       steps can be generated...but the queue shall not be
  //                       empty
  //                       => so even less steps can be done.
  //              Recommendation: keep the duration in the range of ms.
  MoveTimedResultCode moveTimed(int16_t steps, uint32_t duration,
                                uint32_t* actual_duration, bool start = true);

  // ## Low Level Stepper Queue Management (low level access)
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
  // Actually this is necessary, too, in case the queue is full and not
  // started.
  // Return codes for addQueueEntry
  //    positive values mean, that caller should retry later
  AqeResultCode addQueueEntry(const struct stepper_command_s* cmd,
                              bool start = true);

  // ### check functions for command queue being empty, full or running.
  bool isQueueEmpty();
  bool isQueueFull();
  bool isQueueRunning();

  // ### functions to get the fill level of the queue
  //
  // To retrieve the forward planning time in the queue, ticksInQueue()
  // can be used. It sums up all ticks of the not yet processed commands.
  // For commands defining pauses, the summed up value is entry.ticks.
  // For commands with steps, the summed up value is entry.steps*entry.ticks
  uint32_t ticksInQueue();

  // This function can be used to check, if the commands in the queue
  // will last for <min_ticks> ticks. This is again without the
  // currently processed command.
  bool hasTicksInQueue(uint32_t min_ticks);

  // This function allows to check the number of commands in the queue.
  // This is including the currently processed command.
  uint8_t queueEntries();

  // Get the future position of the stepper after all commands in queue are
  // completed
  int32_t getPositionAfterCommandsCompleted();

  // Get the future speed of the stepper after all commands in queue are
  // completed. This is in µs. Returns 0 for stopped motor
  //
  // This value comes from the ramp generator and is not valid for raw command
  // queue
  // ==> Will be renamed in future release
  uint32_t getPeriodInUsAfterCommandsCompleted();
  uint32_t getPeriodInTicksAfterCommandsCompleted();

  // Set the future position of the stepper after all commands in queue are
  // completed. This has immediate effect to getCurrentPosition().
  void setPositionAfterCommandsCompleted(int32_t new_pos);

  // This function provides info, in which state the high level stepper control
  // is operating. The return value is an `or` of RAMP_STATE_... and
  // RAMP_DIRECTION_... flags. Definitions are above
  inline uint8_t rampState() { return _rg.rampState(); }

  // returns true, if the ramp generation is active
  inline bool isRampGeneratorActive() { return _rg.isRampGeneratorActive(); }

  // These functions allow to detach and reAttach a step pin for other use.
  // Pretty low level, use with care or not at all
  void detachFromPin();
  void reAttachToPin();

  // ## ESP32 only: Free pulse counter
  // These four functions are only available on esp32.
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
  // These functions are intended primarily for testing, because the library
  // should always output the correct amount of pulses. Possible application
  // usage would be an immediate and interrupt friendly version for
  // getCurrentPosition()
  //
  // The pulse counter counts up towards high_value.
  // If the high_value is reached, then the pulse counter is reset to 0.
  // Similarly, if direction pin is configured and set to count down,
  // then the pulse counter counts towards low_value. When the low value is hit,
  // the pulse counter is reset to 0.
  //
  // If low_value and high_value are set to zero, then the pulse counter is just
  // counting like any int16_t counter: 0...32767,-32768,-32767,...,0 and
  // backwards accordingly
  //
  // Possible application:
  // Assume the stepper, to which the pulse counter attached to, needs 3200
  // steps/revolution. If now attachToPulseCounter is called with -3200 and 3200
  // for the low and high values respectively, then the momentary angle of the
  // stepper (at exact this moment) can be retrieved just by reading the pulse
  // counter. If the value is negative, then just add 3200.
  //
  // In case external direction pin is used and the dir pin is available on one
  // of the GPIOs, then the additional dir_pin_readback parameter informs
  // about this pin.
  //
  // Update for idf5 version:
  // The pcnt_unit value is not used, because the available units are managed
  // by the system. The parameter is kept for compatibility.
  //
#if defined(SUPPORT_ESP32_PULSE_COUNTER) && (ESP_IDF_VERSION_MAJOR == 5)
  bool attachToPulseCounter(uint8_t unused_pcnt_unit = 0,
                            int16_t low_value = -16384,
                            int16_t high_value = 16384,
                            uint8_t dir_pin_readback = PIN_UNDEFINED);
  int16_t readPulseCounter();
  void clearPulseCounter();
  inline bool pulseCounterAttached() { return _attached_pulse_unit != NULL; }
#endif
#if defined(SUPPORT_ESP32_PULSE_COUNTER) && (ESP_IDF_VERSION_MAJOR == 4)
  bool attachToPulseCounter(uint8_t pcnt_unit, int16_t low_value = -16384,
                            int16_t high_value = 16384,
                            uint8_t dir_pin_readback = PIN_UNDEFINED);
  int16_t readPulseCounter();
  void clearPulseCounter();
  inline bool pulseCounterAttached() { return _attached_pulse_cnt_unit >= 0; }
#endif

 private:
  void performOneStep(bool count_up, bool blocking = false);
#ifdef SUPPORT_EXTERNAL_DIRECTION_PIN
  bool externalDirPinChangeCompletedIfNeeded();
#endif
  void fill_queue();
  void updateAutoDisable();
  void blockingWaitForForceStopComplete();
  bool needAutoDisable();
  bool agreeWithAutoDisable();
  bool usesAutoEnablePin(uint8_t pin);
  void getCurrentSpeedInTicks(struct actual_ticks_s* speed, bool realtime);

  FastAccelStepperEngine* _engine;
  RampGenerator _rg;
  uint8_t _stepPin;
  uint8_t _dirPin;
  bool _dirHighCountsUp;
  bool _autoEnable;
  uint8_t _enablePinLowActive;
  uint8_t _enablePinHighActive;
  uint8_t _queue_num;

  uint16_t _dir_change_delay_ticks;
  uint32_t _on_delay_ticks;
  uint16_t _off_delay_count;
  uint16_t _auto_disable_delay_counter;

  uint32_t _forward_planning_in_ticks;

#if defined(SUPPORT_ESP32_PULSE_COUNTER) && (ESP_IDF_VERSION_MAJOR == 5)
  pcnt_unit_handle_t _attached_pulse_unit;
#endif
#if defined(SUPPORT_ESP32_PULSE_COUNTER) && (ESP_IDF_VERSION_MAJOR == 4)
  int16_t _attached_pulse_cnt_unit;
#endif
#if (TEST_MEASURE_ISR_SINGLE_FILL == 1)
  uint32_t max_micros;
#endif

  friend class FastAccelStepperEngine;
  friend class FastAccelStepperTest;
};

#endif
