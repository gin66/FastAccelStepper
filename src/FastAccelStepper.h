#if defined(ARDUINO_ARCH_AVR) || defined(TEST)
#ifndef TEST
#include <Arduino.h>
#else
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include "stubs.h"
#endif
#include <stdint.h>
#include "PoorManFloat.h"

#define MIN_DELTA_TICKS (16000000L / 50000)
#define ABSOLUTE_MAX_TICKS (254L * 16384 + 32767)

class FastAccelStepper {
 public:
  // stable API functions
  FastAccelStepper(bool channelA);
  void setDirectionPin(uint8_t dirPin);
  void setEnablePin(uint8_t enablePin);

  void setAutoEnable(bool auto_enable);
  void enableOutputs();
  void disableOutputs();

  int32_t getPositionAfterCommandsCompleted();
  int32_t getCurrentPosition();
  bool isRunning();
  void move(int32_t move);
  void moveTo(int32_t position);

  // unstable API functions

  // stepper queue management (low level access)
  inline int addQueueEntry(uint32_t start_delta_ticks, uint8_t steps,
                           bool dir_high, int16_t change_ticks);
  // Return codes for add_queue_entry
#define AQE_OK 0
#define AQE_FULL -1
#define AQE_TOO_HIGH -2
#define AQE_TOO_LOW -3
#define AQE_CHANGE_TOO_HIGH -4
#define AQE_CHANGE_TOO_LOW -5
#define AQE_CUMULATED_CHANGE_TOO_LOW -6
#define AQE_STEPS_ERROR -7

  void addQueueStepperStop();
  bool isQueueEmpty();
  bool isQueueFull();
  bool isStopped();

  // For stepper movement control by FastAccelStepper
  //
  // setSpeed expects as parameter the minimum time between two steps.
  // If for example 5 steps/s shall be the maximum speed of the stepper,
  // then this call will be
  //      t = 0.2 s/steps = 200000 us/step
  //      setSpeed(200000);
  //
  void setSpeed(uint32_t min_step_us);
  //  set Acceleration expects as parameter the change of speed
  //  as step/s².
  //  If for example the speed should ramp up from 0 to 10000 steps/s within
  //  10s, then the acceleration is 10000 steps/s / 10s = 1000 steps/s²
  //
  void setAcceleration(uint32_t step_s_s);

#define RAMP_STATE_IDLE 0
#define RAMP_STATE_ACCELERATE 1
#define RAMP_STATE_DECELERATE_TO_STOP 2
#define RAMP_STATE_DECELERATE 3
#define RAMP_STATE_COAST 4
  uint8_t ramp_state;  // updated by isr_fill_queue
  int32_t target_pos;
  bool isr_speed_control_enabled;
  inline void isr_fill_queue();  // MUST BE ONLY CALLED FROM THIS MODULE'S
                                 // INTERRUPT SERVICE ROUTINE !
  inline void isr_single_fill_queue();  // MUST BE ONLY CALLED FROM THIS
                                        // MODULE'S INTERRUPT SERVICE ROUTINE !

  uint32_t max_micros;

 private:
  uint8_t _dirPin;
  int32_t _pos_at_queue_end;    // in steps
  int32_t _ticks_at_queue_end;  // in timer ticks, 0 on stopped stepper
  bool _dir_high_at_queue_end;  // direction high corresponds to position
                                // counting upwards

  void _calculate_move(int32_t steps);
  bool _channelA;
  uint8_t _auto_enablePin;
  uint8_t _enablePin;

  uint32_t _min_travel_ticks;  // in ticks, means 0.25us

  // used in interrupt routine isr_update_move
  uint32_t _deceleration_start;  // in steps
  upm_float _upm_inv_accel2;

  uint32_t _performed_ramp_up_steps;
};

class FastAccelStepperEngine {
 public:
  // stable API functions
  void init();
  FastAccelStepper* stepperA();
  FastAccelStepper* stepperB();

  // unstable API functions
  //
  // If this is called, then the periodic task will let the associated LED
  // blink with 1 Hz
  void setDebugLed(uint8_t ledPin);
};
#else
#error “This library only supports boards with an AVR processor.”
#endif
