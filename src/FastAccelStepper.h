#if defined(ARDUINO_ARCH_AVR) || defined(ARDUINO_ARCH_ESP32) || defined(TEST)
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

#define TEST_MEASURE_ISR_SINGLE_FILL 0
#define TEST_CREATE_QUEUE_CHECKSUM 0

#if defined(TEST)
#define MAX_STEPPER 2
#endif
#if defined(ARDUINO_ARCH_AVR)
#define MAX_STEPPER 2
#endif
#if defined(ARDUINO_ARCH_ESP32)
#define MAX_STEPPER 6
#endif

#define MIN_DELTA_TICKS (F_CPU / 50000)
#define ABSOLUTE_MAX_TICKS (254L * 16384 + 32767)

class FastAccelStepper {
 public:
  // This should be only called by FastAccelStepperEngine !
  init(uint8_t num, uint8_t step_pin);

  // stable API functions
  void setDirectionPin(uint8_t dirPin);
  void setEnablePin(uint8_t enablePin);
  uint8_t getStepPin();

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
  int addQueueEntry(uint32_t start_delta_ticks, uint8_t steps, bool dir_high,
                    int16_t change_ticks);
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

#if (TEST_MEASURE_ISR_SINGLE_FILL == 1)
  uint32_t max_micros;
#endif
#if (TEST_CREATE_QUEUE_CHECKSUM == 1)
  uint8_t checksum;
#endif

 private:
  uint8_t _stepPin;
  uint8_t _dirPin;
  uint8_t _auto_enablePin;
  uint8_t _enablePin;

  int32_t _pos_at_queue_end;    // in steps
  int32_t _ticks_at_queue_end;  // in timer ticks, 0 on stopped stepper
  bool _dir_high_at_queue_end;  // direction high corresponds to position
                                // counting upwards

  void _calculate_move(int32_t steps);
  void _update_ramp_steps();

  // stepper_num's meaning depends on processor type:
  //		AVR:	0 => OC1B
  //		    	1 => OC1A
  //		ESP32:	0 => MCPWM0 Timer 0
  //				1 => MCPWM0 Timer 1
  //				2 => MCPWM0 Timer 2
  //				3 => MCPWM1 Timer 0
  //				4 => MCPWM1 Timer 1
  //				5 => MCPWM1 Timer 2
  uint8_t _stepper_num;

  uint32_t _min_travel_ticks;  // in ticks, means 0.25us
  uint32_t _ramp_steps;        // ramp steps from 0 to max speed

  // used in interrupt routine isr_update_move
  uint32_t _deceleration_start;  // in steps
  upm_float _upm_inv_accel2;

  uint32_t _performed_ramp_up_steps;
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

#if defined(ARDUINO_ARCH_AVR)
#define stepperA() stepperConnectToPin(9)
#define stepperB() stepperConnectToPin(10)
#endif

  // unstable API functions
  //
  // If this is called, then the periodic task will let the associated LED
  // blink with 1 Hz
  void setDebugLed(uint8_t ledPin);

 private:
  uint8_t _next_stepper_num;
  FastAccelStepper* _stepper[MAX_STEPPER];
};
#else
#error “This library only supports boards with an AVR or ESP32 processor.”
#endif
