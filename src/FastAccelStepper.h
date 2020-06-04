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

// Return codes for add_queue_entry
#define AQE_OK 0
#define AQE_FULL -1
#define AQE_TOO_HIGH -2
#define AQE_TOO_LOW -3
#define AQE_CHANGE_TOO_HIGH -4
#define AQE_CHANGE_TOO_LOW -5
#define AQE_STEPS_ERROR -6

// Limit to 40.000 Steps/s
#define ABSOLUTE_MIN_TICKS (16000000 / 40000)

class FastAccelStepper {
 public:
  // stable API functions
  FastAccelStepper(bool channelA);
  void setDirectionPin(uint8_t dirPin);
  void setEnablePin(uint8_t enablePin);

  void set_auto_enable(bool auto_enable);
  void enableOutputs();
  void disableOutputs();

  int32_t getPositionAfterCommandsCompleted();
  int32_t getCurrentPosition();
  bool isRunning();
  void move(int32_t move);
  void moveTo(int32_t position);

  // unstable API functions
  uint32_t min_delta_ticks();  // this translates into maximum speed

  // stepper queue management (low level access)
  inline int add_queue_entry(uint32_t start_delta_ticks, uint8_t steps,
                             bool dir_high, int16_t change_ticks);
  void add_queue_stepper_stop();
  bool isQueueEmpty();
  bool isQueueFull();
  bool isStopped();

  // For stepper movement control by FastAccelStepper
  void set_dynamics(uint32_t min_travel_ticks, float accel);
  int32_t target_pos;
  bool isr_speed_control_enabled;
  inline void isr_fill_queue();  // MUST BE ONLY CALLED FROM THIS MODULE'S
                                 // INTERRUPT SERVICE ROUTINE !

 private:
  uint8_t _dirPin;
  int32_t _pos_at_queue_end;       // in steps
  int32_t _ticks_at_queue_end;     // in timer ticks, 0 on stopped stepper
  bool _dir_high_at_queue_end;  // direction high corresponds to position
                                // counting upwards

  void _calculate_move(int32_t steps);
  bool _channelA;
  uint8_t _auto_enablePin;
  uint8_t _enablePin;
  float _speed;  // in steps/s
  float _accel;  // in steps/s²

  uint32_t _min_steps;    // in steps
  uint32_t _min_travel_ticks;  // in ticks, means 0.25us
  uint32_t _starting_ticks;    // in ticks, means 0.25us

  // used in interrupt routine isr_update_move
  uint32_t _deceleration_start;  // in steps
  uint32_t _dec_time_ms;                  // in ms
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
