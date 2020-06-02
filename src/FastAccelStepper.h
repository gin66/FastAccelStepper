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

class FastAccelStepper {
 public:
  // stable API functions
  FastAccelStepper(bool channelA);
  void setDirectionPin(uint8_t dirPin);
  void setEnablePin(uint8_t enablePin);

  void set_auto_enable(bool auto_enable);
  void enableOutputs();
  void disableOutputs();

  long getPositionAfterCommandsCompleted();
  long getCurrentPosition();
  bool isRunning();
  void move(long move);
  void moveTo(long position);

  // unstable API functions
  uint32_t min_delta_ticks();

  // stepper queue management (low level access)
  inline int add_queue_entry(uint32_t start_delta_ticks, uint8_t steps,
                             bool dir_high, int16_t change_ticks);
  bool isQueueEmpty();
  bool isQueueFull();

  // For stepper movement control by FastAccelStepper
  void set_dynamics(float speed, float accel);
  long target_pos;
  bool isr_speed_control_enabled;
  inline void isr_fill_queue();  // MUST BE ONLY CALLED FROM THIS MODULE'S
                                 // INTERRUPT SERVICE ROUTINE !

  long _last_ms;  // in ms

 private:
  uint8_t _dirPin;
  long _pos_at_queue_end;       // in steps
  long _ticks_at_queue_end;     // in timer ticks, 0 on stopped stepper
  bool _dir_high_at_queue_end;  // direction high corresponds to position
                                // counting upwards

  void _calculate_move(long steps);
  bool _channelA;
  uint8_t _auto_enablePin;
  uint8_t _enablePin;
  float _speed;  // in steps/s
  float _accel;  // in steps/s²

  unsigned long _min_steps;  // in steps

  // used in interrupt routine isr_update_move
  unsigned long _deceleration_start;  // in steps
  long _dec_time_ms;                  // in ms

  // current state
  float _curr_speed;  // in steps/s
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
