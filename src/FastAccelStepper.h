#if defined(ARDUINO_ARCH_AVR)
#include <Arduino.h>
#include <stdint.h>

class FastAccelStepper {
   public:
      // stable API functions
      FastAccelStepper(bool channelA);
      void setDirectionPin(uint8_t dirPin);
      void setEnablePin(uint8_t enablePin);

      inline uint8_t auto_enablePin();
      void set_auto_enable(bool auto_enable);
      void enableOutputs();
      void disableOutputs();

      long getCurrentPosition();
      bool isRunning();
      void move(long move);
      void moveTo(long position);

      // unstable API functions
      void set_dynamics(float speed, float accel);

      inline bool add_queue_entry(uint8_t msb, uint16_t lsw, uint8_t steps, bool dir_high);

      // MUST BE ONLY CALLED FROM THIS MODULES INTERRUPT SERVICE ROUTINE !
      inline void isr_update_move(long remaining_steps);
      unsigned long pos_at_queue_end;    // in steps
      bool dir_high_at_queue_end;        // direction high corresponds to position counting upwards
      uint8_t _dirPin;

   private:
      void _calculate_move(long steps);
      bool _channelA;
      uint8_t _auto_enablePin;
      uint8_t _enablePin;
      float _speed;             // in steps/s
      float _accel;             // in steps/s²

      unsigned long _min_steps;          // in steps

      // used in interrupt routine isr_update_move
      long _last_ms;            // in ms
      unsigned long _deceleration_start; // in steps
      long _dec_time_ms;        // in ms

      // current state
      float _curr_speed;                 // in steps/s
};

class FastAccelStepperEngine {
   public:
      // stable API functions
      void init();
      FastAccelStepper *stepperA();
      FastAccelStepper *stepperB();

      // unstable API functions
      //
      // If this is called, then the periodic task will let the associated LED blink with 1 Hz
      void setDebugLed(uint8_t ledPin);
};
#else
  #error “This library only supports boards with an AVR processor.”
#endif
