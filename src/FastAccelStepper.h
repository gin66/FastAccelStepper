#if defined(ARDUINO_ARCH_AVR)
#include <Arduino.h>
#include <stdint.h>

class FastAccelStepper {
   public:
      FastAccelStepper(bool channelA);
      void setDirectionPin(uint8_t dirPin);
      void setEnablePin(uint8_t enablePin);
      void set_dynamics(float speed, float accel);
      void calculate_move(long steps);
      void start();

      inline uint8_t auto_enablePin();
      void set_auto_enable(bool auto_enable);

      long current_pos();

      // MUST BE ONLY CALLED FROM THIS MODULES INTERRUPT SERVICE ROUTINE !
      inline void isr_update_move(unsigned long remaining_steps);

   private:
      bool _channelA;
      uint8_t _auto_enablePin;
      uint8_t _dirPin;
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
      FastAccelStepperEngine();
      void init();
      void setDebugLed(uint8_t ledPin);
      FastAccelStepper *stepperA(uint8_t dirPin);
      FastAccelStepper *stepperB(uint8_t dirPin);

   private:
};
#else
  #error “This library only supports boards with an AVR processor.”
#endif
