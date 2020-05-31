#if defined(ARDUINO_ARCH_AVR)
#include <stdint.h>
#include <Arduino.h>

class FastAccelStepper {
   public:
      FastAccelStepper(bool channelA);
      void setDirectionPin(uint8_t dirPin);
      void set_dynamics(float speed, float accel);
      void calculate_move(unsigned long steps);
      unsigned long update_move(unsigned long remaining_steps);

   private:
      bool _channelA;
      uint8_t _dirPin;
      uint8_t _enablePin;
      float _speed;             // in steps/s
      float _accel;             // in steps/s²
      long _last_ms;            // in ms
      long _dec_time_ms;        // in ms

      // target
      unsigned long _min_steps; // in steps
      unsigned long _deceleration_start; // in steps

      // current state
      float _curr_speed;        // in steps/s
};

class FastAccelStepperEngine {
   public:
      FastAccelStepperEngine();
      FastAccelStepper *stepperA(uint8_t dirPin);
      FastAccelStepper *stepperB(uint8_t dirPin);

   private:
      FastAccelStepper _stepperA = FastAccelStepper(true);
      FastAccelStepper _stepperB = FastAccelStepper(false);
};
#else
  #error “This library only supports boards with an AVR processor.”
#endif
