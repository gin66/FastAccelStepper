#ifndef FASTACCELSTEPPER_ENGINE_H
#define FASTACCELSTEPPER_ENGINE_H
#include <stdint.h>
#include "fas_arch/common.h"

class FastAccelStepper;

class FastAccelStepperEngine {
 public:
#if defined(SUPPORT_CPU_AFFINITY)
  void init(uint8_t cpu_core = 255);
#else
  void init();
#endif

#if defined(SUPPORT_SELECT_DRIVER_TYPE)
#define DRIVER_MCPWM_PCNT FasDriver::MCPWM_PCNT
#define DRIVER_RMT FasDriver::RMT
#if defined(SUPPORT_ESP32_I2S)
#define DRIVER_I2S_DIRECT FasDriver::I2S_DIRECT
#define DRIVER_I2S_MUX FasDriver::I2S_MUX
#endif
#define DRIVER_DONT_CARE FasDriver::DONT_CARE
  FastAccelStepper* stepperConnectToPin(
      uint8_t step_pin, FasDriver driver_type = DRIVER_DONT_CARE);
#else
  FastAccelStepper* stepperConnectToPin(uint8_t step_pin);
#endif

#if defined(SUPPORT_TASK_RATE_CHANGE)
  inline void task_rate(uint8_t delay_ms) { _delay_ms = delay_ms; };
  uint8_t _delay_ms;
#endif

  void setExternalCallForPin(bool (*func)(uint8_t pin, uint8_t value));
  void setDebugLed(uint8_t ledPin);

  void manageSteppers();

 private:
  bool isDirPinBusy(uint8_t dirPin, uint8_t except_stepper);

  uint8_t _stepper_cnt;
  FastAccelStepper* _stepper[MAX_STEPPER];

  bool (*_externalCallForPin)(uint8_t pin, uint8_t value);

#if defined(SUPPORT_RP_PICO)
  uint8_t claimed_pios;
  PIO pio[NUM_PIOS];
#endif

  friend class FastAccelStepper;
  friend class StepperQueue;
};

#endif
