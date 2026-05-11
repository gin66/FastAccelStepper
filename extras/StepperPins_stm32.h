#ifndef STEPPERPINS_STM32_H
#define STEPPERPINS_STM32_H

// ====================================================================
// Default step pin mapping for STM32 platforms
//
// These defaults map steppers 0-3 to PA0-PA3 (TIM2 channels 1-4).
// Override by defining before including this header:
//
//   #define STEP_PIN_STEPPER_0 PB0
//   #include "StepperPins_stm32.h"
//
// Note: On STM32, any GPIO pin can be used as a step pin.
// PA0-PA3 are only a convention — the timer is used only for
// interrupt timing, not direct pin output.
// ====================================================================

#ifndef STEP_PIN_STEPPER_0
#define STEP_PIN_STEPPER_0 PA0
#endif

#ifndef STEP_PIN_STEPPER_1
#define STEP_PIN_STEPPER_1 PA1
#endif

#ifndef STEP_PIN_STEPPER_2
#define STEP_PIN_STEPPER_2 PA2
#endif

#ifndef STEP_PIN_STEPPER_3
#define STEP_PIN_STEPPER_3 PA3
#endif

#endif /* STEPPERPINS_STM32_H */