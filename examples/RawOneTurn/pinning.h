#include "FastAccelStepper.h"

#if defined(ARDUINO_ARCH_AVR)
#include "AVRStepperPins.h"
#define vTaskDelay(xx) \
  {                    \
  }
#define stepPinStepper stepPinStepper1A
#define enablePinStepper 6
#define dirPinStepper 5
#elif defined(ARDUINO_ARCH_ESP32)
#define stepPinStepper 17
#define enablePinStepper 26
#define dirPinStepper 18
#elif defined(ARDUINO_ARCH_STM32)
#define stepPinStepper PA0
#define enablePinStepper PA4
#define dirPinStepper PB0
#define vTaskDelay(xx) \
  {                    \
  }
#else
#define vTaskDelay(xx) \
  {                    \
  }
#define stepPinStepper 17
#define enablePinStepper 26
#define dirPinStepper 18
#endif
