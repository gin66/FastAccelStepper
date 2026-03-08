#ifndef FAS_ARCH_ARDUINO_AVR_H
#define FAS_ARCH_ARDUINO_AVR_H

#define SUPPORT_AVR

// this is an arduino platform, so include the Arduino.h header file
#include <Arduino.h>

#include "AVRStepperPins.h"
// for AVR processors a reentrant version of disabling/enabling interrupts is
// used
#define fasDisableInterrupts() \
  uint8_t prevSREG = SREG;     \
  cli()
#define fasEnableInterrupts() SREG = prevSREG

#endif /* FAS_ARCH_ARDUINO_AVR_H */
