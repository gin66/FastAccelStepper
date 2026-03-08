#ifndef FAS_ARCH_ARDUINO_SAM_H
#define FAS_ARCH_ARDUINO_SAM_H

#define SUPPORT_SAM

// this is an arduino platform, so include the Arduino.h header file
#include <Arduino.h>
// on SAM just use the arduino macros
#define fasEnableInterrupts interrupts
#define fasDisableInterrupts noInterrupts

#endif /* FAS_ARCH_ARDUINO_SAM_H */
