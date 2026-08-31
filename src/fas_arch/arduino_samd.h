#ifndef FAS_ARCH_ARDUINO_SAMD_H
#define FAS_ARCH_ARDUINO_SAMD_H

// SAMD51 only. SAMD21 differs in TCC clocking/sync and instance count and is
// not implemented (see fas_arch/common.h dispatch).
#define SUPPORT_SAMD51

// this is an arduino platform, so include the Arduino.h header file
#include <Arduino.h>

// on SAMD just use the arduino macros
#define fasEnableInterrupts interrupts
#define fasDisableInterrupts noInterrupts

#endif /* FAS_ARCH_ARDUINO_SAMD_H */
