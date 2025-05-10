#ifndef FAS_ARCH_ARDUINO_RP_PICO_H
#define FAS_ARCH_ARDUINO_RP_PICO_H

// this is an arduino platform, so include the Arduino.h header file
#include <Arduino.h>

// For pico using arduino, just use arduino definition
#define fasEnableInterrupts interrupts
#define fasDisableInterrupts noInterrupts

#include "fas_arch/common_rp_pico.h"

#endif /* FAS_ARCH_ARDUINO_RP_PICO_H */
