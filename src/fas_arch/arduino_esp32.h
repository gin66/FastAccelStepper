#ifndef FAS_ARCH_ARDUINO_ESP32_H
#define FAS_ARCH_ARDUINO_ESP32_H

// this is an arduino platform, so include the Arduino.h header file
#include <Arduino.h>

// For esp32 using arduino, just use arduino definition
#define fasEnableInterrupts interrupts
#define fasDisableInterrupts noInterrupts

#include "fas_arch/common_esp32.h"

#endif /* FAS_ARCH_ARDUINO_ESP32_H */
