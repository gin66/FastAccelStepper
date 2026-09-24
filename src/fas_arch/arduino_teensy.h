#ifndef FAS_ARCH_ARDUINO_TEENSY_H
#define FAS_ARCH_ARDUINO_TEENSY_H

// Teensy 4.0/4.1 (i.MX RT1062, ARM Cortex-M7), via Teensyduino.
// EXPERIMENTAL - see pd_teensy/pd_config.h and pd_teensy/teensy_queue.cpp.
#define SUPPORT_TEENSY4

// this is an arduino platform, so include the Arduino.h header file
#include <Arduino.h>

#define fasEnableInterrupts interrupts
#define fasDisableInterrupts noInterrupts

#endif /* FAS_ARCH_ARDUINO_TEENSY_H */
