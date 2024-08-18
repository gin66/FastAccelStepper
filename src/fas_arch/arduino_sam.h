#ifndef FAS_ARCH_ARDUINO_SAM_H
#define FAS_ARCH_ARDUINO_SAM_H

#define SUPPORT_SAM

// this is an arduino platform, so include the Arduino.h header file
#include <Arduino.h>
// on SAM just use the arduino macros
#define fasEnableInterrupts interrupts
#define fasDisableInterrupts noInterrupts

// queue definitions for SAM
#define MAX_STEPPER 6
#define NUM_QUEUES 6
#define QUEUE_LEN 32

// timing definitions for SAM
#define TICKS_PER_S 21000000L
#define MIN_CMD_TICKS (TICKS_PER_S / 5000)
#define MIN_DIR_DELAY_US (MIN_CMD_TICKS / (TICKS_PER_S / 1000000))
#define MAX_DIR_DELAY_US (65535 / (TICKS_PER_S / 1000000))
#define DELAY_MS_BASE 2

// debug led timing
#define DEBUG_LED_HALF_PERIOD 50

#define noop_or_wait

#define SUPPORT_DIR_PIN_MASK uint32_t

// TO BE CHECKED
#define SUPPORT_QUEUE_ENTRY_END_POS_U16

#endif /* FAS_ARCH_ARDUINO_SAM_H */
