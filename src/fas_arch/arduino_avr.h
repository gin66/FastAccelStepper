#ifndef FAS_ARCH_ARDUINO_AVR_H
#define FAS_ARCH_ARDUINO_AVR_H

#define SUPPORT_AVR
#define SUPPORT_UNSAFE_ABS_SPEED_LIMIT_SETTING 1

// this is an arduino platform, so include the Arduino.h header file
#include <Arduino.h>

#include "AVRStepperPins.h"
// for AVR processors a reentrant version of disabling/enabling interrupts is
// used
#define fasDisableInterrupts() \
  uint8_t prevSREG = SREG;     \
  cli()
#define fasEnableInterrupts() SREG = prevSREG

// Here are shorthand definitions for number of queues, the queues/channel
// relation and queue length This definitions are derivate specific
#define QUEUE_LEN 16
#define TICKS_PER_S F_CPU
#define MIN_CMD_TICKS (TICKS_PER_S / 25000)
#define MIN_DIR_DELAY_US 40
#define MAX_DIR_DELAY_US (65535 / (TICKS_PER_S / 1000000))
#define DELAY_MS_BASE (65536000 / TICKS_PER_S)

// debug led timing
#define DEBUG_LED_HALF_PERIOD (TICKS_PER_S / 65536 / 2)

#define noop_or_wait

#define SUPPORT_DIR_TOGGLE_PIN_MASK uint8_t

#define SUPPORT_QUEUE_ENTRY_END_POS_U16

//==========================================================================
//
// AVR derivate ATmega 168/328/P
//
//==========================================================================
#if (defined(__AVR_ATmega168__) || defined(__AVR_ATmega168P__) || \
     defined(__AVR_ATmega328__) || defined(__AVR_ATmega328P__))
#define SUPPORT_EXTERNAL_DIRECTION_PIN
#define MAX_STEPPER 2
#define NUM_QUEUES 2
#define fas_queue_A fas_queue[0]
#define fas_queue_B fas_queue[1]
enum channels { channelA, channelB };
//==========================================================================
//
// AVR derivate ATmega 2560
//
//==========================================================================
#elif defined(__AVR_ATmega2560__)
#define SUPPORT_EXTERNAL_DIRECTION_PIN
#define MAX_STEPPER 3
#define NUM_QUEUES 3
#define fas_queue_A fas_queue[0]
#define fas_queue_B fas_queue[1]
#define fas_queue_C fas_queue[2]
enum channels { channelA, channelB, channelC };
//==========================================================================
//
// AVR derivate ATmega 32U4
//
//==========================================================================
#elif defined(__AVR_ATmega32U4__)
#define MAX_STEPPER 3
#define NUM_QUEUES 3
#define fas_queue_A fas_queue[0]
#define fas_queue_B fas_queue[1]
#define fas_queue_C fas_queue[2]
enum channels { channelA, channelB, channelC };
//==========================================================================
//
// For all unsupported AVR derivates
//
//==========================================================================
#else
#error "Unsupported AVR derivate"
#endif

#endif /* FAS_ARCH_ARDUINO_AVR_H */
