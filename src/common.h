#ifndef COMMON_H
#define COMMON_H

//	ticks is multiplied by (1/TICKS_PER_S) in s
//	If steps is 0, then a pause is generated
struct stepper_command_s {
  uint16_t ticks;
  uint8_t steps;
  bool count_up;
};

struct queue_end_s {
  volatile int32_t pos;  // in steps
  volatile bool count_up;
  volatile bool dir;
};

// use own min/max function, because the lib versions are messed up
#define fas_min(a, b) ((a) > (b) ? (b) : (a))
#define fas_max(a, b) ((a) > (b) ? (a) : (b))

//==============================================================================
// All architecture specific definitions should be located here
//==============================================================================

// disable inject_fill_interrupt() for all real devices
#ifndef TEST
#define inject_fill_interrupt(x)
#endif

//==========================================================================
#if defined(TEST)
// For pc-based testing like to have assert-macro
#include <assert.h>

// and some more includes
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include "../tests/pc_based/stubs.h"

// For pc-based testing, the macro TEST is defined. The pc-based testing does
// not support the concept of interrupts, so provide an empty definition
#define fasEnableInterrupts()
#define fasDisableInterrupts()

// The TEST target needs a couple of arduino like definitions
#define LOW 0
#define HIGH 1

// queue definitions for pc based testing
#define MAX_STEPPER 2
#define NUM_QUEUES 2
#define fas_queue_A fas_queue[0]
#define fas_queue_B fas_queue[1]
#define QUEUE_LEN 16

// timing definitions for pc-based testing
#define TICKS_PER_S 16000000L
#define MIN_DELTA_TICKS (TICKS_PER_S / 50000)
#define MIN_DIR_DELAY_US (MIN_CMD_TICKS / (TICKS_PER_S / 1000000))
#define MAX_DIR_DELAY_US (65535 / (TICKS_PER_S / 1000000))
#define DELAY_MS_BASE 1

// Busy wait
#define yield_if_supported()

//==========================================================================
#elif defined(ARDUINO_ARCH_ESP32)
// this is an arduino platform, so include the Arduino.h header file
#include <Arduino.h>

// Some more esp32 specific includes
#include <driver/gpio.h>
#include <driver/mcpwm.h>
#include <driver/pcnt.h>
#include <esp_task_wdt.h>
//#include <math.h>
#include <soc/mcpwm_reg.h>
#include <soc/mcpwm_struct.h>
#include <soc/pcnt_reg.h>
#include <soc/pcnt_struct.h>

// For esp32 using arduino, just use arduino definition
#define fasEnableInterrupts interrupts
#define fasDisableInterrupts noInterrupts

// Only since esp-idf v4.4 MCPWM_TIMER0_PHASE_DIRECTION_S is defined. So use
// this to distinguish between the two versions
#if defined(MCPWM_TIMER0_PHASE_DIRECTION_S)
#define __ESP32_IDF_V44__
#endif

// Esp32 queue definitions
#define MAX_STEPPER 6
#define NUM_QUEUES 6
#define QUEUE_LEN 32

// Esp32 timing definition
#define TICKS_PER_S 16000000L
#define MIN_DELTA_TICKS (TICKS_PER_S / 200000)
#define MIN_DIR_DELAY_US (MIN_CMD_TICKS / (TICKS_PER_S / 1000000))
#define MAX_DIR_DELAY_US (65535 / (TICKS_PER_S / 1000000))
#define DELAY_MS_BASE 4

// debug led timing
#define DEBUG_LED_HALF_PERIOD 50

// have support for pulse counter
#define SUPPORT_ESP32_PULSE_COUNTER

// have more than one core
#define SUPPORT_CPU_AFFINITY

// esp32 is a multitasking system, so yield for other tasks
#define yield_if_supported() yield()

//==========================================================================
#elif defined(ESP_PLATFORM)
// esp32 specific includes
#include <driver/gpio.h>
#include <driver/mcpwm.h>
#include <driver/pcnt.h>
#include <esp_task_wdt.h>
#include <math.h>
#include <soc/mcpwm_reg.h>
#include <soc/mcpwm_struct.h>
#include <soc/pcnt_reg.h>
#include <soc/pcnt_struct.h>

// on espidf need to use portDISABLE/ENABLE_INTERRUPTS
//
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#define fasDisableInterrupts portDISABLE_INTERRUPTS
#define fasEnableInterrupts portENABLE_INTERRUPTS

// Only since esp-idf v4.4 MCPWM_TIMER0_PHASE_DIRECTION_S is defined. So use
// this to distinguish between the two versions
#if defined(MCPWM_TIMER0_PHASE_DIRECTION_S)
#define __ESP32_IDF_V44__
#endif

// Esp32 queue definitions
#define MAX_STEPPER 6
#define NUM_QUEUES 6
#define QUEUE_LEN 32

// Esp32 timing definition
#define TICKS_PER_S 16000000L
#define MIN_DELTA_TICKS (TICKS_PER_S / 200000)
#define MIN_DIR_DELAY_US (MIN_CMD_TICKS / (TICKS_PER_S / 1000000))
#define MAX_DIR_DELAY_US (65535 / (TICKS_PER_S / 1000000))
#define DELAY_MS_BASE 4

// The espidf-platform needs a couple of arduino like definitions
#define LOW 0
#define HIGH 1
#define OUTPUT GPIO_MODE_OUTPUT
#define pinMode(pin, mode) gpio_set_direction((gpio_num_t)pin, mode)
#define digitalWrite(pin, level) gpio_set_level((gpio_num_t)pin, level)

// debug led timing
#define DEBUG_LED_HALF_PERIOD 50

// have support for pulse counter
#define SUPPORT_ESP32_PULSE_COUNTER

// have more than one core
#define SUPPORT_CPU_AFFINITY

// esp32 is a multitasking system, so yield for other tasks
#define yield_if_supported() yield()

//==========================================================================
#elif defined(ARDUINO_ARCH_SAM)
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
#define MIN_DELTA_TICKS (TICKS_PER_S / 50000)
#define MIN_DIR_DELAY_US (MIN_CMD_TICKS / (TICKS_PER_S / 1000000))
#define MAX_DIR_DELAY_US (65535 / (TICKS_PER_S / 1000000))
#define DELAY_MS_BASE 2

// debug led timing
#define DEBUG_LED_HALF_PERIOD 50

// Busy wait
#define yield_if_supported()

//==========================================================================
#elif defined(ARDUINO_ARCH_AVR)
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
#if defined(__AVR_ATmega328P__)
#define MAX_STEPPER 2
#define NUM_QUEUES 2
#define fas_queue_A fas_queue[0]
#define fas_queue_B fas_queue[1]
enum channels { channelA, channelB };
#elif defined(__AVR_ATmega2560__)
#define MAX_STEPPER 3
#define NUM_QUEUES 3
#define fas_queue_A fas_queue[0]
#define fas_queue_B fas_queue[1]
#define fas_queue_C fas_queue[2]
enum channels { channelA, channelB, channelC };
#elif defined(__AVR_ATmega32U4__)
#define MAX_STEPPER 3
#define NUM_QUEUES 3
#define fas_queue_A fas_queue[0]
#define fas_queue_B fas_queue[1]
#define fas_queue_C fas_queue[2]
enum channels { channelA, channelB, channelC };
#else
#error "Unsupported derivate"
#endif

// AVR:
// tests on arduino nano indicate, that at 40ksteps/s in dual stepper mode,
// the main task is freezing (StepperDemo).
// Thus the limitation set here is set to 25kSteps/s as stated in the README.
#define TICKS_PER_S F_CPU
#define MIN_DELTA_TICKS (TICKS_PER_S / 25000)
#define MIN_DIR_DELAY_US (MIN_DELTA_TICKS / (TICKS_PER_S / 1000000))
#define MAX_DIR_DELAY_US (65535 / (TICKS_PER_S / 1000000))
#define DELAY_MS_BASE (65536000 / TICKS_PER_S)
// debug led timing
#define DEBUG_LED_HALF_PERIOD (TICKS_PER_S / 65536 / 2)

// Busy wait
#define yield_if_supported()

//==========================================================================
#else

// If come here, then the device is not supported
#error "Unsupported derivate"

#endif

#endif /* COMMON_H */
