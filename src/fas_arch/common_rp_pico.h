#ifndef FAS_ARCH_COMMON_RP_PICO_H
#define FAS_ARCH_COMMON_RP_PICO_H

#include <FreeRTOS.h>
#include <task.h>

#define SUPPORT_RP_PICO
#define SUPPORT_EXTERNAL_DIRECTION_PIN
#define SUPPORT_UNSAFE_ABS_SPEED_LIMIT_SETTING 1
// #define SUPPORT_QUEUE_ENTRY_START_POS_U16

// pico queue definitions
#define NUM_QUEUES 4 * NUM_PIOS
#define MAX_STEPPER (NUM_QUEUES)
#define QUEUE_LEN 32

// Pico timing definition. We use FastAccelStepper standard and recalculate to
// pico system clock.
#define TICKS_PER_S 16000000L
#define MIN_CMD_TICKS (TICKS_PER_S / 5000)
#define MIN_DIR_DELAY_US (MIN_CMD_TICKS / (TICKS_PER_S / 1000000))
#define MAX_DIR_DELAY_US (65535 / (TICKS_PER_S / 1000000))
#define DELAY_MS_BASE 4

// debug led timing
#define DEBUG_LED_HALF_PERIOD 50

#define noop_or_wait vTaskDelay(1)

// have more than one core
// #define SUPPORT_CPU_AFFINITY

// have adjustable stepper task rate
#define SUPPORT_TASK_RATE_CHANGE

//#define LL_TOGGLE_PIN(dirPin)                  \
//  gpio_ll_set_level(&GPIO, (gpio_num_t)dirPin, \
//                    gpio_ll_get_level(&GPIO, (gpio_num_t)dirPin) ^ 1)

#endif /* FAS_ARCH_COMMON_RP_PICO_H */
