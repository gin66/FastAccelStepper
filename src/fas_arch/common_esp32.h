#ifndef FAS_ARCH_COMMON_ESP32_H
#define FAS_ARCH_COMMON_ESP32_H

#include <sdkconfig.h>

#define SUPPORT_ESP32
#define SUPPORT_EXTERNAL_DIRECTION_PIN
#define SUPPORT_UNSAFE_ABS_SPEED_LIMIT_SETTING 1

// Some more esp32 specific includes
#include <hal/gpio_ll.h>
#include <driver/gpio.h>
#include <esp_task_wdt.h>

#if ESP_IDF_VERSION_MAJOR == 5
#if ESP_IDF_VERSION_MINOR < 3
#error "FastAccelStepper requires esp-idf >= 5.3.0"
#endif
#include "fas_arch/common_esp32_idf5.h"
#elif ESP_IDF_VERSION_MAJOR == 4
#include "fas_arch/common_esp32_idf4.h"
#elif ESP_IDF_VERSION_MAJOR <= 3
#pragma "Last supported by FastAccelStepper 0.30.15"
#endif

// Esp32 queue definitions
#define NUM_QUEUES (QUEUES_MCPWM_PCNT + QUEUES_RMT)
#define MAX_STEPPER (NUM_QUEUES)
#define QUEUE_LEN 32

// Esp32 timing definition
#define TICKS_PER_S 16000000L
#define MIN_CMD_TICKS (TICKS_PER_S / 5000)
#define MIN_DIR_DELAY_US (MIN_CMD_TICKS / (TICKS_PER_S / 1000000))
#define MAX_DIR_DELAY_US (65535 / (TICKS_PER_S / 1000000))
#define DELAY_MS_BASE 4

#define SUPPORT_QUEUE_ENTRY_START_POS_U16

// debug led timing
#define DEBUG_LED_HALF_PERIOD 50

#define noop_or_wait vTaskDelay(1)

// have more than one core
#define SUPPORT_CPU_AFFINITY

// have adjustable stepper task rate
#define SUPPORT_TASK_RATE_CHANGE

#define LL_TOGGLE_PIN(dirPin)                  \
  gpio_ll_set_level(&GPIO, (gpio_num_t)dirPin, \
                    gpio_ll_get_level(&GPIO, (gpio_num_t)dirPin) ^ 1)

//==========================================================================
// determine, if driver type selection should be supported
#if defined(QUEUES_MCPWM_PCNT) && defined(QUEUES_RMT)
#if (QUEUES_MCPWM_PCNT > 0) && (QUEUES_RMT > 0)
#define SUPPORT_SELECT_DRIVER_TYPE
#endif
#endif

#endif /* FAS_ARCH_COMMON_ESP32_H */
