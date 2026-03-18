// pd_esp32/pd_config.h - ESP32 platform configuration
//
// This file defines ESP32-specific constants for the FastAccelStepper library:
// - Queue topology (MAX_STEPPER, NUM_QUEUES, QUEUE_LEN)
// - Timing constants (TICKS_PER_S, MIN_CMD_TICKS, delays)
// - Driver type selection (RMT, MCPWM/PCNT, I2S)
// - Feature flags for ESP32-specific behavior
//
// Includes pd_config_idfX.h for IDF version-specific chip configurations.
// Included by fas_arch/common.h during platform dispatch.

#ifndef PD_ESP32_CONFIG_H
#define PD_ESP32_CONFIG_H

#include <soc/soc_caps.h>

#define SUPPORT_UNSAFE_ABS_SPEED_LIMIT_SETTING

#if ESP_IDF_VERSION_MAJOR == 6
#include "pd_esp32/pd_config_idf6.h"
#elif ESP_IDF_VERSION_MAJOR == 5
#define SUPPORT_DYNAMIC_ALLOCATION
#if ESP_IDF_VERSION_MINOR < 3
#error "FastAccelStepper requires esp-idf >= 5.3.0"
#endif
#include "pd_esp32/pd_config_idf5.h"
#elif ESP_IDF_VERSION_MAJOR == 4
#include "pd_esp32/pd_config_idf4.h"
#elif ESP_IDF_VERSION_MAJOR <= 3
#pragma "Last supported by FastAccelStepper 0.30.15"
#endif

// Esp32 queue definitions
#if defined(SUPPORT_DYNAMIC_ALLOCATION)
#if defined(SUPPORT_ESP32_I2S)
#define QUEUES_I2S_MUX 32
#define QUEUES_I2S_DIRECT 3
#else
#define QUEUES_I2S_MUX 0
#define QUEUES_I2S_DIRECT 0
#endif
#define QUEUES_I2S (QUEUES_I2S_MUX + QUEUES_I2S_DIRECT)
#define NUM_QUEUES \
  (QUEUES_MCPWM_PCNT + QUEUES_RMT + QUEUES_I2S_MUX + QUEUES_I2S_DIRECT)
#else
#if defined(SUPPORT_ESP32_I2S)
#if SOC_I2S_NUM >= 2
#define QUEUES_I2S 64
#else
#define QUEUES_I2S 32
#endif
#else
#define QUEUES_I2S 0
#endif
#define NUM_QUEUES (QUEUES_MCPWM_PCNT + QUEUES_RMT + QUEUES_I2S)
#endif
#define MAX_STEPPER (NUM_QUEUES)
#define QUEUE_LEN 32

// Esp32 timing definition
#define TICKS_PER_S 16000000L
#define MIN_CMD_TICKS (TICKS_PER_S / 5000)
#define MIN_DIR_DELAY_US (MIN_CMD_TICKS / (TICKS_PER_S / 1000000))
#define MAX_DIR_DELAY_US (65535 / (TICKS_PER_S / 1000000))
#define DELAY_MS_BASE 4

#define SUPPORT_QUEUE_ENTRY_START_POS_U16

#define DEBUG_LED_HALF_PERIOD 50

#define noop_or_wait vTaskDelay(1)

#define SUPPORT_CPU_AFFINITY

#define SUPPORT_TASK_RATE_CHANGE

#if defined(SUPPORT_ESP32_I2S)
#define PIN_I2S_FLAG 0x40
#endif

#define LL_TOGGLE_PIN(dirPin)                  \
  gpio_ll_set_level(&GPIO, (gpio_num_t)dirPin, \
                    gpio_ll_get_level(&GPIO, (gpio_num_t)dirPin) ^ 1)

//==========================================================================
// determine, if driver type selection should be supported
// Enable if at least two different driver types have queues
#if (defined(QUEUES_MCPWM_PCNT) && defined(QUEUES_RMT)) || \
    (defined(QUEUES_RMT) && defined(QUEUES_I2S))
#if ((QUEUES_MCPWM_PCNT > 0) && (QUEUES_RMT > 0)) || \
    ((QUEUES_RMT > 0) && (QUEUES_I2S > 0))
#define SUPPORT_SELECT_DRIVER_TYPE
#endif
#endif

#if defined(SUPPORT_SELECT_DRIVER_TYPE)
enum class FasDriver : uint8_t {
#if (defined(QUEUES_MCPWM_PCNT) && (QUEUES_MCPWM_PCNT > 0))
  MCPWM_PCNT = 0,
#endif
#if (defined(QUEUES_RMT) && (QUEUES_RMT > 0))
  RMT = 1,
#endif
#if defined(SUPPORT_ESP32_I2S) && (defined(QUEUES_I2S) && (QUEUES_I2S > 0))
  I2S_DIRECT = 2,
  I2S_MUX = 3,
#endif
  DONT_CARE = 255
};
#endif

#endif /* PD_ESP32_CONFIG_H */
