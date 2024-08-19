#ifndef FAS_ARCH_COMMON_ESP32_H
#define FAS_ARCH_COMMON_ESP32_H

#include <sdkconfig.h>

#define SUPPORT_ESP32
#define SUPPORT_EXTERNAL_DIRECTION_PIN
#define SUPPORT_UNSAFE_ABS_SPEED_LIMIT_SETTING 1

// Some more esp32 specific includes
#include <driver/gpio.h>
#include <esp_task_wdt.h>

//==========================================================================
//
// ESP32 derivate - the first one
//
//==========================================================================

#if CONFIG_IDF_TARGET_ESP32
#define SUPPORT_ESP32_MCPWM_PCNT
#define SUPPORT_ESP32_RMT
#define SUPPORT_ESP32_PULSE_COUNTER 8
#define HAVE_ESP31_RMT
#define QUEUES_MCPWM_PCNT 6
#define QUEUES_RMT 8
#define NEED_RMT_HEADERS
#define NEED_MCPWM_HEADERS
#define NEED_PCNT_HEADERS

//==========================================================================
//
// ESP32 derivate - ESP32S2
//
//==========================================================================
#elif CONFIG_IDF_TARGET_ESP32S2
#define SUPPORT_ESP32_RMT
#define SUPPORT_ESP32_PULSE_COUNTER 4
#define HAVE_ESP32S3_PULSE_COUNTER
#define HAVE_ESP32_RMT
#define QUEUES_MCPWM_PCNT 0
#define QUEUES_RMT 4
#define NEED_RMT_HEADERS
#define NEED_PCNT_HEADERS

//==========================================================================
//
// ESP32 derivate - ESP32S3
//
//==========================================================================
#elif CONFIG_IDF_TARGET_ESP32S3
#define SUPPORT_ESP32_MCPWM_PCNT
#define SUPPORT_ESP32_RMT
#define SUPPORT_ESP32_PULSE_COUNTER 8
#define HAVE_ESP32S3_PULSE_COUNTER
#define HAVE_ESP32S3_RMT

#define QUEUES_MCPWM_PCNT 4
#define QUEUES_RMT 4
#define NEED_RMT_HEADERS
#define NEED_MCPWM_HEADERS
#define NEED_PCNT_HEADERS


//==========================================================================
//
// ESP32 derivate - ESP32C3
//
//==========================================================================
#elif CONFIG_IDF_TARGET_ESP32C3
#define SUPPORT_ESP32_RMT
#define HAVE_ESP32C3_RMT
#define QUEUES_MCPWM_PCNT 0
#define QUEUES_RMT 2
#define NEED_RMT_HEADERS

//==========================================================================
//
// For all unsupported ESP32 derivates
//
//==========================================================================
#else
#error "Unsupported derivate"
#endif

#if ESP_IDF_VERSION_MAJOR == 5
#error "Not supported"

#elif ESP_IDF_VERSION_MAJOR == 4

#if ESP_IDF_VERSION_MINOR == 4
#define __ESP32_IDF_V44__
#endif

#include <driver/periph_ctrl.h>
#include <soc/periph_defs.h>

#ifdef NEED_MCPWM_HEADERS
#include <driver/mcpwm.h>
#include <soc/mcpwm_reg.h>
#include <soc/mcpwm_struct.h>
#endif

#ifdef NEED_PCNT_HEADERS
#include <driver/pcnt.h>
#include <soc/pcnt_reg.h>
#include <soc/pcnt_struct.h>
#endif

#ifdef NEED_RMT_HEADERS
#include <driver/rmt.h>
#include <soc/rmt_periph.h>
#include <soc/rmt_reg.h>
#include <soc/rmt_struct.h>

#define FAS_RMT_MEM(channel) ((uint32_t *)RMTMEM.chan[channel].data32)
#endif

#elif ESP_IDF_VERSION_MAJOR == 3

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

//==========================================================================
// determine, if driver type selection should be supported
#if defined(QUEUES_MCPWM_PCNT) && defined(QUEUES_RMT)
#if (QUEUES_MCPWM_PCNT > 0) && (QUEUES_RMT > 0)
#define SUPPORT_SELECT_DRIVER_TYPE
#endif
#endif

#endif /* FAS_ARCH_COMMON_ESP32_H */
