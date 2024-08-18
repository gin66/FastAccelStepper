#ifndef FAS_ARCH_ARDUINO_ESP32_H
#define FAS_ARCH_ARDUINO_ESP32_H

// this is an arduino platform, so include the Arduino.h header file
#include <Arduino.h>
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
#include <driver/mcpwm.h>
#include <driver/pcnt.h>
#include <soc/mcpwm_reg.h>
#include <soc/mcpwm_struct.h>
#include <soc/pcnt_reg.h>
#include <soc/pcnt_struct.h>

#define SUPPORT_ESP32_MCPWM_PCNT
#define SUPPORT_ESP32_RMT
#include <driver/rmt.h>
#define QUEUES_MCPWM_PCNT 6
#define QUEUES_RMT 8

// have support for pulse counter
#define SUPPORT_ESP32_PULSE_COUNTER
#define FAS_RMT_MEM(channel) ((uint32_t *)RMT_CHANNEL_MEM(channel))

//==========================================================================
//
// ESP32 derivate - ESP32S2
//
//==========================================================================
#elif CONFIG_IDF_TARGET_ESP32S2
#define SUPPORT_ESP32S3_PULSE_COUNTER
#define SUPPORT_ESP32_RMT
#include <driver/pcnt.h>
#include <driver/periph_ctrl.h>
#include <driver/rmt.h>
#include <soc/pcnt_reg.h>
#include <soc/pcnt_struct.h>
#define QUEUES_MCPWM_PCNT 0
#define QUEUES_RMT 4
#define FAS_RMT_MEM(channel) ((uint32_t *)RMTMEM.chan[channel].data32)

//==========================================================================
//
// ESP32 derivate - ESP32S3
//
//==========================================================================
#elif CONFIG_IDF_TARGET_ESP32S3
#define SUPPORT_ESP32_MCPWM_PCNT
#define SUPPORT_ESP32S3_MCPWM_PCNT
#include <driver/mcpwm.h>
#include <driver/pcnt.h>
#include <soc/mcpwm_reg.h>
#include <soc/mcpwm_struct.h>
#include <soc/pcnt_reg.h>
#include <soc/pcnt_struct.h>

#define SUPPORT_ESP32_RMT
#define SUPPORT_ESP32S3_RMT
#include <driver/periph_ctrl.h>
#include <driver/rmt.h>
#include <soc/rmt_periph.h>
#include <soc/rmt_reg.h>
#include <soc/rmt_struct.h>
#define FAS_RMT_MEM(channel) ((uint32_t *)RMTMEM.chan[channel].data32)

#define QUEUES_MCPWM_PCNT 4
#define QUEUES_RMT 4

// have support for pulse counter
#define SUPPORT_ESP32_PULSE_COUNTER

//==========================================================================
//
// ESP32 derivate - ESP32C3
//
//==========================================================================
#elif CONFIG_IDF_TARGET_ESP32C3
#define SUPPORT_ESP32_RMT
#define SUPPORT_ESP32C3_RMT
#include <driver/periph_ctrl.h>
#include <driver/rmt.h>
#include <soc/rmt_periph.h>
#include <soc/rmt_reg.h>
#include <soc/rmt_struct.h>
#define QUEUES_MCPWM_PCNT 0
#define QUEUES_RMT 2
#define FAS_RMT_MEM(channel) ((uint32_t *)RMTMEM.chan[channel].data32)

//==========================================================================
//
// For all unsupported ESP32 derivates
//
//==========================================================================
#else
#error "Unsupported derivate"
#endif

// For esp32 using arduino, just use arduino definition
#define fasEnableInterrupts interrupts
#define fasDisableInterrupts noInterrupts

#if ESP_IDF_VERSION_MAJOR == 4
#define __ESP32_IDF_V44__
#include <driver/periph_ctrl.h>
#include <soc/periph_defs.h>
#include <soc/rmt_periph.h>
#include <soc/rmt_reg.h>
#include <soc/rmt_struct.h>
#elif ESP_IDF_VERSION_MAJOR == 3
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

#endif /* FAS_ARCH_ARDUINO_ESP32_H */
