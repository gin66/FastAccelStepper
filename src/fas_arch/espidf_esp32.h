#ifndef FAS_ARCH_ESPIDF_ESP32_H
#define FAS_ARCH_ESPIDF_ESP32_H

#define SUPPORT_ESP32
#define SUPPORT_UNSAFE_ABS_SPEED_LIMIT_SETTING 1

#define SUPPORT_ESP32_RMT
#define SUPPORT_ESP32_MCPWM_PCNT

// esp32 specific includes
#include <driver/gpio.h>
#include <driver/mcpwm.h>
#include <driver/pcnt.h>
#include <driver/rmt.h>
#include <esp_task_wdt.h>
#include <math.h>
#include <soc/mcpwm_reg.h>
#include <soc/mcpwm_struct.h>
#include <soc/pcnt_reg.h>
#include <soc/pcnt_struct.h>
#include <soc/rmt_periph.h>
#include <soc/rmt_reg.h>
#include <soc/rmt_struct.h>

// on espidf need to use portDISABLE/ENABLE_INTERRUPTS
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
#define QUEUES_MCPWM_PCNT 6
#define QUEUES_RMT 8
#define QUEUE_LEN 32
#define SUPPORT_EXTERNAL_DIRECTION_PIN

// Esp32 timing definition
#define TICKS_PER_S 16000000L
#define MIN_CMD_TICKS (TICKS_PER_S / 2000)
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

#define noop_or_wait vTaskDelay(1)

#define SUPPORT_QUEUE_ENTRY_START_POS_U16

// have support for pulse counter
#define SUPPORT_ESP32_PULSE_COUNTER

// have more than one core
#define SUPPORT_CPU_AFFINITY

#if defined(ARDUINO_ESP32_RELEASE_3_0_0)
#else
// have support for pulse counter
#define SUPPORT_ESP32_PULSE_COUNTER
#define FAS_RMT_MEM(channel) ((uint32_t *)RMT_CHANNEL_MEM(channel))
#endif

#ifdef __ESP32_IDF_V44__
#include <driver/periph_ctrl.h>
#include <soc/periph_defs.h>
#endif /* __ESP32_IDF_V44__ */

//==========================================================================
// determine, if driver type selection should be supported
#if defined(QUEUES_MCPWM_PCNT) && defined(QUEUES_RMT)
#if (QUEUES_MCPWM_PCNT > 0) && (QUEUES_RMT > 0)
#define SUPPORT_SELECT_DRIVER_TYPE
#endif
#endif

#endif /* FAS_ARCH_ESPIDF_ESP32_H */
