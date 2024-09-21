#ifndef FAS_ARCH_COMMON_ESP32_IDF5_H
#define FAS_ARCH_COMMON_ESP32_IDF5_H

//==========================================================================
//
// ESP32 derivate - the first one
//
//==========================================================================

#if CONFIG_IDF_TARGET_ESP32
// #define SUPPORT_ESP32_MCPWM_PCNT
#define SUPPORT_ESP32_RMT
#define SUPPORT_ESP32_PULSE_COUNTER 8
#define HAVE_ESP32_RMT
#define RMT_SIZE 64

// #define QUEUES_MCPWM_PCNT 6
#define QUEUES_MCPWM_PCNT 0
#define QUEUES_RMT 8

#define NEED_RMT_HEADERS
// #define NEED_MCPWM_HEADERS
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
#define RMT_SIZE 64
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
// #define SUPPORT_ESP32_MCPWM_PCNT
#define SUPPORT_ESP32_RMT
#define SUPPORT_ESP32_PULSE_COUNTER 8
#define HAVE_ESP32S3_PULSE_COUNTER
#define HAVE_ESP32_RMT
#define RMT_SIZE 48

// #define QUEUES_MCPWM_PCNT 4
#define QUEUES_MCPWM_PCNT 0
#define QUEUES_RMT 4
#define NEED_RMT_HEADERS
// #define NEED_MCPWM_HEADERS
#define NEED_PCNT_HEADERS

//==========================================================================
//
// ESP32 derivate - ESP32C3
//
//==========================================================================
#elif CONFIG_IDF_TARGET_ESP32C3
#define SUPPORT_ESP32_RMT
#define HAVE_ESP32_RMT
#define RMT_SIZE 48
#define QUEUES_MCPWM_PCNT 0
#define QUEUES_RMT 2
#define NEED_RMT_HEADERS

//==========================================================================
//
// ESP32 derivate - ESP32C6
//
//==========================================================================
#elif CONFIG_IDF_TARGET_ESP32C6
#define SUPPORT_ESP32_RMT
#define SUPPORT_ESP32_PULSE_COUNTER 4
#define HAVE_ESP32_RMT
#define RMT_SIZE 48
#define QUEUES_MCPWM_PCNT 0
#define QUEUES_RMT 2
#define NEED_RMT_HEADERS
#define NEED_PCNT_HEADERS

//==========================================================================
//
// ESP32 derivate - ESP32H2
//
//==========================================================================
#elif CONFIG_IDF_TARGET_ESP32H2
#define SUPPORT_ESP32_RMT
#define SUPPORT_ESP32_PULSE_COUNTER 4
#define HAVE_ESP32_RMT
#define RMT_SIZE 48
#define QUEUES_MCPWM_PCNT 0
#define QUEUES_RMT 2
#define NEED_RMT_HEADERS
#define NEED_PCNT_HEADERS

//==========================================================================
//
// For all unsupported ESP32 derivates
//
//==========================================================================
#else
#error "Unsupported derivate"
#endif

// #include <driver/periph_ctrl.h>
#include <soc/periph_defs.h>
#include <soc/gpio_sig_map.h>

#ifdef NEED_MCPWM_HEADERS
#include <driver/mcpwm.h>
#include <soc/mcpwm_reg.h>
#include <soc/mcpwm_struct.h>
#endif

#ifdef NEED_PCNT_HEADERS
#include <driver/pulse_cnt.h>
// #include <soc/pcnt_reg.h>
// #include <soc/pcnt_struct.h>
#include <soc/pcnt_periph.h>
#include <driver/gpio.h>
#include <rom/gpio.h>
#include <hal/gpio_ll.h>
#include <esp_rom_gpio.h>
#endif

#ifdef NEED_RMT_HEADERS
#include <driver/rmt_tx.h>
#include <soc/rmt_periph.h>
#include <soc/rmt_reg.h>
#include <soc/rmt_struct.h>

#define RMT_CHANNEL_T rmt_channel_handle_t
#define FAS_RMT_MEM(channel) ((uint32_t *)RMTMEM.chan[channel].data32)
#endif

// in order to avoid spikes, first set the value and then make an output
// esp32 idf5 does not like this approach => output first, then value
#define PIN_OUTPUT(pin, value)  \
  {                             \
    pinMode(pin, OUTPUT);       \
    digitalWrite(pin, (value)); \
  }

#endif /* FAS_ARCH_COMMON_ESP32_IDF5_H */
