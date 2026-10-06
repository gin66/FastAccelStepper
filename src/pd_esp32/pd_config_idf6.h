#ifndef PD_ESP32_CONFIG_IDF6_H
#define PD_ESP32_CONFIG_IDF6_H

//--------------------------------------------------------------------------
// QUEUES_I2S_DIRECT -- one I2S TX channel per I2sManager
//
// What this number does: it sizes the pool. MAX_STEPPER bounds
// `_stepper[MAX_STEPPER]` (FastAccelStepperEngine.h) and fas_queue[NUM_QUEUES]
// (queue_init.cpp), and tryAllocateQueue() refuses past it -- and both of those
// exist under SUPPORT_DYNAMIC_ALLOCATION too, which changes the elements from
// inline queues to pointers but not the array bounds. So the constant is not
// dead weight under dynamic allocation; it is the sum's upper bound.
//
// What it does NOT do: enforce the limit. The i2s_direct path has no
// _i2s_direct_allocated counter, unlike mcpwm_pcnt and rmt which each keep
// one. It asks I2sManager::create(), which returns nullptr when
// i2s_new_channel() fails, and the hardware answers. So overstating this
// number costs RAM (two pointer arrays, 8 B per slot on ESP32) and buys
// nothing -- which is the other half of why the old 3 was wrong.
//
// Stated per variant, and NOT read from I2S_LL_INST_NUM below even though this
// SDK defines it and it is the exact number. The count is a property of the
// silicon and is identical on IDF 5 and IDF 6 -- only the symbol moved
// (SOC_I2S_NUM -> I2S_LL_INST_NUM). Reading it from the SDK buys nothing and
// costs a build that only works on one of them.
//
// Measured, not assumed: on the ESP32, n=1 and n=2 connect and n=3..N are
// refused inside i2s_new_channel() with ESP_ERR_NO_MEM, on IDF 5.5.3 and 6.1.0
// alike.
//--------------------------------------------------------------------------

//==========================================================================
//
// ESP32 derivate - the first one
//
//==========================================================================

#if CONFIG_IDF_TARGET_ESP32
#define SUPPORT_ESP32_MCPWM_PCNT
#define SUPPORT_ESP32_RMT_V2
#define SUPPORT_ESP32_PULSE_COUNTER 8
#define HAVE_ESP32_RMT
#define RMT_SIZE 64

#define QUEUES_MCPWM_PCNT 6
#define QUEUES_I2S_DIRECT 2
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
#define SUPPORT_ESP32_RMT_V2
#define SUPPORT_ESP32_PULSE_COUNTER 4
// #define HAVE_ESP32S3_PULSE_COUNTER
#define HAVE_ESP32_RMT
#define RMT_SIZE 64
#define QUEUES_MCPWM_PCNT 0
#define QUEUES_I2S_DIRECT 1
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
#define SUPPORT_ESP32_RMT_V2
#define SUPPORT_ESP32_PULSE_COUNTER 8
// #define HAVE_ESP32S3_PULSE_COUNTER
#define HAVE_ESP32_RMT
#define RMT_SIZE 48

#define QUEUES_MCPWM_PCNT 4
#define QUEUES_I2S_DIRECT 2
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
#define SUPPORT_ESP32_RMT_V2
#define HAVE_ESP32_RMT
#define RMT_SIZE 48
#define QUEUES_MCPWM_PCNT 0
#define QUEUES_I2S_DIRECT 1
#define QUEUES_RMT 2
#define NEED_RMT_HEADERS

//==========================================================================
//
// ESP32 derivate - ESP32C6
//
//==========================================================================
#elif CONFIG_IDF_TARGET_ESP32C6
#define SUPPORT_ESP32_RMT_V2
#define SUPPORT_ESP32_PULSE_COUNTER 4
#define HAVE_ESP32_RMT
#define RMT_SIZE 48
#define QUEUES_MCPWM_PCNT 0
#define QUEUES_I2S_DIRECT 1
#define QUEUES_RMT 2
#define NEED_RMT_HEADERS
#define NEED_PCNT_HEADERS

//==========================================================================
//
// ESP32 derivate - ESP32H2
//
//==========================================================================
#elif CONFIG_IDF_TARGET_ESP32H2
#define SUPPORT_ESP32_RMT_V2
#define SUPPORT_ESP32_PULSE_COUNTER 4
#define HAVE_ESP32_RMT
#define RMT_SIZE 48
#define QUEUES_MCPWM_PCNT 0
#define QUEUES_I2S_DIRECT 1
#define QUEUES_RMT 2
#define NEED_RMT_HEADERS
#define NEED_PCNT_HEADERS

//==========================================================================
//
// ESP32 derivate - ESP32P4
//
//==========================================================================
#elif CONFIG_IDF_TARGET_ESP32P4
#define SUPPORT_ESP32_RMT_V2
#define SUPPORT_ESP32_PULSE_COUNTER CONFIG_SOC_PCNT_UNITS_PER_GROUP
#define HAVE_ESP32_RMT
#define RMT_SIZE CONFIG_SOC_RMT_MEM_WORDS_PER_CHANNEL
#define QUEUES_MCPWM_PCNT 0
#define QUEUES_I2S_DIRECT 3
#define QUEUES_RMT CONFIG_SOC_RMT_TX_CANDIDATES_PER_GROUP
#define NEED_RMT_HEADERS
#define NEED_PCNT_HEADERS

//==========================================================================
//
// For all unsupported ESP32 derivates
//
//==========================================================================
#else
// cppcheck-suppress preprocessorErrorDirective
#error "Unsupported derivate"
#endif

// #include <driver/periph_ctrl.h>
#include <soc/periph_defs.h>
#include <soc/gpio_sig_map.h>

#ifdef NEED_MCPWM_HEADERS
#include <driver/mcpwm_timer.h>
#include <driver/mcpwm_oper.h>
#include <driver/mcpwm_cmpr.h>
#include <driver/mcpwm_gen.h>
#include <soc/mcpwm_reg.h>
#include <soc/mcpwm_struct.h>
#include <hal/mcpwm_ll.h>
// IDF 6.x removed SOC_MCPWM_GROUPS / SOC_MCPWM_TIMERS_PER_GROUP from soc_caps;
// the per-chip topology now lives in the MCPWM LL layer.
#define SOC_MCPWM_GROUPS MCPWM_LL_GROUP_NUM
#define SOC_MCPWM_TIMERS_PER_GROUP MCPWM_LL_TIMERS_PER_GROUP
#endif

#ifdef NEED_PCNT_HEADERS
#include <driver/pulse_cnt.h>
#include <soc/pcnt_reg.h>
#include <soc/pcnt_struct.h>
#include <hal/pcnt_periph.h>
#include <driver/gpio.h>
#include <hal/gpio_ll.h>
#include <esp_rom_gpio.h>
#endif

#ifdef NEED_RMT_HEADERS
#include <driver/rmt_tx.h>
#include <esp_idf_version.h>
// cppcheck-suppress syntaxError
#if ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(6, 0, 0)
#include <hal/rmt_periph.h>
#else
#include <soc/rmt_periph.h>
#endif
#include <soc/rmt_reg.h>
#include <soc/rmt_struct.h>

#define RMT_CHANNEL_T rmt_channel_handle_t
// #define FAS_RMT_MEM(channel) ((uint32_t *)RMTMEM.chan[channel].data32)
// PART_SIZE shall be even.
#define PART_SIZE (RMT_SIZE >> 1)

// RMT V2 fill model: cap every RMT sub-entry (including the step high) at
// RMT_MAX_SYMBOL_TICKS. A symbol holds TWO sub-entries, so one symbol spans at
// most 2*RMT_MAX_SYMBOL_TICKS and one RMT half (PART_SIZE symbols) at most
// 2*PART_SIZE*RMT_MAX_SYMBOL_TICKS. The divisor is 2*PART_SIZE, not PART_SIZE:
// with PART_SIZE the half covered 2*RMT_BLOCK_TICKS instead of one, which
// doubled the whole buffer and left the direction-change drain (sized from the
// buffer) too short to do its job.
#define RMT_BLOCK_COUNT 2
#define RMT_BLOCK_TICKS 8000
#define RMT_MAX_INFLIGHT_TICKS (RMT_BLOCK_COUNT * RMT_BLOCK_TICKS)
// One RMT half holds at most RMT_BLOCK_TICKS, so the buffer's playback content
// is RMT_BUFFER_TICKS. This is what a pause must exceed to keep a direction
// change behind the pipeline: see esp32_before_pause_ticks().
#define RMT_MAX_SYMBOL_TICKS (RMT_BLOCK_TICKS / (2 * PART_SIZE))
#define RMT_BUFFER_TICKS (2 * PART_SIZE * 2 * RMT_MAX_SYMBOL_TICKS)
// Pause the driver injects before a direction change. It must EXCEED
// RMT_BUFFER_TICKS: the toggle runs at encode time, so a pause that fits
// in the buffer delays it not at all. One RMT_BLOCK_TICKS of margin.
#define RMT_DIR_DRAIN_TICKS (RMT_BUFFER_TICKS + RMT_BLOCK_TICKS)
#endif

// Gated on the per-variant count above, not on I2S_LL_INST_NUM. The two agree
// on this SDK, and the gate is then the same number QUEUES_I2S_DIRECT is
// derived from, so "has I2S" and "how many I2S queues" cannot disagree. The LL
// header is still needed below for the mux's frame timing.
#include <hal/i2s_ll.h>
#if QUEUES_I2S_DIRECT >= 1
#define SUPPORT_ESP32_I2S
#endif

// in order to avoid spikes, first set the value and then make an output
// esp32 idf5 does not like this approach => output first, then value
#define PIN_OUTPUT(pin, value)  \
  {                             \
    pinMode(pin, OUTPUT);       \
    digitalWrite(pin, (value)); \
  }

#endif /* PD_ESP32_CONFIG_IDF6_H */
