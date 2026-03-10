#ifndef PD_ESP32_QUEUE_H
#define PD_ESP32_QUEUE_H

#include "FastAccelStepper.h"
#include "fas_queue/base.h"

#include <hal/gpio_ll.h>

#if defined(SUPPORT_ESP32_I2S)
#include "pd_esp32/i2s_fill.h"
#include "pd_esp32/i2s_manager.h"
#endif

class StepperQueue : public StepperQueueBase {
 public:
#include "../fas_queue/protocol.h"

#if defined(SUPPORT_SELECT_DRIVER_TYPE)
#if defined(SUPPORT_DYNAMIC_ALLOCATION)
  // dynamic allocation only for espidf >=5.3, so no mcpwm/pcnt
  static uint8_t queues_allocated;
#ifdef SUPPORT_ESP32_RMT
  static uint8_t _rmt_allocated;
#endif
#if defined(SUPPORT_ESP32_I2S)
  static bool _i2s_mux_initialized;
  static uint32_t _i2s_mux_allocated_bitmask;
  static I2sManager* _i2s_mux_manager;
#endif
  static void initVars() {
    StepperQueue::queues_allocated = 0;
#ifdef SUPPORT_ESP32_RMT
    StepperQueue::_rmt_allocated = 0;
#endif
#if defined(SUPPORT_ESP32_I2S)
    StepperQueue::_i2s_mux_initialized = false;
    StepperQueue::_i2s_mux_allocated_bitmask = 0;
    StepperQueue::_i2s_mux_manager = nullptr;
#endif
  }
#endif
#elif defined(SUPPORT_DYNAMIC_ALLOCATION)
  static uint8_t queues_allocated;
  static void initVars() { StepperQueue::queues_allocated = 0; }
#endif  // SUPPORT_SELECT_DRIVER_TYPE

  volatile bool _isRunning;
  bool _nextCommandIsPrepared;
  uint8_t _step_pin;

  inline void _pd_initVars() {
    _isRunning = false;
    _nextCommandIsPrepared = false;
  }

  inline bool isRunning() const { return _isRunning; }
  bool isReadyForCommands() const;
#ifdef SUPPORT_ESP32_RMT
  bool use_rmt;
#endif
#ifdef SUPPORT_ESP32_I2S
  bool use_i2s;
#endif
#ifdef SUPPORT_ESP32_MCPWM_PCNT
  bool use_mcpwm_pcnt;
#endif

  // module specific variables
  union {
#ifdef SUPPORT_ESP32_MCPWM_PCNT
    const void* driver_data;
#endif
#ifdef SUPPORT_ESP32_RMT
    struct {
      RMT_CHANNEL_T channel;
      bool _rmtStopped;
      bool lastChunkContainsSteps;
#if defined(SUPPORT_ESP32_RMT_V2)
      rmt_encoder_handle_t _tx_encoder;
#if ESP_IDF_VERSION_MAJOR >= 5
      bool _channel_enabled;
#endif
#endif
    };
#endif
#ifdef SUPPORT_ESP32_I2S
    struct {
      struct i2s_fill_state _fill_state;
      I2sManager* i2s_mgr;
      uint8_t _i2s_mux_step_byte_offset;
      uint8_t _i2s_mux_step_bit_mask;
    };
#endif
  };

  uint16_t _getPerformedPulses() const;

  // Module specific functions
#ifdef SUPPORT_ESP32_MCPWM_PCNT
  bool isReadyForCommands_mcpwm_pcnt() const;
  bool init_mcpwm_pcnt(uint8_t channel_num, uint8_t step_pin);
  void startQueue_mcpwm_pcnt();
  void forceStop_mcpwm_pcnt();
  uint16_t _getPerformedPulses_mcpwm_pcnt() const;
  void connect_mcpwm_pcnt();
  void disconnect_mcpwm_pcnt();
#endif
#ifdef SUPPORT_ESP32_RMT
  bool isReadyForCommands_rmt() const;
  bool init_rmt(uint8_t channel_num, uint8_t step_pin);
  void startQueue_rmt();
#if ESP_IDF_VERSION_MAJOR == 4
  void stop_rmt(bool both);
#endif
  void forceStop_rmt();
  uint16_t _getPerformedPulses_rmt() const;
  void connect_rmt();
  void disconnect_rmt();
#endif
#ifdef SUPPORT_ESP32_I2S
  bool init_i2s(uint8_t step_pin);
  void startQueue_i2s();
  void forceStop_i2s();
  bool isReadyForCommands_i2s() const;
  uint16_t _getPerformedPulses_i2s() const;
  void fill_i2s_buffer(uint8_t* buf);
#endif

  void setDirPin(uint8_t dir_pin, bool _dirHighCountsUp) {
    dirPin = dir_pin;
    dirHighCountsUp = _dirHighCountsUp;
#ifdef SUPPORT_ESP32_I2S
    if ((dirPin & PIN_I2S_FLAG) && use_i2s && i2s_mgr->_is_mux) {
      uint8_t slot = dirPin & 0x1F;
      if (slot >= 32) {
        return;
      }
      uint32_t bit = 1UL << slot;
      if (_i2s_mux_allocated_bitmask & bit) {
        return;
      }
      _i2s_mux_allocated_bitmask |= bit;
      if (_i2s_mux_manager != nullptr) {
        _i2s_mux_manager->i2sMuxSetBit(slot, _dirHighCountsUp);
      }
    }
#endif
  }

 private:
  static bool isValidStepPin(uint8_t step_pin);
};

#if defined(SUPPORT_ESP32_RMT)
void rmt_fill_buffer(StepperQueue* q, bool fill_part_one, uint32_t* data);
void rmt_apply_command(StepperQueue* q, bool fill_part_one, uint32_t* data);
#endif

//==========================================================================
// ESP32 PROTOCOL MACROS AND HELPER FUNCTIONS
//==========================================================================

#if defined(SUPPORT_ESP32_I2S)
static inline void esp32_set_enable_pin_state(StepperQueue* q, uint8_t pin,
                                              bool high) {
  if (pin & PIN_I2S_FLAG) {
    uint8_t slot = pin & 0x1F;
    if (StepperQueue::_i2s_mux_manager != nullptr) {
      StepperQueue::_i2s_mux_manager->i2sMuxSetBit(slot, high);
    }
  } else {
    gpio_ll_set_level(&GPIO, (gpio_num_t)pin, high ? 1 : 0);
  }
}

static inline void esp32_set_direction_pin_state(StepperQueue* q, bool high) {
  if (q->dirPin & PIN_I2S_FLAG) {
    uint8_t slot = q->dirPin & 0x1F;
    if (StepperQueue::_i2s_mux_manager != nullptr) {
      StepperQueue::_i2s_mux_manager->i2sMuxSetBit(slot, high);
    }
  } else {
    gpio_ll_set_level(&GPIO, (gpio_num_t)q->dirPin, high ? 1 : 0);
  }
}

static inline uint16_t esp32_before_dir_change_delay_ticks(StepperQueue* q) {
#if defined(SUPPORT_SELECT_DRIVER_TYPE)
  if (q->use_rmt) return MIN_CMD_TICKS;
  if (q->use_i2s) return I2S_BLOCK_TICKS;
  return 0;
#elif defined(SUPPORT_ESP32_RMT)
  return MIN_CMD_TICKS;
#elif defined(SUPPORT_ESP32_I2S)
  return I2S_BLOCK_TICKS;
#else
  return 0;
#endif
}

static inline uint16_t esp32_after_dir_change_delay_ticks(StepperQueue* q) {
#if defined(SUPPORT_ESP32_I2S)
  return q->use_i2s ? I2S_BLOCK_TICKS : 0;
#else
  return 0;
#endif
}

#define SET_DIRECTION_PIN_STATE(q, high) \
  esp32_set_direction_pin_state((q), (high))

#define SET_ENABLE_PIN_STATE_NEED_QUEUE
#define SET_ENABLE_PIN_STATE(q, pin, high) \
  esp32_set_enable_pin_state((q), (pin), (high))
#define BEFORE_DIR_CHANGE_DELAY_TICKS(q) \
  esp32_before_dir_change_delay_ticks((q))
#define AFTER_DIR_CHANGE_DELAY_TICKS(q) esp32_after_dir_change_delay_ticks((q))

#else

#define SET_DIRECTION_PIN_STATE(q, high)                               \
  do {                                                                 \
    gpio_ll_set_level(&GPIO, (gpio_num_t)(q)->dirPin, (high) ? 1 : 0); \
  } while (0)

#define SET_ENABLE_PIN_STATE(q, pin, high)                       \
  do {                                                           \
    gpio_ll_set_level(&GPIO, (gpio_num_t)(pin), (high) ? 1 : 0); \
  } while (0)

#if defined(SUPPORT_ESP32_RMT)
#define BEFORE_DIR_CHANGE_DELAY_TICKS(q) (MIN_CMD_TICKS)
#endif

#endif  // SUPPORT_ESP32_I2S

#endif
