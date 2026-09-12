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

#if defined(SUPPORT_DYNAMIC_ALLOCATION)
  static uint8_t queues_allocated;
#if defined(SUPPORT_SELECT_DRIVER_TYPE)
#ifdef SUPPORT_ESP32_RMT
  static uint8_t _rmt_allocated;
#endif
#ifdef SUPPORT_ESP32_MCPWM_PCNT
  static uint8_t _mcpwm_pcnt_allocated;
#endif
#if defined(SUPPORT_ESP32_I2S)
  static bool _i2s_mux_initialized;
  static uint32_t _i2s_mux_allocated_bitmask;
  static I2sManager* _i2s_mux_manager;
#endif
#endif  // SUPPORT_SELECT_DRIVER_TYPE
#endif  // SUPPORT_DYNAMIC_ALLOCATION

#if defined(SUPPORT_SELECT_DRIVER_TYPE)
  FasDriver _driver_type;
#endif

  volatile bool _isRunning;
  bool _nextCommandIsPrepared;
  uint8_t _step_pin;

  inline void _pd_initVars() {
    _step_pin = PIN_UNDEFINED;
    max_speed_in_ticks = 80;
#if defined(SUPPORT_SELECT_DRIVER_TYPE)
    _driver_type = FasDriver::DONT_CARE;
#endif
  }

  inline bool isRunning() const { return _isRunning; }
  bool isReadyForCommands() const;

  // module specific variables
  union {
#ifdef SUPPORT_ESP32_MCPWM_PCNT
    const void* driver_data;
#endif
#ifdef SUPPORT_ESP32_RMT
    struct {
      RMT_CHANNEL_T channel;
      bool _rmtStopped;
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
  void init_mcpwm_pcnt(uint8_t channel_num, uint8_t step_pin);
  void startQueue_mcpwm_pcnt();
  void forceStop_mcpwm_pcnt();
  uint16_t _getPerformedPulses_mcpwm_pcnt() const;
  void connect_mcpwm_pcnt();
  void disconnect_mcpwm_pcnt();
#endif
#ifdef SUPPORT_ESP32_RMT
  bool isReadyForCommands_rmt() const;
  void init_rmt(uint8_t channel_num, uint8_t step_pin);
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
  void init_i2s(uint8_t step_pin);
  void startQueue_i2s();
  void forceStop_i2s();
  bool isReadyForCommands_i2s() const;
  uint16_t _getPerformedPulses_i2s() const;
  void fill_i2s_buffer(uint8_t* buf);
#endif

  void setDirPin(uint8_t dir_pin, bool _dirHighCountsUp) {
#if defined(SUPPORT_ESP32_I2S) && defined(SUPPORT_SELECT_DRIVER_TYPE)
    if (_driver_type == FasDriver::I2S_DIRECT) {
      dir_pin &= (uint8_t)~PIN_I2S_FLAG;
    }
#endif
    dirPin = dir_pin;
    dirHighCountsUp = _dirHighCountsUp;
#ifdef SUPPORT_ESP32_I2S
    if ((dirPin & PIN_I2S_FLAG)) {
#if defined(SUPPORT_SELECT_DRIVER_TYPE)
      if (_driver_type == FasDriver::I2S_MUX)
#endif
      {
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

#define SET_DIRECTION_PIN_STATE(q, high) \
  esp32_set_direction_pin_state((q), (high))

#define SET_ENABLE_PIN_STATE_NEED_QUEUE
#define SET_ENABLE_PIN_STATE(q, pin, high) \
  esp32_set_enable_pin_state((q), (pin), (high))

#else

#define SET_DIRECTION_PIN_STATE(q, high)                               \
  do {                                                                 \
    gpio_ll_set_level(&GPIO, (gpio_num_t)(q)->dirPin, (high) ? 1 : 0); \
  } while (0)

#define SET_ENABLE_PIN_STATE(q, pin, high)                       \
  do {                                                           \
    gpio_ll_set_level(&GPIO, (gpio_num_t)(pin), (high) ? 1 : 0); \
  } while (0)

#endif  // SUPPORT_ESP32_I2S

//==========================================================================
// DIRECTION CHANGE PAUSE INSERTION
//==========================================================================
//
// Buffered drivers (RMT, I2S) toggle the direction pin while the driver fill
// routine processes the queue entry, i.e. before the content has actually
// reached the output. A direction change therefore needs the output pipeline
// drained first, so that no step in the old direction is emitted after the
// change. The drain pauses are generated one per addQueueEntry() call: such a
// call may insert a single pause command (old direction) and then return
// AQE_DIR_CHANGE_PAUSE_INJECTED; the caller retries until the recorded pause
// state (SUPPORT_PAUSE_CMD_COUNTING) satisfies the driver's requirement.
//
//   driver        drain pauses before dir change     dir change/delay pause
//   RMT (idf4)    1 x MIN_CMD_TICKS                  user dir_change_delay
//   RMT (idf5/6)  2 x MIN_CMD_TICKS                  user dir_change_delay
//   I2S GPIO DIR  1 x 2*I2S_BLOCK_TICKS (old dir)    user dir_change_delay
//                 both DMA blocks must be pause:
//                 GPIO DIR is async at fill time
//   I2S mux DIR   none                               max(I2S_BLOCK_TICKS,
//                 (mask applies to the next block)   user dir_change_delay)
//   MCPWM/PCNT    none (idf4)                        user dir_change_delay
//   MCPWM/PCNT    1 x MIN_CMD_TICKS (idf5+)          user dir_change_delay
//                 Pause commands interrupt at MCPWM
//                 compare (tick 1), so the next
//                 command — and DIR — is applied at
//                 the start of that pause (one-command
//                 pipeline). STEP stays low for the
//                 rest of MIN_CMD_TICKS.
//                 IDF5/6 generator actions latch at
//                 TEZ/TEP. startQueue therefore applies
//                 the first command with immediate
//                 update: leftover utea=2 from the
//                 previous run would otherwise emit a
//                 step at the first TEA of a leading
//                 pause, with DIR still old (#370).
//
// IDF4 vs IDF5 RMT pause count is not a guess: it is the number of PART_SIZE
// halves the hardware fills ahead of the wire. Each queue pause occupies
// exactly one half, so N pauses put a pause in the other half when the
// toggle command is filled — the same slot the 1.2.7 fill-time inject used.
//
//   IDF4 ping-pongs one half per interrupt (and startQueue's double-fill
//   puts the drain in part 2, so the first ISR fill is the toggle while
//   that drain plays). One pause is therefore the same as one inject:
//   last step part -> pause part -> fill that toggles DIR.
//   A second pause shifts the toggle one half later, so DIR no longer
//   sits on the pause that immediately follows the last steps.
//
//   IDF5/6's encoder is invoked two PART_SIZE chunks ahead, so two pauses
//   are required for the same "other half is a pause" condition.
//
// The fill path must not insert pauses of its own: extra ticks would be
// invisible to addQueueEntry()/moveTimed().
//
// Once the pipeline is drained, the pause that carries the direction change is
// inserted (count_up = cmd->count_up). The driver toggles the pin while
// processing that entry and then idles, so the user requested
// dir_change_delay_ticks is honored as part of the direction change.
//
// needed_pauses is RMT-only (each pause occupies one RMT half). I2S uses
// needed_pause_ticks: GPIO DIR needs 2*I2S_BLOCK_TICKS before the change
// (in-flight block and the block being filled must contain no steps). Mux-slot
// DIR (PIN_I2S_FLAG) updates _mux_state for the next block, so the after-pause
// is I2S_BLOCK_TICKS and there is no extra before-pause. IDF5 MCPWM uses
// needed_pause_ticks = MIN_CMD_TICKS: pause TEA at compare=1 applies the
// following command (DIR) at the start of the pause (IDF4 MCPWM does not).
static inline bool esp32_driver_is_rmt(StepperQueue* q) {
#if defined(SUPPORT_SELECT_DRIVER_TYPE)
#if defined(SUPPORT_ESP32_RMT)
  return q->_driver_type == FasDriver::RMT;
#else
  return false;
#endif
#elif defined(SUPPORT_ESP32_RMT)
  return true;
#else
  return false;
#endif
}

static inline bool esp32_driver_is_mcpwm(StepperQueue* q) {
#if defined(SUPPORT_SELECT_DRIVER_TYPE)
#if defined(SUPPORT_ESP32_MCPWM_PCNT)
  return q->_driver_type == FasDriver::MCPWM_PCNT;
#else
  return false;
#endif
#elif defined(SUPPORT_ESP32_MCPWM_PCNT)
  return true;
#else
  return false;
#endif
}

static inline bool esp32_driver_is_i2s(StepperQueue* q) {
#if defined(SUPPORT_SELECT_DRIVER_TYPE)
#if defined(SUPPORT_ESP32_I2S)
  return q->_driver_type == FasDriver::I2S_DIRECT ||
         q->_driver_type == FasDriver::I2S_MUX;
#else
  return false;
#endif
#elif defined(SUPPORT_ESP32_I2S)
  return true;
#else
  return false;
#endif
}

static inline uint8_t esp32_before_pause_count(StepperQueue* q) {
  if (esp32_driver_is_rmt(q)) {
#if defined(SUPPORT_ESP32_RMT_V2)
    return 2;
#else
    return 1;
#endif
  }
  return 0;
}

static inline bool esp32_i2s_dir_is_mux_slot(StepperQueue* q) {
#if defined(SUPPORT_ESP32_I2S)
  if (!esp32_driver_is_i2s(q) || ((q->dirPin & PIN_I2S_FLAG) == 0)) {
    return false;
  }
#if defined(SUPPORT_SELECT_DRIVER_TYPE)
  if (q->_driver_type == FasDriver::I2S_DIRECT) {
    return false;
  }
#endif
  return true;
#else
  (void)q;
  return false;
#endif
}

static inline uint16_t esp32_before_pause_ticks(StepperQueue* q) {
  if (esp32_driver_is_rmt(q)) {
    return MIN_CMD_TICKS;
  }
#if defined(SUPPORT_ESP32_MCPWM_PCNT) && (ESP_IDF_VERSION_MAJOR >= 5)
  if (esp32_driver_is_mcpwm(q)) {
    return MIN_CMD_TICKS;
  }
#endif
#if defined(SUPPORT_ESP32_I2S)
  if (esp32_driver_is_i2s(q) && !esp32_i2s_dir_is_mux_slot(q)) {
    return (uint16_t)(2 * I2S_BLOCK_TICKS);
  }
#endif
  return 0;
}

static inline uint16_t esp32_after_pause_ticks(StepperQueue* q) {
#if defined(SUPPORT_ESP32_I2S)
  if (esp32_driver_is_i2s(q) && esp32_i2s_dir_is_mux_slot(q)) {
    return I2S_BLOCK_TICKS;
  }
#else
  (void)q;
#endif
  return 0;
}

static inline AqeResultCode esp32_enqueue_pause(StepperQueue* q,
                                                uint16_t pause_ticks,
                                                bool start, bool count_up) {
  struct stepper_command_s pause_cmd = {
      .ticks = pause_ticks, .steps = 0, .count_up = count_up};
  return q->addQueueEntry(&pause_cmd, start);
}

inline AqeResultCode StepperQueue::addDirChangePauseToQueue(
    const struct stepper_command_s* cmd, bool start,
    uint16_t dir_change_delay_ticks) {
  uint8_t needed_pauses = esp32_before_pause_count(this);
  uint16_t needed_pause_ticks = esp32_before_pause_ticks(this);
  if ((_nr_of_pauses < needed_pauses) ||
      (_last_pause_ticks < needed_pause_ticks)) {
    AqeResultCode res = esp32_enqueue_pause(this, needed_pause_ticks, start,
                                            queue_end.count_up);
    if (res != AQE_OK) {
      return res;
    }
    return AQE_DIR_CHANGE_PAUSE_INJECTED;
  }
  // Before-pause requirement is met. Enforce the direction-change/delay pause.
  uint16_t delay_ticks =
      (uint16_t)fas_max(dir_change_delay_ticks, esp32_after_pause_ticks(this));
  bool needs_toggle_pause = false;
  if (cmd->steps == 0) {
    if ((delay_ticks != 0) && (cmd->ticks < delay_ticks)) {
      needs_toggle_pause = true;
      delay_ticks = (uint16_t)fas_max(delay_ticks - cmd->ticks, MIN_CMD_TICKS);
    }
  } else if (delay_ticks != 0) {
    needs_toggle_pause = true;
    delay_ticks = (uint16_t)fas_max(delay_ticks, MIN_CMD_TICKS);
  }
  if (!needs_toggle_pause) {
    return AQE_OK;
  }
  AqeResultCode res =
      esp32_enqueue_pause(this, delay_ticks, start, cmd->count_up);
  if (res != AQE_OK) {
    return res;
  }
  return AQE_DIR_CHANGE_PAUSE_INJECTED;
}

#endif
