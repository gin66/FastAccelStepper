#ifndef PD_ESP32_QUEUE_H
#define PD_ESP32_QUEUE_H

#include "FastAccelStepper.h"
#include "fas_queue/base.h"

#if defined(SUPPORT_ESP32_I2S)
#include "pd_esp32/i2s_fill.h"
#include "pd_esp32/i2s_manager.h"
#endif

class StepperQueue : public StepperQueueBase {
 public:
#include "../fas_queue/protocol.h"

#if !defined(SUPPORT_SELECT_DRIVER_TYPE)
#error "SUPPORT_SELECT_DRIVER_TYPE is supported for esp32"
#endif  // SUPPORT_SELECT_DRIVER_TYPE

#if defined(SUPPORT_DYNAMIC_ALLOCATION)
// dynamic allocation only for espidf >=5.3, so no mcpwm/pcnt
static uint8_t queues_allocated;
#ifdef SUPPORT_ESP32_RMT
  static uint8_t _rmt_allocated;
#endif
#if defined(SUPPORT_ESP32_I2S)
  static bool _i2s_mux_initialized;
  static uint32_t _i2s_mux_allocated_bitmask;
#endif
static void initVars() {
  StepperQueue::queues_allocated = 0;
  #ifdef SUPPORT_ESP32_RMT
    StepperQueue::_rmt_allocated = 0;
  #endif
  #if defined(SUPPORT_ESP32_I2S)
    StepperQueue::_i2s_mux_initialized = false;
    StepperQueue::_i2s_mux_allocated_bitmask = 0;
    _fill_state = {0,0,0};
  #endif
}
#endif

  volatile bool _isRunning;
  bool _nextCommandIsPrepared;
  uint8_t _step_pin;

  inline bool isRunning() { return _isRunning; }
  bool isReadyForCommands();
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
      int8_t _i2s_step_slot; // TODO: only works if this variable is defined 
      struct i2s_fill_state _fill_state;
      I2sManager* i2s_mgr;
    };
#endif
  };

  uint16_t _getPerformedPulses();

  // Module specific functions
#ifdef SUPPORT_ESP32_MCPWM_PCNT
  bool isReadyForCommands_mcpwm_pcnt();
  bool init_mcpwm_pcnt(uint8_t channel_num, uint8_t step_pin);
  void startQueue_mcpwm_pcnt();
  void forceStop_mcpwm_pcnt();
  uint16_t _getPerformedPulses_mcpwm_pcnt();
  void connect_mcpwm_pcnt();
  void disconnect_mcpwm_pcnt();
#endif
#ifdef SUPPORT_ESP32_RMT
  bool isReadyForCommands_rmt();
  bool init_rmt(uint8_t channel_num, uint8_t step_pin);
  void startQueue_rmt();
#if ESP_IDF_VERSION_MAJOR == 4
  void stop_rmt(bool both);
#endif
  void forceStop_rmt();
  uint16_t _getPerformedPulses_rmt();
  void connect_rmt();
  void disconnect_rmt();
#endif
#ifdef SUPPORT_ESP32_I2S
  bool init_i2s(uint8_t step_pin);
  void startQueue_i2s();
  void forceStop_i2s();
  bool isReadyForCommands_i2s();
  uint16_t _getPerformedPulses_i2s();
  void fill_i2s_buffer(uint8_t* buf);
#endif

  void setDirPin(uint8_t dir_pin, bool _dirHighCountsUp) {
    dirPin = dir_pin;
    dirHighCountsUp = _dirHighCountsUp;
  }
};

#if defined(SUPPORT_ESP32_RMT)
void rmt_fill_buffer(StepperQueue* q, bool fill_part_one, uint32_t* data);
void rmt_apply_command(StepperQueue* q, bool fill_part_one, uint32_t* data);
#endif

#endif
