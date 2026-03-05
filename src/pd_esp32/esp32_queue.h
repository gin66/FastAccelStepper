#ifndef PD_ESP32_QUEUE_H
#define PD_ESP32_QUEUE_H

#include "FastAccelStepper.h"
#include "fas_queue/base.h"

#if defined(SUPPORT_ESP32_I2S)
#include "pd_esp32/i2s_fill.h"
#endif

class StepperQueue : public StepperQueueBase {
 public:
#include "../fas_queue/protocol.h"

#if !defined(SUPPORT_SELECT_DRIVER_TYPE)
#error "SUPPORT_SELECT_DRIVER_TYPE is supported for esp32"
#endif  // SUPPORT_SELECT_DRIVER_TYPE
  static FasDriver tryAllocateDriver(FasDriver driver);
#ifdef SUPPORT_ESP32_MCPWM_PCNT
  static uint8_t _mcpwm_pcnt_allocated;
#endif
#ifdef SUPPORT_ESP32_RMT
  static uint8_t _rmt_allocated;
#endif
#if defined(SUPPORT_ESP32_I2S)
  static uint8_t _i2s0_mode;
#if SOC_I2S_NUM >= 2
  static uint8_t _i2s1_mode;
#endif
#if SOC_I2S_NUM >= 3
  static uint8_t _i2s2_mode;
#endif
  static uint32_t _i2s0_mux_allocated_bitmask;
#if SOC_I2S_NUM >= 2
  static uint32_t _i2s1_mux_allocated_bitmask;
#endif
#if SOC_I2S_NUM >= 3
  static uint32_t _i2s2_mux_allocated_bitmask;
#endif
#endif  // SUPPORT_ESP32_I2S

  volatile bool _isRunning;
  bool _nextCommandIsPrepared;
  uint8_t _step_pin;

  inline bool isRunning() { return _isRunning; }
  bool isReadyForCommands();

  // module specific variables
  union {
#ifdef SUPPORT_ESP32_MCPWM_PCNT
    struct {
      bool use_mcpwm_pcnt;
      const void* driver_data;
    };
#endif
#ifdef SUPPORT_ESP32_RMT
    struct {
      bool use_rmt;
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
      bool use_i2s;
      int8_t _i2s_step_slot;
      struct i2s_fill_state _fill_state;
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
  bool init_i2s(uint8_t channel_num, uint8_t step_pin);
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
