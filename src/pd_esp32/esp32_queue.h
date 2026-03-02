#ifndef PD_ESP32_QUEUE_H
#define PD_ESP32_QUEUE_H

#include "FastAccelStepper.h"
#include "fas_queue/base.h"

#if defined(SUPPORT_ESP32_I2S)
#include "pd_esp32/i2s_fill.h"
#endif

class StepperQueue : public StepperQueueBase {
 public:
  volatile bool _isRunning;
  bool _nextCommandIsPrepared;
  bool use_rmt;
  bool use_i2s;
  uint8_t _step_pin;

  inline bool isRunning() { return _isRunning; }
  bool isReadyForCommands();

#ifdef SUPPORT_ESP32_MCPWM_PCNT
  const void* driver_data;
#endif
#ifdef SUPPORT_ESP32_RMT
  RMT_CHANNEL_T channel;
  bool _rmtStopped;
  bool lastChunkContainsSteps;
#if defined(SUPPORT_ESP32_RMT_V2)
  rmt_encoder_handle_t _tx_encoder;
#if ESP_IDF_VERSION_MAJOR >= 5
  bool _channel_enabled;
#endif
#endif
#endif

  uint16_t _getPerformedPulses();

  AqeResultCode addQueueEntry(const struct stepper_command_s* cmd, bool start);
  int32_t getCurrentPosition();
  uint32_t ticksInQueue();
  bool hasTicksInQueue(uint32_t min_ticks);
  bool getActualTicksWithDirection(struct actual_ticks_s* speed);

  bool init(FastAccelStepperEngine* engine, uint8_t queue_num,
            uint8_t step_pin);
  void startQueue();
  void forceStop();
  void _initVars();
  void connect();
  void disconnect();

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
  int8_t _i2s_step_slot;
  uint8_t _i2s_drain;
  uint8_t _write_block;
  struct i2s_fill_state _fill_state;

  bool init_i2s(uint8_t channel_num, uint8_t step_pin);
  void startQueue_i2s();
  void forceStop_i2s();
  bool isReadyForCommands_i2s();
  uint16_t _getPerformedPulses_i2s();
  void fill_i2s_buffer(uint8_t busy_block);
#endif

  static bool isValidStepPin(uint8_t step_pin);

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
