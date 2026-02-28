#include <stdint.h>

#include "FastAccelStepper.h"
#include "fas_arch/common.h"
#include "fas_queue/base.h"

// Here are the global variables to interface with the interrupts

class StepperQueue : public StepperQueueBase {
 public:
  // a word to isRunning():
  //    if isRunning() is false, then the _QUEUE_ is not running.
  //
  // For esp32 this does NOT mean, that the HW is finished.
  // The timer is still counting down to zero until it stops at 0.
  // But there will be no interrupt to process another command.
  // So the queue requires startQueue() again
  //
  // Due to the rmt version of esp32, there has been the needed to
  // provide information, that device is not yet ready for new commands.
  // This has been called isReadyForCommands().
  //
#if defined(SUPPORT_RP_PICO)
  bool _isActive;  // indicates that the sm should be serviced by the ISR
  bool isRunning();
  bool isReadyForCommands();
  uint8_t _step_pin;
  uint16_t adjust_80MHz;
  PIO pio; /* not set in init */
  uint sm; /* not set in init */
  bool claim_pio_sm(FastAccelStepperEngine* engine);
  void setupSM();
  int32_t pos_offset;  // offset between pico step count and position
  int32_t getCurrentStepCount();
  void attachDirPinToStatemachine();
  void setDirPinState(bool high);
#endif
#if defined(SUPPORT_ESP32)
  volatile bool _isRunning;
  bool _nextCommandIsPrepared;
  inline bool isRunning() { return _isRunning; }
  bool isReadyForCommands();
  bool use_rmt;
  uint8_t _step_pin;
  uint16_t _getPerformedPulses();
#endif
#ifdef SUPPORT_ESP32_MCPWM_PCNT
  const void* driver_data;
#endif
#ifdef SUPPORT_ESP32_RMT
  RMT_CHANNEL_T channel;
  bool _rmtStopped;
  bool lastChunkContainsSteps;
#if defined(SUPPORT_ESP32_RMT_V2)
  rmt_encoder_handle_t _tx_encoder;
#endif
#endif
#if defined(SUPPORT_AVR)
  volatile bool _noMoreCommands;
  volatile bool _isRunning;
  inline bool isRunning() { return _isRunning; }
  inline bool isReadyForCommands() { return true; }
  enum channels channel;
#endif
#if defined(SUPPORT_SAM)
  uint8_t _step_pin;
  uint8_t _queue_num;
  void* driver_data;
  volatile bool _hasISRactive;
  bool isRunning();
  bool _connected;
  inline bool isReadyForCommands() { return true; }
  volatile bool _pauseCommanded;
  volatile uint32_t timePWMInterruptEnabled;
#endif
#if defined(TEST)
  volatile bool _isRunning;
  inline bool isReadyForCommands() { return true; }
  inline bool isRunning() { return _isRunning; }
#endif

  AqeResultCode addQueueEntry(const struct stepper_command_s* cmd, bool start);
  int32_t getCurrentPosition();
  uint32_t ticksInQueue();
  bool hasTicksInQueue(uint32_t min_ticks);
  bool getActualTicksWithDirection(struct actual_ticks_s* speed);

  bool init(FastAccelStepperEngine* engine, uint8_t queue_num,
            uint8_t step_pin);
  // startQueue is always called
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
#else
  bool _channel_enabled;
#endif
  void forceStop_rmt();
  uint16_t _getPerformedPulses_rmt();
  void connect_rmt();
  void disconnect_rmt();
#endif
  void setDirPin(uint8_t dir_pin, bool _dirHighCountsUp) {
    dirPin = dir_pin;
    dirHighCountsUp = _dirHighCountsUp;
#if defined(SUPPORT_DIR_PIN_MASK)
    if ((dir_pin != PIN_UNDEFINED) && ((dir_pin & PIN_EXTERNAL_FLAG) == 0)) {
      _dirPinPort = portOutputRegister(digitalPinToPort(dir_pin));
      _dirPinMask = digitalPinToBitMask(dir_pin);
    }
#endif
#if defined(SUPPORT_DIR_TOGGLE_PIN_MASK)
    if ((dir_pin != PIN_UNDEFINED) && ((dir_pin & PIN_EXTERNAL_FLAG) == 0)) {
      _dirTogglePinPort = portInputRegister(digitalPinToPort(dir_pin));
      _dirTogglePinMask = digitalPinToBitMask(dir_pin);
    }
#endif
#if defined(SUPPORT_RP_PICO)
    attachDirPinToStatemachine();
#endif
  }
#if defined(NEED_ADJUSTABLE_MAX_SPEED_DEPENDING_ON_STEPPER_COUNT)
  void adjustSpeedToStepperCount(uint8_t steppers);
#endif
  static bool isValidStepPin(uint8_t step_pin);
#if defined(NEED_FIXED_QUEUE_TO_PIN_MAPPING)
  static int8_t queueNumForStepPin(uint8_t step_pin);
#endif
};

extern StepperQueue fas_queue[NUM_QUEUES];

#if defined(SUPPORT_ESP32_RMT)
void rmt_fill_buffer(StepperQueue* q, bool fill_part_one, uint32_t* data);
void rmt_apply_command(StepperQueue* q, bool fill_part_one, uint32_t* data);
#endif

#if defined(SUPPORT_CPU_AFFINITY)
void fas_init_engine(FastAccelStepperEngine* engine, uint8_t cpu_core);
#else
void fas_init_engine(FastAccelStepperEngine* engine);
#endif
