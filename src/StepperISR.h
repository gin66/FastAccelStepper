#include <stdint.h>

#include "FastAccelStepper.h"
#include "fas_arch/common.h"

// Here are the global variables to interface with the interrupts

// These variables control the stepper timing behaviour
#define QUEUE_LEN_MASK (QUEUE_LEN - 1)

struct queue_entry {
  uint8_t steps;  // if 0,  then the command only adds a delay
  uint8_t toggle_dir : 1;
  uint8_t countUp : 1;
  uint8_t moreThanOneStep : 1;
  uint8_t hasSteps : 1;
#if defined(SUPPORT_EXTERNAL_DIRECTION_PIN)
  // if repeat_entry==1, then this entry shall be repeated.
  // This mechanism only works for pauses (steps == 0)
  // Used for external direction pin
  uint8_t repeat_entry : 1;
  uint8_t dirPinState : 1;
#endif
  uint16_t ticks;
#if defined(SUPPORT_QUEUE_ENTRY_END_POS_U16)
  uint16_t end_pos_last16;
#endif
#if defined(SUPPORT_QUEUE_ENTRY_START_POS_U16)
  uint16_t start_pos_last16;
#endif
};

class StepperQueue {
 public:
  struct queue_entry entry[QUEUE_LEN];

  // In case of forceStopAndNewPosition() the adding of commands has to be
  // temporarily suspended
  volatile bool ignore_commands;
  volatile uint8_t read_idx;  // ISR stops if readptr == next_writeptr
  volatile uint8_t next_write_idx;
  bool dirHighCountsUp;
  uint8_t dirPin;

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
  bool _isStarting;
  bool isRunning();
  bool isReadyForCommands();
  uint8_t _step_pin;
  PIO pio; /* not set in init */
  uint sm; /* not set in init */
  bool claim_pio_sm(FastAccelStepperEngine* engine);
  void setupSM();
  int32_t pos_offset; // offset between pico step count and position
  int32_t getCurrentStepCount();
  void attachDirPinToStatemachine();
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
#if defined(SUPPORT_DIR_PIN_MASK)
  volatile SUPPORT_DIR_PIN_MASK* _dirPinPort;
  SUPPORT_DIR_PIN_MASK _dirPinMask;
#endif
#if defined(SUPPORT_DIR_TOGGLE_PIN_MASK)
  volatile SUPPORT_DIR_TOGGLE_PIN_MASK* _dirTogglePinPort;
  SUPPORT_DIR_TOGGLE_PIN_MASK _dirTogglePinMask;
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

  struct queue_end_s queue_end;
  uint16_t max_speed_in_ticks;

  bool init(FastAccelStepperEngine* engine, uint8_t queue_num,
            uint8_t step_pin);
  inline uint8_t queueEntries() {
    fasDisableInterrupts();
    uint8_t rp = read_idx;
    uint8_t wp = next_write_idx;
    fasEnableInterrupts();
    inject_fill_interrupt(0);
    return (uint8_t)(wp - rp);
  }
  inline bool isQueueFull() { return queueEntries() == QUEUE_LEN; }
  inline bool isQueueEmpty() { return queueEntries() == 0; }
#if defined(SUPPORT_EXTERNAL_DIRECTION_PIN)
  inline bool isOnRepeatingEntry() {
    return entry[read_idx & QUEUE_LEN_MASK].repeat_entry == 1;
  }
  inline uint8_t dirPinState() {
    return entry[read_idx & QUEUE_LEN_MASK].dirPinState;
  }
  inline void clearRepeatingFlag() {
    entry[read_idx & QUEUE_LEN_MASK].repeat_entry = 0;
  }
#endif

  AqeResultCode addQueueEntry(const struct stepper_command_s* cmd, bool start);
  int32_t getCurrentPosition();
  uint32_t ticksInQueue();
  bool hasTicksInQueue(uint32_t min_ticks);
  bool getActualTicksWithDirection(struct actual_ticks_s* speed);

  inline uint16_t getMaxSpeedInTicks() { return max_speed_in_ticks; }

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
#if defined(SUPPORT_UNSAFE_ABS_SPEED_LIMIT_SETTING)
  void setAbsoluteSpeedLimit(uint16_t ticks) { max_speed_in_ticks = ticks; }
#endif
#if defined(NEED_ADJUSTABLE_MAX_SPEED_DEPENDING_ON_STEPPER_COUNT)
  void adjustSpeedToStepperCount(uint8_t steppers);
#endif
  static bool isValidStepPin(uint8_t step_pin);
  static int8_t queueNumForStepPin(uint8_t step_pin);
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
