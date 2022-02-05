#include <stdint.h>

#include "FastAccelStepper.h"
#include "common.h"

// Here are the global variables to interface with the interrupts

// These variables control the stepper timing behaviour
#define QUEUE_LEN_MASK (QUEUE_LEN - 1)

#if defined(ARDUINO_ARCH_SAM)
typedef struct _PWMCHANNELMAPPING {
  uint8_t pin;
  uint32_t channel;
  Pio* port;
  uint32_t channelMask;
} PWMCHANNELMAPPING;
#endif
struct queue_entry {
  uint8_t steps;  // if 0,  then the command only adds a delay
  uint8_t toggle_dir : 1;
  uint8_t countUp : 1;
  uint8_t moreThanOneStep : 1;
  uint8_t hasSteps : 1;
  uint16_t ticks;
#if defined(ARDUINO_ARCH_AVR)
  uint16_t end_pos_last16;
#else
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

#if defined(SUPPORT_ESP32)
  volatile bool _isRunning;
  bool _nextCommandIsPrepared;
  bool isRunning() { return _isRunning; }
  bool isReadyForCommands();
  bool use_rmt;
#ifdef SUPPORT_ESP32_MCPWM_PCNT
  const void *driver_data;
  const struct mapping_s* mapping;
  bool isReadyForCommands_mcpwm_pcnt();
#endif
#ifdef SUPPORT_ESP32_RMT
  rmt_channel_t channel;
  bool isReadyForCommands_rmt();
  bool _rmtStopped;
#endif
#elif defined(ARDUINO_ARCH_AVR)
  volatile uint8_t* _dirPinPort;
  uint8_t _dirPinMask;
  volatile bool _prepareForStop;
  volatile bool _isRunning;
  inline bool isRunning() { return _isRunning; }
  inline bool isReadyForCommands() { return true; }
  enum channels channel;
#elif defined(ARDUINO_ARCH_SAM)
  volatile uint32_t* _dirPinPort;
  uint32_t _dirPinMask;
  volatile bool _hasISRactive;
  bool isRunning();
  const PWMCHANNELMAPPING* mapping;
  bool _connected;
  inline bool isReadyForCommands() { return true; }
  volatile bool _pauseCommanded;
  volatile uint32_t timePWMInterruptEnabled;
#else
  volatile bool _isRunning;
  inline bool isReadyForCommands() { return true; }
  inline bool isRunning() { return _isRunning; }
#endif

  struct queue_end_s queue_end;
  uint16_t max_speed_in_ticks;

  void init(uint8_t queue_num, uint8_t step_pin);
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

  int8_t addQueueEntry(const struct stepper_command_s* cmd, bool start);
  int32_t getCurrentPosition();
  uint32_t ticksInQueue();
  bool hasTicksInQueue(uint32_t min_ticks);
  uint16_t getActualTicks();

  volatile uint16_t getMaxSpeedInTicks() { return max_speed_in_ticks; }

  // startQueue is always called
  void startQueue();
  void forceStop();
  void _initVars();
#if defined(SUPPORT_ESP32)
  uint8_t _step_pin;
  uint16_t _getPerformedPulses();
#endif
#if defined(ARDUINO_ARCH_SAM)
  uint8_t _step_pin;
  uint8_t _queue_num;
#endif
  void connect();
  void disconnect();

#if defined(SUPPORT_ESP32_PULSE_COUNTER)
bool _esp32_attachToPulseCounter(uint8_t pcnt_unit, FastAccelStepper* stepper,
                                 int16_t low_value, int16_t high_value);
void _esp32_clearPulseCounter(uint8_t pcnt_unit);
int16_t _esp32_readPulseCounter(uint8_t pcnt_unit);
#endif

#ifdef SUPPORT_ESP32_MCPWM_PCNT
  void init_mcpwm_pcnt(uint8_t channel_num, uint8_t step_pin);
  void startQueue_mcpwm_pcnt();
  void forceStop_mcpwm_pcnt();
  uint16_t _getPerformedPulses_mcpwm_pcnt();
  void connect_mcpwm_pcnt();
  void disconnect_mcpwm_pcnt();
#endif
#ifdef SUPPORT_ESP32_RMT
  void init_rmt(uint8_t channel_num, uint8_t step_pin);
  void startQueue_rmt();
  void forceStop_rmt();
  uint16_t _getPerformedPulses_rmt();
  void connect_rmt();
  void disconnect_rmt();
#endif
  void setDirPin(uint8_t dir_pin, bool _dirHighCountsUp) {
    dirPin = dir_pin;
    dirHighCountsUp = _dirHighCountsUp;
#if defined(ARDUINO_ARCH_AVR) || defined(ARDUINO_ARCH_SAM)
    if (dir_pin != PIN_UNDEFINED) {
      _dirPinPort = portOutputRegister(digitalPinToPort(dir_pin));
      _dirPinMask = digitalPinToBitMask(dir_pin);
    }
#endif
  }
  void adjustSpeedToStepperCount(uint8_t steppers);
  static bool isValidStepPin(uint8_t step_pin);
  static int8_t queueNumForStepPin(uint8_t step_pin);
};

extern StepperQueue fas_queue[NUM_QUEUES];

void fas_init_engine(FastAccelStepperEngine* engine, uint8_t cpu_core);
