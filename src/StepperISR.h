#if defined(ARDUINO_ARCH_ESP32) || defined(ARDUINO_ARCH_AVR)
#include <Arduino.h>
#endif
#include <stdint.h>

#include "FastAccelStepper.h"
#include "common.h"

// Here are the global variables to interface with the interrupts

#if defined(TEST)
#define NUM_QUEUES 2
#define fas_queue_A fas_queue[0]
#define fas_queue_B fas_queue[1]
#define QUEUE_LEN 16
#elif defined(ARDUINO_ARCH_AVR)
#if defined(__AVR_ATmega328P__)
#define NUM_QUEUES 2
#define fas_queue_A fas_queue[0]
#define fas_queue_B fas_queue[1]
#define QUEUE_LEN 16
enum channels { channelA, channelB };
#elif defined(__AVR_ATmega2560__)
#define NUM_QUEUES 3
#define fas_queue_A fas_queue[0]
#define fas_queue_B fas_queue[1]
#define fas_queue_C fas_queue[2]
#define QUEUE_LEN 16
enum channels { channelA, channelB, channelC };
#else
#error "Unsupported derivate"
#endif
#elif defined(ARDUINO_ARCH_ESP32)
#define NUM_QUEUES 6
#define QUEUE_LEN 32
#else
#define NUM_QUEUES 6
#define QUEUE_LEN 32
#endif

// These variables control the stepper timing behaviour
#define QUEUE_LEN_MASK (QUEUE_LEN - 1)

#ifndef TEST
#define inject_fill_interrupt(x)
#endif

#define TICKS_FOR_STOPPED_MOTOR 0xffffffff

#if defined(ARDUINO_ARCH_ESP32)
#include <driver/gpio.h>
#include <driver/mcpwm.h>
#include <driver/pcnt.h>
#include <soc/mcpwm_reg.h>
#include <soc/mcpwm_struct.h>
#include <soc/pcnt_reg.h>
#include <soc/pcnt_struct.h>
struct mapping_s {
  mcpwm_unit_t mcpwm_unit;
  uint8_t timer;
  mcpwm_io_signals_t pwm_output_pin;
  pcnt_unit_t pcnt_unit;
  uint8_t input_sig_index;
  uint32_t cmpr_tea_int_clr;
  uint32_t cmpr_tea_int_ena;
  uint32_t cmpr_tea_int_raw;
};

bool _esp32_attachToPulseCounter(uint8_t pcnt_unit, FastAccelStepper* stepper);
int16_t _esp32_readPulseCounter(uint8_t pcnt_unit);
#endif

struct queue_entry {
  uint8_t steps;  // if 0,  then the command only adds a delay
  bool toggle_dir;
  uint16_t ticks;
};
class StepperQueue {
 public:
  struct queue_entry entry[QUEUE_LEN];
  uint8_t read_idx;  // ISR stops if readptr == next_writeptr
  uint8_t next_write_idx;
  uint8_t dirPin;
  bool dirHighCountsUp;
#if defined(ARDUINO_ARCH_ESP32)
  volatile bool _hasISRactive;
  bool isRunning();
  const struct mapping_s* mapping;
#elif defined(ARDUINO_ARCH_AVR)
  volatile bool _prepareForStop;
  volatile bool _isRunning;
  bool isRunning() { return _isRunning; }
  enum channels channel;
#else
  volatile bool _isRunning;
  bool isRunning() { return _isRunning; }
#endif
#if (TEST_CREATE_QUEUE_CHECKSUM == 1)
  uint8_t checksum;
#endif

  struct queue_end_s queue_end;

  void init(uint8_t queue_num, uint8_t step_pin);
  inline uint8_t queueEntries() {
    noInterrupts();
    uint8_t rp = read_idx;
    uint8_t wp = next_write_idx;
    interrupts();
    inject_fill_interrupt(0);
    return (uint8_t)(wp - rp);
  }
  inline bool isQueueFull() { return queueEntries() == QUEUE_LEN; }
  inline bool isQueueEmpty() { return queueEntries() == 0; }

  int8_t addQueueEntry(const struct stepper_command_s* cmd, bool start) {
    if (cmd == NULL) {
      if (start) {
        return startPreparedQueue();
      }
      return AQE_OK;
    }
    if (isQueueFull()) {
      return AQE_QUEUE_FULL;
    }
    uint16_t period = cmd->ticks;
    uint8_t steps = cmd->steps;
    // Serial.print(period);
    // Serial.print(" ");
    // Serial.println(steps);

    uint32_t command_rate_ticks = period;
    if (steps > 1) {
      command_rate_ticks *= steps;
    }
    if (command_rate_ticks < MIN_CMD_TICKS) {
      return AQE_ERROR_TICKS_TOO_LOW;
    }

    uint8_t wp = next_write_idx;
    struct queue_entry* e = &entry[wp & QUEUE_LEN_MASK];
    if (steps == 0) {
      // This is a pause
      uint32_t tfls = queue_end.ticks_from_last_step;
      if (tfls <= 0xffff0000) {
        queue_end.ticks_from_last_step = tfls + cmd->ticks;
      }
    } else {
      queue_end.pos += cmd->count_up ? steps : -steps;
      uint32_t tfls = queue_end.ticks_from_last_step;
      if (tfls <= 0xffff0000) {
        queue_end.ticks = tfls + cmd->ticks;
      } else {
        queue_end.ticks = tfls;
      }
      queue_end.ticks_from_last_step = 0;
    }
    bool dir = (cmd->count_up == dirHighCountsUp);
    bool toggle_dir = false;
    if (dirPin != PIN_UNDEFINED) {
      if (isQueueEmpty()) {
        // set the dirPin here. Necessary with shared direction pins
        digitalWrite(dirPin, dir);
        queue_end.dir = dir;
      } else {
        toggle_dir = (dir != queue_end.dir) ? true : false;
      }
    }
    e->steps = steps;
    e->toggle_dir = toggle_dir;
    e->ticks = period;
    queue_end.dir = dir;
    queue_end.count_up = cmd->count_up;
#if (TEST_CREATE_QUEUE_CHECKSUM == 1)
    {
      // checksum is in the struct and will updated here
      unsigned char* x = (unsigned char*)e;
      for (uint8_t i = 0; i < sizeof(struct queue_entry); i++) {
        if (checksum & 0x80) {
          checksum <<= 1;
          checksum ^= 0xde;
        } else {
          checksum <<= 1;
        }
        checksum ^= *x++;
      }
    }
#endif
    commandAddedToQueue(start);
    return AQE_OK;
  }
  uint32_t ticksInQueue() {
    noInterrupts();
    uint8_t rp = read_idx;
    uint8_t wp = next_write_idx;
    interrupts();
    if (wp == rp) {
      return 0;
    }
    uint32_t ticks = 0;
    rp++;  // ignore currently processed entry
    while (wp != rp) {
      struct queue_entry* e = &entry[rp++ & QUEUE_LEN_MASK];
      ticks += e->ticks;
      uint8_t steps = e->steps;
      if (steps > 1) {
        uint32_t tmp = e->ticks;
        tmp *= steps - 1;
        ticks += tmp;
      }
    }
    return ticks;
  }
  bool hasTicksInQueue(uint32_t min_ticks) {
    noInterrupts();
    uint8_t rp = read_idx;
    uint8_t wp = next_write_idx;
    interrupts();
    if (wp == rp) {
      return false;
    }
    rp++;  // ignore currently processed entry
    while (wp != rp) {
      struct queue_entry* e = &entry[rp & QUEUE_LEN_MASK];
      uint32_t tmp = e->ticks;
      uint8_t steps = max(e->steps, 1);
      tmp *= steps;
      if (tmp >= min_ticks) {
        return true;
      }
      min_ticks -= tmp;
      rp++;
    }
    return false;
  }

  // startQueue is always called
  void commandAddedToQueue(bool start);
  int8_t startPreparedQueue();
  void forceStop();
  void _initVars() {
    dirPin = PIN_UNDEFINED;
    read_idx = 0;
    next_write_idx = 0;
    queue_end.dir = true;
    queue_end.count_up = true;
    queue_end.pos = 0;
    queue_end.ticks = TICKS_FOR_STOPPED_MOTOR;
    queue_end.ticks_from_last_step = 0xffffffff;
    dirHighCountsUp = true;
#if defined(ARDUINO_ARCH_AVR)
    _isRunning = false;
    _prepareForStop = false;
#elif defined(ARDUINO_ARCH_ESP32)
    _hasISRactive = false;
#else
    _isRunning = false;
#endif
#if (TEST_CREATE_QUEUE_CHECKSUM == 1)
    checksum = 0;
#endif
  }
#if defined(ARDUINO_ARCH_ESP32)
  uint8_t _step_pin;
#endif
  void connect();
  void disconnect();
  static bool isValidStepPin(uint8_t step_pin);
  static int8_t queueNumForStepPin(uint8_t step_pin);
};

extern StepperQueue fas_queue[NUM_QUEUES];
