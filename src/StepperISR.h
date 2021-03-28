#if defined(ARDUINO_ARCH_ESP32) || defined(ARDUINO_ARCH_AVR)
#include <Arduino.h>
#else
#include <assert.h>
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

bool _esp32_attachToPulseCounter(uint8_t pcnt_unit, FastAccelStepper* stepper,
                                 int16_t low_value, int16_t high_value);
void _esp32_clearPulseCounter(uint8_t pcnt_unit);
int16_t _esp32_readPulseCounter(uint8_t pcnt_unit);
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
  uint8_t read_idx;  // ISR stops if readptr == next_writeptr
  uint8_t next_write_idx;
  bool dirHighCountsUp;
  uint8_t dirPin;
#if defined(ARDUINO_ARCH_ESP32)
  volatile uint32_t* _dirPinPort;
  uint32_t _dirPinMask;
  volatile bool _hasISRactive;
  bool _nextCommandIsPrepared;
  bool isRunning();
  const struct mapping_s* mapping;
#elif defined(ARDUINO_ARCH_AVR)
  volatile uint8_t* _dirPinPort;
  uint8_t _dirPinMask;
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
    // Just to check if, if the struct has the correct size
    // if (sizeof(entry) != 6 * QUEUE_LEN) {
    //  return -1;
    //}
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
    bool dir = (cmd->count_up == dirHighCountsUp);
    bool toggle_dir = false;
    if (dirPin != PIN_UNDEFINED) {
      if (isQueueEmpty()) {
        // set the dirPin here. Necessary with shared direction pins
        digitalWrite(dirPin, dir);
        queue_end.dir = dir;
      } else {
        toggle_dir = (dir != queue_end.dir);
      }
    }
    e->steps = steps;
    e->toggle_dir = toggle_dir;
    e->countUp = cmd->count_up ? 1 : 0;
    e->moreThanOneStep = steps > 1 ? 1 : 0;
    e->hasSteps = steps > 0 ? 1 : 0;
    e->ticks = period;
#if defined(ARDUINO_ARCH_AVR)
    queue_end.pos += cmd->count_up ? steps : -steps;
    e->end_pos_last16 = (uint32_t)queue_end.pos & 0xffff;
#else
    e->start_pos_last16 = (uint32_t)queue_end.pos & 0xffff;
    queue_end.pos += cmd->count_up ? steps : -steps;
#endif
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

  int32_t getCurrentPosition() {
    noInterrupts();
    uint32_t pos = (uint32_t)queue_end.pos;
    uint8_t rp = read_idx;
    bool is_empty = (rp == next_write_idx);
    struct queue_entry* e = &entry[rp & QUEUE_LEN_MASK];
#if defined(ARDUINO_ARCH_AVR)
    uint16_t pos_last16 = e->end_pos_last16;
#else
    uint16_t pos_last16 = e->start_pos_last16;
#endif
    uint8_t steps = e->steps;
#if defined(ARDUINO_ARCH_ESP32)
    // pulse counter should go max up to 255 with perhaps few pulses overrun, so
    // this conversion is safe
    int16_t done_p = (int16_t)_getPerformedPulses();
#endif
    interrupts();
#if defined(ARDUINO_ARCH_ESP32)
    if (done_p == 0) {
      // fix for possible race condition described in issue #68
      noInterrupts();
      rp = read_idx;
      is_empty = (rp == next_write_idx);
      e = &entry[rp & QUEUE_LEN_MASK];
      pos_last16 = e->start_pos_last16;
      steps = e->steps;
      done_p = (int16_t)_getPerformedPulses();
      interrupts();
    }
#endif
    if (!is_empty) {
      int16_t adjust = 0;

      uint16_t pos16 = pos & 0xffff;
      uint8_t transition = ((pos16 >> 12) & 0x0c) | (pos_last16 >> 14);
      switch (transition) {
        case 0:   // 00 00
        case 5:   // 01 01
        case 10:  // 10 10
        case 15:  // 11 11
          break;
        case 1:   // 00 01
        case 6:   // 01 10
        case 11:  // 10 11
        case 12:  // 11 00
          pos += 0x4000;
          break;
        case 4:   // 01 00
        case 9:   // 10 01
        case 14:  // 11 10
        case 3:   // 00 11
          pos -= 0x4000;
          break;
        case 2:   // 00 10
        case 7:   // 01 11
        case 8:   // 10 00
        case 13:  // 11 01
          break;  // TODO: ERROR
      }
      pos = (int32_t)((pos & 0xffff0000) | pos_last16);

      if (steps != 0) {
        if (e->countUp) {
#if defined(ARDUINO_ARCH_AVR)
          adjust = -steps;
#elif defined(ARDUINO_ARCH_ESP32)
          adjust = done_p;
#endif
        } else {
#if defined(ARDUINO_ARCH_AVR)
          adjust = steps;
#elif defined(ARDUINO_ARCH_ESP32)
          adjust = -done_p;
#endif
        }
        pos += adjust;
      }
    }
    return pos;
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
  uint16_t getActualTicks() {
    // Retrieve current step rate from the current view.
    // This is valid only, if the command describes more than one step
    noInterrupts();
    uint8_t rp = read_idx;
    uint8_t wp = next_write_idx;
    interrupts();
    if (wp == rp) {
      return 0;
    }
    struct queue_entry* e = &entry[rp & QUEUE_LEN_MASK];
    if (e->hasSteps) {
      if (e->moreThanOneStep) {
        return e->ticks;
      }
      if (wp != ++rp) {
        if (entry[rp & QUEUE_LEN_MASK].hasSteps) {
          return e->ticks;
        }
      }
    }
    return 0;
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
    dirHighCountsUp = true;
#if defined(ARDUINO_ARCH_AVR)
    _isRunning = false;
    _prepareForStop = false;
#elif defined(ARDUINO_ARCH_ESP32)
    _hasISRactive = false;
    _nextCommandIsPrepared = false;
#else
    _isRunning = false;
#endif
#if (TEST_CREATE_QUEUE_CHECKSUM == 1)
    checksum = 0;
#endif
  }
#if defined(ARDUINO_ARCH_ESP32)
  uint8_t _step_pin;
  uint16_t _getPerformedPulses();
#endif
  void connect();
  void disconnect();
  void setDirPin(uint8_t dir_pin, bool _dirHighCountsUp) {
    dirPin = dir_pin;
    dirHighCountsUp = _dirHighCountsUp;
#if defined(ARDUINO_ARCH_ESP32) || defined(ARDUINO_ARCH_AVR)
    if (dir_pin != PIN_UNDEFINED) {
      _dirPinPort = portOutputRegister(digitalPinToPort(dir_pin));
      _dirPinMask = digitalPinToBitMask(dir_pin);
    }
#endif
  }
  static bool isValidStepPin(uint8_t step_pin);
  static int8_t queueNumForStepPin(uint8_t step_pin);
};

extern StepperQueue fas_queue[NUM_QUEUES];
