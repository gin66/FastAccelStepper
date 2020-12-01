#if defined(ARDUINO_ARCH_ESP32) || defined(ARDUINO_ARCH_AVR)
#include <Arduino.h>
#endif
#include <stdint.h>

#include "FastAccelStepper.h"
#include "common.h"

#if defined(ARDUINO_ARCH_AVR)
#define stepPinStepperA 9  /* OC1A */
#define stepPinStepperB 10 /* OC1B */
#endif

// Here are the global variables to interface with the interrupts

// CURRENT QUEUE IMPLEMENTATION WASTES ONE UNUSED ENTRY => BUG/TODO

#if defined(TEST)
#define NUM_QUEUES 2
#define fas_queue_A fas_queue[0]
#define fas_queue_B fas_queue[1]
#define QUEUE_LEN 16
#elif defined(ARDUINO_ARCH_AVR)
#define NUM_QUEUES 2
#define fas_queue_A fas_queue[0]
#define fas_queue_B fas_queue[1]
#define QUEUE_LEN 16
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
  int input_sig_index;
  uint32_t timer_tez_int_clr;
  uint32_t timer_tez_int_ena;
};
#endif

struct queue_entry {
  uint8_t steps_dir;  // coding is bit7..1 is nr of steps and bit 0 is direction
  uint8_t n_periods;  // number of PERIOD_TICKS delays
  uint16_t period;    // remaining period time in addition to
                      // n_periods*PERIOD_TICKS delays
};
class StepperQueue {
 public:
  struct queue_entry entry[QUEUE_LEN];
  uint8_t read_idx;  // ISR stops if readptr == next_writeptr
  uint8_t next_write_idx;
  uint8_t dirPin;
  bool dirHighCountsUp;
  volatile bool isRunning;
#if defined(ARDUINO_ARCH_ESP32)
  const struct mapping_s* mapping;
  // These two variables are for the mcpwm interrupt
  uint8_t current_period;
  uint8_t current_n_periods;
#endif
#if defined(ARDUINO_ARCH_AVR)
  bool isChannelA;
  // This is used in the timer compare unit as extension of the 16 timer
  uint8_t skip;
#endif
  uint16_t period;
#if (TEST_CREATE_QUEUE_CHECKSUM == 1)
  uint8_t checksum;
#endif

  struct queue_end_s queue_end;

  void init(uint8_t queue_num, uint8_t step_pin);
  inline bool isQueueFull() {
    noInterrupts();
    uint8_t rp = read_idx;
    uint8_t wp = next_write_idx;
    interrupts();
    rp += QUEUE_LEN;
    return (wp == rp);
  }
  inline bool isQueueEmpty() {
    noInterrupts();
    bool res = (next_write_idx == read_idx);
    interrupts();
    inject_fill_interrupt(0);
    return res;
  }
  int addQueueEntry(struct stepper_command_s* cmd) {
    if (isQueueFull()) {
      return AQE_FULL;
    }
    // For ticks > 65536, there will be several fixed delays with PERIOD_TICKS
    // inserted. The formula behind is:
    //		ticks = n_periods * PERIOD_TICKS + period
    // In order to avoid division/remainder operation (on avr actually), the
    // division is approximated, because PERIOD_TICKS is close to 65536. Perform
    // first "division by 65536", which serves too identifying the need of
    // adding fixed delays.
    uint32_t period_ticks = cmd->ticks;
    uint8_t n_periods = period_ticks >> 16;
    if (n_periods > 0) {
      // In this case, PERIOD_TICKS delays need to be inserted
      // Based on division by 65536, n_periods is off by 0..11 in case F_CPU =
      // 16MHz
      uint32_t fixed_ticks = PERIOD_TICKS;
      fixed_ticks *= n_periods;
      period_ticks -= fixed_ticks;
      // Consequently a loop is acceptable compared to 32bit/16bit division and
      // remainder operations.
      while (period_ticks > 4 * 65535) {
        n_periods += 4;
        period_ticks -= 4 * PERIOD_TICKS;
      }
      while (period_ticks > 2 * 65535) {
        n_periods += 2;
        period_ticks -= 2 * PERIOD_TICKS;
      }
      while (period_ticks > 65535) {
        n_periods += 1;
        period_ticks -= PERIOD_TICKS;
      }
    }
    uint16_t period = period_ticks;

    uint8_t wp = next_write_idx;
    struct queue_entry* e = &entry[wp & QUEUE_LEN_MASK];
    uint8_t steps = cmd->steps;
    queue_end.pos += cmd->count_up ? steps : -steps;
    steps <<= 1;
    if (steps == 0) {
      // This is a pause
      uint32_t tfls = queue_end.ticks_from_last_step;
      if (tfls <= 0xffff0000) {
        queue_end.ticks_from_last_step = tfls + cmd->ticks;
      }
    } else {
      uint32_t tfls = queue_end.ticks_from_last_step;
      if (tfls <= 0xffff0000) {
        queue_end.ticks = tfls + cmd->ticks;
      }
	  else {
        queue_end.ticks = cmd->ticks;
	  }
      queue_end.ticks_from_last_step = 0;
    }
    e->period = period;
    e->n_periods = n_periods;
    // check for dir pin value change
    queue_end.count_up = cmd->count_up;
    bool dir = (cmd->count_up == dirHighCountsUp);
    e->steps_dir = (dir != queue_end.dir) ? steps | 0x01 : steps;
    queue_end.dir = dir;
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
    wp++;
    noInterrupts();
    next_write_idx = wp;
    bool run = isRunning;
    interrupts();
    if (!run) {
      startQueue();
    }
    return AQE_OK;
  }
  bool hasTicksInQueue(uint32_t min_ticks) {
    noInterrupts();
    uint8_t rp = read_idx;
    uint8_t wp = next_write_idx;
    interrupts();
    if (wp == rp) {
      return 0;
    }
    rp++;  // ignore currently processed entry
    while (wp != rp) {
      struct queue_entry* e = &entry[rp & QUEUE_LEN_MASK];
      uint8_t steps = e->steps_dir >> 1;
      uint32_t tmp = e->period;
      tmp *= steps;
      if (tmp >= min_ticks) {
        return true;
      }
      min_ticks -= tmp;
      tmp = e->n_periods;
      tmp *= steps;
      tmp *= PERIOD_TICKS;
      if (tmp >= min_ticks) {
        return true;
      }
      min_ticks -= tmp;
      rp++;
    }
    return false;
  }

  // startQueue is called, if motor is not running.
  void startQueue();
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
    isRunning = false;
#if (TEST_CREATE_QUEUE_CHECKSUM == 1)
    checksum = 0;
#endif
  }
};

extern StepperQueue fas_queue[NUM_QUEUES];
