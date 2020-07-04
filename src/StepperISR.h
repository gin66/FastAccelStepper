#ifndef TEST
#include <Arduino.h>
#endif
#include <stdint.h>

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
#endif
#if defined(ARDUINO_ARCH_AVR)
#define NUM_QUEUES 2
#define fas_queue_A fas_queue[0]
#define fas_queue_B fas_queue[1]
#define QUEUE_LEN 16
#endif
#if defined(ARDUINO_ARCH_ESP32)
#define NUM_QUEUES 6
#define QUEUE_LEN 32
#endif

// These variables control the stepper timing behaviour
#define QUEUE_LEN_MASK (QUEUE_LEN - 1)

struct queue_entry {
  uint8_t steps;  // coding is bit7..1 is nr of steps and bit 0 is direction
#if defined(TEST)
  uint8_t delta_msb;
  uint16_t delta_lsw;    // using small values is not safe
  int16_t delta_change;  // change of delta on each step. delta_lsw +
                         // steps*delta_change must not over/underflow
#endif
#if defined(ARDUINO_ARCH_AVR)
  uint8_t delta_msb;
  uint16_t delta_lsw;    // using small values is not safe
  int16_t delta_change;  // change of delta on each step. delta_lsw +
                         // steps*delta_change must not over/underflow
#endif
#if defined(ARDUINO_ARCH_ESP32)
  uint8_t prescaler;
  uint16_t period;
#endif
};
class StepperQueue {
 public:
  struct queue_entry entry[QUEUE_LEN];
  uint8_t read_ptr;  // ISR stops if readptr == next_writeptr
  uint8_t next_write_ptr;
  uint8_t autoEnablePin;
  uint8_t dirPin;
#if defined(ARDUINO_ARCH_ESP32)
  uint8_t queueNum;
  bool isRunning;
#endif
  // This is used in the timer compare unit as extension of the 16 timer
#if defined(ARDUINO_ARCH_AVR)
  uint8_t skip;
#endif

  int32_t pos_at_queue_end;    // in steps
  int32_t ticks_at_queue_end;  // in timer ticks, 0 on stopped stepper
  bool dir_high_at_queue_end;  // direction high corresponds to position
                                // counting upwards

  void init(uint8_t queue_num, uint8_t step_pin);
  bool isQueueFull() {
	return (((next_write_ptr + 1) & QUEUE_LEN_MASK) == read_ptr);
  }
  bool isQueueEmpty() {
	return (read_ptr == next_write_ptr);
  }
  bool isStopped() {
	  return ticks_at_queue_end == 0;
  }
  void addQueueStepperStop() {
	  ticks_at_queue_end = 0;
  }
  int addQueueEntry(uint32_t start_delta_ticks, uint8_t steps, bool dir_high,
                    int16_t change_ticks);
 private:
  void _initVars() {
	  dirPin = 255;
	  autoEnablePin = 255;
	  read_ptr = 0;
	  next_write_ptr = 0;
	  dir_high_at_queue_end = true;
	  pos_at_queue_end = 0;
	  ticks_at_queue_end = 0;
  }
};

extern struct StepperQueue fas_queue[NUM_QUEUES];
