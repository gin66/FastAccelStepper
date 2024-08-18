#ifndef FAS_ARCH_TEST_PC_H
#define FAS_ARCH_TEST_PC_H

// For pc-based testing like to have assert-macro
#include <assert.h>

// and some more includes
#include <math.h>
#include <stdio.h>
#include <stdlib.h>

#include "../extras/tests/pc_based/stubs.h"

// For pc-based testing, the macro TEST is defined. The pc-based testing does
// not support the concept of interrupts, so provide an empty definition
#define fasEnableInterrupts()
#define fasDisableInterrupts()

// The TEST target needs a couple of arduino like definitions
#define LOW 0
#define HIGH 1

// queue definitions for pc based testing
#define MAX_STEPPER 2
#define NUM_QUEUES 2
#define fas_queue_A fas_queue[0]
#define fas_queue_B fas_queue[1]
#define QUEUE_LEN 16

// timing definitions for pc-based testing
#define TICKS_PER_S 16000000L
#define MIN_CMD_TICKS (TICKS_PER_S / 5000)
#define MIN_DIR_DELAY_US (MIN_CMD_TICKS / (TICKS_PER_S / 1000000))
#define MAX_DIR_DELAY_US (65535 / (TICKS_PER_S / 1000000))
#define DELAY_MS_BASE 1
#define SUPPORT_UNSAFE_ABS_SPEED_LIMIT_SETTING 0

#define noop_or_wait

#define SUPPORT_QUEUE_ENTRY_END_POS_U16

#endif /* FAS_ARCH_TEST_PC_H */
