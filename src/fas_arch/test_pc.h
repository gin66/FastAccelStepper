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

#endif /* FAS_ARCH_TEST_PC_H */
