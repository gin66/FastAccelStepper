#ifndef FAS_ARCH_TEST_PC_H
#define FAS_ARCH_TEST_PC_H

// For pc-based testing like to have assert-macro
#include <assert.h>

// and some more includes
#include <math.h>
#include <stdio.h>
#include <stdlib.h>

// Arduino/AVR compatibility stubs for PC-based testing
#define PROGMEM
#define pgm_read_byte_near(x) (*(x))

// For inducing interrupts while testing
void noInterrupts();
void interrupts();
void inject_fill_interrupt(int mark);

#define _BV(x) 0
#define ISR(x) void x()
#define inline
#define micros() 0

#define abs(x) ((x) > 0 ? (x) : -(x))
#define min(a, b) ((a) > (b) ? (b) : (a))
#define max(a, b) ((a) > (b) ? (a) : (b))

#define digitalWrite(a, b) \
  {                        \
  }
#define pinMode(a, b) \
  {                   \
  }

extern char TCCR1A;
extern char TCCR1B;
extern char TCCR1C;
extern char TIMSK1;
extern char TIFR1;
extern unsigned short OCR1A;
extern unsigned short OCR1B;

#define test(x, msg) \
  if (!(x)) {        \
    puts(msg);       \
    assert(false);   \
  };

// For pc-based testing, the macro TEST is defined. The pc-based testing does
// not support the concept of interrupts, so provide an empty definition
#define fasEnableInterrupts()
#define fasDisableInterrupts()

// The TEST target needs a couple of arduino like definitions
#define LOW 0
#define HIGH 1

#endif /* FAS_ARCH_TEST_PC_H */
