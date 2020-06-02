#define noInterrupts() \
  {}
#define interrupts() \
  {}
#define _BV(x) 0
#define ISR(x) void x()
#define inline

#include <math.h>

#define abs(x) (x > 0 ? x : -x)
#define min(a, b) (a > b ? b : a)
#define max(a, b) (a > b ? a : b)

#define digitalWrite(a, b) \
  {}
#define pinMode(a, b) \
  {}

extern char TCCR1A;
extern char TCCR1B;
extern char TCCR1C;
extern char TIMSK1;
extern char TIFR1;
extern unsigned short OCR1A;
extern unsigned short OCR1B;
