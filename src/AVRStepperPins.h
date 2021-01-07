#ifndef AVRSTEPPERPINS_H_YFXPAB9M
#define AVRSTEPPERPINS_H_YFXPAB9M
#include <Arduino.h>

/** * Warning: Other libraries may also use the timers!
 *
 * For example Serial library and delay() functions for example.
 * Using the same timer may cause strange effects, you are best to avoid using
 * those other libraries at the some time or use a different pin where
 * possible!
 */

#if defined(ARDUINO_ARCH_AVR)
#if !(defined(__AVR_ATmega328P__) || defined(__AVR_ATmega2560__))
#error "Unsupported AVR derivate"
#endif
#endif

// The ATmega328P has one 16 bit timer: Timer 1
// The ATmega2560 has four 16 bit timers: Timer 1, 3, 4 and 5
#if defined(__AVR_ATmega328P__)
#define stepPinStepper1A 9  /* OC1A */
#define stepPinStepper1B 10 /* OC1B */
#elif defined(__AVR_ATmega2560__)
#define stepPinStepper1A 11 /* OC1A */
#define stepPinStepper1B 12 /* OC1B */
#define stepPinStepper1C 13 /* OC1B */
#define stepPinStepper3A 5  /* OC3A */
#define stepPinStepper3B 2  /* OC3B */
#define stepPinStepper3C 3  /* OC3C */
#define stepPinStepper4A 6  /* OC4A */
#define stepPinStepper4B 7  /* OC4B */
#define stepPinStepper4C 8  /* OC4C */
#define stepPinStepper5A 46 /* OC5A */
#define stepPinStepper5B 45 /* OC5B */
#define stepPinStepper5C 44 /* OC5C */
#endif

#endif // AVRSTEPPERPINS_H_YFXPAB9M
