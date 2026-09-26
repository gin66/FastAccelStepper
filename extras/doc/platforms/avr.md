# AVR (ATmega) Platform

[Back to README](../../../README.md) | [Platform index](../hardware.md)

## Usage limits

### ATmega 168/168P/328/328P

* allows up to 50000 generated steps per second for single stepper operation, 37000 for dual stepper
* supports up to two stepper motors using Step/Direction/Enable Control (Direction and Enable is optional)
* Step signal only on Pin 9 and 10
* Uses `F_CPU` Macro for the relation tick value to time, so it should now not be limited to 16 MHz CPU frequency (untested)
* Steppers' command queue depth: 16

### ATmega 32u4

* allows up to 50000 generated steps per second for single stepper operation, 37000 for dual stepper and 20000 for three steppers
* supports up to three stepper motors using Step/Direction/Enable Control (Direction and Enable is optional)
* Step signal only on Pin 9, 10 and 11
* Uses `F_CPU` Macro for the relation tick value to time, so it should now not be limited to 16 MHz CPU frequency (untested)
* Steppers' command queue depth: 16

### ATmega 2560

* allows up to 50000 generated steps per second for single stepper operation, 37000 for dual stepper and 20000 for three steppers
* supports up to three stepper motors using Step/Direction/Enable Control (Direction and Enable is optional)
* Step signal only on Pin 6, 7 and 8 by default
* Uses `F_CPU` Macro for the relation tick value to time, so it should now not be limited to 16 MHz CPU frequency (untested)
* Steppers' command queue depth: 16
* This device has four 16 bit timers, so extension up to 12 steppers should be possible (not implemented)

## Selecting the timer (ATmega2560)

By default timer 4 is used. For users of platformio, the used timer can be
changed to either 1, 3, 4 or 5. For e.g. timer module 3 add to platformio.ini
under `build_flags`:

```
build_flags = -DFAS_TIMER_MODULE=3
```

or better:

```
build_flags = -Werror -Wall -DFAS_TIMER_MODULE=3
```

For arduino users, the same can be done by defining the flag *before* including
the `FastAccelStepperEngine.h` header (as per info ixil), but apparently to
[issue #50](https://github.com/gin66/FastAccelStepper/issues/50), this approach
does not work for everyone. e.g.

```
sketch.ino
----------
#include <Arduino.h>
#define FAS_TIMER_MODULE 3
#include <FastAccelStepper.h>
/* ... */
```

This allows to change the timer to other triples: 11/12/13 Timer 1, 5/2/3
Timer 3 or 46/45/44 Timer 5 with the `FAS_TIMER_MODULE` setting.

## Implementation

### ATmega168/328 and ATmega32u4

The timer 1 is used with prescaler 1. With the arduino nano running at 16 MHz,
timer overflow interrupts are generated every ~4 ms. This timer overflow
interrupt is used for adjusting the speed.

The timer compare unit toggles the step pin from Low to High precisely. The
transition High to Low is done in the timer compare interrupt routine, thus the
High state is only few us.

After stepper movement is completed, the timer compare unit is disconnected from
the step pin. Thus the application could change the state freely, while the
stepper is not controlled by this library.

Measurement of the acceleration/deacceleration aka timer overflow interrupt
yields: one calculation round needs around 300us. Thus it can keep up with the
chosen 10 ms planning ahead time.

### ATmega2560

Similar to ATmega328, but instead of timer 1, timer 4 is used.

See [Selecting the timer](#selecting-the-timer-atmega2560) above.
