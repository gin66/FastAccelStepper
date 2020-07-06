# FastAccelStepper 
 
![C/C++ CI](https://github.com/gin66/FastAccelStepper/workflows/C/C++%20CI/badge.svg)

This is an high speed alternative for the [AccelStepper library](http://www.airspayce.com/mikem/arduino/AccelStepper/). It makes use of the 16 bit Timer 1 ot the Atmega 328 and as such supports one or two stepper motors.

The stepper motors should be connected via a driver IC (like 4988) with a 1, 2 or 3-wire connection:
* Step Signal
	- This must be connected for stepper A to Pin 9 and for Stepper B to Pin 10.
	- Step should be done on transition Low to High. High time will be only a few us.
* Direction Signal (optional)
	- This can be any port pin.
* Enable Signal (optional)
	- This can be any port pin.
    - Stepper shall be enabled on LOW

FastAccelStepper offers the following features:
* 1-pin operation for e.g. peristaltic pump => only positive move
* 2-pin operation for e.g. axis control
* 3-pin operation for power reduction
* Lower limit of 3.8 steps/s @ 16MHz
* fully interrupt driven - no periodic task to be called
* supports acceleration and deceleration with per stepper max speed/acceleration
* speed/acceleration can be varied while stepper is running (need call to move/moveTo for application of new values, not done implicitly)
* Auto enable mode: stepper motor is enabled before movement and disabled afterwards
* No float calculation (use own implementation of poor man float: 8 bit mantissa+8 bit exponent)
* Provide API to each steppers' command queue. Those commands are tied to timer ticks aka the CPU frequency!

### AVR

* allows up to roughly 25000 generated steps per second in dual stepper operation (depends on worst ISR routine in the system)
* supports up to two stepper motors using Step/Direction/Enable Control (Direction and Enable is optional)
* Uses F_CPU Macro for the relation tick value to time, so it should now not be limited to 16 MHz CPU frequency (untested)
* Steppers' command queue depth: 15

### ESP32

* supports up to six stepper motors using Step/Direction/Enable Control (Direction and Enable is optional)
* Steppers' command queue depth: 31

The library is in use with A4988, but other driver ICs could work, too.

## Usage

### AVR

Using the high level interface with ramp up/down:

```
#include "FastAccelStepper.h"

#define dirPinStepperA    5
#define enablePinStepperA 6
//#define stepPinStepperA   9  // OC1A

FastAccelStepperEngine fas_engine = FastAccelStepperEngine();
FastAccelStepper stepperA = fas_engine.stepperA();
//FastAccelStepper stepperB = fas_engine.stepperB();

void setup() {
   fas_engine.init();
   stepperA->setDirectionPin(dirPinStepperA);

   stepperA->setEnablePin(enablePinStepperA);
   stepperA->setAutoEnable(true);

   stepperA->setSpeed(1000);       // the parameter is us/step !!!
   stepperA->setAcceleration(100);
   stepperA->move(1000);
}

void loop() {
}
```

Using the low level interface to stepper command queue:

```
#include "FastAccelStepper.h"

#define dirPinStepperA    5
#define enablePinStepperA 6
//#define stepPinStepperA   9  // OC1A

FastAccelStepperEngine fas_engine = FastAccelStepperEngine();
FastAccelStepper stepperA = fas_engine.stepperA();
//FastAccelStepper stepperB = fas_engine.stepperB();

void setup() {
   fas_engine.init();
   stepperA->setDirectionPin(dirPinStepperA);

   stepperA->setEnablePin(enablePinStepperA);
   stepperA->setAutoEnable(true);
}

uint32_t dt = ABSOLUTE_MAX_TICKS;
bool up = true;

void loop() {
  // Issue command with parameters via addQueueEntry:
  //          time delta:            dt  [*0.25us/Step]
  //          steps:                 2
  //          direction pin:         high
  uint8_t steps = min(max(100000L/dt,1), 127);
  if (stepperA->addQueueEntry(dt, steps, true) == AQE_OK) {
     if (up) {
        dt -= dt / 100;
        if (dt < F_CPU/40000) { // 40000 steps/s
            up = false;
        }
     }
     else {
        dt += 1;
        if (dt == ABSOLUTE_MAX_TICKS) {
	        up = true;
        }
     }
  }
}
```

### ESP32

Check the SingleStepper_esp32 example for an example.

## Behind the curtains

### AVR

The timer 1 is used with prescaler 1. With the arduino nano running at 16 MHz, timer overflow interrupts are generated every ~4 ms. This timer overflow interrupt is used for adjusting the speed. 

The timer compare unit toggles the step pin from Low to High precisely. The transition High to Low is done in the timer compare interrupt routine, thus the High state is only few us.

After stepper movement is completed, the timer compare unit is disconnected from the step pin. Thus the application could change the state freely, while the stepper is not controlled by this library.

The compare interrupt routines uses two staged tick counters. One byte (msb) + one word (lsw). The max tick counter value is 4,194,303. At 16 MHz this comes down to 0.2621s. Thus the lower speed limit is approx 3.82 steps/s.

The acceleration/deacceleration aka timer overflow interrupt reports to perform one calculation round in around 300us. Thus it can keep up with the chosen 10 ms planning ahead time.

### ESP32

This stepper driver use mcpwm modules of the esp32: for the first three stepper motors mcpwm0 and mcpwm1 for the steppers four to six. In addition the pulse counter module is used starting from unit_0 to unit_5. This driver uses the pcnt_isr_service, so unallocated modules can be used by the application.

The mcpwm modules' outputs are fed into the pulse counter by direct gpio_matrix-modification.

### BOTH

The used formula is just s = 1/2 * a * t² = v² / (2 a) with s = steps, a = acceleration, v = speed and t = time. The performed square root is an 8 bit table lookup. Sufficient exact for this purpose.

The low level command queue for each stepper allows direct speed control - when high level ramp generation is not operating. This allows precise control of the stepper, if the code, generating the commands, can cope with the stepper speed (beware of any Serial.print in your hot path).

## TODO

* API stabilization
* Better API documentation
* Introduce command queue of speed/accel commands - one per stepper.
* Add command to set current position
* Extend command queue entry to perform delay only without step (steps=0) to reduce the 3.8 steps/s
* Calculation on pc and on arduino do not create same commands. Queue checksum differ (recheck) !
* Support different values for acceleration and deceleration
* Add preprocessor constant: TICKS_PER_S for raw commands

## ISSUES

* Speed changes at low speed and high acceleration do not come
* Queue is filled too much, which cause slow response to speed/acceleration changes
* esp32: getCurrentPosition() does not take into account the current pulses

## NOT TODO

* Change in direction requires motor stop ! => it's a feature
* Using constant acceleration leads to force jumps at start and max speed => smooth this out => will not happen

## Lessons Learned

* Spent more than half a day debugging the esp32-code, till I have found out, that just the cable to the stepper was broken.
* In one setup, operating A4988 without microsteps lead to erratic behaviour and some specific low speed (erratic means step forward/backward). No issue with 16 microstep

