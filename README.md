# FastAccelStepper

This is an high speed alternative for the [AccelStepper library](http://www.airspayce.com/mikem/arduino/AccelStepper/). It makes use of the 16 bit Timer 1 ot the Atmega 328 and as such supports one or two stepper motors.

The stepper motors should be connected via a driver IC (like 4988) with a 1/2 or 3-wire connection:
* Step Signal
	- This must be connected for stepper A to Pin 9 and for Stepper B to Pin 10.
	- Step should be done on transition Low to High. High time will be only a few us.
* Direction Signal (optional)
	- This can be any port pin.
* Enable Signal (optional)
	- This can be any port pin.

FastAccelStepper offers the following features:
* 1-pin operation e.g. peristaltic pump
* 2-pin operation e.g. axis control
* 3-pin operation for power reduction
* supports up to two stepper motors using DIR/STEP/Enable Control (Direction and Enable is optional)
* allows up to roughly 32000 generated steps per second in dual stepper operation
* fully interrupt driven - no periodic task to be called
* supports acceleration and deceleration with per stepper max speed/acceleration
* speed/acceleration can be varied while stepper is running
* Auto enable mode: stepper motor is enabled before movement and disabled afterwards

The library is used with A4988, but other driver ICs could work, too.

## Usage

```
#include "FastAccelStepper.h"

#define dirPinStepper1    5
#define enablePinStepper1 6
#define stepPinStepper1   9  /* OC1A, just for information here */

FastAccelStepperEngine fas_engine = FastAccelStepperEngine();
FastAccelStepper stepper1 = fas_engine.stepperA();
//FastAccelStepper stepper2 = fas_engine.stepperB();

void setup() {
   engine.init();
   engine.setDebugLed(LED);
   stepper1->setDirectionPin(dirPinStepper1);
   stepper1->setEnablePin(enablePinStepper1);

   stepper1->set_auto_enable(true);

   stepper1->set_dynamics(100.0, 10.0);
   stepper1->move(1000);
}

void loop() {
}
```

## TODO

1. Change from speed [steps/s] into time delta [µs/steps]
2. Avoid float after use of time delta
3. Introduce command queue of speed/accel commands - one per stepper
4. Change in direction requires motor stop !
5. Using constant acceleration leads to force jumps at start and max speed => smooth this out


