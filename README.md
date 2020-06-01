# FastAccelStepper

This is an alternative for the [AccelStepper library](http://www.airspayce.com/mikem/arduino/AccelStepper/), which makes use of the 16 bit Timer 1.

FastAccelStepper offers the following features:
* supports up to two stepper motors using DIR/STEP Control (Enable is optional)
* allows up to roughly 40000 generated steps per second
* support two stepper connected with step pin to Pin 9 using Timer 1 A and Pin 10 using Timer 1 B Compare unit
* fully interrupt driven - no period task to be called
* supports acceleration and deceleration with per stepper max speed/acceleration
* speed/acceleration can be varied while stepper is running
* Deceleration can stop quickly, if needed

The library is used with A4988, but others with two/three pin control should work, too.

## Usage

```
#include "FastAccelStepper.h"

#define dirPinStepper1    5
#define enablePinStepper1 6
#define stepPinStepper1   9  /* OC1A */

FastAccelStepperEngine fas_engine = FastAccelStepperEngine();
FastAccelStepper stepper1 = fas_engine.stepperA(DirPin);
//FastAccelStepper stepper2 = fas_engine.stepperB(DirPin);

void setup() {
   engine.init();
   engine.setDebugLed(LED);
   stepper1->setEnablePin(enablePinStepper1);

   stepper1->set_auto_enable(true);

   stepper1->set_dynamics(speed*1.0, accel*1.0);
   stepper1->calculate_move(move);
   stepper1->start();
}

void loop() {
}
```

## TODO

1. Change from speed [steps/s] into time delta [µs/steps]
2. Avoid float after use of time delta
