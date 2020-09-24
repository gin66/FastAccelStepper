# FastAccelStepper 
 
![C/C++ CI](https://github.com/gin66/FastAccelStepper/workflows/C/C++%20CI/badge.svg)

This is an high speed alternative for the [AccelStepper library](http://www.airspayce.com/mikem/arduino/AccelStepper/). Supported are avr (Atmega 328) and esp32.

The stepper motors should be connected via a driver IC (like A4988) with a 1, 2 or 3-wire connection:
* Step Signal
	- avr: This must be connected for stepper A to Pin 9 and for Stepper B to Pin 10.
	- esp32: This can be any output capable port pin.
	- Step should be done on transition Low to High. High time will be only a few us.
      On esp32 is the high time fixed to 20us.
* Direction Signal (optional)
	- This can be any port pin.
    - Position counting up on direction pin high or low, as per optional parameter to setDirectionPin(). Default is high.
* Enable Signal (optional)
	- This can be any port pin.
    - Stepper will be enabled on pin high or low, as per optional parameter to setEnablePin(). Default is low.

FastAccelStepper offers the following features:
* 1-pin operation for e.g. peristaltic pump => only positive move
* 2-pin operation for e.g. axis control
* 3-pin operation to reduce power dissipation of driver/stepper
* Lower limit of ~1 steps/s @ 16MHz
* fully interrupt driven - no periodic task to be called
* supports acceleration and deceleration with per stepper max speed/acceleration
* speed/acceleration can be varied while stepper is running (call to functions move or moveTo is needed in order to apply the new values)
* Auto enable mode: stepper motor is enabled before movement and disabled afterwards
* No float calculation (use own implementation of poor man float: 8 bit mantissa+8 bit exponent)
* Provide API to each steppers' command queue. Those commands are tied to timer ticks aka the CPU frequency!

### AVR

* allows up to roughly 25000 generated steps per second in dual stepper operation (depends on worst ISR routine in the system)
* supports up to two stepper motors using Step/Direction/Enable Control (Direction and Enable is optional)
* Uses F_CPU Macro for the relation tick value to time, so it should now not be limited to 16 MHz CPU frequency (untested)
* Steppers' command queue depth: 15

### ESP32

* allows up to roughly 50000 generated steps per second
* supports up to six stepper motors using Step/Direction/Enable Control (Direction and Enable is optional)
* Steppers' command queue depth: 31

The library is in use with A4988, but other driver ICs could work, too.

## Usage

For the API definition please consult the ![FastAccelStepper.h](https://github.com/gin66/FastAccelStepper/blob/master/src/FastAccelStepper.h) header file.

The module defines the global variable fas_queue. Do not use or redefine this variable.

### AVR

Using the high level interface with ramp up/down:

```
#include "FastAccelStepper.h"

#define dirPinStepperA    5
#define enablePinStepperA 6
#define stepPinStepperA   9  // OC1A

FastAccelStepperEngine engine = FastAccelStepperEngine();
FastAccelStepper *stepperA = NULL;

void setup() {
   engine.init();
   stepperA = engine.stepperConnectToPin(stepPinStepperA);

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
#define stepPinStepperA   9  // OC1A

FastAccelStepperEngine engine = FastAccelStepperEngine();
FastAccelStepper *stepper1 = NULL;
FastAccelStepper *stepper2 = NULL;

void setup() {
   engine.init();
   stepperA = engine.stepperConnectToPin(stepPinStepperA);

   stepperA->setDirectionPin(dirPinStepperA);

   stepperA->setEnablePin(enablePinStepperA);
   stepperA->setAutoEnable(true);
}

uint32_t dt = ABSOLUTE_MAX_TICKS;
bool up = true;

void loop() {
  // Issue command with parameters via addQueueEntry:
  //          time delta:            dt  [*1/TICKS_PER_S s]
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

Check the examples: SingleStepper_esp32 and RawAccess_esp32.
RawAccess_esp32 drives the stepper up to 80000 steps/s.

## Behind the curtains

### AVR

The timer 1 is used with prescaler 1. With the arduino nano running at 16 MHz, timer overflow interrupts are generated every ~4 ms. This timer overflow interrupt is used for adjusting the speed. 

The timer compare unit toggles the step pin from Low to High precisely. The transition High to Low is done in the timer compare interrupt routine, thus the High state is only few us.

After stepper movement is completed, the timer compare unit is disconnected from the step pin. Thus the application could change the state freely, while the stepper is not controlled by this library.

Measurement of the acceleration/deacceleration aka timer overflow interrupt yields: one calculation round needs around 300us. Thus it can keep up with the chosen 10 ms planning ahead time.

### ESP32

This stepper driver use mcpwm modules of the esp32: for the first three stepper motors mcpwm0, and mcpwm1 for the steppers four to six. In addition, the pulse counter module is used starting from unit_0 to unit_5. This driver uses the pcnt_isr_service, so unallocated modules can still be used by the application.

The mcpwm modules' outputs are fed into the pulse counter by direct gpio_matrix-modification.

### BOTH

The used formula is just s = 1/2 * a * t² = v² / (2 a) with s = steps, a = acceleration, v = speed and t = time. The performed square root is an 8 bit table lookup. Sufficient exact for this purpose.

The compare interrupt routines use two staged tick counters. One byte (n_periods) + one word (period). The max tick counter value is 254*65535. At 16 MHz this comes down to 1.04s. Thus the lower speed limit is approx 1 step per second.

The low level command queue for each stepper allows direct speed control - when high level ramp generation is not operating. This allows precise control of the stepper, if the code, generating the commands, can cope with the stepper speed (beware of any Serial.print in your hot path).

## TODO

* API stabilization
* Better API documentation
* Simplify the examples, avoid the loop for serial readin.
* Introduce command queue of speed/accel commands - one per stepper.
* Calculation on pc and on arduino do not create same commands. Queue checksum differ (recheck) !
* Support different values for acceleration and deceleration.

## ISSUES

* Speed changes at very low speed with high acceleration values are not always performed
	==> Implementation changed. Need to verify, if issue is closed
* Queue is filled too much, which cause slow response to speed/acceleration changes
* esp32: getCurrentPosition() does not take into account the current pulses, because the pulse counter is not read
* Following will lead to interesting behavior:
       1.) let stepper slowly increase speed to v1
       2.) before v1 is reached, increase accel and call stopMove()
       => result is, that the stepper first increases speed before ramping down
* Change in direction requires motor stop ! => it's a feature
* Strategy should be clarified. Under this topic belongs:
	* TODO: Introduce command queue of speed/accel commands - one per stepper.
	* Change in direction requires motor stop ! => it's a feature
  Currently any move/moveTo - while running - adjusts the ongoing move/moveTo's values.
  With a command queue, this would lead to additional commands to be executed after the running one.
  Update of new values for acceleration/speed would require another API function, because move/moveTo would enqueue a new command.
  The question is, if this complexity in the driver is a good idea or better left to the application.
* The revisions 0.4.x and 0.5.x are not usable for AVR in case channel B is needed => upgrade to 0.6

## Not planned for now

* Using constant acceleration leads to force jumps at start and max speed => smooth this out => will not happen
* Extend command queue entry to perform delay only without step (steps=0) to reduce the 1.0 steps/s

## Lessons Learned

* Spent more than half a day debugging the esp32-code, till I have found out, that just the cable to the stepper was broken.
* In one setup, operating A4988 without microsteps has led to erratic behaviour at some specific low speed (erratic means step forward/backward, while DIR is kept low). No issue with 16 microstep

