# FastAccelStepper 
 
![Run tests](https://github.com/gin66/FastAccelStepper/workflows/Run%20tests/badge.svg)
![Build examples](https://github.com/gin66/FastAccelStepper/workflows/Build%20examples/badge.svg)

This is an high speed alternative for the [AccelStepper library](http://www.airspayce.com/mikem/arduino/AccelStepper/). Supported are avr (Atmega 328) and esp32.

The stepper motors should be connected via a driver IC (like A4988) with a 1, 2 or 3-wire connection:
* Step Signal
	- avr: This must be connected for stepper A to Pin 9 and for Stepper B to Pin 10.
	- esp32: This can be any output capable port pin.
	- Step should be done on transition Low to High. High time will be only a few us.
      On esp32 the high time is fixed to 10us.
* Direction Signal (optional)
	- This can be any output capable port pin.
    - Position counting up on direction pin high or low, as per optional parameter to setDirectionPin(). Default is high.
* Enable Signal (optional)
	- This can be any output capable port pin.
    - Stepper will be enabled on pin high or low, as per optional parameter to setEnablePin(). Default is low.

FastAccelStepper offers the following features:
* 1-pin operation for e.g. peristaltic pump => only positive move
* 2-pin operation for e.g. axis control
* 3-pin operation to reduce power dissipation of driver/stepper
* Lower limit of ~1 steps/s @ 16MHz
* fully interrupt driven - no periodic task to be called
* supports acceleration and deceleration with per stepper max speed/acceleration
* speed/acceleration can be varied while stepper is running (call to functions move or moveTo is needed in order to apply the new values)
* Auto enable mode: stepper motor is enabled before movement and disabled afterwards with configurable delays
* No float calculation (use own implementation of poor man float: 8 bit mantissa+8 bit exponent)
* Provide API to each steppers' command queue. Those commands are tied to timer ticks aka the CPU frequency!

General behaviour:
* The desired end position to move to is set by calls to moveTo() and move()
* The desired end position is in case of moveTo() given as absolute position
* For move() the delta is added to the latest desired end position
* The stepper tries to reach the given desired end position as fast as possible with adherence to acceleration/deceleration
* If the stepper is e.g. running towards position 1000 and moveTo(0) is called at position 500, then the stepper will 
	1. decelerate, which means it will overshoot position 500 
    2. stop and accelerate towards 0
    3. eventually coast for a while and then decelerate
    4. stop

### AVR

* allows up to roughly 25000 generated steps per second in dual stepper operation (depends on worst ISR routine in the system)
* supports up to two stepper motors using Step/Direction/Enable Control (Direction and Enable is optional)
* Uses F_CPU Macro for the relation tick value to time, so it should now not be limited to 16 MHz CPU frequency (untested)
* Steppers' command queue depth: 16

### ESP32

* allows up to roughly 50000 generated steps per second
* supports up to six stepper motors using Step/Direction/Enable Control (Direction and Enable is optional)
* Steppers' command queue depth: 32

The library is in use with A4988, but other driver ICs could work, too.

## Usage

For the API definition please consult the header file [FastAccelStepper.h](https://github.com/gin66/FastAccelStepper/blob/master/src/FastAccelStepper.h).
Please check the examples for application and how to use the low level interface.

The module defines the global variable fas_queue. Do not use or redefine this variable.

Using the high level interface with ramp up/down:

```
#include "FastAccelStepper.h"

#define dirPinStepper    5
#define enablePinStepper 6
#define stepPinStepper   9  // OC1A in case of AVR

FastAccelStepperEngine engine = FastAccelStepperEngine();
FastAccelStepper *stepper = NULL;

void setup() {
   engine.init();
   stepper = engine.stepperConnectToPin(stepPinStepper);
   if (stepper) {
      stepper->setDirectionPin(dirPinStepper);
      stepper->setEnablePin(enablePinStepper);
      stepper->setAutoEnable(true);

      stepper->setSpeed(1000);       // the parameter is us/step !!!
      stepper->setAcceleration(100);
      stepper->move(1000);
   }
}

void loop() {
}
```

Few comments to auto enable/disable:

* If the motor is operated with micro stepping, then the disable/enable will cause the stepper to jump to/from the closest full step position.
* Some drivers need time to e.g. stabilize voltages until stepping should start. For this the start on delay has been added. See [issue #5](https://github.com/gin66/FastAccelStepper/issues/5).
* The turn off delay is realized in the cyclic task for esp32 or cyclic interrupt for avr. The esp32 task uses 10ms delay, while the avr repeats every ~4 ms at 16 MHz. Thus the turn off delay is a multiple (n>=2) of those period times and actual turning off takes place approx [(n-1)..n] * (10 or 4) ms after the last step.


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

The used formula is just s = 1/2 * a * t² = v² / (2 a) with s = steps, a = acceleration, v = speed and t = time. In order to determine the speed for a given step, the calculation is v = sqrt(2 * a * s). The performed square root is an 8 bit table lookup. Sufficient exact for this purpose.

The compare interrupt routines use two staged tick counters. One byte (n_periods) + one word (period). The max tick counter value is 254*65535. At 16 MHz this comes down to 1.04s. Thus the lower speed limit is approx 1 step per second.

The low level command queue for each stepper allows direct speed control - when high level ramp generation is not operating. This allows precise control of the stepper, if the code, generating the commands, can cope with the stepper speed (beware of any Serial.print in your hot path).

## TODO

See [project](https://github.com/gin66/FastAccelStepper/projects/1)

## PLATFORMIO

If you prefer platformio and you are running Linux, then platformio version of the examples are created by executing

```
ci/build-platformio.sh
```

This will create a directory pio_dirs, which contains all examples. Can be executed by e.g.

```
cd pio_dirs/StepperDemo
pio run -e avr --target upload --upload-port /dev/ttyUSB0
```


## ISSUES

* Speed changes at very low speed with high acceleration values are not always performed
* Queue is filled too much, which cause slow response to speed/acceleration changes
* esp32: getCurrentPosition() does not take into account the current pulses, because the pulse counter is not read

## Not planned for now

* Using constant acceleration leads to force jumps at start and max speed => smooth this out => will not happen
* Extend command queue entry to perform delay only without step (steps=0) to reduce the 1.0 steps/s

## Lessons Learned

* Spent more than half a day debugging the esp32-code, till I have found out, that just the cable to the stepper was broken.
* In one setup, operating A4988 without microsteps has led to erratic behaviour at some specific low speed (erratic means step forward/backward, while DIR is kept low). No issue with 16 microstep

