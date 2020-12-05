# FastAccelStepper 
 
![Run tests](https://github.com/gin66/FastAccelStepper/workflows/Run%20tests/badge.svg)
![Build examples](https://github.com/gin66/FastAccelStepper/workflows/Build%20examples/badge.svg)

This is an high speed alternative for the [AccelStepper library](http://www.airspayce.com/mikem/arduino/AccelStepper/). Supported are avr (Atmega 328) and esp32.

The stepper motors should be connected via a driver IC (like A4988) with a 1, 2 or 3-wire connection:
* Step Signal
	- avr: This must be connected for stepper A to Pin 9 and for Stepper B to Pin 10.
	- esp32: This can be any output capable port pin.
	- Step should be done on transition Low to High. High time will be only a few us.
      On esp32 the high time is for slow speed fixed to ~2ms and high speed to 50% duty cycle
* Direction Signal (optional)
	- This can be any output capable port pin.
    - Position counting up on direction pin high or low, as per optional parameter to setDirectionPin(). Default is high.
* Enable Signal (optional)
	- This can be any output capable port pin.
    - Stepper will be enabled on pin high or low, as per optional parameter to setEnablePin(). Default is low.

FastAccelStepper offers the following features:
* 1-pin operation for e.g. peristaltic pump => only positive move
* 2-pin operation for e.g. axis control (even though use for X/Y-coordinated movement is not recommended)
* 3-pin operation to reduce power dissipation of driver/stepper
* Lower limit of 260s per step @ 16MHz aka one step every four minute
* fully interrupt driven - no periodic task to be called
* supports acceleration and deceleration with per stepper max speed/acceleration
* Allow the motor to continuously run in the current direction until stopMove() is called.
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
* The stepper position is a 32bit integer variable, which wraps around for continuous movement.
  Example:
	- Assume counting up turns stepper clockwise, and counting down, anti-clockwise.
    - Current position is -2.000.000.000, move to 2.000.000.000.
    - Apparently the position has to count up, and count should run clockwise.
    - Implementation is done via difference of 32bit signed numbers, which can overflow (being legal).
    - The calculation is then:
			2.000.000.000 - (-2.000.000.000) = 4.000.000.000
	- But 4.000.000.000 interpreted as signed 32bit is -294.967.296 => count down, turn anti-clockwise

### AVR

* allows up to 25000 generated steps per second in dual stepper operation (depends on worst ISR routine in the system)
* supports up to two stepper motors using Step/Direction/Enable Control (Direction and Enable is optional)
* Uses F_CPU Macro for the relation tick value to time, so it should now not be limited to 16 MHz CPU frequency (untested)
* Steppers' command queue depth: 16

### ESP32

* allows up 200000 generated steps per second
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
* More than one stepper can be connected to one auto enable pin. Behaviour is like this:
	1. If stepper #1 needs enable, then it will enable it with its defined on delay time.
    2. If stepper #2, which is connected to same enable pin, starts after stepper one, then it still will wait its defined on delay time and set the enable pin, again (no-op).
       The stepper #2 is not aware, that another stepper (stepper #1) has enabled the outputs already.
    3. If e.g. stepper #1 stops, then stepper #1's delay off counter is started.
    4. When stepper #1's counter is finished, then the FastAccelStepperEngine will ask all steppers, if they agree to stepper #1's disable request.
       If stepper #2 is still running, then stepper #2 will not agree and the output will stay enabled.
    5. When stepper #2 stops, then stepper #2's delay off counter is started.
    6. When stepper #2's counter is finished, then the FastAccelStepperEngine will ask all steppers, if they agree to stepper #2's disable request.
       Stepper #1 agrees, because it is not running. So the engine will call Stepper #2's _AND_ Stepper #1's disableOutputs().

  The library does not consider the case, that Low/High Active enable may be mixed.
  This means stepper #1 uses the enable pin as High Active and stepper #2 the same pin as Low Active.
  => This situation will not be identified and will lead to unexpected behaviour

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

The compare interrupt routines use 16bit tick counters, which translates to approx. 4ms. For longer time between pulses, pauses without step output can be added. With this approach the ramp generation supports up to one step per 268s. 

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

## CHANGELOG

See [changelog](https://github.com/gin66/FastAccelStepper/blob/master/CHANGELOG)

## ISSUES

* esp32: getCurrentPosition() does not take into account the current pulses, because the pulse counter is not read

## Lessons Learned

* Spent more than half a day debugging the esp32-code, till I have found out, that just the cable to the stepper was broken.
* In one setup, operating A4988 without microsteps has led to erratic behaviour at some specific low speed (erratic means step forward/backward, while DIR is kept low). No issue with 16 microstep
* The pulse counters in esp32 have several comparators to trigger interrupts. What the documentation does not mention: All those reference values are only forwarded to the actual comparator on pulse counter reset. Thus the pulse counters cannot be used as lower 16bit of the position, unfortunately.


