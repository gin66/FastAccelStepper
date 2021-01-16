# FastAccelStepper 
 
![Run tests](https://github.com/gin66/FastAccelStepper/workflows/Run%20tests/badge.svg)
![Build examples](https://github.com/gin66/FastAccelStepper/workflows/Build%20examples/badge.svg)
![Simvar tests](https://github.com/gin66/FastAccelStepper/workflows/Run%20tests%20with%20simavr/badge.svg)

This is an high speed alternative for the [AccelStepper library](http://www.airspayce.com/mikem/arduino/AccelStepper/). Supported are avr (ATmega 328, ATmega2560) and esp32.

The stepper motors should be connected via a driver IC (like A4988) with a 1, 2 or 3-wire connection:
* Step Signal
	- avr atmega328p: only Pin 9 and Pin 10.
	- avr atmega2560: only Pin 6, 7 and 8.
      On platformio, this can be changed to other triples: 11/12/13 Timer 1, 5/2/3 Timer 3 or 46/45/44 Timer 5 with FAS_TIMER_MODULE setting.
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
* Enable pins can be shared between motors
* Direction pins can be shared between motors
* External callback function can be used to drive the enable pins (e.g. connected to shift register)
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
	  Means the position will count:
```
		-2.000.000.000
		-2.000.000.001
		-2.000.000.002
			:
		-2.147.483.647
		-2.147.483.648
		 2.147.483.647
		 2.147.483.646
		 2.147.483.645
			:
		 2.000.000.000
```
Comments to pin sharing:
* Enable pin sharing: the common pin will be enabled for as long as one motor is running + delay off.
  Every motor will adhere to its auto enable delay, even if other motors already have enabled the pin.
* Direction pin sharing: The direction pin will be exclusively driven by one motor. If one motor is operating, another motor will wait until the direction pin comes available

### AVR ATMega 328

* allows up to 25000 generated steps per second in dual stepper operation (depends on worst ISR routine in the system)
* supports up to two stepper motors using Step/Direction/Enable Control (Direction and Enable is optional)
* Uses F_CPU Macro for the relation tick value to time, so it should now not be limited to 16 MHz CPU frequency (untested)
* Steppers' command queue depth: 16

### AVR ATMega 2560

* allows up to 25000 generated steps per second in dual stepper operation (depends on worst ISR routine in the system)
* supports up to three stepper motors using Step/Direction/Enable Control (Direction and Enable is optional)
* Uses F_CPU Macro for the relation tick value to time, so it should now not be limited to 16 MHz CPU frequency (untested)
* Steppers' command queue depth: 16
* This device has four 16 bit timers, so extension up to 12 steppers should be possible (not implemented)

### ESP32

* allows up 200000 generated steps per second
* supports up to six stepper motors using Step/Direction/Enable Control (Direction and Enable is optional)
* Steppers' command queue depth: 32

The library is in use with A4988, but other driver ICs could work, too.

## Usage

For the API definition please consult the header file [FastAccelStepper.h](https://github.com/gin66/FastAccelStepper/blob/master/src/FastAccelStepper.h).
Please check the examples for application and how to use the low level interface.

The module defines the global variable fas_queue. Do not use or redefine this variable.

Using the high level interface with ramp up/down as in [UsageExample.ino](https://github.com/gin66/FastAccelStepper/blob/master/examples/UsageExample/UsageExample.ino).

```
#include "FastAccelStepper.h"
#include "AVRStepperPins.h" // Only required for AVR controllers

#define dirPinStepper    5
#define enablePinStepper 6
#define stepPinStepper   9

// If using an AVR device use the definitons provided in AVRStepperPins
//    stepPinStepper1A
//
// or even shorter (for 2560 the correct pin on the chosen timer is selected):
//    stepPinStepperA

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
* The turn off delay is realized in the cyclic task for esp32 or cyclic interrupt for avr. The esp32 task uses 4ms delay, while the avr repeats every ~4 ms at 16 MHz. Thus the turn off delay is a multiple (n>=2) of those period times and actual turning off takes place approx [(n-1)..n] * 4 ms after the last step.
* The turn on delay is min `MIN_DELTA_TICKS` for avr or `MIN_CMD_TICKS` for esp32.
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

### AVR ATmega328

The timer 1 is used with prescaler 1. With the arduino nano running at 16 MHz, timer overflow interrupts are generated every ~4 ms. This timer overflow interrupt is used for adjusting the speed. 

The timer compare unit toggles the step pin from Low to High precisely. The transition High to Low is done in the timer compare interrupt routine, thus the High state is only few us.

After stepper movement is completed, the timer compare unit is disconnected from the step pin. Thus the application could change the state freely, while the stepper is not controlled by this library.

Measurement of the acceleration/deacceleration aka timer overflow interrupt yields: one calculation round needs around 300us. Thus it can keep up with the chosen 10 ms planning ahead time.

### AVR ATmega2560

Similar to ATmega328, but instead of timer 1, timer 4 is used.

For users of platformio, the used timer can be changed to either 1, 3, 4 or 5. For e.g. timer module 3 add to platformio.ini under build_flags:
```
build_flags = -DFAS_TIMER_MODULE=3
```

or better:
```
build_flags = -Werror -Wall -DFAS_TIMER_MODULE=3
```

For arduino users, the same can be done by defining the flag *before* including the `FastAccelStepperEngine.h` header (as per info ixil).
e.g.
```
sketch.ino
----------
#include <Arduino.h>
#define FAS_TIMER_MODULE 3
#include <FastAccelStepper.h>
/* ... */
```

### ESP32

This stepper driver uses mcpwm modules of the esp32: for the first three stepper motors mcpwm0, and mcpwm1 for the steppers four to six. In addition, the pulse counter module is used starting from unit_0 to unit_5. This driver uses the pcnt_isr_service, so unallocated modules can still be used by the application.

The mcpwm modules' outputs are fed into the pulse counter by direct gpio_matrix-modification.

A note to MIN_CMD_TICKS: The current implementation uses one interrupt per command in the command queue. This is much less interrupt rate than for avr. Nevertheless at 200kSteps/s the switch from one command to the next one should be ideally serviced before the next step. This means within 5us. As this cannot be guaranteed, the driver remedies an overrun (at least by design) to deduct the overrun pulses from the next command. The overrun pulses will then be run at the former command's tick rate. For real life stepper application, this should be ok. To be considered for raw access: Do not run many steps at high rate e.g. 200kSteps/s followed by a pause. 

### ALL

The used formula is just s = 1/2 * a * t² = v² / (2 a) with s = steps, a = acceleration, v = speed and t = time. In order to determine the speed for a given step, the calculation is v = sqrt(2 * a * s). The performed square root is an 8 bit table lookup. Sufficient exact for this purpose.

The compare interrupt routines use 16bit tick counters, which translates to approx. 4ms. For longer time between pulses, pauses without step output can be added. With this approach the ramp generation supports up to one step per 268s. 

The low level command queue for each stepper allows direct speed control - when high level ramp generation is not operating. This allows precise control of the stepper, if the code, generating the commands, can cope with the stepper speed (beware of any Serial.print in your hot path).

## Usage for multi-axis applications

For coordinated movement of two or more axis, the current ramp generation will not provide good results. The planning of steps needs to take into consideration max.speed/acceleration of all steppers and eventually the net speed/acceleration of the resulting movement together with its restrictions. Nice example of multi-axis forward planning can be found within the [marlin-project](https://github.com/MarlinFirmware/Marlin/tree/2.0.x/Marlin/src/module). If this kind of multi-dimensional planning is used, then FastAccelStepper is a good solution to execute the raw commands (without ramp generation). Only missing feature for this is a synchronized start of several steppers. With the tick-exact execution of commands, the synchronization will not be lost as long as the command queues are not running out of commands.

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

## TEST STRATEGY

The library is tested with different kind of tests:
* PC only, which reside in tests/.
  These tests focussing primarily the ramp generator and part of the API
* simavr based for avr
  The simavr is an excellent simulator for avr microcontrollers. This allows to check the avr implementation thoroughly and even count the number of steps generated. Tested code is mainly the StepperDemo, which gets fed in a line of commands to execute.
  These tests focus on avr and help to check the whole library code and helps for esp32
* esp32 tests with another pulse counter attached
  The FastAccelStepper-API supports to attach another free pulse counter to a stepper's step and dir pins. This counter counts in the range of -16383 to 16383 with wrap around to 0. The test condition is, that the library's view of the position should match the independently counted one. These tests are still evolving
* Test for pulse generation using examples/Pulses
  This has been intensively used to debug the esp32 ISR code
* manual tests using StepperDemo
  These are unstructured tests with listening to the motor and observing the behavior

## CHANGELOG

See [changelog](https://github.com/gin66/FastAccelStepper/blob/master/CHANGELOG)

## ISSUES

* esp32: getCurrentPosition() does not take into account the current pulses, because the pulse counter is not read
* avr: three steppers at high speed does not work due too interrupt load
* Very high acceleration value e.g. 10.000.000 and high speed may be silently not executed, if the high speed in us is smaller than MIN_CMD_TICKS. This corresponds to 2500 steps/s for avr and 5000 steps/s.
* StepperDemo test case 07 yields quite a deviation between esp32 and avr timing for identical ramp
* There is an issue with the esp32 mcpwm: as soon as the mcpwm timer is running on every cycle an interrupt is serviced - even though no interrupt is enabled. If several steppers are running at high step rate, the interrupt load for this nonsense interrupt could be quite high for the CPU. Need further investigation, but till now haven't found the root cause.


## Lessons Learned

* Spent more than half a day debugging the esp32-code, till I have found out, that just the cable to the stepper was broken.
* In one setup, operating A4988 without microsteps has led to erratic behaviour at some specific low speed (erratic means step forward/backward, while DIR is kept low). No issue with 16 microstep
* The pulse counters in esp32 have several comparators to trigger interrupts. What the documentation does not mention: All those reference values are only forwarded to the actual comparator on pulse counter reset. Thus the pulse counters cannot be used as lower 16bit of the position, unfortunately.

## Thanks

- Thanks ixil for pull request (https://github.com/gin66/FastAccelStepper/pull/19) for ATmega2560
