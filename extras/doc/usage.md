# Usage

[Back to README](../../README.md)

The library is in use with A4988, but other driver ICs should work, too.

For the API definition please consult the header file
[FastAccelStepper.h](../../src/FastAccelStepper.h) or the generated
[markdown file](FastAccelStepper_API.md).

Please check the examples for application and how to use the low level interface.
Some info is in [Issue #86](https://github.com/gin66/FastAccelStepper/issues/86).

The module defines the global variable `fas_queue`. Do not use or redefine this
variable.

## High level interface (ramp up/down)

Using the high level interface with ramp up/down as in
[UsageExample.ino](../../examples/UsageExample/UsageExample.ino).

```
#include "FastAccelStepper.h"

#define dirPinStepper    5
#define enablePinStepper 6
#define stepPinStepper   9

FastAccelStepperEngine engine = FastAccelStepperEngine();
FastAccelStepper *stepper = NULL;

void setup() {
   engine.init();
   stepper = engine.stepperConnectToPin(stepPinStepper);
   if (stepper) {
      stepper->setDirectionPin(dirPinStepper);
      stepper->setEnablePin(enablePinStepper);
      stepper->setAutoEnable(true);

      stepper->setSpeedInHz(500);       // 500 steps/s
      stepper->setAcceleration(100);    // 100 steps/s²
      stepper->move(1000);
   }
}

void loop() {
}
```

## Auto enable/disable

Few comments to auto enable/disable:

* If the motor is operated with micro stepping, then the disable/enable will cause
  the stepper to jump to/from the closest full step position.
* Some drivers need time to e.g. stabilize voltages until stepping should start.
  For this the start on delay has been added. See
  [issue #5](https://github.com/gin66/FastAccelStepper/issues/5).
* The turn off delay is realized in the cyclic task for esp32 or cyclic interrupt
  for avr. The esp32 task uses 4ms delay, while the avr repeats every ~4 ms at
  16 MHz and atmel sam due every 2ms at 21MHz. Thus the turn off delay is a
  multiple (n>=2) of those period times and actual turning off takes place approx
  [(n-1)..n] * 4 ms resp. 2ms after the last step.
* The turn on delay is minimal `MIN_CMD_TICKS`.
* More than one stepper can be connected to one auto enable pin. Behaviour is like this:
    1. If stepper #1 needs enable, then it will enable it with its defined on delay time.
    2. If stepper #2, which is connected to same enable pin, starts after stepper
       one, then it still will wait its defined on delay time and set the enable
       pin, again (no-op). The stepper #2 is not aware, that another stepper
       (stepper #1) has enabled the outputs already.
    3. If e.g. stepper #1 stops, then stepper #1's delay off counter is started.
    4. When stepper #1's counter is finished, then the FastAccelStepperEngine will
       ask all steppers, if they agree to stepper #1's disable request. If
       stepper #2 is still running, then stepper #2 will not agree and the output
       will stay enabled.
    5. When stepper #2 stops, then stepper #2's delay off counter is started.
    6. When stepper #2's counter is finished, then the FastAccelStepperEngine will
       ask all steppers, if they agree to stepper #2's disable request. Stepper #1
       agrees, because it is not running. So the engine will call Stepper #2's
       _AND_ Stepper #1's `disableOutputs()`.

  The library does not consider the case, that Low/High Active enable may be mixed.
  This means stepper #1 uses the enable pin as High Active and stepper #2 the same
  pin as Low Active. => This situation will not be identified and will lead to
  unexpected behaviour.

## Low level interface

The low level command queue for each stepper allows direct speed control - when
high level ramp generation is not operating. This allows precise control of the
stepper, if the code generating the commands can cope with the stepper speed
(beware of any `Serial.print` in your hot path).

See [Multi-axis applications](multi_axis.md) and the
[driver architecture](driver_architecture.md) for details.
