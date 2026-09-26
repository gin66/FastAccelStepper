# Troubleshooting

[Back to README](../../README.md)

## Known issues

* There is an issue with the esp32 mcpwm: as soon as the mcpwm timer is running,
  on every cycle an interrupt is serviced - even though no interrupt is enabled.
  If several steppers are running at high step rate, the interrupt load for this
  nonsense interrupt could be quite high for the CPU. Need further investigation,
  but till now haven't found the root cause.
* Compilation using esp-idf 4.4 will yield a deprecation warning for
  `mcpwm_isr_register()`. This has been raised as
  [issue](https://github.com/espressif/esp-idf/issues/7890) at espressif
* `framework-arduinoespressif32 @ 3.10006.210326` and later will lead to compile
  error for esp32, if using compiler options `-Werror -Wall` !!! The problem can
  be circumvented by applying `-Wno-error=incompatible-pointer-types`

## Error investigation

In case the stepper does not run smoothly, then StepperDemo contains commands to
simulate two type of error causes. For avr the commands are `r` and `e`. For
esp32 only `e` can be used.

- `r`: The `digitalRead()` of arduino is a fancy implementation, which checks, if
  the pin being read is connected to a timer to generate PWM and if yes, turns
  this off (actually IMHO a broken implementation: only 1 of the needed 2 bits
  are cleared, and the activation by force compare is missing). As
  FastAccelStepper controls the step pin, the `digitalRead` can disturb the step
  pin (even though I have expected step loss, only difference in noise can be
  heard). The error simulation in StepperDemo reads the pins in the main loop(),
  thus the symptom occurs quite reliably.
- `e`: This blocks repeatedly interrupts for ~100us during 64ms out of 256ms. On
  AVR to see this problem popping up, the stepper rate has to be <~106 us (avr,
  one stepper running). >~106us it runs quite smoothly. The 106us = 100us block +
  ~6us ISR runtime. For ESP32 this has no effect.

For avr: cause of long interrupt being blocked can be e.g.:

- long section of codes between `noInterrupts()/interrupts()` in the application
  (or used libraries)
- long interrupt service routines in the application (or used libraries).
- port interrupts connected to noisy/bouncy switches causing bursts of interrupts

Especially in interrupt service routines, the `digitalRead()/digitalWrite()` must
be avoided. Alternative solution is described e.g. here:
[blog](https://masteringarduino.blogspot.com/2013/10/fastest-and-smallest-digitalread-and.html),
or [digitalWriteFast](https://github.com/NicksonYap/digitalWriteFast), or
[fast versions](https://forum.arduino.cc/index.php?topic=46896.0).

This feature of StepperDemo allows to compare non-smooth running stepper in an
application with these error types.

## Lessons learned

* Spent more than half a day debugging the esp32-code, till I have found out, that
  just the cable to the stepper was broken.
* In one setup, operating A4988 without microsteps has led to erratic behaviour at
  some specific low speed (erratic means step forward/backward, while DIR is kept
  low). No issue with 16 microstep. These two youtube videos show similar behavior:
  [hard disc stepper](https://youtu.be/DsYgw3GFHZo) and
  [axes movement](https://youtu.be/Nw18B81Ylhk)
* The pulse counters in esp32 have several comparators to trigger interrupts. What
  the documentation does not mention: All those reference values are only
  forwarded to the actual comparator on pulse counter reset. Thus the pulse
  counters cannot be used as lower 16bit of the position, unfortunately.
* The [issue #60](https://github.com/gin66/FastAccelStepper/issues/60) was raised
  due to wrong position on negative moves with esp32. Apparently the issue was
  with proper ground and/or power lines to the stepper driver. If similar issue
  is encountered, please check on this issue
* ESP32C3: USBSerial works only under Arduino IDE. platformio support for
  USBSerial is missing
