# Raspberry Pi Pico / Pico 2 Platform

[Back to README](../../../README.md) | [Platform index](../hardware.md) | [PIO details](../pico_pio.md)

## Usage limits

* allows up to 200000 generated steps per second
* In theory supports up to eight stepper motors for pico and twelve stepper
  motors for pico 2. Using arduino framework and arm core, only four for pico and
  eight for pico 2 can be allocated. riscv should be able to allocate all
  steppers.
* Steppers' command queue depth: 32
* Step signal can be any GPIO up to 31.
* Direction delay is recommended.

## Implementation

Uses the pio module. Pico (rp2040) offers two pios and pico 2 (rp2350) offers
three pios. Each pio contains four state machines and every state machine can
drive one stepper. So in theory can allocate 8 steppers for pico and 12 steppers
for pico 2.

Arduino framework allocates one pio module, if not compiled for
[riscv](https://github.com/earlephilhower/arduino-pico/blob/b8864711cd7801b7062c10f6b84d67f90284723b/cores/rp2040/RP2040Support.h#L206).
Then only 4 resp. 8 steppers are available. SoftwareSerial is implemented via pio,
too. So the number of available steppers depends on the sw configuration.

Integration with applications using pio: FastAccelStepper claims always a complete
pio. This means all four state machines are not available for the app. The second
pio will be claimed, when allocating a fifth stepper. The third - on pico 2, when
allocating the nineth stepper. Unused state machines of a pio cannot be used,
because FastAccelStepper's pio code needs 100% of the available program space
(32 words - none left).

Important: Without direction delay set up, the time between a direction transition
to step low to high transition can be 50ns @ 80 MHz. Best to call
`setDirectionPin()` with time parameter for delay.

Important to note: The standard pico platform appears to not work with pio
(#339). So please use an alternative platform for rp2040 like:

```
[env:rpipico]
platform = https://github.com/maxgerhardt/platform-raspberrypi.git
framework = arduino
board = rpipico
build_flags = -Wall -Wextra -D__FREERTOS=1
```

and for rp2350:

```
[env:rpipico2]
platform = https://github.com/maxgerhardt/platform-raspberrypi.git
framework = arduino
board = rpipico2
build_flags = -Wall -Wextra -D__FREERTOS=1
```

See [pico PIO program flow](../pico_pio.md) for the generated state machine
diagram.
