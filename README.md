# FastAccelStepper

![GitHub tag](https://img.shields.io/github/v/tag/gin66/FastAccelStepper.svg?sort=semver&no_cache_0.28.1)
[![PlatformIO Registry](https://badges.registry.platformio.org/packages/gin66/library/FastAccelStepper.svg)](https://registry.platformio.org/libraries/gin66/FastAccelStepper)
[![arduino-library-badge](https://www.ardu-badge.com/badge/FastAccelStepper.svg?)](https://www.ardu-badge.com/FastAccelStepper)
![Run tests](https://github.com/gin66/FastAccelStepper/workflows/Run%20tests/badge.svg?no_cache_0.28.1)
![Simvar tests](https://github.com/gin66/FastAccelStepper/workflows/Run%20tests%20with%20simavr/badge.svg?no_cache_0.28.1)
[![Build examples](https://github.com/gin66/FastAccelStepper/actions/workflows/build_arduino_examples_matrix.yml/badge.svg)](https://github.com/gin66/FastAccelStepper/actions/workflows/build_arduino_examples_matrix.yml)
[![Build espidf](https://github.com/gin66/FastAccelStepper/actions/workflows/build_idf_examples_matrix.yml/badge.svg)](https://github.com/gin66/FastAccelStepper/actions/workflows/build_idf_examples_matrix.yml)

## Overview

This is a high speed alternative for the
[AccelStepper library](http://www.airspayce.com/mikem/arduino/AccelStepper/).
Supported are avr (ATmega 168/328/P, ATmega2560, ATmega32u4), atmelsam due,
Microchip SAMD51, esp32, esp32s2, esp32s3, esp32c3, esp32c6, esp32p4, Raspberry
pi pico and pico2, and (experimentally) Teensy 4.0/4.1.

The stepper motors are connected via a driver IC (like A4988) with a 1, 2 or
3-wire connection (step / direction / enable). See
[Hardware & Pin Connection](extras/doc/hardware.md) for the pin restrictions of
each platform.

There is an excellent AI-generated description of FastAccelStepper with deep
implementation details and diagrams at
[deepwiki.com/gin66/FastAccelStepper](https://deepwiki.com/gin66/FastAccelStepper).
For memory footprint information across supported architectures, see the
[Memory Report](extras/doc/memory_report.md).

## Supported platforms

| Platform | Max steps/s | Steppers | Queue | Doc |
|----------|-------------|----------|-------|-----|
| AVR ATmega 168/328 | 50000 | 1-2 | 16 | [avr.md](extras/doc/platforms/avr.md) |
| AVR ATmega32u4 | 50000 | 3 | 16 | [avr.md](extras/doc/platforms/avr.md) |
| AVR ATmega2560 | 50000 | 3 | 16 | [avr.md](extras/doc/platforms/avr.md) |
| ESP32 (IDF 4.x) | 200000 | 14 | 32 | [esp32.md](extras/doc/platforms/esp32.md) |
| ESP32 (IDF 5.3+) | 200000 | 14 + 32 (I2S) | 32 | [esp32.md](extras/doc/platforms/esp32.md) |
| ESP32-S2 | 200000 | 4 + 32 (I2S) | 32 | [esp32.md](extras/doc/platforms/esp32.md) |
| ESP32-S3 | 200000 | 4 + 32 (I2S) | 32 | [esp32.md](extras/doc/platforms/esp32.md) |
| ESP32-C3 | 200000 | 2 + 32 (I2S) | 32 | [esp32.md](extras/doc/platforms/esp32.md) |
| ESP32-C6 | 200000 | 2 + 32 (I2S) | 32 | [esp32.md](extras/doc/platforms/esp32.md) |
| ESP32-P4 | 200000 | 4 + 32 (I2S) | 32 | [esp32.md](extras/doc/platforms/esp32.md) |
| Raspberry Pi Pico | 200000 | 4 (8 w/ riscv) | 32 | [pico.md](extras/doc/platforms/pico.md) |
| Raspberry Pi Pico 2 | 200000 | 8 (12 w/ riscv) | 32 | [pico.md](extras/doc/platforms/pico.md) |
| Atmel SAM Due | 50000 | 6 | 32 | [sam.md](extras/doc/platforms/sam.md) |
| Microchip SAMD51 | — | 3-5 | 32 | [samd51.md](extras/doc/platforms/samd51.md) |
| Teensy 4.0/4.1 (exp.) | 200000 | 16 | 32 | [teensy.md](extras/doc/platforms/teensy.md) |

The full ESP32 driver comparison (MCPWM/PCNT vs RMT vs I2S Mux) is in
[platforms/esp32.md](extras/doc/platforms/esp32.md).

## Features

* 1-pin operation for e.g. peristaltic pump => only positive move
* 2-pin operation for e.g. axis control
* 3-pin operation to reduce power dissipation of driver/stepper
* Lower limit of 260s per step @ 16MHz aka one step every four minute
  (esp32/avr/samd51), 198s for sam due
* fully interrupt/task driven - no periodic function to be called from application loop
* supports acceleration and deceleration with per stepper max speed/acceleration
* Allows the motor to continuously run in the current direction until `stopMove()` is called.
* speed/acceleration can be varied while stepper is running (call to `move` or
  `moveTo` is needed in order to apply the new values)
* Constant acceleration control: In this mode the motor can be controled by
  acceleration values and with acceleration=0 will keep current speed
* Linear acceleration increase from/to standstill using cubic speed function -
  configurable by `setLinearAcceleration()`
* Jump start from standstill - configurable by `setJumpStart()`
* Auto enable mode: stepper motor is enabled before movement and disabled
  afterwards with configurable delays
* Enable pins can be shared between motors
* Direction pins can be shared between motors
* Configurable delay between direction change and following step
* External callback function can be used to drive the enable pins (e.g. connected
  to shift register) and, only esp32 derivates: the direction pins
* No float calculation (log2 representation in range -64..64 with 16bit integer
  representation and 1/512th resolution)
* Provide API to each steppers' command queue. Those commands are tied to timer
  ticks aka the CPU frequency!
* Command queue can be filled with commands and then started. This allows near
  synchronous start of several steppers for multi axis applications.

## Quick Start

```cpp
#include "FastAccelStepper.h"
#include "AVRStepperPins.h" // Only required for AVR controllers

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

More details in [Usage](extras/doc/usage.md) and the
[UsageExample.ino](examples/UsageExample/UsageExample.ino).

## Documentation

| Topic | Document |
|-------|----------|
| Installation (Arduino / PlatformIO / ESP-IDF) | [installation.md](extras/doc/installation.md) |
| Hardware & pin connection | [hardware.md](extras/doc/hardware.md) |
| Usage & auto enable | [usage.md](extras/doc/usage.md) |
| Move semantics & position wraparound | [move_semantics.md](extras/doc/move_semantics.md) |
| Multi-axis applications | [multi_axis.md](extras/doc/multi_axis.md) |
| API reference | [FastAccelStepper_API.md](extras/doc/FastAccelStepper_API.md), [FastAccelStepper.h](src/FastAccelStepper.h) |
| Driver architecture | [driver_architecture.md](extras/doc/driver_architecture.md) |
| Ramp generator | [ramp.md](extras/doc/ramp.md), [ramp_cubic_quadratic.md](extras/doc/ramp_cubic_quadratic.md) |
| Planner modes (AFAP vs timed) | [planner_modes.md](extras/doc/planner_modes.md) |
| ESP32 I2S driver | [esp32_i2s_driver.md](extras/doc/esp32_i2s_driver.md) |
| Pico PIO program flow | [pico_pio.md](extras/doc/pico_pio.md) |
| SAMD51 design notes | [samd51/CONTEXT.md](extras/doc/samd51/CONTEXT.md) |
| n-axis whitepaper | [n_axes_whitepaper.md](extras/doc/n_axes_whitepaper.md) |
| Physical stepper simulation | [physical_stepper_whitepaper.md](extras/doc/physical_stepper_whitepaper.md) |
| Test strategy | [testing.md](extras/doc/testing.md) |
| Troubleshooting | [troubleshooting.md](extras/doc/troubleshooting.md) |
| Showcase (videos) | [showcase.md](extras/doc/showcase.md) |
| Contributors & supporters | [contributors.md](extras/doc/contributors.md) |
| Memory footprint | [memory_report.md](extras/doc/memory_report.md) |
| Changelog | [CHANGELOG.md](CHANGELOG.md) |
| TODO / roadmap | [GitHub project](https://github.com/gin66/FastAccelStepper/projects/1) |

## Source Code Structure

| Directory | Purpose |
|-----------|---------|
| `fas_arch/` | Platform abstraction: compiler/framework detection, interrupt macros, Arduino-like polyfills |
| `pd_avr/` | AVR pulse driver: timer-based stepper control for ATmega chips |
| `pd_esp32/` | ESP32 pulse driver: RMT, MCPWM/PCNT, and I2S-based stepper control |
| `pd_pico/` | RP2040/RP2350 pulse driver: PIO-based stepper control |
| `pd_sam/` | SAM Due pulse driver: timer-based stepper control |
| `pd_samd/` | SAMD51 pulse driver: TCC PWM-based stepper control |
| `pd_teensy/` | Teensy 4.0/4.1 pulse driver: QuadTimer-based stepper control (EXPERIMENTAL) |
| `pd_test/` | Test platform pulse driver: PC-based testing simulation |
| `fas_queue/` | Queue implementation: command queue management |
| `fas_ramp/` | Ramp calculation: acceleration/deceleration curves |
| `log2/` | Log2 representation: fixed-point arithmetic for speed calculations |

Platform-specific configuration constants (queue sizes, timing, feature flags)
are defined in `pd_*/pd_config.h` files, which are included by
`fas_arch/common.h` during the platform dispatch. See
[driver_architecture.md](extras/doc/driver_architecture.md) for details.

## Project status

* Arduino core v3.0.x uses esp-idf v5.0 up to v5.1 and FastAccelStepper will fail
  to compile. Arduino core 3.1.0 supports ESP-IDF V5.3.0 (based on RC1).
* TODO / roadmap: [GitHub project](https://github.com/gin66/FastAccelStepper/projects/1)
* Known issues and debugging tips: [troubleshooting.md](extras/doc/troubleshooting.md)

## Star History

[![Star History Chart](https://star-history.dera.page/svg?repos=gin66/FastAccelStepper&type=Date)](https://star-history.dera.page/#gin66/FastAccelStepper&Date)

## Contributing

Contributions are welcome. See
[contributors.md](extras/doc/contributors.md) for the list of people who helped.
Before committing, format the code with `bash extras/scripts/format_code.sh` and
run the PC-based tests with `make test` in `extras/tests/pc_based`.
