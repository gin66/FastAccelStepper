# FastAccelStepper

![GitHub tag](https://img.shields.io/github/v/tag/gin66/FastAccelStepper.svg?sort=semver&no_cache_0.28.1)
[![PlatformIO Registry](https://badges.registry.platformio.org/packages/gin66/library/FastAccelStepper.svg)](https://registry.platformio.org/libraries/gin66/FastAccelStepper)
[![arduino-library-badge](https://www.ardu-badge.com/badge/FastAccelStepper.svg?)](https://www.ardu-badge.com/FastAccelStepper)
![Run tests](https://github.com/gin66/FastAccelStepper/actions/workflows/test.yml/badge.svg)
![Simvar tests](https://github.com/gin66/FastAccelStepper/actions/workflows/test_avr.yml/badge.svg)
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

| Platform | Max step rate [kSteps/s] | Steppers | Cmd queue depth | Doc |
|----------|--------------------------|----------|-----------------|-----|
| AVR ATmega 168/328 | 50 | 1-2 | 16 | [avr.md](extras/doc/platforms/avr.md) |
| AVR ATmega32u4 | 50 | 3 | 16 | [avr.md](extras/doc/platforms/avr.md) |
| AVR ATmega2560 | 50 | 3 | 16 | [avr.md](extras/doc/platforms/avr.md) |
| ESP32 (IDF 4.x) | 200 | 14 (6 MCPWM/PCNT + 8 RMT) | 32 | [esp32.md](extras/doc/platforms/esp32.md) |
| ESP32 (IDF 5.3+) | 200 [1] | 14 (6 MCPWM/PCNT + 8 RMT) + 32 I2S * | 32 | [esp32.md](extras/doc/platforms/esp32.md) |
| ESP32-S2 | 200 [1] | 4 RMT + 32 I2S * | 32 | [esp32.md](extras/doc/platforms/esp32.md) |
| ESP32-S3 | 200 [1] | 4 MCPWM/PCNT + 4 RMT + 32 I2S * | 32 | [esp32.md](extras/doc/platforms/esp32.md) |
| ESP32-C3 | 200 [1] | 2 RMT + 32 I2S * | 32 | [esp32.md](extras/doc/platforms/esp32.md) |
| ESP32-C6 | 200 [1] | 2 MCPWM/PCNT + 2 RMT + 32 I2S * | 32 | [esp32.md](extras/doc/platforms/esp32.md) |
| ESP32-P4 | 200 [1] | 4 RMT + 32 I2S * [2] | 32 | [esp32.md](extras/doc/platforms/esp32.md) |
| Raspberry Pi Pico | 200 | 4 (8 w/ riscv) | 32 | [pico.md](extras/doc/platforms/pico.md) |
| Raspberry Pi Pico 2 | 200 | 8 (12 w/ riscv) | 32 | [pico.md](extras/doc/platforms/pico.md) |
| Atmel SAM Due | 50 | 6 | 32 | [sam.md](extras/doc/platforms/sam.md) |
| Microchip SAMD51 | 100 | 3-5 | 32 | [samd51.md](extras/doc/platforms/samd51.md) |
| Teensy 4.0/4.1 (exp.) | 200 | 16 | 32 | [teensy.md](extras/doc/platforms/teensy.md) |

Notes:

1. 200 kSteps/s applies to the MCPWM/PCNT and RMT drivers. I2S Mux is
   limited to 40 kSteps/s; I2S Direct channels reach up to 200 kSteps/s.
2. The ESP32-P4 hardware provides 2 MCPWM groups and 4 PCNT units, but
   MCPWM/PCNT is not implemented for P4 yet. ESP32-S2 has PCNT but no MCPWM,
   ESP32-C3 has neither, so only RMT/I2S are available there.

\* The 32 I2S steppers require an external demultiplexer (e.g. 74HC154, or a
74HC138 cascade / 74HC595 shift registers) on the I2S data line. Using I2S
slots directly (I2S Direct mode) provides only 1-3 extra steppers.

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
* **EXPERIMENTAL** multi-axis planner `FasNAxis` (`src/FasNAxis.h`): drives N
  stepper queues from one polyline so the axes stay time-synchronized. API not
  stable yet. See the
  [n-axis whitepaper](extras/doc/n_axes_whitepaper.md), the
  [NaxesAFAP example](examples/NaxesAFAP/README.md), and
  [Future work](#future-work).

## Quick Start

```cpp
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

More details in [Usage](extras/doc/usage.md) and the
[UsageExample.ino](examples/UsageExample/UsageExample.ino).

## Multi-axis planner (FasNAxis)

> **Experimental.** The `FasNAxis` API is **not stable yet** and may change
> without notice. It runs a polyline as fast as the motors and geometry allow,
> with no requested speed or time. The call list is
> [FasNAxis.md](extras/doc/FasNAxis.md). A waypoint that carries its own
> duration is [FasTimed](extras/doc/FasTimed.md). Other open work is listed
> under [Future work](#future-work).

For coordinated motion, `FasNAxis` drives N stepper queues from one polyline
so the axes stay time-synchronized. The hot path has no float, division, or
64-bit integers. Full example: [examples/NaxesAFAP](examples/NaxesAFAP/README.md); the
theory is in the [n-axis whitepaper](extras/doc/n_axes_whitepaper.md).

`FasNAxis` is `FasNAxis<NAXES, HORIZON = 64, Stepper = FastAccelStepper,
Engine = FastAccelStepperEngine>` (`src/FasNAxis.h`). `HORIZON` caps how many
future polyline points are buffered and planned: a small value slows the track
(a speed cap, never an error — see `isSpeedLimitedByLookahead()`), a larger one
smooths the ramp at the cost of RAM.

```cpp
#include "FastAccelStepper.h"
#include "FasNAxis.h"   // not pulled in by FastAccelStepper.h

FastAccelStepperEngine engine = FastAccelStepperEngine();
FastAccelStepper* axes[2] = {NULL, NULL};
FasNAxis<2, 32> planner(FasNAxisConfig{}, engine);   // 2 axes, HORIZON = 32

void setup() {
   engine.init();
   axes[0] = engine.stepperConnectToPin(9);
   axes[1] = engine.stepperConnectToPin(10);
   for (uint8_t i = 0; i < 2; i++) {
      axes[i]->setDirectionPin(5 + i);
      axes[i]->setSpeedInHz(4000);      // steps/s
      axes[i]->setAcceleration(2000);   // steps/s²
      planner.addAxis(i, axes[i]);
   }
   planner.setLimitsFromSteppers();     // freeze v_max / a_max per axis

   int32_t origin[2] = {0, 0};
   planner.setCurrentPosition(origin);  // open the path at the current position
}

void loop() {
   // Append exact-chord target vertices; backpressures when HORIZON is full.
   static const int32_t path[][2] = {
       {1000, 0}, {1000, 1000}, {0, 1000}, {0, 0}};
   static uint8_t wp = 0;
   if (wp < 4) {
      if (planner.addWaypoint(path[wp])) {
         wp++;
      }
      if (wp == 4) {
         planner.endPath();              // last point is rest
      }
   }
   planner.pump();   // plan and feed the committed path into the queues
}
```

## Documentation

| Topic | Document |
|-------|----------|
| Installation (Arduino / PlatformIO / ESP-IDF) | [installation.md](extras/doc/installation.md) |
| Hardware & pin connection | [hardware.md](extras/doc/hardware.md) |
| Usage & auto enable | [usage.md](extras/doc/usage.md) |
| Move semantics & position wraparound | [move_semantics.md](extras/doc/move_semantics.md) |
| Multi-axis applications | [multi_axis.md](extras/doc/multi_axis.md) |
| Stepper and engine API | [FastAccelStepper_API.md](extras/doc/FastAccelStepper_API.md), from `header2markdown.sh` on [FastAccelStepper.h](src/FastAccelStepper.h) and [FastAccelStepperEngine.h](src/FastAccelStepperEngine.h) |
| FasNAxis calls | [FasNAxis.md](extras/doc/FasNAxis.md) |
| FasTimed calls | [FasTimed.md](extras/doc/FasTimed.md), example [NaxesTimed](examples/NaxesTimed/README.md) |
| Driver architecture | [driver_architecture.md](extras/doc/driver_architecture.md) |
| Ramp generator | [ramp.md](extras/doc/ramp.md), [ramp_cubic_quadratic.md](extras/doc/ramp_cubic_quadratic.md) |
| Planner modes (AFAP vs timed) | [planner_modes.md](extras/doc/planner_modes.md) |
| ESP32 I2S driver | [esp32_i2s_driver.md](extras/doc/esp32_i2s_driver.md) |
| Pico PIO program flow | [pico_pio.md](extras/doc/pico_pio.md) |
| SAMD51 design notes | [samd51/CONTEXT.md](extras/doc/samd51/CONTEXT.md) |
| n-axis whitepaper | [n_axes_whitepaper.md](extras/doc/n_axes_whitepaper.md) |
| Future work | [extras/todo/README.md](extras/todo/README.md) |
| Physical stepper simulation | [physical_stepper_whitepaper.md](extras/doc/physical_stepper_whitepaper.md) |
| Test strategy | [testing.md](extras/doc/testing.md) |
| Troubleshooting | [troubleshooting.md](extras/doc/troubleshooting.md) |
| Showcase (videos) | [showcase.md](extras/doc/showcase.md) |
| Contributors & supporters | [contributors.md](extras/doc/contributors.md) |
| Memory footprint | [memory_report.md](extras/doc/memory_report.md) |
| Changelog | [CHANGELOG.md](CHANGELOG.md) |

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
| `fas_naxis/` | Experimental multi-axis planner (`FasNAxis`), header-only |
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

## Future work

Open work is one file per item in
[`extras/todo`](extras/todo/README.md). The filename prefix is the priority,
three digits wide (`050_name.md`). Numbers step by 10, so a new item can take
a free number between two existing ones.

The list is library-wide. It currently covers per-platform synchronized
start, a cubic ramp start, delta-step input for `FasNAxis`, a single-axis
`moveTo` with an arrival in ticks, and a smooth stop at the end of a path.
Timed chunks are [FasTimed](extras/doc/FasTimed.md). The n-axis design is the
[whitepaper](extras/doc/n_axes_whitepaper.md).

## Star History

[![Star History Chart](https://star-history.dera.page/svg?repos=gin66/FastAccelStepper&type=Date)](https://star-history.dera.page/#gin66/FastAccelStepper&Date)

## A Bit of Nostalgia


For FastAccelStepper, the way software is developed is changing fundamentally. It has been a long way from 2020 to today. Back then, I simply needed a fast stepper driver for AVR, and with my embedded know-how I went for it, hand-optimizing the ISR code until I was satisfied. Then I added ESP32 support and found several ways to generate the pulses. Today I no longer write the documentation and implementation code myself; LLMs and coding agents are increasingly doing that work. Almost inadvertently, my role is shifting toward defining requirements, consulting on the architecture, selectively reviewing the generated code, and testing it.

Nowadays, coding agents can understand the existing code base surprisingly well and carry out most of the required modifications. This makes implementing new features significantly faster.

That said, I still take pride in hand-writing the Pico PIO code. There is something deeply satisfying about getting all the required functionality for a step/direction pin into just 50 words of PIO code. That kind of low-level optimization and craftsmanship is something I value.

Perhaps there is also a little nostalgia in this. It feels like one of those moments when a craft that has shaped the way you have worked throughout your lifetime is slowly becoming obsolete — not because it has lost its value, but because there is now a faster and more powerful way of achieving the same result. Like the expert typist who could produce flawless pages at remarkable speed, the old skills don't necessarily become less beautiful; they simply become less necessary.

Those 50 words of PIO code are perhaps one of the last pieces of FastAccelStepper where I can say, quite honestly, *I wrote every word myself.*

## Contributing

Contributions are welcome. See [contributors.md](extras/doc/contributors.md) for the list of people who have helped.

Before committing, format the code with `bash extras/scripts/format_code.sh` and run the PC-based tests with `make test` in `extras/tests/pc_based`.

With implementation becoming faster, testing is increasingly becoming the bottleneck for maintaining quality. Any new feature idea should therefore ideally come with a patch and some level of automated testing—preferably in a `pc_based` or `simavr_based` form.

The current architecture makes it straightforward to add new multi-axes planners and test them on a PC. Adding a new pulse driver, however, requires testing on real hardware. Any contribution that introduces a new PD instance must therefore include confirmation that it has been tested on the relevant hardware.

It would be great to see the introduction of more cycle-accurate, embeddable emulators that can run in GitHub Actions. `simavr` was, and still is, extremely valuable to me.

Having said that, I want to point out that contributions using float, double, or 64-bit arithmetic in production code will not make it into the codebase.
