# Raspberry Pi Pico / Pico 2 Platform

[Back to README](../../../README.md) | [Platform index](../hardware.md) | [PIO details](../pico_pio.md)

## Usage limits

* allows up to 200000 generated steps per second
* RP2040 has two PIO blocks and RP2350 has three. Each block has four state
  machines, and one loaded stepper program serves all four. The builds offer
  8 steppers on the Pico and 12 on the Pico 2.
* RISC-V is the Pico 2 only. The RP2040 has no RISC-V core.
* Measured with `-D__FREERTOS=1` on 2026-11-08. The Pico connects 4 of 8.
  The Pico 2 Cortex image connects 8 of 12. On both boards one PIO block
  already holds the core's 2-word cycle counter, and the next connect fails.
  The mechanism is in the next section.
* Steppers' command queue depth: 32
* Step signal can be any GPIO up to 31.
* Direction delay is recommended.

## Implementation

Uses the pio module. Pico (rp2040) offers two pios and pico 2 (rp2350) offers
three pios. Each pio contains four state machines and every state machine can
drive one stepper. So in theory can allocate 8 steppers for pico and 12 steppers
for pico 2.

SoftwareSerial is implemented via pio, too. Another program that has already
used instruction words in a block leaves that block unusable for
FastAccelStepper.

## Cycle-counter state machine

Checked against the core installed for these PlatformIO builds:
`~/.platformio/packages/framework-arduinopico`, `platform.txt`
`version=6.1.0`. The older link in this file (earlephilhower blob `b8864711`)
is a different revision of the same header.

**Verified in that 6.1.0 source.** `cores/rp2040/RP2040Support.h`,
`RP2040::begin()`:

```c
#if !defined(__riscv) && !defined(__PROFILE) && !defined(__FREERTOS)
    // ARM SysTick
#else
    // core 0 only: PIOProgram(&ccount_program)->prepare(), then enable
#endif
```

`getCycleCount()` uses the same condition. `cores/rp2040/ccount.pio.h` sets
`.length = 2` (`jmp x--, 0` and `jmp y--, 0`). `main()` calls
`rp2040.begin(0)` before `setup()`. FastAccelStepper does not call
`getCycleCount()`. `micros()` and `delay()` use `time_us_32()`.

Hazard3 has no ARM SysTick. This core does not read `mcycle` / `rdcycle`.
No flag in `begin()` skips the program when `__riscv` is set.
`core1_disable_systick` only skips `begin(1)` on core 1.

`PIOProgram::prepare()` (`cores/rp2040/PIOProgram.cpp`) adds those 2 words
with `pio_add_program` at a free origin. A later 32-word program fails
`pio_can_add_program` on that block, so the block is gone, not one state
machine. `add_step()` in `src/pd_pico/pico_pio.cpp` stores at most 32 words,
and the stepper program is loaded at origin 0.

Every Pico environment in `extras/ci/platformio.ini` passes
`build_flags = -D__FREERTOS=1`. On 6.1.0 that macro is the preprocessor
symbol in the `#if` above (the core's own FreeRTOS menu uses `-D__FREERTOS`).
Cortex with the macro defined takes the PIO path. RISC-V takes it because
`__riscv` is set. RISC-V exists on the RP2350 only. The RP2040 image is
Cortex. With the counter loaded it connects 4 steppers, measured on
`/dev/cu.usbmodem4101`.

The previous wording on this page said the Arduino core allocates one PIO
when the build is not RISC-V, and that a RISC-V build can use every stepper.
That wording does not match `begin()` in 6.1.0.

**Verified on the `rpipico2` compile line (2026-11-08).**
`RP2040Support.cpp` is compiled with `-D__FREERTOS=1`. `build_flags` does
reach the core. The same command has `-DF_CPU=150000000L`. `boards.txt`
in the core package says 125 MHz for `rpipico2`; the PlatformIO board
the saleae env resolves overrides that to 150 MHz. One PIO cycle on this
firmware is 1/150 MHz.

**Measured on a Pico (RP2040), 2026-11-08.** Env `rpipico` with
`-D__FREERTOS=1`, port `/dev/cu.usbmodem4101`. No logic analyzer. The
sketch called `stepperConnectToPin()` for GPIO 0 through 7. Pins 0
through 3 succeeded and pins 4 through 7 failed: 4 steppers. Before any
connect, PIO0 was empty. PIO1 had one state machine claimed. A 30-word
program still fit there and a 31-word program did not, so that block
held 2 instruction words. After the four connects, PIO0 had four state
machines claimed and accepted no further program. PIO1 was unchanged.

**Measured on a Pico 2, Cortex, 2026-11-08.** Env `rpipico2` with
`-D__FREERTOS=1`, port `/dev/cu.usbmodem833201`. A throwaway sketch called
`stepperConnectToPin()` for GPIO 0 through 11. Pins 0 through 7 succeeded
and pins 8 through 11 failed: 8 steppers. Before any connect, PIO0 and
PIO1 were empty (`pio_can_add_program` accepted 32 words). PIO2 had one
state machine claimed. A 30-word program still fit there and a 31-word
program did not, so that block held 2 instruction words. After the eight
connects, PIO0 and PIO1 each had four state machines claimed and accepted
no further program. PIO2 was unchanged. Instruction memory reads back as
zero, so the word count is this fit test, not a read of `instr_mem`.

The Saleae firmware cannot make this count: `SALEAE_STEPPER_BOUND` in
`extras/tests/saleae_based/common/saleae_app.cpp` is 8, so `CONFIG 9` is
refused by the test app.

**Measured on a Pico 2, RISC-V, without `-D__FREERTOS=1`, 2026-11-08.**
USB comes up on that image. The image built with `-D__FREERTOS=1` still
does not. Before any stepper program is added, PIO2 has one state
machine claimed. A 30-word program fits there and a 31-word program
does not, so that block holds 2 instruction words. PIO0 and PIO1 are
empty. The stepper program is 32 words at origin 0, so it cannot be
loaded into the block that holds the cycle counter.

`pio_can_add_program` builds its occupancy mask with `1u << length`.
A shift of 32 is undefined. This RISC-V compiler emits `bset` of bit
`length & 31`, so a length of 32 produces a mask of 0. The 32-word
program is then reported as fitting, and the write is not recorded as
used. A probe that trusted that check claimed 11 state machines and
wrote the 32-word image over the counter. Those 11 do not coexist with
the counter.

Upstream arduino-pico master, which this tree does not build, tests a
runtime `__isFreeRTOS` on Cortex and still always takes the PIO path on
RISC-V. 6.1.0 tests the macro at compile time.

## Non-Arduino

No Pico environment in this tree uses a `framework` other than `arduino`.

`src/fas_arch/common.h` also has a branch on `PICO_SDK_RP2350`. That macro
is not defined by pico-sdk or by framework-arduinopico 6.1.0 (it occurs only
in this repository). The header it selects, `src/fas_arch/pico_sdk_rp_pico.h`,
calls FreeRTOS `portENTER_CRITICAL` / `portEXIT_CRITICAL`, and
`src/pd_pico/pd_config.h` includes `FreeRTOS.h`. That branch is not a
bare-metal port, and it is not built in CI.

The cycle counter is started by the Arduino class `RP2040::begin()`. A
pico-sdk image does not call it. In the pico-sdk shipped inside
framework-arduinopico 6.1.0, the only `pio_add_program` is the implementation
in `hardware_pio/pio.c`. A non-Arduino image therefore keeps every PIO block,
on Cortex and on RISC-V. FastAccelStepper still has to be ported to build
that way: the hook that exists today requires FreeRTOS, and no such platform
is packaged here.

`platform = raspberrypi` from the PlatformIO registry is a separate failure
(issue #339). It does not avoid the cycle counter.

## Cortex-M33 and Hazard3

Both cores share `clk_sys`. The boot ROM starts one architecture. They do
not run at the same time.

Step edges are produced by the PIO at `clk_sys`. Comparing step periods
between `rpipico2` and `rpipico2-riscv` on a logic analyzer measures the PIO
clock, which is the same clock. It does not measure which CPU is faster.
The comparison that tests this page is how many steppers connect.

Which core finishes a given integer workload first depends on that code.
A secondary CoreMark table (kreier.github.io/benchmark/CoreMark, update
2024-08-09, not a run from this repository) lists about 4.02/MHz for the
M33 and 3.81/MHz for Hazard3 at the 150 MHz rating. The M33 has a hardware
FPU and Hazard3 does not (`boards.txt` RISC-V flags are
`rv32imac`, integer only). The FastAccelStepper ramp is integer arithmetic,
so the FPU is not on the queue-fill path. The step-rate limit stays the PIO
unless the queue underruns.

`boards.txt` for `rpipico2` sets `build.f_cpu=125000000`. The Saleae
firmware's `QINFO` line does not report `clock_get_hz(clk_sys)`. Read the
clock from the board, or from `-DF_CPU` on the compile line, before turning
a capture into nanoseconds. One PIO cycle is `1/clk_sys` either way.

Integration with applications using pio: FastAccelStepper claims always a complete
pio. This means all four state machines are not available for the app. With every
block free, the second pio is claimed for the fifth stepper, and on pico 2 the
third is claimed for the ninth. When the cycle-counter program above already
holds one block, the ninth connect fails and pico 2 stops at eight. Unused state
machines of a claimed pio cannot be used, because FastAccelStepper's pio code
needs 100% of the available program space (32 words - none left).

## Loop budget, measured

The command duration matches the requested time. `pio_calc_loops()`
subtracts the 3-cycle preamble, solves for the period, and adds the
preamble back into `actual_cycles`. That is the time the state machine
spends: the preamble once, then `loop_cnt` passes of `26 + 3*period`.
`adjust_80MHz` keeps `requested - actual_cycles` for the next command.

The period moves in steps of 3 cycles, and a step is two passes, so one
command lands on a 6-cycle grid. A single command can be a few cycles
off the request. The following commands take up the remainder, and a
run of the same request adds up to the requested duration. The preamble
stays in front of the first pass, so the first step of a multi-step
command is 3 cycles longer than each later step. Those 3 cycles are
inside the command length.

One step of 3200 ticks is 200 µs, 30000 cycles at 150 MHz. This tree
emits 29995 cycles, then 30001 five times. Those six commands are
180000 cycles, and thirty of them are exactly 30 × 200 µs. Tag 2.0.1
left the preamble out of `actual_cycles` while the machine still
executed it, so the adjustment was taken from a short budget. The same
six commands there are 18 cycles long, and thirty commands are 90
cycles long.

The loaded 32-word program is unchanged. `add_step()` stores nothing
past word 32.

Measured 2026-11-08 on the Pico 2 saleae firmware, env `rpipico2`,
`-DF_CPU=150000000L`. GPIO2 (analyzer D0), 30 one-step commands of 3200
ticks (`QFILL` kept 30 of 32). 48 MS/s on this fx2lafw returns a few
thousand samples and then times out on USB; 24 MS/s held a 200 ms
window. Thirty rising edges give 29 intervals, command lengths 0
through 28. At 24 MS/s, 200 µs is 4800 samples, so 29 × 200 µs is
139200 samples. This tree's budget for those 29 lengths is 869999
cycles, 139199.8 samples: one cycle under, because command 30 holds
the extra cycle.

| Firmware | Sum of the 29 intervals |
|---|---|
| this tree | 139197 samples |
| `pico_pio.cpp` / `pico_pio.h` from tag 2.0.1, rest of the tree at HEAD | 139210 samples |

139197 samples is 125 ns under 29 × 200 µs. The tag capture is 10
samples over that request. The gap between the two captures is 13
samples, 542 ns. The two budgets differ by 84 cycles, 560 ns, 13.4
samples. Each capture sits about 3 samples under its own budget, so
that offset is in the capture and cancels in the gap. A single step,
and the catalogue line `period 10.0 us`, do not show it: one short
command is about one sample at 24 MS/s. The capture record is
[pico_pio_loop_timing.md](../implemented/pico_pio_loop_timing.md).

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
