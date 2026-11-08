# Pico cycle-counter PIO — one block gone, RISC-V and FreeRTOS

Priority: **032**. Published stepper counts in
`extras/doc/platforms/pico.md` and the README follow from this.

Status: Pico 2 Cortex measured 2026-11-08. Env `rpipico2` with
`-D__FREERTOS=1` connected 8 steppers. Before any connect, one PIO
block had one state machine claimed and exactly 2 instruction words
used (a 30-word program fit, a 31-word program did not). The other two
blocks were empty and then took four steppers each. `rpipico2-riscv` with `-D__FREERTOS=1` does not start USB, so that
library image was not counted. Without that flag, PIO2 holds 2
instruction words (a 30-word program fits, a 31-word program does not)
and one state machine. A length-32 check is not usable on this RISC-V
compiler: `1u << 32` becomes a mask of 0. RP2040 has no RISC-V core. Env `rpipico` with
`-D__FREERTOS=1` on `/dev/cu.usbmodem4101` connected 4 steppers. PIO1
held 2 instruction words before any connect; PIO0 then took four.
Findings are also in `extras/doc/platforms/pico.md`.

## Why the state machine exists

**Verified in source (framework-arduinopico 6.1.0).**
`RP2040::begin()` in `cores/rp2040/RP2040Support.h` enables ARM SysTick
only when `__riscv`, `__PROFILE`, and `__FREERTOS` are all undefined.
Every other build, on core 0, loads `ccount`. `ccount.pio.h` in that
package sets `.length = 2`: `jmp x--, 0` and `jmp y--, 0`.
`getCycleCount()` uses the same `#if` and then calls `ccount_read()`.

Hazard3 has no ARM SysTick, so a RISC-V build always takes the PIO
path. The same path is taken for `__PROFILE`, and for a Cortex build
that defines `__FREERTOS`. 6.1.0 tests the macro at compile time. It
does not consult a runtime `__isFreeRTOS`. `_freertos.h` includes the
FreeRTOS headers under `#ifdef __FREERTOS` and does not define
`__isFreeRTOS`.

Upstream arduino-pico master (not the package this tree builds) uses
`#if !defined(__riscv) && !defined(__PROFILE)` and then a runtime
`__isFreeRTOS` test. RISC-V still always loads `ccount` there.

Hazard3 implements `rdcycle` / `mcycle`. This Arduino core does not
read it. `mcountinhibit.CY` left set holds `mcycle` at 0, so a
replacement has to clear that bit. That inhibit bit was not re-checked
in the 6.1.0 headers during this pass; treat it as a note from the
architecture, not as a line cited from this package.

`micros()` and `delay()` use `time_us_32()`. FastAccelStepper does not
call `getCycleCount()`. `main()` calls `rp2040.begin(0)` before
`setup()`, and that is what claims the program.

## Can it be turned off?

**Verified in 6.1.0 `begin()`.** No switch skips the program on a
RISC-V build. `core1_disable_systick` only skips `rp2040.begin(1)`.

On Cortex the program stays unloaded when `__FREERTOS` and
`__PROFILE` are both unset. Every FAS Pico env passes
`build_flags = -D__FREERTOS=1`, which is the symbol the `#if` tests.
The core menu spells the same symbol `-D__FREERTOS`.

**Verified on the compile line (2026-11-08, env `rpipico2`).**
`RP2040Support.cpp` is built with `-D__FREERTOS=1` and
`-DF_CPU=150000000L`. The Cortex image takes the PIO path. The RISC-V
image takes it because `__riscv` is set, whether or not the macro is
present. On the Pico 2 Cortex image the measured connect count is 8.
The FreeRTOS RISC-V image does not start USB. Without that flag the
RISC-V boot state is the same 2-word block. An RP2040 with this flag connected
4 steppers, measured 2026-11-08 on `/dev/cu.usbmodem4101`.

The program is 2 words and 1 state machine. `PIOProgram::prepare()`
adds it with `pio_add_program`. FastAccelStepper's program is 32 words
at origin 0, so `pio_can_add_program` fails for that block. An RP2350
then has 8 steppers left, an RP2040 has 4. The Pico 2 Cortex count
was measured on 2026-11-08 and is 8. The RP2040 count was measured
the same day and is 4.

Freeing the block is a core change: leave `ccount` unloaded when the
application does not call `getCycleCount()`, or read `mcycle` instead.
Neither exists as a flag today. Patching it out of a RISC-V image makes
`getCycleCount()` / `getCycleCount64()` read an SM that was never
started.

`extras/doc/platforms/pico.md` and the README stepper column now follow
this `begin()`. The old "8 w/ riscv" / "12 w/ riscv" wording does not.

## Non-Arduino

**Verified in this tree and in the 6.1.0 package.** No Pico env uses
`framework` other than `arduino`. `PICO_SDK_RP2350` is not a pico-sdk
or arduino-pico macro; the only matches are in this repository.
`src/fas_arch/pico_sdk_rp_pico.h` still uses FreeRTOS
`portENTER_CRITICAL`, and `pd_config.h` includes `FreeRTOS.h`.

The counter starts in `RP2040::begin()`. A pico-sdk image does not
call that. The pico-sdk inside framework-arduinopico 6.1.0 has no
`pio_add_program` caller outside `hardware_pio/pio.c`. A non-Arduino
image keeps every PIO block, on both CPUs. There is no packaged
FastAccelStepper platform that builds that way. The registry platform
`raspberrypi` is issue #339, tracked in `034_pico_registry_platform.md`.

## RISC-V and Cortex speed

Step edges are the PIO at `clk_sys`. A Saleae comparison of step
periods between the two FAS environments does not measure CPU speed.
Both cores share the clock and only one architecture runs.

The comparison that belongs to this item is the stepper count. On
6.1.0 with `-D__FREERTOS=1` the source prediction is 8 for both.

Integer performance of the two cores is close and depends on the
workload. The M33 has an FPU (`-mfloat-abi=softfp` in `boards.txt`)
and Hazard3 is built `-march=rv32imac...` without one. The
FastAccelStepper ramp is integer, so the FPU is not on the queue-fill
path. The 200 kstep/s ceiling is the PIO unless the queue underruns.
A published CoreMark note (not a run from this repository) is cited
in `pico.md`. Re-running CoreMark here would not change the stepper
count or the step period.

## Work items

- Done: `ccount` length is 2. 6.1.0 selects it with
  `#if defined(__FREERTOS)`, not with `__isFreeRTOS`. The `rpipico2`
  command for `RP2040Support.cpp` contains `-D__FREERTOS=1` and
  `-DF_CPU=150000000L`.
- The Saleae `CONFIG` command cannot probe 9..12.
  `SALEAE_STEPPER_BOUND` is `min(MAX_STEPPER, 8)`, so `CONFIG 9` is
  refused by the test firmware. A throwaway sketch that calls
  `stepperConnectToPin()` twelve times and prints the count is the
  measurement. Expect 8 accepted and the 9th refused, on Cortex and
  on `rpipico2-riscv`, if the assumption about `build_flags` holds.
  One process per boot.
- If a Cortex image links without `-D__FREERTOS=1`, that image is the
  SysTick case and should accept 12. `pd_config.h` includes
  `FreeRTOS.h`, so the link may fail first.
- Write the counted connects back here. `pico.md` and the README
  already describe the 6.1.0 prediction and still say the count is
  unmeasured.

## References

- earlephilhower `cores/rp2040/RP2040Support.h` — `RP2040::begin()`,
  `getCycleCount()`.
- earlephilhower `cores/rp2040/ccount.pio.h` — 2-word program.
- `extras/doc/platforms/pico.md` — the stepper-count sentence.
- `src/pd_pico/pico_queue.cpp` — `claim_pio_resources()` loads a 32-word
  program at origin 0, so a PIO with any words used is skipped.
- `extras/tests/saleae_based/reports/rpipico2_platform_matrix.md` —
  SR_31 stopped at 8.
