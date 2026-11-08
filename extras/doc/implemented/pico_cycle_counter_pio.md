# Pico cycle counter occupies one PIO block

Closes todo **032**. The user-facing account is
[pico.md](../platforms/pico.md), section "Cycle-counter state machine".
The follow-up is implemented as
[033](pico_pio_block_claim.md).

## Result

Arduino-Pico 6.1.0 loads a 2-word `ccount` program on core 0 when
`__riscv`, `__PROFILE`, or `__FREERTOS` is set
(`RP2040::begin()` in `cores/rp2040/RP2040Support.h`). These builds
set `-D__FREERTOS=1`. RISC-V has no SysTick path, so it loads the
program whether or not that flag is set. The stepper program is 32
words at origin 0, so that block cannot hold it.

Measured 2026-11-08 with `-D__FREERTOS=1`:

| Board | Image | Connects |
|---|---|---|
| Pico (RP2040) | `rpipico` | 4 of 8 |
| Pico 2 | `rpipico2` | 8 of 12 |

On both, one block already had one state machine claimed. A 30-word
program fit there and a 31-word program did not.

The Pico 2 RISC-V image with `-D__FREERTOS=1` never started USB, so
`stepperConnectToPin()` was not counted on that build. Without the
flag, USB comes up and PIO2 shows the same 2-word block. A length-32
`pio_can_add_program` is not evidence of free space on that compiler:
`1u << 32` becomes a mask of 0.
