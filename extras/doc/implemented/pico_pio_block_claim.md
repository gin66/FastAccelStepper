# Pico — claim a whole PIO block before writing the program

Priority: **033**. Follows
[032](pico_cycle_counter_pio.md). The length-32 occupancy test in
the Pico SDK is wrong on RISC-V, so a stepper program can be written
over the cycle counter.

Status: **implemented** (2026-11-08). Measured on an RP2350: 12 steppers
connect without `__FREERTOS`, and 11 with it, because the core then spends a
PIO state machine on the cycle counter. The Saleae catalogue and scale sweep
pass, and the loop and direction timing analyses are recorded below. The
platform docs and the changelog are updated.

## Implementation

`src/pd_pico/` does both of the options below.

`claim_pio_resources()` claims every free state machine of a block,
from 1 to 4, before `pio_add_program`. The origin is −1. A failed add
unclaims the machines just claimed. A successful load keeps them.

The five direction words are gone, so the program is 27 words and
`LOOP_OVERHEAD` is 22. The direction pin is a CPU GPIO. One command
is already in flight and the TX FIFO holds four more.
`addDirChangePauseToQueue()` queues the missing old-direction pauses
(`PICO_DIR_DRAIN_WORDS` is 4, each `MIN_CMD_TICKS`, `count_up` from
`queue_end`) and returns `AQE_DIR_CHANGE_PAUSE_INJECTED`. The caller's
command stays unqueued. `AQE_QUEUE_FULL` inserts nothing.
`push_command()` puts the following word into the FIFO and writes the
pin only after that put, and only when the entry's level differs. A
full FIFO leaves the pin unchanged.

`BEFORE_DIR_CHANGE_DELAY_TICKS` is `4 * MIN_CMD_TICKS`. A planner pause
of that length is one word; the driver still adds the missing words.

The sentence in Alternative that a 27-word program does not fit beside
2 used words is the origin-0 case. This load is relocatable: in a
`__FREERTOS` build 29 of 32 words are used and the counter's block keeps its
three free machines; without `__FREERTOS` the block keeps all four.

## Verification

`extras/tests/pico_stepper_count` connects GPIO 16 through 31. The
no-`__FREERTOS` build reports `connected=12 first_failed=28`: all three blocks
serve four steppers. The `rpipico2_freertos` build reports `connected=11
first_failed=27`: two blocks serve four each and the cycle-counter block
serves three. The pins must be in 16..31, the range inside all three PIO
windows.

The driver no longer requires the core's FreeRTOS SMP. With `__FREERTOS` a
FreeRTOS task calls `manageSteppers()`; without it a Pico repeating timer
(alarm pool) calls the same function at the same rate.

The `rpipico2` Saleae catalogue passed every implemented scenario, including
the direction-change tests SR_10–SR_12, and `--mode scale --pin-mode nodir`
passed at 1..8 steppers. All PC tests passed. `pio run -d pio_dirs/saleae -e
rpipico2` builds with `-Werror`.

Two timing analyses were run on the Pico 2 Saleae firmware, env `rpipico2`.

**Loop budget.** Thirty one-step commands of 3200 ticks (200 us each) at
24 MS/s gave 30 rising edges and 29 intervals. Two runs summed to 139196
and 139197 samples against the 139200-sample request; the 32-word program
measured 139197. The 27-word pass is `22 + 3*period` instead of
`26 + 3*period`, so `adjust_80MHz` absorbs the 4-cycle difference and the
command duration is unchanged. Histograms: 6×4799 / 21×4800 / 2×4801 and
7×4799 / 18×4800 / 4×4801 samples. Record:
[pico_pio_loop_timing.md](pico_pio_loop_timing.md).

**Direction timing.** SR_10 measures 790.25 us from the DIR edge to the
first STEP edge (785.75 us on the earlier run), which is the four
`MIN_CMD_TICKS` old-direction pause words (4 × 200 us). The previous
32-word program set DIR and STEP from adjacent PIO instructions (~50 ns).
The delay is the cost of draining the FIFO before the new direction
starts; SR_11 and SR_12 confirm the direction level is stable within each
phase and no DIR edge falls inside a step pulse.

## Problem

Each PIO block has 32 instruction words. The stepper program was 32
words at origin 0 (`stepper_make_program()` in
`src/pd_pico/pico_pio.cpp`). The Arduino core's cycle counter is 2
words and one state machine. Both cannot reside in one block.

`pio_can_add_program` builds its mask with `1u << length`
(`hardware_pio/pio.c`, `find_offset_for_program` and
`add_program_at_offset_check`). A shift of 32 is undefined. For a
length of 1 through 31 the shift is defined and the check works on
both CPUs: on the counter's block a 30-word program fits and a
31-word program does not. For a length of 32 this RISC-V compiler
emits `bset` of bit `length & 31`, the mask is 0, and the overlap
test is `used & 0`. The call returns `0`, which is `PICO_OK` and
also a legal load address. No error code is produced, and
`claim_pio_resources()` treats that as success. The write replaces
the counter.

On the Cortex compiler the same expression rejects a 32-word program
as soon as any word is used. That is why the measured Pico 2 Cortex
image kept 8 steppers. The Pico SDK function
`pio_claim_free_sm_and_add_program_for_gpio_range` does not save
this. Its first pass claims one state machine and adds the program.
On RISC-V that add returns success on the counter's block, so the
pass that claims all four state machines never runs.

## Remedy (not used)

In `claim_pio_resources()`, before any program write, claim all four
state machines of a candidate block. Write the program at origin 0
only when all four claims succeed, and keep those four for the four
steppers on that block. If any claim fails, unclaim every state
machine taken on that block and try the next PIO.

Do not release the spare three after a successful write. The
length-32 check stays broken, so a later caller can still be told
the block is empty and overwrite it.

## Alternative (implemented)

Remove the direction-pin sequence from the PIO program and set the
pin in the FIFO filler instead. The sequence was the five words at
`label_step_loop` in `stepper_make_program()`: `out y, 1`,
`jmp !y`, `set pins, 1`, `jmp`, `set pins, 0`. Dropping them takes
the program from 32 words to 27. The SDK length check then uses a
defined shift on both CPUs.

The filler in `push_command()` already has the direction bit when it
builds the FIFO word. The pin must not change at that push. The FIFO
still holds earlier commands, and those commands expect the old
direction until the PIO consumes them. The filler has to apply the
new level at the command the PIO is about to execute, which is the
same moment the five PIO words did it before. The fixed few cycles
between the direction edge and the step edge move to that filler.
