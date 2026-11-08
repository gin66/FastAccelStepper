# Pico — claim a whole PIO block before writing the program

Priority: **033**. Follows
[032](032_pico_cycle_counter_pio.md). The length-32 occupancy test in
the Pico SDK is wrong on RISC-V, so a stepper program can be written
over the cycle counter.

Status: open. Not implemented.

## Problem

Each PIO block has 32 instruction words. The stepper program is 32
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
image keeps 8 steppers. The Pico SDK function
`pio_claim_free_sm_and_add_program_for_gpio_range` does not save
this. Its first pass claims one state machine and adds the program.
On RISC-V that add returns success on the counter's block, so the
pass that claims all four state machines never runs.

## Remedy

In `claim_pio_resources()`, before any program write, claim all four
state machines of a candidate block. Write the program at origin 0
only when all four claims succeed, and keep those four for the four
steppers on that block. If any claim fails, unclaim every state
machine taken on that block and try the next PIO.

Do not release the spare three after a successful write. The
length-32 check stays broken, so a later caller can still be told
the block is empty and overwrite it.

## Alternative

Remove the direction-pin sequence from the PIO program and set the
pin in the FIFO filler instead. The sequence is the five words at
`label_step_loop` in `stepper_make_program()`: `out y, 1`,
`jmp !y`, `set pins, 1`, `jmp`, `set pins, 0`. Dropping them takes
the program from 32 words to 27. The SDK check then works, and the
counter's block is refused because a 27-word program does not fit
beside 2 used words.

The filler in `push_command()` already has the direction bit when it
builds the FIFO word. The pin must not change at that push. The FIFO
still holds earlier commands, and those commands expect the old
direction until the PIO consumes them. The filler has to apply the
new level at the command the PIO is about to execute, which is the
same moment the five PIO words do it today. The fixed few cycles
between the direction edge and the step edge move to that filler.
