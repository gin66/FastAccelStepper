# Pico PIO loop timing — `0f53d4a2` measured against tag 2.0.1

Closes todo **030**. The user-facing account is
[pico.md](../platforms/pico.md), section "Loop budget, measured".

## Result

Measured 2026-11-08 on an RP2350, env `rpipico2`,
`-DF_CPU=150000000L`. Native USB. Analyzer `fx2lafw:conn=8.51`, D0 =
GPIO2. 48 MS/s returned 9216 samples and then timed out. 24 MS/s held
a 200 ms window.

`CONFIG 1 pio nodir`, thirty-two times `QSEG 1 3200 1`, `QFILL`
(kept 30), then `QRUN` about 50 ms after `sigrok-cli` started. Thirty
rising edges, 29 intervals.

| Firmware | Interval sum | Histogram |
|---|---|---|
| HEAD | 139197 | 4×4799, 24×4800, 1×4801 |
| tag 2.0.1, only `pico_pio.cpp` and `pico_pio.h` | 139210 | 19×4800, 10×4801 |

The gap is 13 samples, 542 ns. The 150 MHz budget, with `adjust`
carried, is 84 PIO cycles on those 29 lengths, 560 ns, 13.4 samples.
Both files were restored to HEAD and that firmware was flashed back.

The loaded program is the same 32 words on both sides. `add_step()`
stores nothing past word 32, and the instruction 2.0.1 still listed
after that word was already dropped. What changed is
`pio_calc_loops()`: HEAD counts the 3-cycle preamble in
`actual_cycles`. Tag 2.0.1 leaves it out of the value stored in
`adjust_80MHz`, while the state machine still executes it.

## 27-word program (033)

Re-measured 2026-11-08 on the same RP2350 and rig after the five
direction words left the program (`1795e306`). Same procedure:
`CONFIG 1 pio nodir`, thirty-two `QSEG 1 3200 1`, `QFILL` kept 30,
`QRUN`; the capture is armed on `D0=r` at 24 MS/s. Two runs:

| Run | Interval sum | Histogram |
|---|---|---|
| 1 | 139196 | 6×4799, 21×4800, 2×4801 |
| 2 | 139197 | 7×4799, 18×4800, 4×4801 |

29 × 200 us is 139200 samples at 24 MS/s. The 27-word pass is
`22 + 3*period` instead of `26 + 3*period`, and `pio_calc_loops()`
still hits the same request: both sums are within one sample of the
32-word capture above. The remainder is carried in `adjust_80MHz`;
the 6-cycle grid is unchanged.

Direction changes are outside this budget. Moving DIR to the FIFO
filler and draining four `MIN_CMD_TICKS` words makes the Saleae SR_10
dir→first-step measurement 785.75 us (790.25 us on the re-run), where
the 32-word program changed DIR and STEP from adjacent instructions.
