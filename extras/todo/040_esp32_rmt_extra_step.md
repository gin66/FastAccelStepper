# ESP32 RMT: one spurious step pulse at end of a move

Priority: **040** — high: digital anomaly, pulse counter out of sync, test
fails. Above the 050 platform features.

Status: **open, culprit localised** — seen on IDF5, not on IDF4. The
`FAS_RMT_DEBUG_COUNT` run split the fault (3 of 3 runs failed): the
filler/queue encodes exactly the commanded steps (`enc net == position`),
`short == 0` (no early return, no `ovf_buf`), yet the pin is short by 2
(`pcnt == net − 2`). The extra edges are therefore born **after**
`encode_commands()`, in the IDF5/6 ping-pong driver (`rmt_tx.c` /
`rmt_encode_simple`) — H8. See "Counter-split result".

Historical detail: observed on `idf5` (`esp32_idf_V6_9_0`, ESP-IDF 5.3.1),
classic ESP32, driver `RMT`. Three pin captures (`digital_bad.csv`,
`digital.bad2.csv`, `digital.bad3.csv`). IDF4 ran ~40 large pairs clean.
While hunting on IDF5: `seq_15` ×1 pass, `seq_02` ×7 pass, then `seq_02`
failed again (`digital.bad3.csv`, `@0 [-1]`). The hit rate is still low.

## Current reading

The queue for a given move does not vary. A passing run and a failing run
are the same `steps`/`ticks` stream, and deleting the one inserted
rise-to-rise interval from a bad capture reproduces the good capture to
≤ 0.043 µs. The ramp is not choosing a different command. The extra edge
is introduced downstream, on the way from that stream to the pin.

IDF4 writes `rmt_fill_buffer()` straight into `RMTMEM` from the threshold
ISR (`StepperISR_idf4_esp32_rmt.cpp`, `rmt_apply_command` then `tx_start`).
IDF5 feeds the same filler through Espressif's simple encoder and ping-pong
driver (IDF 5.3.1 `rmt_encoder.c` / `rmt_tx.c`, callback
`encode_commands()`). Working assumption: IDF4 does not have this bug, and
the rare path is in that middleware. The shared queue and the per-core
`noInterrupts()` are the same on both.

Each capture is exactly one extra STEP rising edge. Its high time is the
saturated ~2047.9 µs of every other slow step (`0x7fff` in the
`ticks == 0xffff` branch). The low gap after it is short, and the original
schedule continues, shifted by that one interval. PCNT matches the pin
(odd count, net ±1). A single RMT half sums to at most 65535 ticks
(4096 µs); every inserted gap is longer, so the extra edge is followed by
more than one chunk of low.

The "~6th step" is not a pipeline depth. The first two captures land on
the same speed because acceleration 1000 is symmetric (6th gap from the
start and 6th gap from the end). `digital.bad3.csv` is a different speed
and a different offset: `seq_02` pair 27, n = 1733, backward, extra
interval at index 1731, which is the penultimate gap, immediately before
the final 357871-tick step.

Inserted gap, 16 MHz ticks. Saleae sampled at 24 MHz (one sample is
41.7 ns, 0.67 tick), so a 1-tick disagreement is one sample:

| capture | where | period before the extra | inserted | that number |
|---|---|---|---|---|
| `digital_bad.csv` | end of 6621 backward | 146041 | 105788 | `65535 + first pause` (40253) of that period |
| `digital.bad2.csv` | start of 6621 forward | 160120 | 112827 | `65535 + first pause` (47292) of that period |
| `digital.bad3.csv` | end of 1733 backward | 253044 | 126522 | half of 253044. Pauses 2+3 of that step (`65535 + 60987`). `65535 + first pause` would be 131070 and does not match |

When `T > 65535`, `_getNextCommand()` emits one `steps = 1` command with
`ticks = min(T >> 1, 65535)` and then pauses. Above 131070 ticks the step
command sticks at 65535 (this is the `0xffff` branch, high = 0x7fff) and
a pause above 65535 is halved and capped at 65535. Each of those entries
is one RMT half. The first two gaps are the step chunk plus the pause
after it. The third gap is not that pair. "Always S + first pause" does
not survive `digital.bad3.csv`.

What does survive: one extra `0x7fff` edge in the saturated slow tail,
gap equal to two chunks of the surrounding step, rest of the waveform
identical. The IDF5 channel holds two halves. A rare replay of whichever
two chunks are in that buffer has the right shape. Which pair, and how a
fixed command stream only sometimes replays it, is open.
`rmt_tx_do_transaction()` enables the threshold interrupt before the
task-context prefill returns, so two encoder walks can overlap. That
overlap does not exist on IDF4.

## Symptom

`seq_03_02` fails `check_pcnt_sync()`: after the move sequence returns to
position 0, the API reports `@0` while the hardware pulse counter reads
`[-1]` (or `[1]` in another run). The output contains an **odd number of
STEP pulses**.

The engine status line at the end of the failing run::

    >> M1: @0 [-1] acceleration  [steps/s^2]=1000 speed [us/step]=40  RMT

## Evidence

Saleae export, STEP/DIR, `test_seq_02` (`move(+n)` then `move(−n)`,
n = 1, 2, 3, 5, 7, …, 6621). The same firmware was captured twice:
`digital_good.csv` (pass) and `digital_bad.csv` (fail).

| metric | good | bad |
|---|---|---|
| STEP rising edges | 65,988 (even) | **65,989 (odd)** |
| forward pulses (DIR=1) | 32,994 | 32,994 |
| backward pulses (DIR=0) | 32,994 | 32,995 |
| net (fwd − bwd) | 0 | **−1** |
| direction runs | 68 (34 fwd, 34 bwd) | 68 (34 fwd, 34 bwd) |
| mismatched fwd/bwd pairs | none | **pair 33: 6621/6622** |
| STEP high-time | 194 µs … 2047 µs | same, no glitches |

The bad capture is the good capture **plus exactly one inserted step**: the
last backward run has 6621 rise-to-rise intervals in the good run and 6622 in
the bad run. Removing the single interval at index 6615 from the bad run makes
it match the good run to within 0.043 µs over all remaining 6620 intervals.

The capture is `test_seq_02`. All 33 earlier forward/backward pairs are
exactly equal; the **last pair is fwd = 6621, bwd = 6622**.

The extra pulse is inserted in the deceleration tail of the final backward
move, at t ≈ **93.083542 s** (pulse #6616 of 6622). Interval-by-interval
against the identical final forward move:

```
idx    forward(µs)   backward(µs)
6614    9127.58       9127.58
6615   10007.54       6611.71   <-- extra step emitted early (expect 10007.54)
6616   11183.50      10007.54   <-- ramp resumes, shifted by one
6617   12911.42      11183.46
6618   15815.25      12911.42
6619   22366.96      15815.29
6620        -         22366.96
```

Everything up to idx 6614 agrees to < 0.05 µs. Net fwd − bwd = −1 explains
the odd pulse count and `@0 [-1]`.

In the bad run the extra step has the same STEP high-time as every other step
in that region, 2047.8 µs, but its low gap is only 4563.9 µs, so it appears
6611.7 µs after the previous step where the ramp schedule called for
10007.54 µs. The following step is back on schedule (10007.54 µs), i.e. the
whole remaining tail is shifted by exactly one step.

The ramp is symmetric, so array alignment alone cannot tell whether the extra
step sits at the start or at the end of the move. The captured values do:
in `digital_bad.csv` the 6th interval is normal (9127.58 µs) and the
6th-from-last is short, so there the extra step is at the **end** of the
large backward move. In `digital_bad2.csv` it is the other way around (6th
interval short, tail normal), i.e. at the **start** of the large move. The
side therefore varies.

The same off-by-one also occurs with the opposite sign
(`seq_03_02.log`: `>> M1: @0 [1]`), so it is not a fixed missing pulse but
an extra/lost step at the end of a move.

**Rarity.** Ten consecutive `seq_03_02` runs passed on the same firmware. The
fault is rare and not deterministic.

### Reproduction capture (`digital.bad2.csv`)

`test_seq_15`, 141.74 s, `n = 3` prefix, failure on the 13th large pair:

| metric | value |
|---|---|
| STEP rising edges | **172,219 (odd)** |
| forward pulses (DIR=1) | 86,110 |
| backward pulses (DIR=0) | 86,109 |
| net (fwd − bwd) | **+1** |

Here the extra step is in a large **forward** move (direction varies between
captures). It is at the start of the move, the 6th interval:

```
idx    good(µs)     bad(µs)
4      10007.5      10007.5
5       9127.58      7051.71   <-- extra step too early (low gap 5003.83
6       8455.62      9127.54       vs expected 7079.71 µs)
7       7907.63      8455.62
8       7463.67      7907.63
```

Removing interval idx 5 makes the bad run match the preceding good large
forward run to within 0.043 µs. This is the only anomaly in the capture: the
other 12 large pairs are identical to the reference. In these two files the
short interval is the 6th gap from one end. That is not the rule:
`digital.bad3.csv` (below) is the penultimate gap of a 1733-step move.
Engine line at failure:

    M1: @0 [1] => 0 QueueEnd=0 v=0us/0ticks MANU
    iter 13 n=3 pcnt=1 pos=0 -> extra/missing step

### Third capture (`digital.bad3.csv`)

`seq_02` again, same 24 MHz Saleae export, compared with `digital_good.csv`.
Engine line: `>> M1: @0 [-1]`, acceleration 1000, 40 µs/step, RMT.

| metric | value |
|---|---|
| STEP rising edges | **65,989 (odd)** |
| forward (DIR=1) | 32,994 |
| backward (DIR=0) | 32,995 |
| net | **−1** |
| mismatched run | pair 27, n = **1733**, backward: 1734 pulses vs 1733 |

The other 33 forward/backward pairs match the good capture, including the
final 6621-step pair. The anomaly is not confined to the longest move.

Removing interval index 1731 (7907.625 µs, **126522 ticks**) makes this run
match the good 1733-step backward run to 0.043 µs. One intact interval
follows it, the final step:

```
idx     good (ticks)   bad3 (ticks)
1729    206583         206583
1730    253044         253044
1731    357871         126522   <-- inserted
1732        -          357871
```

126522 is exactly half of the preceding 253044-tick period. That period
splits as `65535 + 65535 + 60987 + 60987` (step, then three pauses; the
first pause is the 65535 cap, not an uncapped half). `65535 + 60987` is
pauses 2+3 and equals 126522. `65535 + first pause` is 131070 and is not
the measurement. The extra pulse's high is again ~2047.9 µs.

So the offset is not "the 6th step", and the gap is not always
`65535 + first pause`. See "Current reading".

## Notes

The simple encoder runs ahead of the wire. The extra edge is in the slow
tail, where a step is a 65535-tick command plus pauses, but it is not
always the last chunk of the move: in `digital.bad3.csv` a full 357871-tick
step still follows it. Start-of-move and end-of-move both occur. Read the
gap in ticks and compare it to the chunk split of the surrounding step
rather than to a fixed index.

## Reproduction

`test_seq_15` (`examples/StepperDemo/test_seq_15.cpp`, ESP32 pulse counter
only) runs a cycle 100 times and checks the counter after each one:

1. a short prefix pair: `n` steps forward, then `n` back (n = 1,2,3,4,5,
   repeating),
2. a forward/backward move of `steps`, starting at `SEQ15_BASE_STEPS` and
   growing by `SEQ15_STEP_INC` each cycle,
3. a 10 ms pause.

After the backward move has stopped, `readPulseCounter()` and
`getCurrentPosition()` must both be zero; a non-zero value prints
`iter <i> n=<n> steps=<s> pcnt=<x> pos=<y> -> extra/missing step` and fails
the sequence. `n` cycles so a direction change shortly before the move is
covered. Run it from test mode with `t 15 R`.

The move length was originally 6621 (the largest `test_seq_02` pair), which
made one run ~17 minutes. Both observed anomalies are within ~6 steps of a
move boundary, and a short move has both edges at the same slow speeds (first
step ~22.4 ms, last step decelerating to ~22.4 ms), so the length now starts
at 50 and grows by 7 per cycle (50, 57, 64, … 743 over 100 cycles). This
sweeps the ramp-length range in ~30 s instead of ~17 min per fixed length.

Results so far with the growing sweep:

| target | runs | result |
|---|---|---|
| IDF4 | 1× ~400 s (~40×6621 pairs) | pass |
| IDF5 | 3 runs (50…743 sweep) | pass |
| IDF5, while hunting | `seq_15` ×1 pass, `seq_02` ×7 pass, `seq_02` ×1 fail (`digital.bad3.csv`, n=1733 backward, `@0 [-1]`) | |

## Root cause hypotheses (brainstorm)

### Constraints the culprit must satisfy

- The anomaly is a real extra STEP high **on the pin** (all three captures
  are the pin, not the PCNT). `check_pcnt_sync` only reports it.
- The commanded stream is the same on a pass and a fail. A pure arithmetic
  bug in the ramp or in `rmt_fill_buffer()` would fire on every move of
  that length. It does not.
- Sign varies (`+1`/`−1`), direction varies, move length varies (1733 and
  6621), and the offset from the end of the move varies (penultimate gap
  in `digital.bad3.csv`, five gaps from the end in `digital_bad.csv`,
  near the start in `digital.bad2.csv`).
- It is in the slow tail, where the step high has saturated at ~2047.9 µs
  (`ticks == 0xffff`). The extra edge has that same high. The low gap is
  the short part, and it spans more than one RMT half.
- Rare. That points at interrupt timing inside the IDF5 driver, not at a
  different queue.

### The 6th-step offset does not hold

`digital_bad.csv` and `digital.bad2.csv` both insert at interval index 5
of one end. That is one ramp speed seen from either end at acceleration
1000, not a 6-deep buffer. `digital.bad3.csv` inserts at index 1731 of a
1733-step move: one gap before the last step, period 253044 ticks before
it and 126522 ticks of extra. A fixed "6th chunk" or "first `read_idx`
advance" does not predict that. Keep both the long 6621 run and the
shorter `seq_02` moves; the failure is not specific to 6621.

### H8 — IDF5 ping-pong repeats two chunks (CONFIRMED as locus)

Classic ESP32, IDF5: `mem_block_symbols = 64`, `PART_SIZE = 32`, so the
channel is two halves and each `encode_commands()` call fills exactly one
half with exactly one queue entry. The hardware buffer therefore always
holds two commands. A second transmit of those two halves is one extra
rising edge when one of them is a `ticks == 0xffff` step (`0x4000FFFF`),
and the rise-to-rise gap equals the sum of the two halves. That matches
the first two captures (step chunk + following pause) and is the shape of
the third (two chunks of the preceding step; the measured sum is pauses
2+3, not the step chunk + first pause).

IDF4 never enters this driver. It fills both halves itself and sets
`tx_start`.

Ways a fixed queue still double-sends, all inside IDF 5.3.1:

- Task-context prefill and the threshold ISR both call the encoder.
  `rmt_tx_do_transaction()` enables `TX_THRES` before
  `rmt_encode_check_result()` returns. A second walk that still has the
  old `read_idx` fills the same two entries into the other half.
- `mem_off` / `mem_end` lose phase, so a half that still holds the step
  symbol is sent again. Phase of those two fields is timing, not data.
- The simple encoder copies through `ovf_buf` only when
  `symbols_free < min_chunk_size`. On the exact-half schedule
  `symbols_free` is 64 then 32 and the overflow path does not run. A
  misaligned `mem_off` is what would take it. Logging `symbols_free` on
  the failing move separates this from a straight double fill.
- `rmt_tx_mark_eof` writes a zero-duration word when the callback reports
  done and the encoder did not also set `RMT_ENCODING_MEM_FULL`. In this
  IDF those two states are exclusive. That truncates more naturally than
  it inserts a 2048 µs high, so it is the weaker branch. It stays listed
  because the stop uses the same ping-pong.

A wrap that simply repeated the current half *during* the step would
split that step's period. The traces keep the preceding period intact and
insert the extra gap after it. The duplicate pair has to reach the wire
after the original chunks have already played. Two walks, the second
stale, can do that; a single-half late refill cannot. The per-callback
log below is what distinguishes them.

### H9 — short free space returns 0 without stopping, and one symbol is repeated (ruled out)

`encode_commands()` bails out before it looks at the queue:

```c
if (symbols_free < PART_SIZE) {
  return 0;            // *done stays false, _rmtStopped stays false
}
if (rp == next_write_idx) {
  _rmtStopped = true;  // only reached when a full half is free
  ENTER_PAUSE(MIN_CMD_TICKS);
  return PART_SIZE;
}
```

A call with `PART_SIZE - 1` (or fewer) free symbols neither stops nor
writes. IDF 5.3.1 does not leave it there: `rmt_encode_simple` treats a
0 return as "encode into `ovf_buf`" and calls back immediately with
`min_chunk_size` (= `PART_SIZE`). That second call can see an empty
queue and emit the pause. The hole matters if that retry does not run,
or if the transmitter wraps onto the old half before the retry's symbols
land. Wrap is always on (`rmt_ll_tx_enable_wrap`).

What a half actually contains around the anomaly. Classic ESP32
`PART_SIZE = 32`. The `PART_SIZE/2`, sometimes `PART_SIZE/2 - 1`,
packing is the other branch: `steps` between `PART_SIZE` and
`2 * PART_SIZE` sets `steps_to_do = PART_SIZE >> 1`, and the stretch
loop then consumes one step to fill the partition, leaving
`PART_SIZE/2 - 1` unstretched steps. Those commands have
`ticks != 0xffff` and a high time below 2048 µs. The three captures
are past that. There `ticks == 65535` and `steps == 1`, which is the
`steps < PART_SIZE/2` arm. One queue entry fills the whole half, and
the only rising edge is the first symbol:

| word | value | levels | ticks |
|---|---|---|---|
| 0 | `0x4000FFFF` | high 32767, low 16384 | 49151 |
| 1 | `0x20001C40` | low 7232, low 8192 | 15424 |
| 2…31 | `0x00100010` | low 16, low 16 | 960 |
| sum | | one rising edge | 65535 |

A pause entry (`steps == 0`) is 31 words of `0x00040004` (low 4 + low 4)
and one final low/low word. For any legal pause the last halves stay
under 32768, so the level bit stays clear. No rising edge in a pause
half. Repeating a pause symbol cannot be this pulse. Repeating word 0
can: its high is exactly the measured ~2047.9 µs.

Likelihood.

- The extra edge is that one symbol. High. Nothing else in these halves
  rises, and the high time matches `0x7fff`.
- The inserted gap is that one symbol's duration. Low. Word 0 is 49151
  ticks (3072 µs). The two-word step pair is 65535. The measured gaps
  are 105788, 112827 and 126522 ticks. A local double-send of word 0
  would show up as a ~3072 µs interval, and none of the captures has
  one.
- "Queue ran out, we returned 0, hardware repeated a symbol." The
  control-flow hole is real: the stop check is unreachable when fewer
  than `PART_SIZE` symbols are free. On this IDF the same-call overflow
  retry usually closes it, and in the steady ping-pong `symbols_free`
  is 32 or 64, so the early return is not on the normal path. It runs
  when `mem_off` is short of `mem_end` by 1…31, which is the "half
  minus one" case. Even then a repeated word 0 only matches the
  captures if the repeat is delayed by the following low symbols and
  the next chunk, i.e. the two-chunk replay of H8, with the edge coming
  from word 0 of the step half. A bare symbol repeat does not produce
  the gap. In the slow tail a half lasts 2–4 ms, so wrap-before-refill
  is unlikely unless the still-live remainder is already a few short
  symbols.

Net: keep it as the description of the edge (one symbol, word 0) and as
a real hole in the early return. It is a weak account of the gap, and
weaker than H8 as the reason a fixed command stream gains a pulse.

### H1 — cross-core ISR race on the queue (ruled out for this symptom)

`fasDisableInterrupts()` on ESP32 is `noInterrupts()` →
`portDISABLE_INTERRUPTS()`, which masks interrupts on the **current core
only** (`src/fas_arch/arduino_esp32.h`, `espidf_esp32.h`). `engine.init()`
defaults to `cpu_core = 255`, so `StepperTask` is created unpinned
(`xTaskCreate`), while the RMT interrupt that runs `encode_commands()` is
bound to whichever core created the channel. Task-side `addQueueEntry()` and
ISR-side `encode_commands()`/`rmt_fill_buffer()` can therefore run on
different cores with no mutual exclusion, even inside the
`fasDisableInterrupts()` critical sections.

Mechanism: `rmt_fill_buffer()` reads `read_idx`/`next_write_idx` and rewrites
`entry[rp].steps`; the task concurrently commits `next_write_idx` and fills
the next entry. One instruction of skew at the exact drain/add boundary can
make the encoder process a one-step command twice, or advance `read_idx`
across a partially committed entry.

A torn `read_idx` only matters if the IDF5 callback is the reader that
races. The bytes committed into the queue are the same on pass and fail.

Run the core-pin and the spinlock only if the counter split below says
the callback consumed an extra step. IDF4 shares this queue layer and is
the control that does not go through the simple encoder. Xtensa store
order also works against the ISR observing `next_write_idx` before the
entry fields: the entry is stored first, and the write buffer is FIFO.

### H2 — premature EOF vs restart (encoder runs ahead)

The encoder fills up to a chunk ahead of the wire. At a move boundary the
queue can look empty to the encoder while `StepperTask` (4 ms tick) is about
to append the next command. The committed path then does: queue empty →
`_rmtStopped = true` → emit one pause partition → `*done = true` → TX-done →
`_isRunning = false` → ramp restarts via `addQueueEntry(NULL, true)` →
`startQueue_rmt()` → `_tx_encoder->reset()` + `rmt_transmit()`.

If the restart lands while the previous transmission's tail is still being
encoded, the encoder reset can cause the last one-step command to be encoded
again. That could fit an extra step at a move boundary. It fits
`digital.bad3.csv` poorly: the extra gap is the penultimate interval, and
a full 357871-tick step still plays after it, so the transmission had not
stopped.

Test: trace `_rmtStopped`, `_isRunning`, `read_idx`, `next_write_idx`, and
reset/transmit counts around a failure (RAM log read out after, or GPIO
probe).

### H3 — stale visibility of `_rmtStopped` / `_isRunning`

`_rmtStopped` is a plain `bool`, not `volatile` (`src/pd_esp32/esp32_queue.h`),
yet it is written in the encoder ISR and read by `isReadyForCommands_rmt()`
in the task. A cached read makes the readiness check briefly wrong at a
boundary, so a command is appended after the encoder committed `done`, or
refused right after a restart.

Test: make it `volatile` (plus a barrier) and A/B the build.

### H4 — one-step command executed twice in the stretch branch (ruled out)

In the `ticks == 0xffff` branch a one-step command is emitted, `steps` is
driven to 0 and `read_idx` advances. If the function is entered twice for the
same `rp` before `read_idx` commits (H1/H2), the step is emitted twice. The
extra step then has the standard 0x7fff high, matching the captures.

Test: log every one-step processing together with `rp` and assert no duplicate
`rp` for the same command.

### H5 — ramp-generator batching nondeterminism (rejected)

A late `StepperTask` tick does not change the commands for these moves.
Pass and fail are the same `steps`/`ticks` stream, and the good and bad
captures agree to < 0.05 µs everywhere except the one inserted gap.
Scheduler load can still move the IDF5 ISR relative to the prefill. That
is H8, not a different queue.

### H6 — PCNT-only artifacts (excluded here)

A PCNT direction/edge glitch could report ±1, but both captures show the edge
on the pin, so PCNT-only explanations are ruled out for these runs. Still
relevant for log-only failures with no capture.

### H7 — channel enable/disable across restart

`startQueue_rmt()` enables the channel only if `!_channel_enabled`;
`forceStop_rmt()` disables it. A restart racing a disable/enable can leave one
symbol latched. Test: count enable/disable/transmit against steps.

### Discriminating experiments

The split that matters is where the extra edge is born. The queue bytes
are the same on a pass and a fail.

1. Three counters, latched when PCNT diverges and printed with the
   existing failure line:
   - steps the ramp committed,
   - steps `encode_commands()` consumed (count the `0x4000FFFF` stores,
     or the `read_idx` advances over step entries),
   - PCNT, which is the pin.
   Consumed == committed and PCNT == committed + 1: the callback ran the
   right number of times and the duplicate is after it (ping-pong replay,
   overflow copy, EOF word). Consumed == PCNT == commanded + 1: the
   callback walked an entry twice.
2. A RAM ring on each callback: `symbols_free`, `read_idx`,
   `next_write_idx`, `steps`, `ticks`, `_rmtStopped`, `done`. Freeze it
   on the mismatch. `symbols_free` other than 32 or 64 means `ovf_buf`
   participated. Every `read_idx` seen once, and `symbols_free` always
   32 or 64, means the duplicate is below the callback.
3. One GPIO toggled at `encode_commands` entry, captured with STEP. The
   extra edge either has a matching toggle or it does not. Same split as
   the counters, next to the gap on the analyzer.
4. Core pin and the queue spinlock only if (1) says the callback walked
   an entry twice.
5. IDF4 stays the control: same queue, direct `RMTMEM` writes.

### IDF4 cross-check — clean so far

`test_seq_15` on IDF4 ran for ~400 s without a single failure, i.e. roughly
40 large 6621-step pairs (plus the small prefix pairs) and no pulse-counter
error.

IDF4 does not use the simple encoder. `startQueue_rmt()` writes both
halves through `rmt_apply_command()` into `RMTMEM` and starts the
transmitter itself. `esp32_before_pause_count()` is 1 on IDF4 and 2 on
IDF5/6, because the IDF5 encoder is invoked two `PART_SIZE` chunks ahead.
A clean IDF4 points at that driver (`StepperISR_idf5_esp32_rmt.cpp` plus
IDF `rmt_encoder.c` / `rmt_tx.c`), not at the shared queue. The assumption
is that longer IDF4 time stays clean. IDF5 with M1=RMT is the config that
fails.

## Debug build (implemented)

Enable with `#define FAS_RMT_DEBUG_COUNT` in `src/pd_esp32/pd_config.h`
(takes effect only with `SUPPORT_ESP32_RMT_V2`, i.e. IDF5/6 RMT). The
counters live in `src/pd_esp32/rmt_debug.h`, are filled from
`rmt_fill_buffer()` / `encode_commands()`, and are reset in the demo when
the pulse counter is attached. One IDF5 RMT build of StepperDemo, then full
`seq_02` until the next `@0 [±1]`. `seq_15` is out of this hunt. No GPIO, no
printf in the callback: both would move the timing of the bug we are trying to
catch. The ramp and the queue bytes stay as they are.

### What is counted

Guard with one macro, for example `FAS_RMT_DEBUG_COUNT`, compiled only
on the IDF5 RMT path (`SUPPORT_ESP32_RMT_V2`). IDF4 does not get the
counters.

In `rmt_fill_buffer()` (`StepperISR_esp32xx_rmt.cpp`), count step
symbols actually stored, not queue entries. A pause entry stores no
step symbol and must not count. Two totals, chosen from the entry's
`count_up` at the store:

- the `ticks == 0xffff` arm, once per `0x40007fff | 0x8000` (word 0 of
  a saturated step, and each further step in that arm),
- the faster arm, once per symbol that carries the step bit.

A double walk of one entry increments the total twice. A hardware
replay of a half that the filler already wrote does not increment it.

In `encode_commands()` (`StepperISR_idf5_esp32_rmt.cpp`), before the
`symbols_free < PART_SIZE` return:

- if `symbols_free` is neither 32 nor 64, increment a short-free
  counter and keep the last value,
- count how often that early return is taken.

Storage is file-scope `volatile uint32_t`. A read function returns
up, down, short-free count, and last short `symbols_free`. The
callback only increments.

### Where it is zeroed and printed

Zero the counters in the same place the demo attaches the pulse
counter (`p` in `StepperDemo.ino`), so position, pulse counter, and
encoded net share one zero for that `seq_02` run. One test per boot
is enough; a second run without re-attach would mix the totals.

`info()` already prints `@pos [pcnt]` every 100 ms while the test
runs, and `stepper_info()` prints it again when the test finishes.
Append the debug fields after `[pcnt]`:

```
enc +<up> -<down> net <up-down> short <n>@<last>
```

`check_pcnt_sync()` matches `^>> M1: @(-?\d+) \[(-?\d+)\]` and does
not anchor at the end of the line, so the suffix does not break
`seq_02.py`. No change to the Python session.

### How to read the failure line

At `@0 [-1]` the ramp's position is 0. Compare that with the encoded
net and with the pulse counter:

| position | encoded net | pulse counter | meaning |
|---|---|---|---|
| 0 | 0 | −1 | the callback consumed the right steps; the extra edge was sent after it (H8, word 0 of a step half replayed) |
| 0 | −1 | −1 | the callback stored one extra backward step symbol (double walk, H1/H4) |
| 0 | 0 | −1, and `short` > 0 | the early return ran on this test; H9 participated, still look at the net |

`[+1]` is the same table with the signs flipped. `short == 0` means
every callback saw a full half (32) or the initial double fill (64),
so the overflow-buffer path was not taken.

After that one failure the next edit follows the row. Encoded net
equal to position goes into the IDF5 ping-pong (`rmt_tx.c` /
`rmt_encode_simple`), not into a queue lock. Encoded net equal to the
pulse counter is the point to pin `StepperTask` and the RMT interrupt
to one core and to replace the ESP32 `fasDisableInterrupts()` around
the queue with a spinlock.

### Counter-split result

Two failing `seq_02` runs on IDF5.3.1, classic ESP32, RMT, M1, with
`FAS_RMT_DEBUG_COUNT`:

```
M1: @40 [39] enc +32994 -32954 net 40 short 0@-1 => 0 QueueEnd=33 ...
M1: @17 [16] enc +32994 -32977 net 17 short 0@-1 => 0 QueueEnd=13 ...
M1: @3 [2]   enc +32994 -32991 net 3  short 0@-1 => 0 QueueEnd=2 ...
>> M1: @0 [-2] enc +32994 -32994 net 0 short 0@-1 acceleration ... RMT
```

```
M1: @1 [-1] enc +65988 -65987 net 1 short 0@-1 => 0 QueueEnd=0 ...
>> M1: @0 [-2] enc +65988 -65988 net 0 short 0@-1 acceleration ... RMT
```

```
M1: @22 [21] enc +98982 -98960 net 22 short 0@-1 => 0 QueueEnd=18 ...
M1: @6 [5]   enc +98982 -98976 net 6  short 0@-1 => 0 QueueEnd=4 ...
M1: @0 [-2]  enc +98982 -98982 net 0  short 0@-1 => 0 QueueEnd=0 ...
>> M1: @0 [-2] enc +98982 -98982 net 0 short 0@-1 acceleration ... RMT
```

All three end `@0` with `enc net == 0` and `short == 0`, but `pcnt == -2`:
the pin has **two** extra backward edges that the encoder never
produced. `short == 0` means every callback saw a full half (32) or
the initial double fill (64); the `symbols_free < PART_SIZE` early
return never ran and the overflow buffer was never used. So on these
runs:

- H1 / H4 (queue/callback double walk): **ruled out**. Encoded net
  tracks the ramp (`@40` net 40, `@17` net 17, `@3` net 3, `@0` net 0).
- H9 (short free space / unstopped early return): **ruled out**.
  `short` stayed 0 for the whole run.
- H8 (IDF5 ping-pong / simple encoder repeats a half): **confirmed as
  the locus**. Net equal to position with the pin ahead by 2 puts the
  extra edges after our filler returns, inside IDF 5.3.1 `rmt_tx.c` /
  `rmt_encode_simple`.

The magnitude is 2 here, not the ±1 of the earlier captures, and the
first run already shows the divergence mid-move (`@40` pos 40, pcnt 39),
so the duplicate is emitted during the move, not only at a move
boundary.

Consequence: the core-pin/spinlock/`volatile` work (H1/H3) does not
address this failure, and the IDF4 control stays clean because it never
enters the simple encoder. The fix is the translator below.

**Hit rate with the debug build.** All **three of three** `seq_02` runs
with `FAS_RMT_DEBUG_COUNT` failed (pcnt −2), where the undecorated
firmware passed seven runs before the next failure. The debug build adds work inside
the encoder callback (a call plus a volatile read-modify-write in
`rmt_fill_buffer()` / `encode_commands()`), which shifts the callback
timing relative to the driver's threshold ISR. That it turns a rare race
into a near-certain one is itself evidence that the fault is a timing
race inside the IDF5 driver. The failure *mode* is unchanged
(`enc net == position`, `short == 0`, pin behind by 2), so the split is
representative; the injected timing only makes it easy to catch. A fix
should be validated with the macro **off**, or the added latency will
mask/alter the race.

## Possible remedy — IDF5/6 translator, not the IDF4 half filler

The IDF5/6 simple encoder is a translator. Each call of
`encode_commands()` is given `symbols_free` and is supposed to turn
queue commands into as many symbols as that space holds, then return
the count. `done` is set only when the queue is finished. IDF 5.3.1
uses the return value 0 for one thing: "this call cannot make
progress; try again with `min_chunk_size`."

`rmt_fill_buffer()` in `StepperISR_esp32xx_rmt.cpp` is the IDF4
contract. A threshold interrupt frees one fixed half, and the filler
always writes exactly `PART_SIZE` symbols for one queue entry (or for
`PART_SIZE/2` steps of a fast entry). `encode_commands()` keeps that
contract on IDF5: it returns 0 unless `symbols_free >= PART_SIZE`,
then asks the shared filler for another full half. The simple
encoder's ping-pong is a different machine. It already tracks
`mem_off` / `mem_end`, the overflow buffer, and the EOF word. Feeding
it IDF4 halves means a queue entry, a half, and a step pulse are the
same object, which is what a repeated half turns into a repeated
pulse.

Remedy idea, IDF5/6 only. Leave `StepperISR_esp32xx_rmt.cpp` to IDF4.
A new filler, called only from `encode_commands()`, translates the
queue into the buffer it was given. Two entry shapes:

- A pause (`steps == 0`) is always exactly `PART_SIZE` symbols, which
  is half the RMT block (`mem_block_symbols / 2`). Same footprint the
  IDF4 half filler uses, so two injected drain pauses still fill the
  whole in-flight memory and the direction toggle on the next entry
  stays safe without reading the RMT. Start a pause only when
  `symbols_free >= PART_SIZE`. If this call would also reach a
  `toggle_dir` entry, return after the pause and toggle on the next
  call.
- A step entry is encoded as short as possible. One step needs two
  free symbols (high and low; a 65535-tick step does not fit in one).
  If `symbols_free < 2`, return 0 and do not touch the entry. Emit as
  many whole steps as fit, then write the remaining `steps` back on
  that queue entry. Advance `read_idx` only when `steps` reaches 0.
  Direction toggle happens when the entry is started, not when a
  later call continues it.

`min_chunk_size` stays `PART_SIZE`. The driver may then call with
fewer free symbols, and returning 0 is allowed; the simple encoder
retries with a full half. A promise of `PART_SIZE` free symbols is
also the smallest promise on which a pause can always start, so the
callback never has to return 0 when the driver forbids it. Inside a
fill that is large enough, steps are then taken two symbols at a time.
The empty-queue stop is checked before a short-buffer return, so the
channel is not left running.

An empty queue still ends the transaction (`_rmtStopped`, then
`done` once the pause or the EOF has been handed over), instead of
returning 0 and leaving wrap to replay the last half.

This does not depend on the debug counters to be worth writing down.
Those counters still say whether the extra edge is born inside the
filler or after the symbols have been handed to the driver. A
translator removes the fixed half that H8 replays and the early
return that H9 leaves unstopped.

## Work items

- ~~Implement the debug build above, then run full `seq_02` on IDF5 RMT
  until the next `@0 [±1]`.~~ Done: two failures captured. Both show
  `enc net == position` and `short == 0` with `pcnt == net − 2` → H8,
  after the encoder. See "Counter-split result".
- The 50…743 sweep has not hit. The moves that have failed are the
  1733-step and 6621-step moves inside `seq_02`.
- Do not bisect the ramp. The command stream matches on pass and fail.
- Core pin, spinlock, and `volatile` on `_rmtStopped`: **dropped**. The
  counter split ruled out H1/H3 for this symptom.
- IDF4 remains the control (direct register writes, same filler).
- Translator is in `StepperISR_idf5_esp32_rmt_encode.cpp` and
  `encode_commands()` calls it. PC coverage is `test_30` (PART_SIZE 24
  and 32). The half filler in `StepperISR_esp32xx_rmt.cpp` stays the
  IDF4 path. A fix still has to be checked on hardware with
  `FAS_RMT_DEBUG_COUNT` off.
- Optional confirmation inside IDF 5.3.1: the per-callback RAM ring
  (experiment 2) around `rmt_tx_do_transaction()` / `rmt_encode_simple`
  to pin the exact replay (`mem_off`/`mem_end` phase vs. the overflow
  copy). Not required to start the translator.

## References

- `extras/tests/esp32_hw_based/digital_bad.csv` — failing capture (seq_02).
- `extras/tests/esp32_hw_based/digital_good.csv` — passing capture (seq_02).
- `extras/tests/esp32_hw_based/digital.bad2.csv` — failing capture (seq_15).
- `extras/tests/esp32_hw_based/digital.bad3.csv` — failing capture (seq_02, n=1733 backward, `@0 [-1]`).
- `extras/tests/esp32_hw_based/seq_03_02.log` — `@0 [1]` variant.
- IDF 5.3.1 `components/esp_driver_rmt/src/rmt_encoder.c` — `rmt_encode_simple`, overflow buffer.
- IDF 5.3.1 `components/esp_driver_rmt/src/rmt_tx.c` — ping-pong prefill, threshold ISR, `rmt_tx_mark_eof`.
- `examples/StepperDemo/test_seq_15.cpp` — reproduction sequence.
- `src/pd_esp32/StepperISR_idf5_esp32_rmt.cpp` — `encode_commands()`.
- `src/pd_esp32/StepperISR_esp32xx_rmt.cpp` — `rmt_fill_buffer()`.
- `src/pd_esp32/rmt_debug.h` — `FAS_RMT_DEBUG_COUNT` counters.
- `extras/tests/esp32_hw_based/serial_session.py` — `check_pcnt_sync()`.
