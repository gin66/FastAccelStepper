# IDF 6 RMT runs sequence 02 about 30 s slow

Priority: **040** — high. The pin trace is longer, not a harness
artifact. Ahead of the 050 platform work.

Status: open, lead identified. Encoded time is proven exact (91.46 s);
the pin trace shows the ~29 s is ~3250 stretched **low** phases of ~9 ms,
one per ~21 ms. The lead is the eager `_rmtStopped` stop in
`encode_commands()` on a *transiently* empty queue (H1): the whole queue
fits in the RMT buffer, the encoder drains it, stops, and restart costs a
task period plus async latency. The exact per-stop cost (~9 ms vs the
model's ~0.7 ms) is still unexplained (H3/H4). A new `test_30` condition,
"every queue command contributes >= PART_SIZE/2 RMT symbols", fails and
captures the read-ahead requirement. The chosen fix is F2: cap the RMT
symbol duration (`ceil(65535/PART_SIZE)` ticks) so the RMT buffer spans
less time than the lookahead and the encoder cannot drain the queue while
running; F1 is rejected, F5 unavailable under IDF 5/6. F2 rewrites the
symbol layout, so it must not regress the confirmed extra-step fix
(`extras/doc/implemented/esp32_rmt_extra_step.md`, H8 ping-pong replay)
and needs re-validation on hardware with `FAS_RMT_DEBUG_COUNT` off. The
min-symbol-period gap is latent, not the 29 s (seq_02 min half = 4). See
"F2 design".

## Measurement

StepperDemo sequence 02, acceleration 1000, 40 µs/step, Saleae on
STEP/DIR. Both captures have the same 65988 rising edges and the same
68 direction runs (lengths 1, 1, 2, 2, …, 6621, 6621).

| capture | image the user flashed | first-to-last edge |
|---|---|---|
| `extras/tests/esp32_hw_based/digital_rmt_idf6.csv` | IDF 6 | 120.8 s |
| `extras/tests/esp32_hw_based/digital_rmt_idf4.csv` | `pio run -e esp32_idf_V6_9_0` | 91.6 s |

The gaps between runs add 0.2 s. The other 28.9 s sits inside the
moves. The 6621-step run is 5.085 s versus 7.110 s. The slow ends of
that run match to 0.04 µs (22367 µs, 15815 µs, …). The extra time is
a hole inserted into the faster part of the ramp.

On the long run there are 223 such holes. They land about every 20 ms
of motion (median 20.9 ms). Each hole is roughly 6–10 ms (extra ticks
from 103411 to 160195, median 146087, about 9.1 ms). 199 of the 223
holes are on steps whose own period is at most 2 ms. Summed, they are
the 2.02 s that run is slow. Shorter runs show the same pattern, and
the step spacing of the holes grows as the ramp gets faster, which is
what a fixed time cadence looks like.

20 ms is the forward-planning window (`_forward_planning_in_ticks =
TICKS_PER_S / 50` in `FastAccelStepper.cpp`). The plausible mechanism:
the IDF 5/6 encoder runs ahead of the pin, sees an empty queue, and
`encode_commands()` sets `_rmtStopped`. `isReadyForCommands()` then
refuses new commands until the transaction has fully stopped, so the
task cannot top the buffer up. The channel restarts one planning
window later and the pin sits idle for about 10 ms. The IDF 4 half
filler does not take that stop path, and its sequence 02 stays near
94 s in `test_all.log`.

`test_all.log` (2026-09-28 14:43 run) agrees that native IDF RMT is
the slow motor, and that I2S and MCPWM are not: `seq_03_02` is 123 s
on idf5 M1 and 122 s on idf6 M1 (both RMT), and 92–94 s on idf5 M9,
idf6 M2, and idf6 M3. The evening Saleae of env `esp32_idf_V6_9_0`
is 92 s, while that same env's afternoon `seq_03_02` was 123 s. Those
two are not the same observation; the pin trace is the one to trust
for the image that was flashed at 17:00.

## `compile_idf4`

`make compile_idf4` is not IDF 5. It builds env `esp32_idf_V5_3_0`,
`platform = espressif32 @ 5.3.0`, and that platform pins
`framework-espidf ~3.40403.0`, which is ESP-IDF 4.4.3. The `5.3.0` in
the name is the PlatformIO platform version. The cmake cache under
`pio_espidf/StepperDemo/.pio/build/esp32_idf_V5_3_0` points at that
4.4.3 package.

The 92 s capture was not produced by `compile_idf4`. It was
`pio run -e esp32_idf_V6_9_0`, which is `compile_idf5`: platform
6.9.0, ESP-IDF 5.3.1. The filename `digital_rmt_idf4.csv` is the
wrong generation.

## PC tests (`extras/tests/pc_based/test_30.cpp`)

The additions to `test_30` narrow the problem (the refill-invariant check
is described in its own section further down):

### The translator is tick-exact

`test_seq_02_total_ticks` drives the real ramp generator through the
whole sequence and drains every command with `rmt_encode_queue()`,
comparing the commanded ticks with the ticks the symbols encode.

```
PART_SIZE 32 and 24:
  pairs=34 commanded=1463342840 symbols=1463342840 edges=65988
```

1463342840 ticks is **91.46 s**, i.e. the IDF 4/IDF 5 duration, and
65988 edges is the capture's step count. The translator loses and
gains nothing for seq_02, so the extra ~29 s is not encoded time.

### The pause prelude overflow is unreachable (guard reverted)

`emit_pause_symbols()` subtracts 8 ticks from each of the first
`PART_SIZE-1` symbols when building a pause. A pause with fewer than
`8*(PART_SIZE-1)+2` ticks would wrap `uint16_t` and become a single
multi-millisecond low (and `8*(PART_SIZE-1)`/`+1` would leave a
zero-duration sub-entry, an RMT stop pattern). That cannot happen:
`addQueueEntry()` rejects any command whose period (times steps) is below
`MIN_CMD_TICKS` (`AQE_ERROR_TICKS_TOO_LOW`), and for a pause `steps == 0`
so the checked period is exactly `cmd->ticks`. With `MIN_CMD_TICKS =
3200` this is far above `8*(PART_SIZE-1)+2` (248 for PART_SIZE 32, 186
for 24). The defensive branch/guard was therefore reverted; the
`test_pause_small_ticks` white-box test fed sub-`MIN_CMD_TICKS` pauses
that the encoder can never receive and was removed with it.

### `emit_step_symbols()` did not report its symbol count

`emit_step_symbols()` writes **two** RMT symbols for `ticks == 0xffff`
(a 15-bit duration cannot hold 32768) and **one** otherwise, but it
returned `void`. The caller re-derived the count with
`per = (e->ticks == 0xffff) ? 2 : 1`, so the 1-or-2 rule was duplicated
in two places. Fixed: the function now returns the number of symbols
written (0 when `symbols_free` is too small) and `rmt_encode_queue()`
advances `written`/`symbols_free` by that return value. Behavior is
unchanged for seq_02 (the counts already agreed), but the caller can no
longer diverge from what was actually emitted.

`test_30` now backs this: `test_two_symbol_cases_report_their_count`
checks the `0xffff` (two symbol) and normal (one symbol) cases under 8, 2
and 1 free symbols with a `0xDEADBEEF` sentinel, so a write past the
returned count is a failure, and `test_seq_02_total_ticks` asserts
`global_min >= 2` (relation 1), `zero_durations == 0` and
`overrun == 0` for the whole sequence. Before this the suite summed only
ticks and edges, so a 1-or-2 symbol accounting error or a sub-2-tick half
could pass unnoticed.

### The empty-queue stop path does not account for the 29 s

`test_seq_02_holes` runs a time-domain model of the IDF 5/6 driver:
RMT memory = `2*PART_SIZE` symbols, task tick = `DELAY_MS_BASE` (4 ms),
`encode_commands()` sets `_rmtStopped` and emits
`ENTER_PAUSE(MIN_CMD_TICKS)` the moment the queue is empty, the task is
locked out until the memory drains, and it restarts a transaction on
its following tick.

```
PART_SIZE 32: wall=93.034s encoded=91.459s injected=0.445s stops=2226  extra=1.575s (0.708ms/stop)
PART_SIZE 24: wall=92.680s encoded=91.459s injected=0.349s stops=1746  extra=1.221s (0.699ms/stop)
capture:      120.760s  -> delta -27.7s / -28.1s
```

The model reproduces the encoded stream and the stop *cadence* (140
stops on the 6621 pair, the same order as the 223 holes quoted above),
but each modeled stop costs only about 0.7 ms. The capture's holes are
6-10 ms. To add 29.3 s over ~2200 stops a stop would have to cost
**~13 ms**, not the ≤4 ms a 4 ms task can produce. The empty-queue stop
path, as modeled, is therefore not sufficient by itself: the real
per-stop latency is much larger, or a further mechanism is involved.

## Capture re-measurement (2026-09-28, this review)

The two CSV files were aligned edge-for-edge (both 65988 rising edges,
same step index) and the per-step rise-to-rise period subtracted. This
removes the ramp shape and isolates the injected time.

```
idf6 first-to-last     120.760 s
idf5.3.1 first-to-last  91.637 s
sum of positive extra   29.146 s
sum of negative extra   -0.022 s
```

Holes (extra > 2 ms) clustered across all 34 moves:

```
holes                 3256
per-hole extra        median 9.0 ms, mean 8.9 ms, max 10 ms
hole shape            exactly one step's LOW phase is stretched;
                      HIGH pulse width is unchanged
hole cadence          median 21 ms of motion, one step wide
```

So the injected time is ~9 ms per hole and the cadence is ~21 ms, i.e.
about one hole per forward-planning window (`TICKS_PER_S / 50` = 20 ms).
The 9 ms is close to two `DELAY_MS_BASE` task ticks (2 x 4 ms) plus the
200 µs `ENTER_PAUSE(MIN_CMD_TICKS)`. This is a *software timing*
signature, not a per-symbol hardware stretch (which would be smeared
over all symbols, not concentrated in single low phases).

### Minimum RMT symbol period in the shared IDF 5/6 encoder

There is **no separate IDF 6 RMT code**. For the RMT V2 driver both IDF 5
and IDF 6 use the same two library files,
`StepperISR_idf5_esp32_rmt.cpp` and `StepperISR_idf5_esp32_rmt_encode.cpp`
(both guarded by `SUPPORT_ESP32_RMT_V2`). Only MCPWM/PCNT, PCNT and the
I2S manager have per-IDF-version files. So "the idf6 encoder" is a
misnomer; the version difference in the 92 s vs 121 s split lives in
ESP-IDF's `esp_driver_rmt`, not in this library (see H4).

The idf4 files document the hardware floor and the shared encoder does
not check it:

- `StepperISR_esp32xx_rmt.cpp:26-38` (also idf4 s3/c3): relation 1
  `3*T_APB + 5*T_RMT_CLK < period*T_CLK_DIV` => `period > 1.6`, i.e. a
  symbol half of **1 tick is illegal**, 0 is the **stop pattern**;
  relation 2 before the end marker wants **period >= 4**.
- `extras/tests/pc_based/AGENTS.md`: "Minimum RMT symbol period is 2
  ticks (hardware limit)".
- `StepperISR_idf5_esp32_rmt_encode.cpp` has no such guard.
  `emit_step_symbols` splits a step into `floor(ticks/2)` and
  `ceil(ticks/2)`; `ticks < 4` yields a half < 2, `ticks == 1` yields a
  0-duration half (stop pattern). `emit_pause_symbols` is not a concern:
  its prelude is 4 ticks and, as established above, every pause is
  >= `MIN_CMD_TICKS` so the prelude cannot underflow.

### RMT refill invariants (`test_30` conditions)

The root failure is that `encode_commands()` sets `_rmtStopped` whenever it is
invoked and the queue is empty. It is invoked with `2*PART_SIZE` free at
transaction start and `PART_SIZE` free at each threshold, and
`rmt_encode_queue()` drains commands into the RMT until the RMT is full *or
the queue is empty*. So the stop fires precisely when **the whole queue fits
in the RMT buffer**. The invariant that prevents it is a bound on the
encoder's read-ahead.

**Primary condition (symbols per command).** Every queue command must
contribute at least `PART_SIZE/2` RMT symbols. Then the full RMT buffer
(`2*PART_SIZE` symbols) holds at most **four** commands, and the ramp's
~20 ms lookahead (>=~5 commands) can never be drained into the RMT. This is
the right unit: the queue is filled and drained per command, and the bound is
deterministic (independent of ISR/task timing).

`test_30` now tracks the emitted symbols per queue command and asserts it.
On seq_02 it fails:

```
PART_SIZE 32: min symbols/command = 1 (steps=1 ticks=63264)  need 16  FAIL
PART_SIZE 24: min symbols/command = 1 (steps=1 ticks=63264)  need 12  FAIL
```

The offender is a single 3.95 ms step, just below the 65535 split. Pauses
already contribute `PART_SIZE` symbols, and fast step commands contribute
~`planning_steps` (~50), so only the 1.3-4.096 ms band is short. Above
4.096 ms the ramp already emits a `steps == 0` pause (`RampControl.cpp:457-465`)
plus a half step, which also gives `PART_SIZE` symbols and the 25%-duty
waveform seen on the scope.

Satisfying the condition by *batching* more steps is not possible at slow
speed (`PART_SIZE/2` steps at 3.95 ms is ~63 ms of lookahead). The only
practical route is to **split a long step's low phase over several RMT
symbols** so the command covers `PART_SIZE/2` symbols while the 20 ms
lookahead is preserved. That requires `emit_step_symbols()` to return more
than 1 or 2 symbols for long periods.

**Secondary / candidate conditions (time per symbol).** Instead of counting
symbols, bound the *time* each symbol may represent. If every symbol covers
at most `65536/PART_SIZE` ticks, the full RMT buffer covers at most
`2*65536` ticks = 8.192 ms, i.e. less than the 20 ms lookahead — the same
read-ahead bound expressed in time. "Even only one fourth" (max
`65536/(4*PART_SIZE)` ticks, buffer <= 2.048 ms) would tighten the staleness
bound further. Note the tension: bounding the buffer time *above* helps the
queue-drain (read-ahead) problem but *hurts* the starvation problem, so this
and the earlier >=2 ms half-buffer idea cannot both be absolute. The symbol
count per command is the safer primary invariant; the time bound is a
backstop for the very slow regime where the lookahead holds fewer than five
commands but one or two commands already buffer tens of ms.

**Still open — more ideas wanted.** Neither condition is proven sufficient
end-to-end:
- the primary condition bounds the drain to four commands, but the ramp can
  still hold fewer than five commands in the very slow regime;
- the earlier sliding `PART_SIZE`-symbol window was downgraded to
  informational because it conflated fast steps with the deliberate 8-tick
  pause prelude (a threshold-early device, not a ramp bug);
- the read-ahead bound and the starvation bound pull in opposite directions
  on symbol duration, so a single monotone condition may not exist.
The next idea should probably state the requirement on the *ramp* in terms of
"time the RMT buffer can still play when the queue empties" versus "worst
case time until the task refills", rather than on symbol counts.

`test_30` asserted neither the minimum nor the symbol count for seq_02.
It now asserts `global_min >= 2`, no zero-duration sub-entry, no write
past the reported symbol count, and the 1-or-2 symbol return, across
PART_SIZE 32 and 24 (the separate per-command refill condition is
currently failing — see above). Its output is `min=4` — that 4 comes from
the `0x00040004` pause prelude, and all step periods in seq_02 are
>= 640 ticks. **The captured seq_02 stream therefore satisfies the
relation.** The gap is real and worth closing, but it is not, by itself,
the 29 s.

## Hypotheses

Each hypothesis carries a judgement: how likely, and why it is (or is not)
worth pursuing. The working order is H1 first, then H4/H3 as timing/driver
explanations, with H2 kept as a latent robustness item and H5 gating H4.

| # | hypothesis | explains | confidence | priority |
|---|---|---|---|---|
| H1 | eager empty-queue stop + restart | the hole itself | high: in our code, matches cadence and idf4 split | 1 |
| H2 | RMT minimum-period violation | latent corruption | excluded for seq_02 (min half = 4) | 5 |
| H3 | threshold-refill starvation | per-stop magnitude | medium: likely why ~9 ms not ~0.7 ms | 3 |
| H4 | IDF 5.3.1 -> 6.1 driver change | idf5 vs idf6 split | low-medium: listed diffs are small | 2 |
| H5 | attribution / measurement control | baseline trust | n/a (precondition, gates H4) | gate |

H1 — Eager empty-queue stop plus restart latency (primary). The encoder
greedily moves every queued command into RMT memory. It is then called
again with the queue momentarily empty, sets `_rmtStopped`, appends
`ENTER_PAUSE(MIN_CMD_TICKS)` + EOF and returns `done=true`.
`isReadyForCommands()` is false for the rest of the transaction, so
`fill_queue` cannot top up; when the transaction ends the StepperTask
(4 ms period) restarts it. `fill_queue` only keeps 20 ms of commands in
the queue, so this repeats once per planning window. Predicted signature:
one hole per ~20 ms, each hole a single low interval of (transaction end
+ task alignment + pause); idf4 does not take this early-stop path and
has no holes. Matches all capture numbers except the exact 9 ms, which
implies the restart needs ~2 task ticks, not <= 1.

*Judgement: strongest, and the one to pursue first.* The mechanism lives
in our own code (`encode_commands()`), it reproduces the cadence
(~one hole per planning window), it explains the idf4 vs idf5/6 split
(the eager `_rmtStopped` is new), and the new read-ahead condition
(`test_30`) fails exactly where the holes are. It is the only hypothesis
that is directly actionable without new hardware measurements. Weakness:
the per-stop cost is not yet explained (model undercounts by ~13x), so
P1/P3 must pin the restart latency before concluding.

H2 — RMT minimum-period violation (the user's observation). If a step or
pause symbol carries a half-period of 0, the RMT reads it as a stop
pattern and ends the transaction early; a 1-tick half is stretched by the
APB synchroniser. Either way the pin can idle while the driver still
believes symbols are queued. Predicted signature: a hole immediately
after the offending short symbol, and a symbol stream whose minimum
half-period is 0 or 1. *Against*: the seq_02 PC stream has min half = 4
even at PART_SIZE 24, so this cannot explain the observed seq_02 holes
unless the on-target command stream differs from `test_30`'s.

*Judgement: excluded for seq_02, keep as latent.* The measured seq_02
minimum is 4 ticks, so the observed 29 s is not this. It only becomes
relevant if P1 shows a hole *starting* at a short symbol, or if a
different command stream (short pauses, single-step moves, user
dir-delay) is shown to reach half < 2. Do not spend the next step here.

H3 — RMT starvation at the threshold refill. The pause prelude drains 31
symbols of 8 ticks each in ~16 us, so the threshold ISR must refill a
whole half-block within that window. If the ISR is delayed (other ISRs,
cache, the I2S task) the hardware reaches the end of the memory block and
stops before the encoder writes the next half. This is IDF-driver
dependent and would explain why IDF 5.3.1 and IDF 6.1 (same `encode.cpp`)
differ. Predicted signature: hole length tracks ISR latency, not the
planning window; worse with Wi-Fi/other interrupts.

*Judgement: plausible secondary, not yet separable.* The 1.04 ms
half-buffer window (see above) makes ISR/refill latency matter, so this
could turn a transient empty queue into a real under-run and is a good
candidate for *why the per-stop cost is ~9 ms rather than ~0.7 ms* in
H1. It is not a mechanism on its own (it needs the queue to empty first)
and cannot be reproduced in the PC tests, which have no ISR timing. Chase
it only via P1, as the explanation for H1's magnitude, not as a
standalone cause.

H4 — IDF driver behaviour change. Both IDF 5.3.1 and IDF 6.1 use
`SUPPORT_ESP32_RMT_V2` and the same encoder, so the 92 s vs 121 s split
must be inside `esp_driver_rmt`. The actual IDF 5.3.1 -> 6.1 changes in
`rmt_encode_simple` are narrower than first stated:

- the symbol offset is byte-based (`mem_off_bytes`) instead of a symbol
  count (`mem_off`); cosmetic for us;
- a new `RMT_ENCODING_WITH_EOF` state bit and a `need_eof_marker`
  argument to `rmt_tx_mark_eof`. Our simple encoder never sets
  `WITH_EOF`, so `need_eof_mark` is always true and EOF marking is
  unchanged;
- `RMT_ENCODING_MEM_FULL` is now `symbol_off >= mem_end` rather than the
  `else` of `is_done`, so completing exactly at the memory end can be
  `COMPLETE|MEM_FULL` and skip that round's `mark_eof`;
- the DMA descriptor plumbing is rewritten (irrelevant: `with_dma = 0`).

The `rmt_tx_mark_eof` call in `rmt_isr_handle_tx_threshold()` exists in
**both** versions (idf5 `rmt_tx.c:914`, idf6 `rmt_tx.c:918`); it is not a
new IDF 6 step. Predicted signature: same app code, only the framework
version changes; the hole count stays, the per-hole cost changes.
Whether these small driver changes actually cause the 92 s vs 121 s is
what P6 must measure, not assume.

*Judgement: necessary to explain the IDF-version split, but weak on its
own.* H1 explains why a hole exists; H4 could explain why IDF 6's holes
are longer/more frequent than IDF 5's, since the encoder is identical.
None of the listed driver diffs obviously turns a stop into a long idle,
so confidence is low until P6 (controlled A/B) and P1 (restart timing)
are done. It is *the* place to look for the IDF-version difference, so it
is second in priority after H1.

H5 — Attribution/measurement control. The doc already flags that the
`digital_rmt_idf4.csv` name is the wrong generation. Until both captures
are reproduced from named envs on one board, the IDF-5.3.1 vs IDF-6
comparison is not a controlled A/B and H4 cannot be separated from chip
or build differences.

*Judgement: not a mechanism — a precondition.* It cannot be "confirmed"
or "refuted"; it gates H4. Do it once (P6) so the other results are
trustworthy. Lowest intellectual priority, highest process priority.

## Fix options (analysis)

The root failure is the queue becoming empty **while the motor is running**:
`encode_commands()` then has nothing to encode, the RMT buffer drains, and the
pin idles. The stop/restart is only the *aftermath*. So a fix is only a
solution if it keeps the queue non-empty while running (or the buffer with
enough time to cover the dry period); removing the stop without that does not
help. Options are grouped accordingly, each with a judgement.

Honest qualification: the cadence (~21 ms) and the exact queue-empty event
are still unexplained. Two different quantities could cause the pin idle and
the options target different ones:

1. **Queue content** (symbols): the encoder drains the queue into the RMT at
   transaction start (up to `2*PART_SIZE` symbols). If the lookahead's symbol
   content is smaller than the RMT capacity, the queue reaches empty.
2. **RMT buffer time**: the pin idles only if the RMT buffer is dry when the
   encoder has nothing to encode. The buffer's playback time at the fast end
   is `2*PART_SIZE * period` = 64 * 40 us = 2.56 ms, which is *less than the
   4 ms task period*.

F2 targets (1); F3 targets (2). We cannot honestly rank them before P1
distinguishes which quantity actually goes to zero when the pin stops. The
judgements below are therefore conditional.

| # | fix | targets | rating |
|---|---|---|---|
| F1 | keep transaction alive | restart latency only | rejected as a solution (queue still empties while running) |
| F2 | read-ahead bound: cap symbol duration at `ceil(65535/PART_SIZE)` t | quantity (1) queue content | **chosen**; design below |
| F3 | bigger RMT buffer | quantity (2) buffer playback time | strong lever if (2) is the cause; needs F1/F4b |
| F4 | cheaper restart | restart latency | mitigation only |
| F5 | idf4-style synchronous ISR refill | read-ahead by construction | rejected: not allowed under IDF 5/6 |
| F6 | continuous / DMA feed | - | rejected |
| F7 | ramp lookahead floor | quantity (1), in time | insufficient alone; part of F2 |

**F1 — Keep the transaction alive until motion really ends.**
In `encode_commands()`, when `read_idx == next_write_idx`, do not set
`_rmtStopped` and do not return `done`; emit the `ENTER_PAUSE(MIN_CMD_TICKS)`
filler and return. Add an explicit "no more commands" signal (set when the
ramp goes idle, e.g. by `fill_queue`/`manageSteppers`) that arms the real stop.
- Pro: removes the restart overhead (the multi-ms latency after a dry buffer);
  reuses the existing stop path for genuine end-of-move; small, local change;
  testable by P3.
- Con: needs a new end-of-motion signal crossing queue <-> stepper (the queue
  ISR does not know the ramp state today); must guarantee the channel always
  stops (else it emits pauses forever); adds up to one `MIN_CMD_TICKS` filler
  per transient empty (200 us, still ~40x less than the 9 ms hole); must not
  break single-stepping (the existing "return done here or single stepping
  fails" note).
- *Judgement: rejected as a solution.* It does not stop the queue from being
  empty while running, so the RMT buffer still drains and the pin still idles;
  the filler adds time instead. It only removes the restart overhead after the
  dry period. Useful only together with F2/F3, never alone.

**F2 — Read-ahead bound: cap the RMT symbol duration (chosen).** Split long
steps and pauses into several RMT symbols so the RMT buffer's playback time is
bounded below the lookahead. Full design in the next section.
- Pro: deterministic, no ISR timing, no end-of-motion signal; keeps the 20 ms
  lookahead; directly prevents mid-move drains (the holes are mid-move).
- Con: variable-length symbols; every emit path must be updated; a step or
  pause must still fit in one half-buffer; needs the split to preserve exactly
  one rising edge and the total ticks.
- *Judgement: chosen solution.* It is the only available option that directly
  prevents the queue emptying while running (F5 is unavailable, F1/F3/F4 only
  treat the aftermath or the symptom).

**F3 — Bigger RMT buffer.** `mem_block_symbols = k*2*PART_SIZE` (several RMT
memory blocks per channel) or a larger `PART_SIZE`.

The quantity that decides whether a momentary queue-empty becomes a pin idle
is the RMT buffer's playback *time*, and F3 is the direct lever on it. At the
fast end the current buffer is only `2*PART_SIZE * 40 us` = 2.56 ms, i.e.
below the 4 ms task period - the exact deficit a bigger buffer removes.

- Pro: one-line config, no CPU; directly raises the buffer's playback time
  above the refill latency; if mechanisms (2) dominates this is the *simplest*
  solution to the pin-idle.
- Con: consumes RMT memory needed by other channels (fewer motors); a larger
  buffer also lets the encoder drain more of the queue into it at transaction
  start, so it can raise the *frequency* of the empty-queue detection; and the
  eager stop still ends the transaction, so if `_rmtStopped` blocks refills the
  extra time is spent playing already-buffered symbols and the hole only moves
  to the end of the buffer.
- *Judgement (revised, honest): a legitimate solution lever for the pin-idle
  symptom, not "counterproductive".* If P1 shows the pin idles because the
  buffer is dry while the queue is momentarily empty, then buffer playback
  time is the quantity to fix and F3 addresses it directly — but it must be
  paired with not letting the eager stop terminate the transaction and block
  refills (F1/F4b), otherwise the bigger buffer is unusable for new commands.
  It is the wrong tool at slow speed (the buffer already covers the window)
  and cannot help if the queue never refills at all. Whether it *suffices* is
  set by the measured ratio `buffer_playback_time / refill_latency`, which P1
  must provide; nothing here should be dismissed or committed before that.

**F4 — Make the restart cheap.**
(a) lower `DELAY_MS_BASE`; (b) stop `isReadyForCommands()` locking the ramp
while `_rmtStopped`; (c) restart from the TX-done ISR / queue the next
transaction (`trans_queue_depth > 1`) instead of waiting for the task.
- Pro: (c) can cut the 4-8 ms task latency to microseconds and is small if the
  stop path is retained; (b) lets the task pre-fill during the stop.
- Con: (a) more CPU/timer load; (b) alone still ends and restarts the
  transaction; (c) the encoder must have the next transaction prepared in
  advance and the stop/`_rmtStopped` bookkeeping gets subtle. F4c is the
  cheapest way to shrink holes only as a stop-gap if F2 is delayed.
- *Judgement: mitigation only.* It shortens the recovery after a dry queue but
  does not keep the queue non-empty while running, so the pin still idles. Low
  value once F2 or F3 lands.

**F5 — Bypass the IDF simple encoder; use the idf4-style synchronous ISR
refill for IDF 5/6 RMT.** Drive RMT registers directly and refill from the
threshold/end ISR (`StepperISR_idf4_esp32_rmt.cpp` pattern).
- Pro: proven fast (idf4 is at 94 s); no async task restart; unifies the fill
  strategy.
- Con: large rewrite; loses the IDF driver abstraction that idf5/6 adopted for
  portability; must handle S3/C3/C6 RMT V2 register and clock differences.
- *Judgement: rejected — not available under IDF 5/6.* On IDF 5/6 the RMT
  peripheral is owned by `esp_driver_rmt`: `rmt_new_tx_channel()` claims the
  channel, its memory and its interrupts, so the idf4 pattern (own ISR + direct
  `RMT.*` register access) is no longer permitted alongside the driver. This
  was the "closest to proven" fallback but cannot be used; it is recorded here
  only to explain why the library moved to the simple encoder and why we must
  fix the behaviour *through* the driver API.

**F6 — Continuous feed (DMA / loop / streaming).** Never stop the channel.
- Pro: no stop/restart at all.
- Con: loop mode repeats the buffer (wrong); RMT-classic DMA is limited; big
  rewrite. Not attractive.
- *Judgement: rejected.* Does not address why the queue drains, and the RMT
  classic cannot do it cleanly.

**F7 — Ramp-level floor on queued commands.** Guarantee the ramp always keeps
>= N commands queued.
- Pro: conceptually simple.
- Con: the encoder drains eagerly, so unless RMT capacity < lookahead (F2) a
  larger lookahead just moves the problem; measured seq_02 is already ~5
  commands. Low value alone.
- *Judgement: insufficient alone, but is the mechanism F2 relies on.* A
  lookahead floor only helps if it is expressed in *symbols* (or in commands
  with F2's per-command floor); a time-only floor still fits in the RMT. Keep
  it as the secondary half of F2, not as a fix by itself.

**Recommendation.** F2 is chosen (design below): cap every RMT symbol at
`MAX_TICKS = 65536/PART_SIZE`, which makes the RMT buffer span less time than
the 20 ms lookahead and keeps the queue non-empty while running. F5 is
unavailable under IDF 5/6, so F2 is the only option that attacks the cause
directly; F3/F4 remain fallbacks/stop-gaps and F1/F6 are rejected.

P1 is still run first as a cheap confirmation and to validate the `MAX_TICKS`
choice: at each pin idle record `read_idx == next_write_idx` (queue empty?)
and the number of symbols still resident in RMT memory. Expected after F2:
queue never empty mid-move. If P1 instead shows the buffer dry with a
non-empty queue, fall back to F3.

## F2 design — cap the RMT symbol duration

### Goal

Make it impossible for the encoder to drain the queue while the motor runs.
The encoder fills the RMT buffer (at most `2*PART_SIZE` symbols); if the
buffer can span less time than the ramp's lookahead, a full fill always leaves
queued commands, so `encode_commands()` never sees an empty queue and the eager
`_rmtStopped` stop cannot fire mid-move.

### Invariants

- **I1 (time cap, primary):** every RMT symbol covers at most `MAX_TICKS`
  ticks, with `2*PART_SIZE*MAX_TICKS < _forward_planning_in_ticks`. Then the
  full buffer spans less time than the lookahead. I1 does not depend on the
  ramp producing a minimum number of commands.
- **I2 (command floor, secondary):** every queue command contributes at least
  `PART_SIZE/2` RMT symbols, so the buffer holds at most four commands. This is
  the form `test_30` already checks. I1 implies I2 for commands whose total
  ticks `T` satisfy `T/MAX_TICKS >= PART_SIZE/2`.

`_forward_planning_in_ticks = TICKS_PER_S/50 = 320000` (20 ms). The two
bounds on `MAX_TICKS`:

```
read-ahead (hard):  MAX_TICKS < 320000/(2*PART_SIZE)  = 5000 (P=32) / 6666 (P=24)
step/pause fit:     MAX_TICKS >= 65535/PART_SIZE      = 2048 (P=32) / 2731 (P=24)
```

Choosing `MAX_TICKS = ceil(65535/PART_SIZE)` sits exactly at the "fit" bound
(the largest step/pause splits into exactly `PART_SIZE` symbols):

```
P=32: MAX=2048, full buffer <= 2*32*2048 = 131072 t = 8.192 ms (<20),
      half <= 65536 t = 4.096 ms, max step/pause = 32 symbols = PART_SIZE
P=24: MAX=2731, full buffer <= 2*24*2731 = 131088 t = 8.193 ms (<20),
      half <= 65544 t = 4.097 ms, max step/pause = 24 symbols = PART_SIZE
```

(`floor(65536/PART_SIZE) = 2730` is too small for P=24: `ceil(65535/2730) =
25 > 24`, so use the ceil.)

"Even only one fourth" (`MAX = 65536/(4*PART_SIZE)` = 512/682) gives ~4x
read-ahead margin but multiplies the symbol count by 4 and is not needed: I1
already keeps the queue non-empty, so the buffer does not have to cover the
4 ms task period. Recommend `MAX_TICKS = ceil(65535/PART_SIZE)` (2048 / 2731),
which also guarantees a single step or pause fits in one `PART_SIZE` half.

Why this makes I2 nearly free: the ramp's fast-speed command length is
`planning_steps = (TICKS_PER_S/500)/curr_ticks`, so a command spans ~32000
ticks and yields `~32000/MAX = PART_SIZE/2` symbols. Slow commands have
`planning_steps = 1` but a long period, so a single step splits into
`~ticks/MAX` symbols. Both regimes land near `PART_SIZE/2`; only commands
shorter than `MAX*PART_SIZE/2 = 32768 t` (2.05 ms) fall below it, which is the
move-tail/short-move case.

### Symbol model

An RMT symbol is two 16-bit sub-entries: `duration[14:0]` (1..32767) plus a
level bit. Consecutive sub-entries with the **same level produce no edge**, so
a long constant phase can be split freely. All emitted symbols also have both
halves at the same level; the only level changes are the step edges.

A step with period `ticks` (`H = ticks>>1` high, `L = ticks - H` low) is
emitted as `ceil(H/MAX)` all-high symbols followed by `ceil(L/MAX)` all-low
symbols. The first high symbol starts the rising edge (the previous step ended
low); the first low symbol starts the falling edge. Total ticks preserved, one
edge pair per step. `ticks == 0xffff` is no longer special - it is just the
largest split.

A pause keeps its **exact `PART_SIZE`-symbol footprint** (one RMT half): the
pause ticks are spread uniformly over `PART_SIZE` all-low symbols, each
`<= MAX`. The count must not change - see the anti-regression section: the
direction-change drain logic (`esp32_before_pause_count() = 2`, `esp32_queue.h`)
and the empty-queue stop both assume one pause == one half.

Constraints:
- every sub-entry in `[1, 32767]`; practically `[2, 32767]` for relation 1
  (so each symbol total >= 4; merge any 1..3-tick tail into the previous
  symbol rather than emitting a 0/1-tick half, which is the stop pattern);
- a single step or pause must fit in one `PART_SIZE` half: guaranteed by
  `MAX >= 65535/PART_SIZE`, so it can always be written into the threshold
  half or the IDF overflow buffer without carrying a partial step across
  calls.

### Changes

- `emit_step_symbols(uint32_t* data, uint16_t ticks, uint32_t symbols_free)`
  -> returns the number of symbols written (0 if `symbols_free` cannot hold the
  whole step); emits the split above.
- `emit_pause_symbols(...)` -> all-low, still exactly `PART_SIZE` symbols,
  uniformly split so each symbol `<= MAX_TICKS`; the 8-tick prelude goes away.
  The count stays `PART_SIZE` (drain logic depends on it).
- `rmt_encode_queue()`: for a step, compute the needed count first and only
  emit if `symbols_free >= needed` (the step path already uses the return
  value); the pause path keeps its `symbols_free >= PART_SIZE` gate.
- `ENTER_PAUSE(MIN_CMD_TICKS)` in `StepperISR_idf5_esp32_rmt.cpp` stays
  `PART_SIZE` symbols but must distribute them so each symbol `<= MAX_TICKS`
  (uniform, no 8-tick prelude).
- `min_chunk_size = PART_SIZE` overflow buffer still works: with
  `MAX >= 65535/PART_SIZE` the largest step/pause is `PART_SIZE` symbols, so it
  fits the overflow buffer and no step is split across calls.

### Test plan (`test_30`)

- assert I1: every emitted symbol total `<= MAX_TICKS` (new `any_symbol_above`);
- keep I2 (`min symbols/command >= PART_SIZE/2`) as the command-level check,
  expected to hold except for commands shorter than `32768 t`;
- keep the exactness checks: total symbol ticks == commanded, one edge per
  step, no zero/one-tick sub-entry, no write past the reported count;
- expected readings for seq_02 after the change: largest step 65535 -> 32
  symbols (P=32); a 4 ms step -> 32 symbols; the max symbol <= 2048;
- the `test_seq_02_holes` model should then never report a stop (queue never
  empties), which is the end-to-end proof for the mechanism;
- **hardware re-validation for the extra-step fix** (cannot be done on PC):
  `seq_02`/`check_pcnt_sync` and the `seq_15` sweep with
  `FAS_RMT_DEBUG_COUNT` off, per `esp32_rmt_extra_step.md`. A green PC run is
  not evidence that H8 did not return.

### Anti-regression vs `esp32_rmt_extra_step.md` (mandatory)

The current encoder is the fix for the confirmed "one spurious step pulse at
end of a move" bug (`extras/doc/implemented/esp32_rmt_extra_step.md`). Its
locus was H8: the IDF 5/6 ping-pong driver can replay a half, and when that
half contains a step pulse (`0x4000FFFF`, high `0x7fff`) the pin gains an
extra rising edge. The fix replaced the IDF4 fixed-half filler with the
translator: `encode_commands()` fills exactly `symbols_free`, a step is as
short as possible, `steps` is written back, and the stop check runs before any
short-buffer return. F2 rewrites that translator's symbol layout, so it can
re-expose H8 and it invalidates the existing hardware validation.

Constraints F2 must obey:

1. **Keep the translator contract.** No return to the fixed-half
   `rmt_fill_buffer()` path (`StepperISR_esp32xx_rmt.cpp`) for IDF 5/6; fill
   exactly the bytes the callback was given; `read_idx`/`steps` stay
   whole-step.
2. **Keep the pause footprint at exactly `PART_SIZE` symbols.** `encode_commands`
   stops by emitting `ENTER_PAUSE(MIN_CMD_TICKS)`, and the direction-change
   drain needs two such pauses to fill the two in-flight halves
   (`esp32_before_pause_count() = 2`, `esp32_queue.h`). A variable-length pause
   would break the "other half is a pause" condition and can move a step across
   a DIR change. Only the *distribution inside* the half changes (each symbol
   `<= MAX`), never the count.
3. **Keep the H9 hole closed.** With variable-length symbols the early return
   must remain "the next unit does not fit", and returning 0 must stay a legal
   retry with `min_chunk_size = PART_SIZE` (`ceil(65535/PART_SIZE)` symbols,
   so the largest unit always fits). Never return 0 in a way that leaves the
   wrap to replay the last half. The empty-queue stop is checked before the
   short-buffer return (already so).
4. **The `emit_step_symbols` return count is load-bearing.** The driver's
   `mem_off`/`mem_end` phase must match the symbols handed over; the
   return-count fix (above) is required by F2, not optional.
5. **Re-check the half-replay exposure.** A saturated slow step's high is now
   `ceil(32767/MAX)` ~ 16 symbols. If a step's pulse can land as one
   self-contained half, a half replay is again an extra step. Either offset the
   split so a pulse is not half-aligned, or argue/measure that the replay
   cannot happen. This must be settled before F2 is trusted.
6. **Re-validate on hardware with the debug macro off.** `test_30` (PC) cannot
   see the timing race. Run `seq_02` / `check_pcnt_sync` and the `seq_15`
   50...743 sweep with `FAS_RMT_DEBUG_COUNT` **off** (the doc shows the macro
   shifts the timing and makes the fault near-certain, so it must be off for
   the confirmation), for both PART_SIZE 32 and, if available, 24.

F2 does not touch IDF4's path, so the IDF4 control stays valid.

### Open questions / risks

- Threshold/prelude behaviour: the pause keeps its `PART_SIZE` count but loses
  the 8-tick prelude, so the threshold fires uniformly inside a pause instead
  of immediately. I1 makes the queue non-empty so refill timing is less
  critical, but verify on target (P1).
- Half-replay exposure (the H8 regression risk above): with a split step the
  pulse spans several symbols; confirm a half cannot become a self-contained
  step pulse, or that the replay path stays unreachable.
- ISR load: more symbols per unit time means the encoder callback and the
  threshold ISR run more often; at the slow end a 65535-tick step is 32
  symbols instead of 1, so the symbol rate rises - acceptable at slow step
  rates, needs a sanity check at the fast end (unchanged, 1 symbol/step).
- The `read_idx`/`steps` write-back must remain whole-step (it does, because a
  step/pause never exceeds a half-buffer); confirm with the existing
  `test_remaining_steps_written_back`-style cases extended to long steps.
- Choose `MAX_TICKS` as a `pd_config.h` constant (per platform) so it is near
  `65535/PART_SIZE` for the RMT variants (P=32, P=24).

## Probe/experiment design (confirmation)

The 91.46 s encoded total and the 65988 edges stay as references. All
experiments below should be run on the same board and the env recorded.

**P1 — Instrument the stop/restart loop (tests H1, H3, H4).**
Use the existing probes in `pd_esp32/test_probe.h` and add one RAM
timestamp per transition, then dump after a move:
- PROBE_1 (`startQueue_rmt`, rmt_transmit) — arm phase.
- PROBE_2 (`queue_done` / on_trans_done) — transaction end.
- PROBE_3 (threshold ISR, where the encoder refills).
- PROBE_4 (read_idx advance / command consumed).
Also record `esp_timer_get_time()` when `_rmtStopped` is set and when the
StepperTask restarts. Saleae the STEP + probes at 1 MS/s. Directly
measure the gap from PROBE_2 to the next PROBE_1 and compare it to the
~9 ms hole. If that gap is ~2 task ticks, H1 is confirmed; if PROBE_2
happens *before* the STEP low ends (transaction ended early), H2/H3 is in
play.

Decisive extra log at each pin idle (this picks F2 vs F3): the queue state
(`read_idx == next_write_idx`?) *and* the number of symbols still resident
in the RMT memory. Queue empty + buffer dry => quantity (1)/(2) both; queue
empty + buffer non-empty but the transaction still stopped => the eager stop
(H1) is the trigger; queue non-empty but pin idle => neither F2 nor F3, look
at H3/H4.

**P2 — Perturb the planning window (tests H1).** Rebuild with
`_forward_planning_in_ticks` and/or `DELAY_MS_BASE` changed and re-run
seq_02. H1 predicts the hole cadence scales with the planning window and
the hole size scales with the task tick. If the cadence is invariant, H1
is wrong and H3/H4 dominate.

**P3 — Disable the eager stop (direct causal test of H1).** Temporarily
make `encode_commands` keep the transaction alive when the queue is empty
(e.g. emit a longer pause / return `done=false` for a bounded number of
calls) so the ramp can catch up. If the 29 s disappears with no other
change, H1 is proven. Keep the current code as the control.

**P4 — Force a minimum-period violation (tests H2).** Extend `test_30`
with an assertion that every emitted half-period is >= 2 and that the
symbol immediately before the EOF is >= 4, and sweep pauses 2..249,
step periods 1..8 ticks and odd periods. This enumerates every violator
off-target. Then feed one computed violator on-target and look for a
transaction abort / extra idle. If the target never sees a half < 2 for
seq_02, H2 is excluded for this item.

**P5 — Extend the `test_30` model (tests H1 vs reality).** The current
`run_move_model` restarts within the same task tick. Parameterise the
per-stop latency `L` and solve for the `L` that reproduces 120.760 s
from the known encoded stream and stop count. If `L` lands on
~2 x `DELAY_MS_BASE`, the model closes; then use P1/P3 to explain why
the real restart takes those two ticks.

**P6 — IDF A/B (tests H4).** Build the identical app with
`esp32_idf_V6_9_0` (IDF 5.3.1) and an IDF 6 env on one board, timestamp
the build, capture STEP, and confirm the hole count vs hole size. This
also removes the file-name generation ambiguity (H5).

Order: P6 (freeze the baseline) -> P1 (see the loop) -> P3 (causality) ->
P2/P5 (model) -> P4 (close the latent min-period gap). The fix work then
follows F2 (the read-ahead bound, with F7's symbol-based floor) once P1/P3
confirm H1; F3 (if the buffer is dry) or F2 (if the queue empties) is the
fix; F4c a stop-gap. F5 is unavailable under IDF 5/6.
