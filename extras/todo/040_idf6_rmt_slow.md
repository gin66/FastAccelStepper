# IDF 6 RMT runs sequence 02 about 30 s slow

Priority: **040** — high. The pin trace is longer, not a harness
artifact. Ahead of the 050 platform work.

Status: **root cause confirmed (H1)**. Encoded time is proven exact
(91.46 s); the pin trace shows the ~29 s is ~3250 stretched **low** phases
of ~9 ms, one per ~21 ms.

Governing principle: with the ramp generator running, `fill_queue()` keeps
the queue filled to `_forward_planning_in_ticks` (20 ms), so **the queue
must not run low while the motor is running**. It does. Hardware event
counters (`FAS_RMT_DEBUG_SLOW`) show `encode_commands()` finds
`read_idx == next_write_idx` repeatedly mid-move and takes the eager
`_rmtStopped` stop (`stopped ~= empty - 46`, through both ACC and RED). The
eager stop is the symptom; the contract violation is that the encoder can
drain the whole queue into the RMT buffer because the buffer holds less
time than the lookahead, and "empty" is then treated as "done" instead of
"waiting for the ramp". That is enough to call H1 confirmed; the exact
per-stop cost (~9 ms vs the model's ~0.7 ms) is no longer load-bearing and
stays as a secondary question (H3/H4).

The chosen fix is F2: cap the RMT symbol duration
(`RMT_MAX_SYMBOL_TICKS = RMT_BLOCK_TICKS/PART_SIZE` ticks, the I2S block
model) so the RMT buffer spans at most `RMT_BLOCK_COUNT*RMT_BLOCK_TICKS` =
16000 ticks (1 ms), far less than the 20 ms lookahead, and the encoder can
never drain the queue while running;
F1 is rejected, F5 unavailable under IDF 5/6. F2 rewrites the symbol
layout, so it must not regress the confirmed extra-step fix
(`extras/doc/implemented/esp32_rmt_extra_step.md`, H8 ping-pong replay)
and needs re-validation on hardware with the debug macro off. The
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
3200` this is far above `8*(PART_SIZE-1)+2` (250 for PART_SIZE 32, 186
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

Governing principle: while the ramp generator is running it keeps the queue
filled to `_forward_planning_in_ticks` (20 ms), so **the queue must never run
low mid-move**. The failure is that it does, and that `encode_commands()`
then treats "empty" as "done" (sets `_rmtStopped`) instead of waiting for the
ramp. It is invoked with `2*PART_SIZE` free at transaction start and
`PART_SIZE` free at each threshold, and `rmt_encode_queue()` drains commands
into the RMT until the RMT is full *or the queue is empty*. So the stop fires
precisely when **the whole queue fits in the RMT buffer**. The invariant that
prevents it is a bound on the encoder's read-ahead: the RMT buffer must span
less time than the lookahead.

This is a general driver-architecture contract, now documented on
`setForwardPlanningTimeInMs()` in `FastAccelStepper.h`: a driver must state
how much it drains out of the queue at most (in flight), and
`forward_planning_ticks` must exceed that. Per driver: AVR/SAM/SAMD/Teensy/
Pico consume at most one command at a time; ESP32 I2S drains up to the DMA
block being filled (`~I2S_BLOCK_TICKS`); ESP32 RMT (idf5/6) can drain the
whole `2*PART_SIZE`-symbol buffer, which is why the symbol cap below is the
driver's declared in-flight bound.

**Candidate condition 1 (symbols per command).** Every queue command must
contribute at least `PART_SIZE/2` RMT symbols. Then the full RMT buffer
(`2*PART_SIZE` symbols) holds at most **four** commands, and the ramp's
~20 ms lookahead (>=~5 commands) can never be drained into the RMT. This is
a candidate unit: the queue is filled and drained per command, and the bound
is deterministic (independent of ISR/task timing). It is *not* the invariant
the chosen design ends up using — see "time per symbol" below and the F2
design, which caps the symbol *duration* and so supersedes this count-based
framing. The count condition survives only as a secondary test.

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

**Candidate condition 2 (time per symbol) — the one that becomes the design.**
Instead of counting symbols, bound the *time* each symbol may represent. The
chosen form is the I2S-referenced block cap
`RMT_MAX_SYMBOL_TICKS = RMT_BLOCK_TICKS/PART_SIZE` (=250/333 ticks): the full
RMT buffer then covers at most `2*RMT_BLOCK_TICKS` = 16000 ticks = 1 ms, i.e.
far less than the 20 ms lookahead. (The earlier looser value `65536/PART_SIZE`
gave an 8.192 ms buffer and only a ~2.4x margin; the I2S shape is preferred.)
The tension noted here — bounding the buffer time *above* helps the
queue-drain (read-ahead) problem but *hurts* the starvation problem — is
resolved by the margin: at 1 ms in flight and 20 ms of lookahead the buffer
cannot drain the queue, and the starvation side is covered by the forward
planning floor. This time bound, not the symbol count, is the design's primary
invariant (see the F2 design and its I1/I2).

**Still open — more ideas wanted.** Neither condition is proven sufficient
end-to-end:
- candidate 1 (symbols per command) bounds the drain to four commands, but the
  ramp can still hold fewer than five commands in the very slow regime;
- the earlier sliding `PART_SIZE`-symbol window was downgraded to
  informational because it conflated fast steps with the deliberate 8-tick
  pause prelude (a threshold-early device, not a ramp bug);
- the read-ahead bound and the starvation bound pull in opposite directions
  on symbol duration, so a single monotone condition may not exist.
The next idea should probably state the requirement on the *ramp* in terms of
"time the RMT buffer can still play when the queue empties" versus "worst
case time until the task refills", rather than on symbol counts.

**Resolved by the F2 design below:** the I2S-referenced block model *is* that
idea. It states the requirement in time — at most
`RMT_BLOCK_COUNT*RMT_BLOCK_TICKS` (=16000 ticks) in flight, with each symbol
`<= RMT_BLOCK_TICKS/PART_SIZE` (=250/333 ticks) — which is deterministic and
does not depend on the ramp producing a minimum number of commands. The
symbol-count condition (candidate 1) is kept only as a secondary check. The
tension noted above is resolved by making the *time* bound the contract and
treating the starvation side as a consequence of the lookahead margin
(20 ms default vs 1 ms in flight).

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

*Judgement: confirmed (root cause).* The mechanism lives in our own code
(`encode_commands()`); it reproduces the cadence (~one hole per planning
window) and the idf4 vs idf5/6 split; the read-ahead condition (`test_30`)
fails exactly where the holes are; and the HW event counters show the
queue emptying at the callback mid-move on essentially every invocation
that finds it empty. Independently of the exact per-stop cost, the queue
running low while the ramp generator is active is itself the contract
violation. The per-stop magnitude is a secondary question (H3/H4) and no
longer gates the conclusion.

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
what P6 must measure, not assume. Of the four, only the
`RMT_ENCODING_MEM_FULL` / `is_done` change can plausibly alter how our
callback interacts with the driver (it decides whether EOF is marked in
this round or deferred to the threshold); the byte offset, `WITH_EOF` and
the DMA plumbing are inert for the simple encoder. H4 confidence is
therefore low and it is one concrete, testable difference, not four.

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
| F2 | read-ahead bound: cap symbol duration at `RMT_MAX_SYMBOL_TICKS = RMT_BLOCK_TICKS/PART_SIZE` t (I2S block model) | quantity (1) queue content | **chosen**; design below |
| F3 | bigger RMT buffer | quantity (2) buffer playback time | strong lever if (2) is the cause; needs F1/F4b |
| F4 | cheaper restart: (a) faster task, (b) unblock ramp while `_rmtStopped`, (c) restart from TX-done / queued transaction | restart latency | mitigation only |
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
- Con: variable-length symbols; a step or pause is split across many
  symbols/blocks, so every emit path and the tick-based direction drain must be
  updated; needs the split to preserve exactly one rising edge and the total
  ticks.
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

**Gating (do not skip).** F2 must not be implemented or merged until
(a) P1 confirms the queue-empty/eager-stop mechanism with the HW event
counters below, and (b) the F2 symbol-layout change re-passes the
extra-step hardware reproduction with `FAS_RMT_DEBUG_SLOW`/`FAS_RMT_DEBUG_COUNT`
off. A green `test_30` is not sufficient: F2 rewrites the exact symbol
layout that the H8 replay depends on, and a misdiagnosis of the 29 s would
re-expose the extra-step bug. Treat "P1 done" and "extra-step re-check
done" as hard milestones before any F2 merge; no calendar deadline is set
here because the gating is evidence-based, not time-based.

**Recommendation.** F2 is chosen (design below): cap every RMT symbol at
`RMT_MAX_SYMBOL_TICKS = RMT_BLOCK_TICKS/PART_SIZE` (250 for PART_SIZE 32,
333 for 24), which makes the RMT buffer span at most
`RMT_BLOCK_COUNT*RMT_BLOCK_TICKS` = 16000 ticks (1 ms), far less than the
20 ms lookahead, so the queue stays non-empty while running. F5 is
unavailable under IDF 5/6, so F2 is the only option that attacks the cause
directly; F3/F4 remain fallbacks/stop-gaps and F1/F6 are rejected.

P1 is still run first as a cheap confirmation and to validate the
`RMT_MAX_SYMBOL_TICKS` choice: at each pin idle record
`read_idx == next_write_idx` (queue empty?)
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

- **I1 (time cap, primary):** every RMT symbol covers at most
  `RMT_MAX_SYMBOL_TICKS` ticks, with
  `2*PART_SIZE*RMT_MAX_SYMBOL_TICKS < _forward_planning_in_ticks`. Then the
  full buffer spans less time than the lookahead. I1 does not depend on the
  ramp producing a minimum number of commands.
- **I2 (command floor, secondary):** every queue command contributes at least
  `PART_SIZE/2` RMT symbols, so the buffer holds at most four commands. This is
  the form `test_30` has checked so far. It is only a secondary/informational
  check after F2: I1 implies I2 for commands whose total ticks `T` satisfy
  `T/RMT_MAX_SYMBOL_TICKS >= PART_SIZE/2` (i.e. `T >= 4000` for P=32), but
  short fast commands (e.g. a single 640-tick step) yield only ~4 symbols and
  legitimately fail it. See the corrected note below.

`_forward_planning_in_ticks = TICKS_PER_S/50 = 320000` (20 ms). The two
bounds on `RMT_MAX_SYMBOL_TICKS`:

Reference is **I2S**, which frames its hardware buffer as fixed time blocks
and does not let a single symbol stretch the buffer:

```
I2S_BLOCK_TICKS = 125 * 64 = 8000 t (500 us);  I2S_BLOCK_COUNT = 2
I2S max in-flight = I2S_BLOCK_COUNT * I2S_BLOCK_TICKS = 16000 t = 1 ms
```

The RMT IDF5/6 driver adopts the same model. A "block" is one
`PART_SIZE`-symbol half and holds a fixed time, so a symbol is capped to that
block time divided by the symbol count:

```
RMT_BLOCK_COUNT        = 2
RMT_BLOCK_TICKS        = 8000            (500 us, match I2S)
RMT_MAX_INFLIGHT_TICKS = RMT_BLOCK_COUNT * RMT_BLOCK_TICKS = 16000 t = 1 ms
RMT_MAX_SYMBOL_TICKS   = RMT_BLOCK_TICKS / PART_SIZE = 250 (P=32) / 333 (P=24)
require: forward_planning_ticks > RMT_MAX_INFLIGHT_TICKS
# worst case incl. ovf: (2*PART_SIZE + min_chunk_size) * RMT_MAX_SYMBOL_TICKS
```

The 20 ms default then has ~20x margin (vs ~1.6x for the symbol-count bound).
Rejected alternative: capping at `ceil(65535/PART_SIZE)` (2048/2731) makes a whole
step/pause fit one half but lets the buffer span 8.19 ms (12.3 ms with the
overflow buffer); it is not the I2S-referenced shape.

I2 is **not** trivially met and must not be asserted as a hard invariant after
F2. Long commands easily exceed `PART_SIZE/2` symbols, but a fast single-step
command (e.g. 640 ticks) yields only `~640/RMT_MAX_SYMBOL_TICKS = 4` symbols,
below `PART_SIZE/2 = 16`. That is fine: the read-ahead is bounded by I1 (time),
not by the command count, and such a short command cannot fill the buffer on
its own. At `RMT_MAX_SYMBOL_TICKS = 250`, a command of `T` ticks contributes
`~T/250` symbols; the ramp still batches fast steps, so the queue is not
drained by symbol count but by the 1 ms in-flight time bound.

### Encoder: I2S-style fill state (chosen mechanism)

Mirror `i2s_fill_buffer_direct()` (`i2s_fill.cpp:72-159`): a per-queue state
machine that tracks the high and low time of the current step and walks the
queue, instead of "one step per symbol". This is what lets any legal command
(`steps<=255`, `ticks<=65535`, e.g. `steps=100, ticks=65535` = 409.6 ms) be
split so every RMT symbol stays `<= RMT_MAX_SYMBOL_TICKS`.

State (per `StepperQueue`, exactly like `i2s_fill_state`):

```c
struct rmt_fill_state {
  uint16_t remaining_low_ticks;   // low time left in the current period
  uint16_t remaining_high_ticks;  // high time (pulse) left in the current step
  uint8_t  off_ticks;             // sub-symbol remainder
};
```

Fill, per call with the given `symbols_free`:

1. While `remaining_high_ticks`/`remaining_low_ticks` are non-zero, emit RMT
   symbols that consume them (high run first, then low run), each symbol total
   `<= RMT_MAX_SYMBOL_TICKS` and each sub-entry in `[2, 32767]`; save state and
   return when `symbols_free` is used up.
2. When both counters hit zero: advance the current entry's `steps` or
   `read_idx` (dir toggle when an entry starts, as in `esp32_queue.h`); load the
   next entry and set the counters from `ticks` (pulse width + rest). A pause
   (`steps==0`) is just a low run of `ticks`.
3. Queue empty: save state and return; the existing idle/stop handling is
   unchanged.

Why this satisfies the contract and the long-command case:

- every symbol `<= RMT_MAX_SYMBOL_TICKS`, so the whole buffer (`2*PART_SIZE`
  symbols) spans `<= 2*PART_SIZE*RMT_MAX_SYMBOL_TICKS = RMT_MAX_INFLIGHT_TICKS`
  for *any* command;
- `steps=100, ticks=65535` becomes ~264 symbols per step across ~8 blocks, not
  one symbol; nothing has to exceed `RMT_MAX_SYMBOL_TICKS`;
- exact tick preservation falls out of the counters (`test_30` guards it).

This replaces `emit_step_symbols()`/`emit_pause_symbols()`'s whole-command
emission and the `per`/return-count plumbing; the return-count is subsumed
because the fill reports symbols as it writes them.

Constraints / follow-ups:

- every sub-entry in `[1,32767]`, practically `[2,32767]` (relation 1); merge a
  1..3-tick tail into the previous symbol rather than emitting a 0/1-tick half;
- the dir drain becomes tick-based (sized to the ovf-inclusive in-flight
  bound, `2*RMT_BLOCK_TICKS` when `min_chunk_size` is small; see
  anti-regression constraint 2), so `esp32_before_pause_count()`/`_ticks()` on
  the RMT IDF5/6 path change;
- the symbol layout changes at all speeds (`steps=100` and even a 40 us step
  split), which is the H8/extra-step surface, so HW re-validation is mandatory.

### Changes (files)

- `pd_esp32/esp32_queue.h`: add `struct rmt_fill_state` (in the RMT union with
  `_tx_encoder`/`channel`), mirroring `i2s_fill_state`.
- `pd_esp32/StepperISR_idf5_esp32_rmt_encode.cpp`: replace
  `emit_step_symbols()`/`emit_pause_symbols()`/`rmt_encode_queue()` with the
  I2S-style fill (`rmt_encode_fill(q, state, symbols, symbols_free)`; distinct
  name from the IDF4 `rmt_fill_buffer`), keeping the dir toggle and
  `read_idx`/`steps` bookkeeping like `i2s_fill.cpp`.
- `pd_esp32/StepperISR_idf5_esp32_rmt.cpp` `encode_commands()`: call the fill
  with the persistent `rmt_fill_state`; drop the `symbols_free < PART_SIZE`
  whole-command gate; treat the queue as empty only when the fill state is also
  drained (see the open-questions note); resolve `min_chunk_size` per
  anti-regression constraint 3 (keep `PART_SIZE`, or make the stop pause
  resumable, and size the DIR drain accordingly).
- `pd_esp32/esp32_queue.h`: `esp32_before_pause_count()`/`_ticks()` for the RMT
  IDF5/6 path go tick-based (value = ovf-inclusive in-flight bound; count 0),
  matching I2S, and this branch must be `#if defined(SUPPORT_ESP32_RMT_V2)` guarded so the IDF4
  RMT path keeps its count-based drain (`count 1`, `MIN_CMD_TICKS`).
- `pd_esp32/StepperISR_idf5_esp32_rmt.cpp` `startQueue_rmt()`/`forceStop_rmt()`:
  reset `rmt_fill_state` at every transaction boundary (mirror the
  `i2s_fill_state` lifecycle).
- `pd_config_idf5.h`/`pd_config_idf6.h`: define `RMT_BLOCK_TICKS`,
  `RMT_MAX_INFLIGHT_TICKS`, `RMT_MAX_SYMBOL_TICKS`.

This is a full encoder rewrite (partial state, tick-based drain) and it
invalidates the current H8/extra-step validation, so the hardware re-check is
mandatory before F2 can be considered done.

TODO(040): once F2 lands, replace the placeholder ESP32 RMT IDF5/6 in-flight
line in the `FastAccelStepper.h` driver-contract comment (and in
`extras/doc/driver_architecture.md`) with the measured bound
(`RMT_BLOCK_COUNT*RMT_BLOCK_TICKS = 16000 t = 1 ms`). RMT IDF4 and MCPWM/PCNT
are already stated there; IDF5/6 is the only one still open.

### Drain (in-flight) contract (I2S-referenced)

Declare, per platform, the maximum the RMT IDF5/6 driver can remove from the
queue before the pin outputs it. Model it on I2S's fixed blocks:

```
RMT_BLOCK_COUNT        = 2
RMT_BLOCK_TICKS        = 8000                  (500 us, matches I2S_BLOCK_TICKS)
RMT_MAX_INFLIGHT_TICKS = RMT_BLOCK_COUNT * RMT_BLOCK_TICKS = 16000 t = 1 ms
RMT_MAX_SYMBOL_TICKS   = RMT_BLOCK_TICKS / PART_SIZE = 250 (P=32) / 333 (P=24)
# worst case incl. ovf: (2*PART_SIZE + min_chunk_size)*RMT_MAX_SYMBOL_TICKS
```

Contract: `forward_planning_ticks > RMT_MAX_INFLIGHT_TICKS` (20 ms default ->
~20x margin). The IDF overflow buffer is a *transient* staging area for the
symbols that did not fit the RMT memory; **if `min_chunk_size` stays at
`PART_SIZE`, it can hold up to `PART_SIZE` more symbols that were already
drained from the queue**, so the true bound is
`(2*PART_SIZE + min_chunk_size)*RMT_MAX_SYMBOL_TICKS` (~1.5 ms for P=32) —
still far below the lookahead, but not exactly
`RMT_BLOCK_COUNT*RMT_BLOCK_TICKS`. Set `min_chunk_size` to 1-2 to make the
declared bound exact (the resumable fill always makes progress with one free
symbol). The symbol-count framing had to add a `3*` factor for the same
buffer; making the bound exact here is the cleaner choice.

Consequences that come with this reference:

- a block holds only 500 us, so a legal command no longer fits one half. A max
  pause (65535 t) is ~263 symbols (P=32) and spans ~8 blocks. The encoder must
  carry partial-command state across calls (like `i2s_fill_state` with
  `remaining_low_ticks` / `remaining_high_ticks`); the per-entry `steps`
  write-back is replaced by a tick-based remainder (a command is still consumed
  whole-command, but may be consumed before its tail ticks have played).
- the direction drain becomes tick-based like I2S (sized to the ovf-inclusive
  in-flight bound), not "two `PART_SIZE` pauses". This changes
  `esp32_before_pause_*()` on the RMT path.
- even a 40 us step (640 t) splits into ~4 symbols (`ceil(320/250)+ceil(320/250)`),
  so the symbol layout changes at all speeds. This is exactly the layout the
  extra-step H8 half-replay depends on, so HW re-validation is mandatory.

Enforce, don't just document: expose `RMT_MAX_INFLIGHT_TICKS` in `pd_config.h`
and have `setForwardPlanningTimeInMs()` clamp/assert against it, so a too-small
forward planning fails loudly instead of draining the queue. This is the RMT
IDF5/6 row of the driver in-flight contract in `driver_architecture.md`.

### Test plan (`test_30`)

- assert I1: every emitted symbol total `<= RMT_MAX_SYMBOL_TICKS`
  (= `RMT_BLOCK_TICKS/PART_SIZE`) (new `any_symbol_above`);
- keep I2 (`min symbols/command >= PART_SIZE/2`) only as a secondary,
  informational check. With `RMT_MAX_SYMBOL_TICKS` a command of `T` ticks
  yields ~`T/RMT_MAX_SYMBOL_TICKS` symbols, so I2 holds for commands with
  `T >= PART_SIZE/2 * RMT_MAX_SYMBOL_TICKS` (= 4000 t for P=32, 3996 t for
  P=24) and is expected to fail for shorter ones; I1 is the invariant, I2 is
  not asserted as a hard requirement after F2;
- keep the exactness checks: total symbol ticks == commanded, one edge per
  step, no zero/one-tick sub-entry, no write past the reported count;
- expected readings for seq_02 after the change: largest step 65535 ->
  `ceil(65535/250) = 263` symbols (P=32); a 4 ms step (~64000 t) -> 256
  symbols; every symbol `<= 250`;
- the `test_seq_02_holes` model should then never report a stop (queue never
  empties), which is the end-to-end proof for the mechanism;
- **hardware re-validation for the extra-step fix** (cannot be done on PC):
  `seq_02`/`check_pcnt_sync` and the `seq_15` sweep with
  `FAS_RMT_DEBUG_COUNT` off, per `esp32_rmt_extra_step.md`. A green PC run is
  not evidence that H8 did not return.

### TDD to-do list (full encoder rewrite)

The rewrite is specified by `test_30`. Work test-first: write each test, watch
it fail (RED), make the minimal encoder change to pass (GREEN), keep the
seq_02 oracle green, then refactor. The encoder is a pure function of
`(queue, rmt_fill_state, symbols_free)`, so everything except the H8 ISR/timing
race is PC-testable. Run: `make -C extras/tests/pc_based test_30 && ./test_30`
(and `make test` before finishing). Because `PART_SIZE = debug_part_size` is a
runtime variable in the PC host build, both `RMT_MAX_SYMBOL_TICKS` and any
array sizing must be runtime expressions (`RMT_BLOCK_TICKS / PART_SIZE`), not
integer-constant expressions.

**Phase 0 — interface/config freeze (new tests compile, all RED)**
- [x] 0.1 Define `RMT_BLOCK_COUNT 2`, `RMT_BLOCK_TICKS 8000`,
  `RMT_MAX_INFLIGHT_TICKS (RMT_BLOCK_COUNT*RMT_BLOCK_TICKS)` (=16000) and
  `RMT_MAX_SYMBOL_TICKS (RMT_BLOCK_TICKS/PART_SIZE)` in `pd_config_idf5.h` /
  `pd_config_idf6.h` (and a `pd_test` fallback so test_30 sees them).
- [x] 0.2 Add `struct rmt_fill_state { uint16_t remaining_low_ticks;
  uint16_t remaining_high_ticks; uint8_t off_ticks; }` to the RMT union in
  `esp32_queue.h`; declare `uint32_t rmt_encode_fill(StepperQueue*, struct
  rmt_fill_state*, uint32_t* symbols, uint32_t symbols_free);` and stub it to
  `return 0`.
- [x] 0.3 Retire the tests bound to the old whole-command model (they are the
  RED baseline): `test_pause_fills_one_half`,
  `test_pause_needs_a_full_half`, `test_short_step_is_one_symbol`,
  `test_max_tick_step_is_two_symbols`, `test_step_needs_two_free_symbols`,
  `test_max_tick_steps_pack_by_two`, `test_two_symbol_cases_report_their_count`.
  Keep `test_pause_then_steps_share_a_call` and
  `test_toggle_waits_for_the_next_call` only after rewriting them for tick
  state, and `test_remaining_steps_written_back` after re-expressing it as a
  tick remainder.

**Phase 1 — I1, the symbol-duration cap (primary invariant)**
- [ ] 1.1 `test_symbol_cap_single_step`: `push(1, 65535)`; fill with a large
  free count; assert every symbol total `<= RMT_MAX_SYMBOL_TICKS`, sum of
  symbol ticks `== 65535`, exactly one rising edge.
- [ ] 1.2 `test_symbol_cap_pause`: `push(0, 65535)`; assert all-low, every
  symbol `<= cap`, sum `== 65535`.
- [ ] 1.3 `test_symbol_cap_sweep`: for `steps in {0,1,2,255}` and `ticks in
  {2,3,4,5,7,8,99,250,251,320,639,640,3200,32767,65535}` assert cap, exact tick
  sum and edge count.
- [ ] 1.4 `test_cap_relation`: assert
  `2*PART_SIZE*RMT_MAX_SYMBOL_TICKS < _forward_planning_in_ticks` with the
  default forward planning (`TICKS_PER_S/50`).
- [ ] 1.5 `test_min_chunk_progress`: with `symbols_free == min_chunk_size`
  (the chosen value, from anti-regression constraint 3) and a non-empty queue
  the fill returns `> 0`; with an empty queue and a drained state it must reach
  the stop rather than return 0 (the IDF overflow-buffer contract,
  `rmt_encoder_simple.c:123-130`).

**Phase 2 — exactness, edges, sub-entry floor**
- [ ] 2.1 `test_long_step_exact`: `steps=100, ticks=65535`; sum `== 100*65535`,
  rising edges `== 100`.
- [ ] 2.2 `test_fast_step_split`: `steps=1, ticks=640`; sum `== 640`, one edge,
  `~4` symbols, every sub-entry `>= 2`.
- [ ] 2.3 `test_no_short_sub_entry`: for `ticks in {2,3,5}` and odd values,
  and for run lengths `k*RMT_MAX_SYMBOL_TICKS + {1,2,3}` with `symbols_free`
  cutoffs at 1, 2 and exactly-full, assert no sub-entry is 0 or 1 and the sum
  stays exact (the relation-1 floor, including across calls).
- [ ] 2.4 `test_level_run_shape`: decode the emitted sequence and assert each
  step is exactly one contiguous HIGH run followed by one contiguous LOW run
  (no level flip inside a pulse), while a split may span several symbols.

**Phase 3 — partial state and resumability**
- [ ] 3.1 `test_split_across_calls_equals_one_call`: fill the same long step
  once with a large buffer and once with `symbols_free == 1` per call; assert
  the concatenated symbol streams are identical.
- [ ] 3.2 `test_state_pending_after_queue_empty`: after the last command is
  loaded but its ticks are still being emitted, `read_idx == next_write_idx`
  while `remaining_high_ticks || remaining_low_ticks`; the encoder must not be
  treated as "done". This is also the Phase 5 stop-predicate.
- [ ] 3.3 `test_read_idx_advance_once`: a command is consumed exactly once
  (no duplicated or skipped entry) as the tick remainder crosses zero; assert
  `read_idx` after each call.

**Phase 4 — direction handling**
- [ ] 4.1 `test_dir_toggle_once_at_entry_start`: a `toggle_dir` entry toggles
  exactly once, before its first symbol and not during the preceding pause
  (rewrite of `test_toggle_waits_for_the_next_call`).
- [ ] 4.2 `test_tick_based_drain`: `esp32_before_pause_count() == 0` and
  `esp32_before_pause_ticks() >=` the ovf-inclusive in-flight bound (= 
  `2*RMT_BLOCK_TICKS` only when `min_chunk_size` is small) for the IDF5/6 RMT
  path, while the IDF4 path keeps count 1 / `MIN_CMD_TICKS` (guard compile
  check).
- [ ] 4.3 `test_dir_delay_pause_preserved`: the user dir-change/delay pause is
  still honored after the variable-length drain.

**Phase 5 — empty-queue stop path (the bug)**
- [ ] 5.1 `test_empty_queue_emits_min_pause`: `read_idx == next_write_idx` and
  drained state makes `encode_commands()` set `_rmtStopped` and write
  `PART_SIZE` `ENTER_PAUSE(MIN_CMD_TICKS)` symbols, each `<= cap`.
- [ ] 5.2 `test_empty_queue_with_pending_state_no_stop`: empty queue but
  non-zero fill state must **not** set `_rmtStopped` and must keep encoding.
  This is the new correctness test; it fails against today's code and is the
  unit-level statement of the contract violation.
- [ ] 5.3 `test_stop_only_after_drained_end`: the stop may fire only once on
  true end (queue empty, state drained, ramp no longer producing).

**Phase 6 — end-to-end seq_02 (oracle / no-regression)**
- [ ] 6.1 Update `test_seq_02_total_ticks`: total symbols `== 1463342840`,
  edges `== 65988`, `zero_durations == 0`, `overrun == 0`,
  `global_min >= 2`, and new `global_max_symbol <= RMT_MAX_SYMBOL_TICKS`.
  Drop the hard `min symbols/command >= PART_SIZE/2` assertion; print it as
  informational only.
- [ ] 6.2 Rewrite/port `test_seq_02_holes` onto the new fill (the current model
  calls `rmt_encode_queue`): with the capped buffer the model must report
  `stops == 0` and `injected == 0` (queue never empties), while the encoded
  total stays 91.459 s. This is the end-to-end proof of the mechanism.
- [ ] 6.3 `test_extra_step_edge_count` (H8 proxy, limited): for every step
  period and `PART_SIZE`, filling with each `symbols_free` from 1..`2*PART_SIZE`
  and concatenating must give exactly one rising edge per step, and `overrun ==
  0` for every call. This verifies only that *our* fill emits the right edges;
  H8 lives below the callback in the IDF ping-pong, so this cannot reproduce it
  and does not replace the HW re-validation (Phase 7.1).
- [ ] 6.4 `test_buffer_time_bound`: over seq_02, max time spanned by any
  `2*PART_SIZE` consecutive symbols `<= RMT_MAX_INFLIGHT_TICKS`, and by any
  `(2*PART_SIZE + min_chunk_size)` consecutive symbols `<=
  RMT_MAX_INFLIGHT_TICKS + min_chunk_size*RMT_MAX_SYMBOL_TICKS` (replaces the
  informational `min_window_ticks`).

**Phase 7 — non-PC gates (block merge, not part of test_30)**
- [ ] 7.1 Hardware re-validation with `FAS_RMT_DEBUG_COUNT` off: `seq_02` /
  `check_pcnt_sync` and the `seq_15` 50...743 sweep, per
  `esp32_rmt_extra_step.md`; both `PART_SIZE` if available.
- [ ] 7.2 `setForwardPlanningTimeInMs()` clamps/asserts against
  `RMT_MAX_INFLIGHT_TICKS` (enforcement, not just documentation), with a PC
  unit test for the clamp.
- [ ] 7.3 Replace the placeholder IDF5/6 in-flight line in
  `FastAccelStepper.h` and `driver_architecture.md` with the measured bound
  (`RMT_BLOCK_COUNT*RMT_BLOCK_TICKS = 16000 t`).

Ordering note: Phases 1-3 are the core RED→GREEN loop and can be built with a
simple "emit into a flat symbol array" harness before touching IDF callbacks;
Phase 5 depends on 3.2; Phase 6 is the regression gate that must stay green
after every step; Phase 7 blocks the merge.

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
   exactly the bytes the callback was given. A command is consumed
   whole-command, never a fraction of `steps`: `read_idx` advances only after
   all the entry's steps have been loaded into the fill state. This is weaker
   than the old "advance when the ticks have played" rule — like
   `i2s_fill.cpp`, the entry may be consumed while its tail ticks are still in
   `rmt_fill_state` (see the pending-state note below).
2. **Convert the direction drain to tick-based — and size it from the real
   in-flight bound.** `encode_commands()` stops by emitting
   `ENTER_PAUSE(MIN_CMD_TICKS)`, and on the *current* RMT IDF5/6 path the
   direction-change drain needs two `PART_SIZE` pauses
   (`esp32_before_pause_count() = 2`, `esp32_queue.h`). `esp32_rmt_extra_step.md`
   could size the drain in whole pauses because a pause filled exactly one RMT
   half. F2 drops the one-pause-per-half property, so
   `esp32_before_pause_count()` / `esp32_before_pause_ticks()` must become
   tick-based like I2S. **The tick count must cover the full worst-case
   in-flight, `(2*PART_SIZE + min_chunk_size)*RMT_MAX_SYMBOL_TICKS` (see
   constraint 3), not merely `2*RMT_BLOCK_TICKS`.** With a resumable stop pause
   and a 1-symbol `min_chunk_size` the add-on is one symbol (negligible);
   otherwise size the drain from the ovf-inclusive bound. If the overflow buffer
   is allowed to hold `PART_SIZE` extra symbols, `2*RMT_BLOCK_TICKS` is short by
   `PART_SIZE*RMT_MAX_SYMBOL_TICKS` and a step can cross a DIR change. Do not
   silently keep the count-based drain against variable-length pauses - that
   would let a step cross a DIR change.
3. **Keep the H9 hole closed, and reconcile `min_chunk_size` with the stop.**
   The IDF simple encoder retries with an overflow buffer of `min_chunk_size`
   symbols when the callback returns 0 with the RMT buffer partly free; if the
   callback then returns 0 *again* it aborts the transaction
   (`rmt_encoder_simple.c:123-130`). So the requirement is: given
   `min_chunk_size` free symbols, the callback must write `> 0` (or set
   `done`). Because the new fill carries partial state, a whole command no
   longer has to fit in one call, so the old rationale
   (`ceil(65535/PART_SIZE)` symbols "so the largest unit always fits") is void.
   But `min_chunk_size` cannot be shrunk blindly: the empty-queue stop emits
   `ENTER_PAUSE(MIN_CMD_TICKS)` as one atomic `PART_SIZE`-symbol run, so if the
   queue is empty and only `min_chunk_size < PART_SIZE` symbols are free, the
   callback cannot both stop and make progress. Resolve it one of two ways:
   (a) keep `min_chunk_size = PART_SIZE` (the stop pause always fits), accept
   that the overflow buffer can add `PART_SIZE` symbols to the in-flight bound,
   and size the DIR drain from constraint 2 accordingly; or (b) feed the stop
   pause through `rmt_fill_state` as well (resumable), allow a small
   `min_chunk_size` (1-2) and keep the exact `2*RMT_BLOCK_TICKS` bound. Note
   that with the new fill the `return 0`/ovf path should almost never run in
   steady state, so the choice is about the stop/edge case, not throughput.
   Never return 0 in a way that leaves the wrap to replay the last half; keep
   the empty-queue stop reachable (checked before any short-buffer return).
4. **The emitter's symbol count is load-bearing.** The driver's
   `mem_off`/`mem_end` phase must match the symbols actually written; the
   fill must report exactly what it wrote, so the return-count bug fixed above
   is required by F2, not optional.
5. **Re-check the half-replay exposure.** A saturated slow step's high is now
   `ceil(32767/RMT_MAX_SYMBOL_TICKS) ~= 132` symbols (P=32). The more symbols
   a pulse spans, the more ways a half replay can reproduce part of it; this is
   the central risk of adopting the I2S block model. F2 does remove the exact
   object H8 replayed: there is no `0x4000FFFF` word (high `0x7fff`) any more,
   because no symbol exceeds `RMT_MAX_SYMBOL_TICKS`, so the observed H8
   signature (one extra ~2047.9 us high) can no longer come from replaying a
   single symbol. What remains is a replay of a *partial run* of the split
   pulse (potentially one extra edge of <=15.6 us), which is why the HW
   re-validation is mandatory and cannot be replaced by the PC edge-count test.
6. **Re-validate on hardware with the debug macro off.** `test_30` (PC) cannot
   see the timing race. Run `seq_02` / `check_pcnt_sync` and the `seq_15`
   50...743 sweep with `FAS_RMT_DEBUG_COUNT` **off** (the doc shows the macro
   shifts the timing and makes the fault near-certain, so it must be off for
   the confirmation), for both PART_SIZE 32 and, if available, 24.

F2 does not touch IDF4's path, so the IDF4 control stays valid.

**Crosscheck vs `esp32_rmt_extra_step.md` (deltas F2 must not break):**

- The extra-step fix's translator contract is "step encoded as short as
  possible, `steps` written back, `read_idx` only when `steps == 0`, toggle on
  entry start, empty-queue stop before the short-buffer return". F2 keeps the
  last three; it replaces whole-step packing with tick state, so "short as
  possible" becomes "each symbol `<= RMT_MAX_SYMBOL_TICKS`". The entry is still
  consumed whole (all `steps` loaded before `read_idx` advances), matching
  `i2s_fill.cpp`.
- The extra-step remedy required a pause to be exactly `PART_SIZE` symbols and
  to start only when `symbols_free >= PART_SIZE`, which is what made the two
  drain pauses and the min_chunk retry safe. F2 intentionally abandons this
  (pauses split into many symbols); the replacements are the tick-based drain
  sized from the real in-flight bound and a reconciled `min_chunk_size` (above).
  This is the main behavioural delta and the reason the old HW validation is
  invalid.
- H9's control-flow hole ("`symbols_free < PART_SIZE` returns 0 without
  stopping") is described in the extra-step doc as closed only by the driver's
  same-call overflow retry. F2 must make the stop reachable with whatever
  `min_chunk_size` is chosen, not rely on the retry.
- H8 was empirically fixed but its exact replay path stayed open; F2 changes the
  replayable object from a whole `0x4000FFFF` command to a partial split pulse.
  The `test_30` edge-count check is only a proxy — it lives entirely above the
  IDF driver and cannot reproduce the ping-pong race.

### Open questions / risks

- Threshold/prelude behaviour: under F2 a pause is split into many small
  symbols and no longer keeps the `PART_SIZE` count or the 8-tick prelude, so
  the threshold fires uniformly inside a pause instead of immediately. I1 makes
  the queue non-empty so refill timing is less critical, but verify on target
  (P1).
- Half-replay exposure (the H8 regression risk above): with a split step the
  pulse spans several symbols; confirm a half cannot become a self-contained
  step pulse, or that the replay path stays unreachable.
- ISR load: more symbols per unit time means the encoder callback and the
  threshold ISR run more often; at the slow end a 65535-tick step is ~264
  symbols instead of 1-2, so the symbol rate rises - acceptable at slow step
  rates, needs a sanity check at the fast end (a 40 us step is ~4 symbols vs 1).
- Command consumption is whole-command, not whole-period: `read_idx` advances
  only after all of an entry's `steps` are loaded, but it may advance while the
  entry's tail ticks are still pending in `rmt_fill_state`. Confirm with the
  existing `test_remaining_steps_written_back`-style cases extended to long
  steps (a 65535-tick step now spans ~264 symbols).
- **Sub-entry floor across call boundaries.** The relation-1 floor is 2 ticks,
  so a run remainder/tail of 0 or 1 tick must be merged into the neighbouring
  sub-entry. The fill must guarantee that for *any* `symbols_free` cutoff,
  i.e. it cannot simply split a run at every `RMT_MAX_SYMBOL_TICKS` boundary
  and leave a 1-tick tail for the next call. Either carry the remainder in
  `rmt_fill_state` (like `i2s_fill_state.off_ticks`) or hold one symbol in
  reserve so the tail can be merged before the symbol is submitted. Otherwise
  the driver writes a 0/1-tick half, which is the stop pattern / stretched
  symbol (H2/H9). Test with `symbols_free == 1` and with run lengths of
  `k*RMT_MAX_SYMBOL_TICKS + {1,2,3}`.
- **Pending fill state vs. the empty-queue check (correctness).** Unlike the
  current whole-command emitter, the new fill advances `read_idx` as soon as a
  command is *loaded* and then emits its ticks over subsequent calls, so
  `read_idx == next_write_idx` can be true while `rmt_fill_state` still holds
  `remaining_high_ticks`/`remaining_low_ticks`. `encode_commands()` must
  therefore treat the queue as empty only when the queue is empty **and** the
  fill state is drained (both counters zero). Otherwise the eager
  `_rmtStopped` stop fires even earlier (mid-split-symbol), making the bug
  worse. This check is also the `done` condition.
- **Reset the fill state at transaction boundaries.** `rmt_fill_state` must be
  zeroed in `startQueue_rmt()` (new transaction) and in `forceStop_rmt()`
  (aborted transaction); otherwise leftovers from a previous transaction leak
  into the next. Mirror `i2s_fill_state`'s lifecycle.
- **Guard the tick-based direction drain to IDF 5/6 only.** The change touches
  the shared `esp32_queue.h`. `esp32_before_pause_count()`/`_ticks()` are also
  used by the IDF4 RMT path (count 1, `MIN_CMD_TICKS`), which must keep its
  count-based drain. Guard the tick-based branch with `SUPPORT_ESP32_RMT_V2`
  so "F2 does not touch IDF4" holds.
- **Overflow-buffer accounting.** If `min_chunk_size` stays at `PART_SIZE`, the
  driver's overflow buffer can hold an extra `PART_SIZE` symbols beyond the
  RMT memory, so the true in-flight bound is
  `(2*PART_SIZE + min_chunk_size) * RMT_MAX_SYMBOL_TICKS` (~1.5 ms for
  P=32), still far below the 20 ms lookahead but no longer exactly
  `RMT_BLOCK_COUNT*RMT_BLOCK_TICKS`. Setting `min_chunk_size` to 1-2 makes the
  declared bound exact and is safe once the fill is resumable. Decide and
  document which one the contract uses.
- Choose `RMT_MAX_SYMBOL_TICKS` as a `pd_config.h` constant (per platform) as
  `RMT_BLOCK_TICKS/PART_SIZE` (250 for P=32, 333 for P=24), *not*
  `65535/PART_SIZE`.

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

### HW event counters (implemented; the cheapest form of P1)

`FAS_RMT_DEBUG_SLOW` in `pd_esp32/pd_config.h` is on. It counts, inside
`encode_commands()` (`StepperISR_idf5_esp32_rmt.cpp`):

- `empty=`: callbacks that found `read_idx == next_write_idx` (queue empty);
- `stopped=`: callbacks that set `_rmtStopped` (the eager stop).

They are read by `fas_rmt_debug_empty_events()` / `..._stopped_events()` and
printed inline in StepperDemo's `info()`:

```
M1: @53 => 230 QueueEnd=53 v=3074us/49184ticks ACC empty=<e> stopped=<s>
```

Read it as:
- `empty` climbing while `@`/`QueueEnd` is still moving mid-move => the queue
  really is drained mid-move (H1 confirmed, quantity (1)); F2 is the fix.
- `stopped` climbing in step with the observed holes => the eager stop fires;
  compare the hole count to the `stopped` delta over the same move.
- both flat while holes occur => H1 is not the trigger; the pin idles for
  another reason (H3/H4, or quantity (2) -> F3).
Reset with `fas_rmt_debug_reset()` at the start of a move (or reboot). The
debug build must be **off** for the final extra-step validation, per
`esp32_rmt_extra_step.md` (the added work shifts the timing).

**First HW result (early, seq_02):** `empty` and `stopped` rise together
(`stopped ~= empty - 45...47`) through both `ACC` and `RED`, i.e. almost
every callback that finds an empty queue immediately takes the eager stop,
mid-move, at the right order of magnitude (a few per status line vs ~47
holes/s in the capture). This confirms H1's *trigger* (queue drains ->
eager stop) but not the *cost*: a stop is not yet shown to be a 9 ms pin
hole, which needs the Saleae correlation (one hole per `stopped` delta).
The small, slowly growing `empty - stopped` offset is the
`symbols_free < PART_SIZE` early return (H9 path) and should be watched.

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
