# 023 i2s_mux in `dir`: 51 phantom steps per capture, from the 24 MS/s sampling race

## Priority

**MEDIUM** — a harness-side measurement defect, not a library one, and the only
thing left wrong with `sync --imux --pin-mode dir`. It reports a driver emitting
steps it did not emit, which is the failure mode this harness exists to catch, so
it has to stop being able to do that.

It is also what remains of the item this one is numbered after. "`i2s_mux` in
`dir` loses the second stepper's slot" was filed HIGH as a library defect on the
strength of a matrix run recording `incomplete_capture: missing_channels:
["S2"]`. It was not a library defect and is closed — the finding itself is not
kept, because a host-side measurement bug with no change in `src/` is the kind
of record the todo directory is the wrong place for. The measurement is in
`extras/tests/saleae_based/AGENTS.md` (§ *The mux in `dir` mode*) and in the
backlog's Done section. In short: measured off the bus wires with no slot map in
the loop, two distinct wire positions each carry 64 bits — 24.931 µs and
49.925 µs mean, i.e. 400 and 800 ticks — interleaved 2:1, which is two steppers
on a shared start at twice the rate. What the run had actually measured was our
own decoder config naming one stepper instead of two.

## Finding

`--mode sync --arch esp32 --framework idf --version 6.13.0 --pin-mode dir
--sync-count 2 --imux`, IDF 5.5.3, **both** steppers on `i2s_mux`. Stepper A
passes, stepper B does not:

```
A ch=S0 ticks=400  steps 64/64 ok  mean_period 24.9306 us  grid 24/28 us
B ch=S2 ticks=800  steps_expected 64  steps_measured 115  extra_steps 51
   period: legal_periods_us [48.0, 52.0]  periods_measured 114
           n_off_grid 51  -- 8177.083 us, and one 29427.083 us
```

8177.083 µs is 2046 frame periods, which is the tell: these are not steps, they
are a periodic event in the decode. IDF 6.1.0 gives the same shape and the same
counts to within a frame or two (`0x1a` 48 against 50, `0x08` 52 against 50).

## Three reasons this cannot be the firmware

Read off the bus without a slot map — group the bclk rising edges into 32s,
read the data at each edge, count which wire positions are high. The capture
holds 125 255 frames of which 201 are corrupt, in **quartets of four** 13 frames
(52 µs) apart, one quartet every 2046 frames, spread evenly over the whole
501 ms capture. Positions counted from the first edge of the group:

| frame offset in the quartet | positions high beyond the two held-high direction bits | what it looks like |
|---|---|---|
| +0 | 15, 13, **11** | a third bit that nothing allocated |
| +13 | 15 | one direction bit dropped for a frame |
| +26 | 15, **13** | **the phantom stepper-B step** |
| +39 | 13 | the other direction bit dropped for a frame |

1. **Slot 4 is unreachable.** Wire position 11 is slot 4 under the wire order
   (below), and no stepper owns it. `_mux_state` is written in exactly two
   places. `init_mux_buffer()` copies it verbatim, and `i2sMuxSetBit()` is the
   only other writer — called from `esp32_set_direction_pin_state()` and
   `esp32_set_enable_pin_state()` with a *connected stepper's* pin, which here
   are the two direction bits. Step bits are set by `i2s_fill_buffer_mux()`
   OR-ing `q->_i2s_mux_step_bit_mask` at `q->_i2s_mux_step_byte_offset`, i.e.
   one of the two step bits. **No path in `src/` can set that bit.**
2. **They start before the move.** The first quartet is at frame 351 and the
   move does not start until frame 71413 — 285 ms of them precede `QRUN`, where
   stepper B cannot have stepped at all.
3. **The same wire decodes differently under a different sampling phase.**
   Re-running the identical grouping over the identical capture, anchored the
   same way, but reading the data one sample *before* the bclk rising edge:

   | decoded word | at the rising edge | one sample earlier |
   |---|---|---|
   | `0x1a` (slot 4 — impossible) | 50 | **0** |
   | `0x02` | 50 | 2 |
   | `0x08` | 50 | 2 |
   | `0x0e` | 83 | 33 |
   | `0x0b` | 32 | 34 |

   The impossible word goes away entirely and the move still decodes to 64 steps
   per stepper, to within a frame or two either way. An emitted pulse is on the
   wire and does not move when you change where you look; a sampling race does.

   This is **not** a decision, and it is not the point `extract_frames()` already
   rejects: that comment rules out the middle of the bclk high time and the
   cell's *last* sample, which at 3 samples per cell is `edge + 2`. What is new
   here is that `edge − 1` is materially better than `edge` on this capture and
   `edge + 1` is worse than useless (19 distinct words in 125 255 frames). One
   capture is not a basis for changing a sample point that four other tests
   depend on.

## The wire order, and why the slot numbers are not the weak part

Only the slot *labels* need the decoder's half-swap — the counts and the
periods above do not, because a wire position is a wire position. That mapping is
checked rather than assumed: position `p` maps to slot `15 - p` for `p < 16`, so
the two positions held high from t=0 land on slots 3 and 1 and the two step
positions on slots 0 and 2 — exactly MAP's `dslots=1,3` and `slots=0,2`, in the
right roles. A wrong swap would put the two always-high bits somewhere other
than the two direction slots MAP reported.

## Why 24 MS/s races

24 MS/s over an 8 MHz bclk is **exactly 3 samples per bit period**, so there is
no margin at all. In the affected frames the bclk rise spacing is 2 and 4
samples where it is 3 everywhere else (14 884 and 4 719 occurrences over the
capture), and the sampling instant lands on the wrong side of the data
transition: at each spurious word the data cell is already high one cell before
the bclk edge that samples it.

There is no rate on this analyzer that fixes it. 48 MS/s gives 6 samples per bit
period, and 48 MS/s truncates an 8-channel capture to 0.18 ms — shorter than any
scenario. 24 MS/s is the documented floor precisely because it is the floor.

**`nodir` does not show it at all**, and that is the confirmation: in `nodir`
the data line is idle-low, so there are almost no transitions to race with. The
same 24 MS/s, the same ~100 frame-alignment faults the decoder reports, and 2
distinct words in 125 255 frames on every `--mode scale --driver i2s_mux` point.
`dir` is the mode where the DIRECTION bits are held high, so the data line
toggles in every single frame — which is why only `dir` is affected.

## Where the decoder stands, and what is left to decide

`extract_frames()` already computes the frame-alignment diagnostics
(`report_faults=True`: 100 of them here) and **nothing records them** —
`decode()` discards them, so the run cannot say "this capture decoded 201
impossible words" and reports a driver defect instead. Three options, none
chosen here:

- **Count steps only inside the commanded run window.** The plan fixes it
  exactly: a scenario's duration is `steps × ticks / TICKS_PER_S` with no ramp
  in the way, so every evaluator knows where the move is and 285 ms of pre-move
  idle is not evidence about a driver. This is the option that makes the
  measurement sound rather than lucky, but it touches every evaluator, not just
  the mux path.
- **Move the sample point one sample earlier** (`edge − 1`). Empirically the
  best of the four phases on this capture, and the reason above: the phase is a
  guess, and this one is testable against the recorded captures without hardware.
  It wants the other four captures in `test_i2s_mux_decoder.py::TestSamplingPoint`
  re-derived before anyone believes it.
- **Reject a mux frame that contradicts its neighbours.** A step is one bit in a
  word that is otherwise idle at a known value; anything else is a decode
  failure. Cheap, and it would hide the rate problem rather than fix it.

## Reproduce

```bash
python3 scripts/harness.py --mode sync --arch esp32 --framework idf \
    --version 6.13.0 --pin-mode dir --sync-count 2 --imux
```

Captures kept: `capture/sync_i2s_mux+i2s_mux_dir_n2_esp32_idf6_13_0_*.vcd` and
`…_idf7_1_2_*.vcd`. The convention-free measurement the whole item rests on is
reproducible without hardware, and deliberately does **not** call
`extract_frames()` — the decoder is the thing under suspicion, so a count taken
with it would be circular:

```bash
python3 - <<'EOF'
import sys, bisect, collections
sys.path.insert(0, 'extras/tests/saleae_based/scripts')
import signal_parser as sp

ch, rate = sp.load_vcd('extras/tests/saleae_based/capture/'
                       'sync_i2s_mux+i2s_mux_dir_n2_esp32_idf6_13_0_'
                       'rmt_syncdir_i2s_muxdirn2.vcd')
data, bclk = ch['D5'], sp.rising_edges(ch['D6'])
anchor = bisect.bisect_left(bclk, sp.falling_edges(ch['D7'])[0])

frames, word, bits = [], 0, 0
for i in range(anchor, len(bclk)):
    word = (word << 1) | int(data[bclk[i]])
    bits += 1
    if bits == 32:
        frames.append((bclk[i], word))
        word = bits = 0

# Which of the 32 wire positions ever carry a bit, and in how many frames.
high = collections.Counter(
    p for _, w in frames for p in range(32) if (w >> (31 - p)) & 1)
for p, n in sorted(high.items()):
    print(f"position {p:2d}: high in {n:6d} of {len(frames)} frames")
EOF
```

which prints, on the IDF 5.5.3 capture:

```
position 11: high in     50 of 125255 frames
position 12: high in 125205 of 125255 frames
position 13: high in    115 of 125255 frames
position 14: high in 125205 of 125255 frames
position 15: high in     64 of 125255 frames
```

Two positions high from the first frame (the held-high direction bits), one high
64 times, one high 115 times, one 50 times on a bit nothing owns. The spacing
histograms are in the item.