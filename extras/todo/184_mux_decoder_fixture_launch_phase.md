# 184 The mux decoder's fixture launched its data at the wrong instant

## Status

**CLOSED** — fixed in the test fixture; `i2s_mux_decoder.py` needed no change.
Verified on the real 24 MS/s hardware captures end-to-end: 64/64 steps on the
correct slots, mean period 24.931 us, identical to the recorded results.

## Priority it had

**HIGH** — the decoder is the only thing standing between this harness and a
measured 32-stepper mux row, and 11 unit tests were failing. They were correct
to fail.

## Finding

`Bus._render()` in `scripts/tests/test_i2s_mux_decoder.py` drew each bit's data
cell starting **at** its bclk rising edge. Real hardware launches it **half a
cell before** the edge that latches it. A waveform with the right periods and the
wrong phase still looks like a square wave on a scope, so nothing caught it
except the decoder's `edge - 1` sample point — which reads one cell early, and
therefore read the *previous* cell's data.

Every bit then landed one position from where it belonged:

| word sent | decoded | meaning |
|---|---|---|
| `0x00000001` | `0x80000000` | slot 31, not slot 0 |
| `0x80000000` | `0x40000000` | slot 30, not slot 31 |
| `0xFFFFFFFF` | `0xFFFF8000` | bits 15..31 |

The fix is one line — launch the cell `bclk_half` samples early — and the 11
failures went to zero.

## Why this was invisible for so long

Every existing test in the file drove **all 32 slots**, or a **symmetric** set,
or compared slot 0 against slot 31. A uniform one-cell shift is a consistent
relabelling, and each of those assertions is invariant under it. A test that
sets **one** slot and asserts that exactly that slot lights up catches it
immediately; there was none. `test_one_slot_lights_up_and_no_other_does`,
`test_every_slot_in_isolation_decodes_to_itself` and
`test_data_is_stable_one_sample_before_the_latching_edge` now exist for that
reason, and the whole 24 MS/s geometry has its own class.

## The first diagnosis of this item was wrong, and why

It is recorded because the wrongness is instructive, not because it was close.

The symptom was a half-swap with no per-half bit reversal: slot 0 decoding as
slot 15. The arithmetic was checked, three cases matched a "swap and reverse each
half" rule, and the 11 failures were attributed to `extract_frames()`.

All of that rested on one number read out of `Bus.render()` — a dict keyed by
role name (`data`/`bclk`/`ws`). `_bus_names()` infers roles as
`sorted(channels)[-3:]`, which is analyzer-channel order; fed role names it
returns `('bclk','data','ws')`, so the decoder read **bclk as data**. The word
that came back was garbage, and the bit-reversal rule was then "verified"
against arithmetic constructed from that garbage. Circular, and it looked like
three independent confirmations.

The test suite never had this problem: `Bus.channels()` names the wires `D5`,
`D6`, `D7` and `roundtrip()` goes through it. Only my ad-hoc probe bypassed it.

The mistake that survived longer was reporting "no new failures" as reassurance
while 11 were red. They were the defect; the framing hid it.

## Ground truth, measured

Both sample rates, from the captures in `capture/` via `signal_parser`:

- **24 MS/s** (`scale_i2s_mux_nodir_n1_...`, `n2_...`): 8 MHz bclk = 3 samples per
  cell. The first step's data cell ran samples **6625361..6625363**, with bclk
  rises at **6625360** and **6625363** — the cell contains its latching edge.
  Grouping from the ws-fall anchor and sampling at `edge - 1` gives exactly **64**
  frames carrying a single bit, always at **wire k = 15**, which the half-swap
  turns into `0x00000001` = slot 0 = the stepper MAP names. The n=2 capture gives
  **S0=64 and S1=64**, the two adjacent slots that run's MAP names.
- **48 MS/s**: the geometry quoted in `extract_frames`' own docstring — data cell
  208..213, bclk rises at 211 and 217 — is the edge at the cell's midpoint, which
  is what "half a cell early" means. That is where the `edge - 1` decision was
  made, and the fixture contradicted it.

So: wire bit k < 16 carries slot 15 - k, the half-swap is correct, the sample
point is correct, and the decoder was never wrong. `sample_rate_hz` remains
accepted and unused by design, and that is not a defect either.

## What is left

- **023 (the 24 MS/s sampling race in `dir`) is untouched and separate.** `nodir`
  is its clean case. Do not close one with the other.
- **`i2s_mux` at n = 32 is still unmeasured**, for 023 and the known intermittent
  dropped pulse, not for anything here. What changed is that the decoder is no
  longer the reason to distrust it.
- **No 48 MS/s mux capture is on disk** — `probe_mux_bits.vcd` is absent, so the
  two hardware tests in `TestAgainstRealCapture` skip. The 48 MS/s geometry above
  is quoted from the decoder's docstring, not re-measured. If someone re-runs
  `probe_mux_bits.py`, that closes the last gap in this item.
- The intermittent dropped pulse (`i2s_fill_buffer_mux()` doing
  `buf[...] |= bit_mask` on memory the DMA is reading) is unaddressed and is not
  a decoder issue.