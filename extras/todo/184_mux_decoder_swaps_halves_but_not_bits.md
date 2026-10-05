# 184 The mux decoder swaps the halves but not the bits inside them

## Priority

**HIGH** — the I2S mux decoder is the only thing standing between this harness
and a measured 32-stepper row, and it currently mirrors slots instead of
decoding them. Every hardware claim about `i2s_mux` above one stepper rests on
it.

## Finding

`i2s_mux_decoder.extract_frames()` collects the 32 bits MSB-first and then undoes
the wire order with one operation:

```python
(word >> 16) | ((word & 0xFFFF) << 16)
```

That swaps the two 16-bit halves. It does **not** reverse the bit order *within*
either half. The measured wire carries slot `15-k` first in the low half
(`AGENTS.md` → "The I2S mux: 32 steppers on 3 wires"), so a word collected
MSB-first has each half byte-reversed relative to slot order, and a half-swap
alone leaves every slot mirrored inside its half of the word.

The 11 unit-test failures in `test_i2s_mux_decoder.py` are this, and they are
correct to fail. Isolated with a one-word bus (`Bus([1 << 0])`, roles passed
explicitly so nothing else is in the way):

| word sent (slot) | collected | after the swap the code does | correct |
|---|---|---|---|
| `0x00000001` (slot 0) | `0x80000000` | `0x00008000` = **slot 15** | `0x00000001` |
| `0x80000000` (slot 31) | `0x00000001` | `0x00010000` = slot 16 | `0x80000000` |
| `0x00000002` (slot 1) | `0x40000000` | `0x00004000` = slot 14 | `0x00000002` |

Swap **and** reverse each half reproduces all three:

```python
def rev16(x): return int(f"{x:016b}"[::-1], 2)
word = rev16(swapped & 0xFFFF) | (rev16(swapped >> 16) << 16)
```

`extract_frames` returns **zero** frames for the fixture's 5-frame bus, with
`1 trailing bit(s) with no frame boundary` — which is the signature of an
anchored-at-the-end shift register, and is what the other eight failures are
downstream of.

## Not the cause

- **Not the sample rate.** `sample_rate_hz` is accepted and unused by design,
  and `AGENTS.md` says so. The fixture renders at 48 MS/s over an 8 MHz bclk and
  decodes identically at any rate, once the roles are right.
- **Not the sampling phase.** `edge - 1` is documented as measured, with the
  three alternatives recorded and rejected. Those tests are in the same file and
  are red for the same reason as this one, not independently.
- **Not the half-swap fix AGENTS.md describes as landed.** That fix made the two
  16-bit halves come out in the right order. The per-half bit order was never
  addressed, and a one-word fixture could not have caught it: every test in the
  file drives all 32 slots or symmetric slot pairs, and a mirrored mapping looks
  correct on those.

## A second, separate problem in the same function

`_bus_names()` infers the bus roles as `sorted(channels)[-3:]` — analyzer-channel
order, which is right for a real capture (`D5` data, `D6` bclk, `D7` ws, the
firmware's `SALEAE_BUS_BASE`) and wrong for the fixture, which hands it a
3-channel dict keyed by role name. `sorted(['bclk','data','ws'])[-3:]` is
`('bclk','data','ws')`, so the decoder reads `bclk` as data and `data` as bclk.

That is a fixture that does not present the interface the function documents, and
it accounts for at least two of the eleven failures on its own. It is worth
fixing in the same pass: a fixture that passes roles positionally has to change
every time the role order changes, which is how this went unnoticed.

## What to find

- **Confirm the bit order on the wire before changing the decoder.** The table
  above is derived from the *documented* wire order, and it is self-consistent,
  but "consistent with the documentation" is not "measured". One `probe_mux_bits`
  run that prints which wire position carries a given slot settles it: drive one
  slot, read the 32 bus positions, and check whether slot 0 is on k = 15 or
  k = 0. If it is k = 0, the bug is in the fixture's renderer instead and the
  decoder is right — which would make this a test bug, not a harness one, and
  both are worth knowing before either is edited.
- **Then fix the decoder and the fixture together**, and re-run
  `--mode scale --driver i2s_mux --pin-mode nodir` for n = 1…4 before trusting
  anything above n = 4. The acceptance bar is that each slot is *distinguishable*
  — a mirrored decoder passes a symmetric test, so the check has to be
  asymmetric: set one low slot, assert its channel goes high and the mirrored
  one stays low.
- **The mux row of the release matrix has never been green.** `183` was filed
  because `--mode scale --driver i2s_mux` is the only thing that reaches n = 32,
  and this is the reason it cannot yet be trusted at any n. `AGENTS.md` records
  the mux sweep as green n = 1…8 in `nodir`; those runs must have been scored by
  a path that did not depend on slot identity (per-stepper adherence, with each
  stepper measured against its own channel, survives a mirrored mapping only if
  the mirror is consistent — which is worth re-checking rather than assuming).
- 023 (the 24 MS/s sampling race in `dir`) is a **separate** defect and is not
  blocked by this one; `nodir` is its clean case. Do not close one with the other.

## Note

Found while closing [183](183_no_catalogue_test_for_the_maximum_stepper_count.md).
The max-count scenario reached its answer on three GPIO drivers — 8, 6 and 2 —
and `i2s_mux` was deliberately not measured rather than assumed green. This is
why that gap existed, and it is the same gap 182 was filed against: the number
could not be *trusted* above n = 8, which is a stronger claim than "could not be
tested" and one nobody had written down.
