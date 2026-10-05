# 072 MAP does not report a multiplexed stepper's direction slot

## Priority

**MEDIUM** — a gap in the firmware's map protocol that the host currently
works around with an assumption. The workaround is correct today and will be
silently wrong the moment slot allocation stops being gapless.

## Finding

`MAP` reports `slots=` with **one entry per stepper**, holding that stepper's
**step** bit. It does not report the **direction** bit, which in `dir` mode is
a second slot of the same 32-bit word and is needed to evaluate every direction
scenario on a multiplexed stepper.

Verified on hardware, not inferred:

```
> CONFIG 2 i2s_mux,i2s_mux dir
OK CONFIG n=2 mode=dir stride=2 drivers=i2s_mux,i2s_mux maxspeed0=400 maxspeed1=400
> MAP
MAP count=2 mode=dir stride=2 ch= bus=5,6,7 slots=0,2 marker=255
```

Two entries for two steppers: stepper A's step bit is slot 0, stepper B's is
slot 2. Slots 1 and 3 are A's and B's direction bits and appear nowhere.

The firmware allocates them with `mux_next_slot()` twice, back to back
(`common/saleae_app.cpp`), from a cursor that `handle_config` resets, with
steppers connected in order — so the pairs come out gapless, 0/1 then 2/3.
That is what lets the host derive a direction slot as `step_slot + 1`.

## Why the host cannot just be told

The host had the opposite rule — `slots` is one entry per *channel* — and that
is the bug that made `sync --imux` report a working driver as dead. It agreed
with itself in `nodir` (where one-per-stepper and one-per-channel are the same
list) and ran off the end of the list in `dir`, handing every stepper past the
first a GPIO channel. Measured: `sync rmt+i2s_mux dir` on IDF 6.1 reported
**0 of 64 steps** for the mux stepper while the capture shows **64 rises on
S0**. Fixed in `read_map()` (`run_tests.py`), with the two unit tests that
encoded the wrong rule corrected to the measured reply and a mixed
mux+GPIO `dir` case added — see the saleae `AGENTS.md` "Firmware protocol"
entry for `MAP`.

That fix is right about the parse and still incomplete about the protocol: it
derives the direction slot rather than reading it, so it is now the host that
holds the assumption.

## Options

- **Report the direction slot in `MAP`.** A second field, or a per-stepper
  `step,dir` pair (`slots=0,1,-,-`). Costs reply-buffer space on AVR, where
  every literal is SRAM (`saleae_str.h` rules, AGENTS.md "Strings and `const`
  tables"), and needs the `i2s_mux_decoder.py` side checked for the same
  one-per-what assumption. This is the honest fix and it makes the assumption
  deletable.
- **Report the allocation is gapless.** One extra flag in the `MAP` reply, e.g.
  `slotalloc=gapless`, which is what the host actually assumes. Cheaper, and it
  makes the assumption checkable instead of implicit — but it documents a
  property of the allocator rather than the thing the host needs.

Until one of those lands, `read_map()` raises rather than guesses when
`step_slot + 1` would fall outside the 32-bit word (`MUX_SLOT_COUNT`), so the
assumption fails loudly at the one boundary where it is currently wrong. It
would still be wrong in the middle of the word if the cursor were ever
resumed rather than reset — which no code path does today.

## Not a bug

`slots=-` for a GPIO stepper is correct and the parse handles it; the mixed
case (`CONFIG 2 i2s_direct,i2s_mux dir` → `slots=-,0`) is now covered by a
unit test. Only the direction bit is missing.
## 2026-10-05 — still open; workaround re-validated on both SDKs

The gap is unchanged: `MAP` reports one slot per stepper (the **step** bit) and
never the direction bit, which in `dir` mode is the next bit of the same word.

Re-checked on both I2S rows of the full matrix. `CONFIG 2 i2s_mux,i2s_mux dir`
still answers `slots=0,2` — the direction slots are still inferred from
`step_slot + 1`, and they are still not reported.

The `nodir` workaround this item's gap forces on the host is now validated on
both SDKs in a single run: `--mode scale --driver i2s_mux --pin-mode nodir` is
green for `n = 1…8` on IDF 5.5.3 and 6.1.0.

Note the compounding with [076](076_i2s_mux_dir_second_slot_not_decoded.md): the
only mux configuration that fails is `dir`, and `dir` is precisely the mode that
needs the direction slot this item does not report. `nodir` works and does not
exercise the gap; `dir` exercises the gap and does not work. Neither item can be
settled from the other's evidence, and both are open.
