# 022 i2s_mux in `dir`: the second stepper's slot is intermittently not decoded

## Priority

**HIGH** — a functional defect in the shipped `i2s_mux` driver, not a
documentation gap: `CONFIG 2 i2s_mux,i2s_mux dir` loses a whole slot, so `dir`
is not a mode a user can rely on. The harness reports it correctly (an
incomplete capture rather than a dead driver), which is why it was not caught
as a crash — the failure is silent on the wire and only visible in the decoded
capture. `nodir`, the mode the library documents and exercises, is unaffected.

Renumbered from 076: this was filed as a characterization item next to the two
other mux findings (070, 072), but it is the only one of the three that is a
library defect rather than a constant or a protocol gap.

## Finding

`--mode sync --pin-mode dir` with two multiplexed steppers,
`CONFIG 2 i2s_mux,i2s_mux dir` → slots 0 (step) and 2 (step), direction bits 1
and 3. The decoded capture contains **S0 and S1 but never S2**:

```
result: failed
incomplete_capture: {"missing_steppers": ["B"], "missing_channels": ["S2"],
                     "channel_map": {"A": {"step": "S0", "dir": "S1"},
                                     "B": {"step": "S2", "dir": "S3"}},
                     "captured": ["S0"]}
```

Measured on **both** SDKs that have I2S queues at all — ESP-IDF 5.5.3 and
6.1.0 — so it is not SDK-specific. The raw capture is fine: the source 8-channel
VCD has all of D0…D7 at the right rate (24 MS/s, all three bus wires toggling),
so this is after the sigrok read, not in the analyzer.

## What has been ruled out

- **Not the sample rate.** 24 MS/s is 3 samples per 8 MHz bit period, the floor
  the harness enforces and the rate at which the same `--imux` flag measured
  `scale --driver i2s_mux` correctly at n = 1…8 on both rows, 64/64 steps every
  point.
- **Not the map.** `MAP` reported `slots=0,2` exactly as expected, and stepper
  A decoded to 64/64 on S0 in the very same capture. That is the run that first
  exposed the `slots=` parsing bug (072), so this item is only visible *after*
  that fix — before it, both steppers were mis-assigned and the run failed for
  a different reason.
- **Not a decode of a slot that was never driven.** In an `rmt+i2s_mux` capture
  from the same session, S0 had 64 rises and S2 had 51, so the decoder does
  recover a second slot when it is there.

## Where that leaves it

This is the same shape as the known intermittent single-step drop in
[`r7_virtual_i2s_mux.md`](../doc/implemented/r7_virtual_i2s_mux.md) §5 —
reproduced once at n=28, slot 16, with a 47.96 µs inter-step period where the
others were 24 — except here the loss is **whole slots**, not one pulse, and it
is reproducible enough to land on two consecutive rows. Suspected in that entry
as `i2s_fill_buffer_mux()` doing `buf[...] |= bit_mask` on memory the DMA is
reading; if that is the cause, a whole slot vanishing and a single step
vanishing are the same bug at two severities, and a slot-wide loss is the
cheaper one to detect and the better one to debug on.

Decide whether it is the decoder (`i2s_mux_decoder.py` losing a bit somewhere
after the shift register) or the wire (the firmware not setting the bit). The
cheapest discriminator is a raw bus capture read without the decoder: if S2 is
absent from the 32-bit word in the raw VCD, it is the firmware.

## Workaround until then

`--mode scale --driver i2s_mux --pin-mode nodir` is the reliable mux
measurement and it is green: n = 1…8, 64/64 steps, 24.93 µs mean period on both
IDF 5.5.3 and 6.1.0. `nodir` is one slot per stepper, so no second slot is
involved. Use `dir` on the mux only for the direction-bit behaviour, which is
what 072 is about.
## 2026-10-05 — full release matrix: same failure, both SDKs, one measurement each

Re-measured from an empty build directory across all six matrix rows
(`reports/esp32_platform_matrix.md`). The finding is unchanged and now has a
stronger claim behind it, because it is no longer one SDK:

| row | ESP-IDF | result | missing |
|---|---|---|---|
| `idf-6.13.0` | 5.5.3 | `failed` | S2 (stepper B) |
| `idf-7.1.2` | 6.1.0 | `failed` | S2 (stepper B) |

The other four rows refuse the configuration outright (`ERR CONFIG no such
driver`), so this is the only failing sync combination in the matrix.

`missing_channels: ["S2"]`, `captured: ["S0"]`, and the direction bit S1 *is*
present — so slot 0 and its paired direction decode, slot 1 decodes, and slot 2
never appears at all. Both `i2s_mux` steppers emit `POS 64 64`, so the firmware
is commanding both; the loss is between the wire and the decoder, or in the
firmware's slot assignment for stepper B.

Two independent SDKs failing identically is the useful part: it argues against
an SDK-specific I2S or DMA behaviour and for something in the library's slot
allocation or in the decoder, which is shared by both. It remains **one**
measurement per SDK, so "intermittent" in the original heading is still not
excluded — the title says intermittently and the evidence cannot yet distinguish
always from usually.

Still cheap to settle, as above: read the raw 3-channel bus capture without the
decoder and look for S2 in the 32-bit word. That distinguishes firmware from
decoder in one capture.
