# 040 ESP32 synchronized start — no native group release, and the backlog entry did not exist

## Priority

**MEDIUM** — a capability gap that users combining drivers on one ESP32 need to
know about, plus a process note: this item had a row in `README.md` and a
priority and an estimate, and **no file**. It is written here for the first time
after the 2026-10-05 release matrix produced the measurement it was asking for.

## Finding

There is no native group start on this platform. Every driver arms its own
hardware and the first step lands when that hardware happens to get going, so
"aligned start" is approximated by issuing the `QRUN` kick-off from one place and
accepting whatever divergence follows.

Measured, `--mode sync`, six matrix rows, step period 25 µs. Full table in
[173](050_cross_driver_skew.md); the shape of it:

| pair | first-step skew | step periods |
|---|---|---|
| `mcpwm_pcnt+mcpwm_pcnt` | 3.0 … 13.3 µs | 0.3 … 1.3 |
| `rmt+rmt` | 17.2 … 67.6 µs | 1.7 … 6.8 |
| `rmt+mcpwm_pcnt` | 15.5 … 80.5 µs | 1.6 … 8.1 |
| any pair with one `i2s_direct` | 683 … 1093 µs | 68 … 109 |
| any pair with one `i2s_mux` | 781 … 1052 µs | 31 … 42 |

Two regimes, one order of magnitude apart:

- **RMT and MCPWM/PCNT are effectively aligned** — under 8 step periods, and
  usually under 2. Their hardware is timer-backed and arming is a register write,
  so a common kick-off lands them close together. `mcpwm_pcnt+mcpwm_pcnt` at
  0.3–1.3 periods is close to genuinely simultaneous.
- **Anything paired with an I2S driver is not aligned at all.** Worst measured
  case, `rmt+i2s_direct`, is **109 step periods** — the faster stepper completes
  its entire 64-step move before the slower one emits its first pulse. This is
  structural: I2S streams from a DMA buffer, so its first step is emitted when
  the DMA starts rather than when the queue is armed.

The part that is not merely "expected" is `i2s_direct+i2s_mux` at a **228 µs
floor** with *both* sides on I2S — neither is the odd one out, so buffer-fill
latency does not explain it. And `i2s_direct+i2s_direct` measured 38.8 µs on IDF
5.5.3 against 412.5 µs on 6.1.0: a 10× spread on one code path, one
measurement each. Both are unexplained and are the substance of this item.

## What would actually fix it

Per the README row this entry carried — "native per-driver release (I2S group,
RMT group start, MCPWM/PCNT) pending":

- **RMT** has `rmt_channel_group_start()` and the hardware supports it; the
  driver starts channels individually today. Group start would collapse the
  `rmt+rmt` and `rmt+mcpwm_pcnt` cases to a common edge.
- **MCPWM/PCNT** has no cross-group start; the two timers are independent, and
  `SOC_MCPWM_GROUPS = 2` means a pair on different groups cannot be made
  simultaneous in software. Worth knowing before it is promised.
- **I2S** can only align two channels by starting their DMAs in the same
  iteration, which the current per-stepper construction does not do. This is the
  large-skew case and the one that needs real work.

## Note on this item's history

`README.md` listed `040 ESP32 synchronized start` with priority, estimate and a
one-line description from the start, linking to `040_esp32_synchronized_start.md`
— which `git log --all --diff-filter=A` confirms **was never committed**. The
link was the only trace of the item, so the backlog silently lost it the way an
unlinked file loses it: nothing failed, nothing warned, and the priority and
estimate kept it looking tracked. Found by checking every `README.md` link in
the backlog against the filesystem after the matrix run, which turned up exactly
this one dangling target.

The other 040 entries (`avr`, `pico`, `sam`, `samd51`, `teensy`) all exist and
are untouched by this run — no hardware for those platforms is connected.