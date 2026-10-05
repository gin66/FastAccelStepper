# 173 Cross-driver start skew — the I2S drivers, not "different driver", dominate

## Priority

**MEDIUM** — not a defect, but a critical characterization result that was
previously hidden by a firmware bug.  Users combining different drivers on
one board need to know the skew.

## Finding

**Superseded in scope by the 2026-10-05 release matrix** — see below. The
original claim was "cross-driver skew is ~66 % worse than same-driver".  The
full combination matrix does not support that as the general rule: the two
distributions overlap heavily. What actually predicts the skew is whether one
side is an **I2S** driver.

First-step skew, `--mode sync`, one measurement per combination per matrix row
(6 rows: arduino 4.4/5.3/6.13 = IDF 4.4.7, idf 5.3 = 4.4.3, idf 6.13 = 5.5.3,
idf 7.1.2 = 6.1.0). Step periods at the commanded 25 µs:

| combination | skew µs | step periods | rows |
|---|---|---|---|
| `mcpwm_pcnt+mcpwm_pcnt` | 3.0 … 13.3 | 0.3 … 1.3 | 6 |
| `rmt+mcpwm_pcnt` | 15.5 … 80.5 | 1.6 … 8.1 | 6 |
| `rmt+rmt` | 17.2 … 67.6 | 1.7 … 6.8 | 6 |
| `i2s_direct+i2s_direct` | 38.8 … 412.5 | 3.9 … 41.3 | 2 |
| `i2s_direct+i2s_mux` | 227.8 … 232.5 | 9.1 … 9.3 | 2 |
| `mcpwm_pcnt+i2s_direct` | 683.2 … 846.1 | 68.4 … 84.7 | 2 |
| `rmt+i2s_direct` | 697.0 … 1093.5 | 69.8 … 109.4 | 2 |
| `mcpwm_pcnt+i2s_mux` | 781.4 … 1051.8 | 31.3 … 42.1 | 2 |
| `rmt+i2s_mux` | 948.4 … 1051.2 | 38.0 … 42.1 | 2 |

Three things the wider sample changed:

- **`rmt+rmt` and `rmt+mcpwm_pcnt` overlap** (17.2 … 67.6 vs 15.5 … 80.5). The
  "~66 % worse" figure came from three consecutive runs each of one
  combination; across six rows the same-driver case is sometimes the *larger*
  of the two. The claim does not survive the wider sample, so it is withdrawn
  rather than restated with a new constant.
- **One I2S stepper costs 1–2 orders of magnitude.** Worst case
  `rmt+i2s_direct` at **1093.5 µs = 109 step periods** — the faster stepper
  finishes its whole 64-step move before the other starts.
- **The non-I2S pairs split by framework.** At the same 25 µs period the
  Arduino rows measure 15–37 µs where the native-IDF rows measure 66–81 µs,
  which is the kind of difference that reads as a driver property until you
  notice it tracking the framework.

Why the I2S pairs are slow is mechanical and expected in outline: `i2s_direct`
and `i2s_mux` stream from a DMA buffer, so their first step is emitted when the
DMA starts rather than when the queue is armed. What is **not** established is
whether the 228 µs floor of `i2s_direct+i2s_mux` — where *both* sides are I2S,
so neither is the odd one out — is a buffer-fill latency that a user could
avoid, or the floor of the mechanism. That is the open question here.

`i2s_direct+i2s_direct` at 38.8 µs on IDF 5.5.3 against 412.5 µs on 6.1.0 is a
10× spread between two SDKs on the same code path, single measurement each, and
is not explained.  Treat the two-column ranges as indicative, not as bounds.

## Original finding (2026-10-04, superseded in scope)

Two `rmt` steppers start **29.5417 µs** apart (0.7385 step periods).  One RMT
and one MCPWM/PCNT start **49.0 µs** apart (**1.2250** periods), i.e. ~66 %
worse, and reproducibly so (three consecutive runs each, to within one sample
at 24 MS/s).

The paper's §5.2 premise — two drivers arming through unrelated hardware must
diverge — is what the hardware does.  Kept because the 29.5417 µs value is
still a correct measurement of `rmt+rmt`; what no longer holds is that the
*ratio* between the two cases is a property of the drivers.

## How the wrong number was produced (previous finding)

The old firmware's `mixed` channel config parsed its per-stepper driver list
into `drivers[]` and then, three lines later, overwrote every entry with the
automatic driver choice:

```c
if (fill == SA_AUTO && n > 1) {
  for (uint8_t i = 0; i < n; i++) drivers[i] = SA_AUTO;   // <- discarded the list
}
```

`fill` was only non-`SA_AUTO` for `4ch_rmt`/`4ch_mcpwm`, so `CONFIG mixed
rmt,mcpwm` was byte-for-byte equivalent to `CONFIG 2ch`: both steppers on
the automatic choice, which under `SUPPORT_DYNAMIC_ALLOCATION` is RMT.

**SR_17 never crossed drivers at all.**  It reported 29.5417 µs — the
same-driver number, because it *was* the same-driver run, agreeing to a sample
at 24 MS/s.  That absurd precision is what gave it away: two independent
hardware paths do not agree to four decimal places.

Verified by flashing the pre-R1 firmware and re-running: SR_17 came back at
29.5417 µs again, and SR_14 at 29.5417 µs, on the old build.

## Consequence for the baseline

The committed `reports/esp32/` SR_17 entry is labelled `rmt+mcpwm_pcnt` but
measured RMT+RMT.  R8 replaces it.
