# 173 Cross-driver start skew ~66% worse than same-driver

## Priority

**MEDIUM** — not a defect, but a critical characterization result that was
previously hidden by a firmware bug.  Users combining different drivers on
one board need to know the skew.

## Finding

Two `rmt` steppers start **29.5417 µs** apart (0.7385 step periods).  One RMT
and one MCPWM/PCNT start **49.0 µs** apart (**1.2250** periods), i.e. ~66 %
worse, and reproducibly so (three consecutive runs each, to within one sample
at 24 MS/s).

The paper's §5.2 premise — two drivers arming through unrelated hardware must
diverge — is what the hardware does.

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
