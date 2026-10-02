# SR_17 — synchronized start across two different drivers

**Test ID:** SR_17
**Goal:** synchronized start across two different drivers
**Program:** `QSEG 200 640 1`
**DUT:** 16000000 ticks/s, `MIN_CMD_TICKS` 3200, `QUEUE_LEN` 32, fastest legal 640 ticks
**Configuration:** mixed_rmt_mcpwm on rmt+mcpwm_pcnt (esp32)
**Tag:** `esp32_rmt_mcpwm_pcnt_mixed_rmt_mcpwm`
**Captured at:** 24000000 Hz
**Result:** **PASS**

## Measured

| stepper | steps | period us (min–max, spread) | pulse high us (min–max, spread) | duty % |
|---|---|---|---|---|
| A | 200 | 39.9167–40, 0.0833 | 15.5833–15.625, 0.0417 | 39.06 |
| B | 200 | 39.9167–40, 0.0833 | 19.875–19.9583, 0.0833 | 49.84 |

**First-step skew between steppers:** 48.9167 us (1.2229 step periods). Both steppers' steady-state periods matching above does *not* mean they started together -- this is the number that says when their first steps landed. Recorded, not asserted.

## Detail

```json
{
  "first_step_skew_us": 48.9167,
  "first_step_us": {
    "A": 2200130.3333,
    "B": 2200179.25
  },
  "invariants": {
    "dir_while_step_high": {},
    "n_dir_while_step_high": 0,
    "ok": true
  },
  "period_us": 40.0,
  "skew_periods": 1.2229,
  "steps_per_stepper": {
    "A": {
      "extra_steps": 0,
      "missing_steps": 0,
      "ok": true,
      "steps_expected": 200,
      "steps_measured": 200
    },
    "B": {
      "extra_steps": 0,
      "missing_steps": 0,
      "ok": true,
      "steps_expected": 200,
      "steps_measured": 200
    }
  }
}
```

## Capture

`/tmp/cap/r8/SR_17.vcd`
