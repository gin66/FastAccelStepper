# SR_14 — 2 steppers, synchronized start

**Test ID:** SR_14
**Goal:** 2 steppers, synchronized start
**Program:** `QSEG 2000 640 1`
**DUT:** 16000000 ticks/s, `MIN_CMD_TICKS` 3200, `QUEUE_LEN` 32, fastest legal 640 ticks
**Configuration:** 2ch on rmt_v2+rmt_v2 (esp32)
**Tag:** `esp32_rmt_v2_rmt_v2_2ch`
**Captured at:** 24000000 Hz
**Result:** **PASS**

## Measured

| stepper | steps | period us (min–max, spread) | pulse high us (min–max, spread) | duty % |
|---|---|---|---|---|
| A | 2000 | 39.9167–40, 0.0833 | 15.5417–15.625, 0.0833 | 39.06 |
| B | 2000 | 39.9167–40, 0.0833 | 15.5417–15.625, 0.0833 | 39.06 |

**First-step skew between steppers:** 29.5833 us (0.7396 step periods). Both steppers' steady-state periods matching above does *not* mean they started together -- this is the number that says when their first steps landed. Recorded, not asserted.

## Detail

```json
{
  "first_step_skew_us": 29.5833,
  "first_step_us": {
    "A": 2189282.5,
    "B": 2189312.0833
  },
  "invariants": {
    "dir_while_step_high": {},
    "n_dir_while_step_high": 0,
    "ok": true
  },
  "period_us": 40.0,
  "skew_periods": 0.7396,
  "steps_per_stepper": {
    "A": {
      "extra_steps": 0,
      "missing_steps": 0,
      "ok": true,
      "steps_expected": 2000,
      "steps_measured": 2000
    },
    "B": {
      "extra_steps": 0,
      "missing_steps": 0,
      "ok": true,
      "steps_expected": 2000,
      "steps_measured": 2000
    }
  }
}
```

## Capture

`/tmp/cap/r8/SR_14.vcd`
