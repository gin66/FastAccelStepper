# SR_14 — 2 steppers, synchronized start

**Test ID:** SR_14
**Goal:** 2 steppers, synchronized start
**Program:** `QSEG 2000 640 1`
**DUT:** 16000000 ticks/s, `MIN_CMD_TICKS` 3200, `QUEUE_LEN` 32, fastest legal 640 ticks
**Configuration:** 2ch on auto (esp32)
**Tag:** `esp32_auto_2ch`
**Captured at:** 24000000 Hz
**Result:** **PASS**

## Measured

| stepper | steps | period us (min–max, spread) | pulse high us (min–max, spread) | duty % |
|---|---|---|---|---|
| A | 2000 | 39.9167–40, 0.0833 | 15.5417–15.625, 0.0833 | 39.06 |
| B | 2000 | 39.9167–40, 0.0833 | 15.5417–15.625, 0.0833 | 39.06 |

## Detail

```json
{
  "first_step_skew_us": 29.5417,
  "first_step_us": {
    "A": 2219633.75,
    "B": 2219663.2917
  },
  "invariants": {
    "dir_while_step_high": {},
    "n_dir_while_step_high": 0,
    "ok": true
  },
  "period_us": 40.0,
  "skew_periods": 0.7385,
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

`/tmp/cap/rep/SR_14.vcd`
