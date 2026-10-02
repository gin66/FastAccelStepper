# SR_15 — aligned start, then each stepper at its own period

**Test ID:** SR_15
**Goal:** aligned start, then each stepper at its own period
**Program:** `stepper 0: QSEG 0 200 640 1 ; stepper 1: QSEG 1 200 1280 1`
**DUT:** 16000000 ticks/s, `MIN_CMD_TICKS` 3200, `QUEUE_LEN` 32, fastest legal 640 ticks
**Configuration:** 2ch on auto (esp32)
**Tag:** `esp32_auto_2ch`
**Captured at:** 24000000 Hz
**Result:** **PASS**

## Measured

| stepper | steps | period us (min–max, spread) | pulse high us (min–max, spread) | duty % |
|---|---|---|---|---|
| A | 200 | 39.9167–40, 0.0833 | 15.5417–15.625, 0.0833 | 39.06 |
| B | 200 | 79.875–79.9583, 0.0833 | 15.5417–15.625, 0.0833 | 19.53 |

## Detail

```json
{
  "A": {
    "mean_period_us": 39.9665,
    "period": {
      "expected_period_us": 40.0,
      "long_periods_us": [],
      "n_long": 0,
      "n_short": 0,
      "ok": true,
      "periods_measured": 199,
      "short_periods_us": [],
      "tolerance_us": 2.0
    },
    "steps": {
      "extra_steps": 0,
      "missing_steps": 0,
      "ok": true,
      "steps_expected": 200,
      "steps_measured": 200
    },
    "ticks": 640
  },
  "B": {
    "mean_period_us": 79.9322,
    "period": {
      "expected_period_us": 80.0,
      "long_periods_us": [],
      "n_long": 0,
      "n_short": 0,
      "ok": true,
      "periods_measured": 199,
      "short_periods_us": [],
      "tolerance_us": 4.0
    },
    "steps": {
      "extra_steps": 0,
      "missing_steps": 0,
      "ok": true,
      "steps_expected": 200,
      "steps_measured": 200
    },
    "ticks": 1280
  },
  "first_step_skew_us": 27.1667,
  "invariants": {
    "dir_while_step_high": {},
    "n_dir_while_step_high": 0,
    "ok": true
  },
  "speed_ratio": 2
}
```

## Capture

`/tmp/cap/rep/SR_15.vcd`
