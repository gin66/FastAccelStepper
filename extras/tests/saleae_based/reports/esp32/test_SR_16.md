# SR_16 — does a second stepper perturb the first?

**Test ID:** SR_16
**Goal:** does a second stepper perturb the first?
**Program:** `QSEG 64 640 1`
**DUT:** 16000000 ticks/s, `MIN_CMD_TICKS` 3200, `QUEUE_LEN` 32, fastest legal 640 ticks
**Configuration:** 2ch on rmt_v2+rmt_v2 (esp32)
**Tag:** `esp32_rmt_v2_rmt_v2_2ch`
**Captured at:** 24000000 Hz
**Result:** **PASS**

## Measured

| stepper | steps | period us (min–max, spread) | pulse high us (min–max, spread) | duty % |
|---|---|---|---|---|
| A | 64 | 39.9167–40, 0.0833 | 15.5833–15.625, 0.0417 | 39.06 |
| B | 64 | 39.9167–40, 0.0833 | 15.5833–15.625, 0.0417 | 39.06 |

## Detail

```json
{
  "invariants": {
    "dir_while_step_high": {},
    "n_dir_while_step_high": 0,
    "ok": true
  },
  "per_stepper": {
    "A": {
      "max_period_us": 40.0,
      "mean_period_us": 39.9656,
      "min_period_us": 39.9167,
      "period": {
        "expected_period_us": 40.0,
        "long_periods_us": [],
        "n_long": 0,
        "n_short": 0,
        "ok": true,
        "periods_measured": 63,
        "short_periods_us": [],
        "tolerance_us": 2.0
      },
      "steps": {
        "extra_steps": 0,
        "missing_steps": 0,
        "ok": true,
        "steps_expected": 64,
        "steps_measured": 64
      }
    },
    "B": {
      "max_period_us": 40.0,
      "mean_period_us": 39.9663,
      "min_period_us": 39.9167,
      "period": {
        "expected_period_us": 40.0,
        "long_periods_us": [],
        "n_long": 0,
        "n_short": 0,
        "ok": true,
        "periods_measured": 63,
        "short_periods_us": [],
        "tolerance_us": 2.0
      },
      "steps": {
        "extra_steps": 0,
        "missing_steps": 0,
        "ok": true,
        "steps_expected": 64,
        "steps_measured": 64
      }
    }
  },
  "ticks": 640
}
```

## Capture

`/tmp/cap/r8/SR_16.vcd`
