# SR_01 — inter-step period equals ticks

**Test ID:** SR_01
**Goal:** inter-step period equals ticks
**Program:** `QSEG 8 640 1`
**DUT:** 16000000 ticks/s, `MIN_CMD_TICKS` 3200, `QUEUE_LEN` 32, fastest legal 640 ticks
**Configuration:** 1ch on rmt_v2 (esp32)
**Tag:** `esp32_rmt_v2_1ch`
**Captured at:** 24000000 Hz
**Result:** **PASS**

## Measured

| stepper | steps | period us (min–max, spread) | pulse high us (min–max, spread) | duty % |
|---|---|---|---|---|
| A | 8 | 39.9583–40, 0.0417 | 15.5833–15.625, 0.0417 | 39.06 |

## Detail

```json
{
  "adherence": {
    "commanded_period_us": 40.0,
    "commanded_rate_hz": 25000.0,
    "jitter_pct": 0.104,
    "max_period_us": 40.0,
    "mean_period_us": 39.9702,
    "measurable": true,
    "min_period_us": 39.9583,
    "n_out_of_tolerance": 0,
    "ok": true,
    "periods_measured": 7,
    "rate_max_hz": 25026.07,
    "rate_mean_hz": 25018.62,
    "rate_min_hz": 25000.0,
    "sag_pct": -0.074,
    "tolerance_us": 0.8,
    "worst_deviation_us": 0.0417
  },
  "invariants": {
    "dir_while_step_high": {},
    "n_dir_while_step_high": 0,
    "ok": true
  },
  "period": {
    "expected_period_us": 40.0,
    "long_periods_us": [],
    "n_long": 0,
    "n_short": 0,
    "ok": true,
    "periods_measured": 7,
    "short_periods_us": [],
    "tolerance_us": 2.0
  },
  "steps": {
    "extra_steps": 0,
    "missing_steps": 0,
    "ok": true,
    "steps_expected": 8,
    "steps_measured": 8
  },
  "ticks": 640,
  "ticks_per_s": 16000000
}
```

## Capture

`/tmp/cap/r8/SR_01.vcd`
