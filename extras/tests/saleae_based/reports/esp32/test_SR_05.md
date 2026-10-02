# SR_05 — pulse high time

**Test ID:** SR_05
**Goal:** pulse high time
**Program:** `QSEG 16 640 1`
**DUT:** 16000000 ticks/s, `MIN_CMD_TICKS` 3200, `QUEUE_LEN` 32, fastest legal 640 ticks
**Configuration:** 1ch on rmt_v2 (esp32)
**Tag:** `esp32_rmt_v2_1ch`
**Captured at:** 24000000 Hz
**Result:** **PASS**

## Measured

| stepper | steps | period us (min–max, spread) | pulse high us (min–max, spread) | duty % |
|---|---|---|---|---|
| A | 16 | 39.9167–40, 0.0833 | 15.5833–15.625, 0.0417 | 39.07 |

## Detail

```json
{
  "adherence": {
    "commanded_period_us": 40.0,
    "commanded_rate_hz": 25000.0,
    "jitter_pct": 0.208,
    "max_period_us": 40.0,
    "mean_period_us": 39.9639,
    "measurable": true,
    "min_period_us": 39.9167,
    "n_out_of_tolerance": 0,
    "ok": true,
    "periods_measured": 15,
    "rate_max_hz": 25052.19,
    "rate_mean_hz": 25022.59,
    "rate_min_hz": 25000.0,
    "sag_pct": -0.09,
    "tolerance_us": 0.8,
    "worst_deviation_us": 0.0833
  },
  "avg_high_us": 15.6146,
  "avg_low_us": 24.35,
  "duty_percent": 39.07,
  "expected_period_us": 40.0,
  "frequency_hz": 25022.59,
  "invariants": {
    "dir_while_step_high": {},
    "n_dir_while_step_high": 0,
    "ok": true
  },
  "min_pulse_high_us": 15.5833,
  "period": {
    "expected_period_us": 40.0,
    "long_periods_us": [],
    "n_long": 0,
    "n_short": 0,
    "ok": true,
    "periods_measured": 15,
    "short_periods_us": [],
    "tolerance_us": 2.0
  },
  "steps": {
    "extra_steps": 0,
    "missing_steps": 0,
    "ok": true,
    "steps_expected": 16,
    "steps_measured": 16
  },
  "ticks": 640,
  "ticks_per_s": 16000000
}
```

## Capture

`/tmp/cap/r8/SR_05.vcd`
