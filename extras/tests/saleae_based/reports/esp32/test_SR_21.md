# SR_21 — long RMT run: no gap at a buffer split

**Test ID:** SR_21
**Goal:** long RMT run: no gap at a buffer split
**Program:** `QSEG 200 640 1`
**DUT:** 16000000 ticks/s, `MIN_CMD_TICKS` 3200, `QUEUE_LEN` 32, fastest legal 640 ticks
**Configuration:** 1ch on auto (esp32)
**Tag:** `esp32_auto_1ch`
**Captured at:** 24000000 Hz
**Result:** **PASS**

## Measured

| stepper | steps | period us (min–max, spread) | pulse high us (min–max, spread) | duty % |
|---|---|---|---|---|
| A | 200 | 39.9167–40, 0.0833 | 15.5417–15.625, 0.0833 | 39.06 |

## Detail

```json
{
  "adherence": {
    "commanded_period_us": 40.0,
    "commanded_rate_hz": 25000.0,
    "jitter_pct": 0.208,
    "max_period_us": 40.0,
    "mean_period_us": 39.9663,
    "measurable": true,
    "min_period_us": 39.9167,
    "n_out_of_tolerance": 0,
    "ok": true,
    "periods_measured": 199,
    "rate_max_hz": 25052.19,
    "rate_mean_hz": 25021.09,
    "rate_min_hz": 25000.0,
    "sag_pct": -0.084,
    "tolerance_us": 0.8,
    "worst_deviation_us": 0.0833
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
  "ticks": 640,
  "ticks_per_s": 16000000
}
```

## Capture

`/tmp/cap/rep/SR_21.vcd`
