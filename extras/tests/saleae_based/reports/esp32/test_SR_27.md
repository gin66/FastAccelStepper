# SR_27 — single step in one command

**Test ID:** SR_27
**Goal:** single step in one command
**Program:** `QSEG 1 3200 1`
**DUT:** 16000000 ticks/s, `MIN_CMD_TICKS` 3200, `QUEUE_LEN` 32, fastest legal 640 ticks
**Configuration:** 1ch on rmt_v2 (esp32)
**Tag:** `esp32_rmt_v2_1ch`
**Captured at:** 24000000 Hz
**Result:** **PASS**

## Measured

| stepper | steps | period us (min–max, spread) | pulse high us (min–max, spread) | duty % |
|---|---|---|---|---|
| A | 1 | —–—, — | 15.625–15.625, 0 | 100 |

## Detail

```json
{
  "adherence": {
    "commanded_period_us": 200.0,
    "commanded_rate_hz": 5000.0,
    "measurable": false,
    "ok": true,
    "periods_measured": 0,
    "reason": "fewer than 2 inter-step periods: rate adherence cannot be measured for a single-step command"
  },
  "invariants": {
    "dir_while_step_high": {},
    "n_dir_while_step_high": 0,
    "ok": true
  },
  "period": {
    "expected_period_us": 200.0,
    "long_periods_us": [],
    "n_long": 0,
    "n_short": 0,
    "ok": true,
    "periods_measured": 0,
    "short_periods_us": [],
    "tolerance_us": 10.0
  },
  "steps": {
    "extra_steps": 0,
    "missing_steps": 0,
    "ok": true,
    "steps_expected": 1,
    "steps_measured": 1
  },
  "ticks": 3200,
  "ticks_per_s": 16000000
}
```

## Capture

`/tmp/cap/r8/SR_27.vcd`
