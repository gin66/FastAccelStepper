# SR_25 — STOP mid-run: pulses cease

**Test ID:** SR_25
**Goal:** STOP mid-run: pulses cease
**Program:** `QSEG 20000 640 1`
**DUT:** 16000000 ticks/s, `MIN_CMD_TICKS` 3200, `QUEUE_LEN` 32, fastest legal 640 ticks
**Configuration:** 1ch on rmt_v2 (esp32)
**Tag:** `esp32_rmt_v2_1ch`
**Captured at:** 24000000 Hz
**Result:** **PASS**

## Measured

| stepper | steps | period us (min–max, spread) | pulse high us (min–max, spread) | duty % |
|---|---|---|---|---|
| A | 11475 | 39.9167–40, 0.0833 | 15.5417–15.625, 0.0833 | 39.06 |

## Detail

```json
{
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
    "periods_measured": 11474,
    "short_periods_us": [],
    "tolerance_us": 2.0
  },
  "requested_steps": 20000,
  "steps": {
    "extra_steps": 0,
    "missing_steps": 0,
    "ok": true,
    "steps_expected": 11475,
    "steps_measured": 11475
  },
  "steps_before_stop": 11475,
  "stopped_early": true,
  "truncated": true,
  "unterminated_pulses": 0
}
```

## Capture

`/tmp/cap/r8/SR_25.vcd`
