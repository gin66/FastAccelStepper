# SR_03 — at the speed floor

**Test ID:** SR_03
**Goal:** at the speed floor
**Program:** `QSEG 8 3200 1`
**DUT:** 16000000 ticks/s, `MIN_CMD_TICKS` 3200, `QUEUE_LEN` 32, fastest legal 640 ticks
**Configuration:** 1ch on auto (esp32)
**Tag:** `esp32_auto_1ch`
**Captured at:** 24000000 Hz
**Result:** **PASS**

## Measured

| stepper | steps | period us (min–max, spread) | pulse high us (min–max, spread) | duty % |
|---|---|---|---|---|
| A | 8 | 199.7917–199.875, 0.0833 | 15.5417–15.625, 0.0833 | 7.81 |

## Detail

```json
{
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
    "periods_measured": 7,
    "short_periods_us": [],
    "tolerance_us": 10.0
  },
  "steps": {
    "extra_steps": 0,
    "missing_steps": 0,
    "ok": true,
    "steps_expected": 8,
    "steps_measured": 8
  },
  "ticks": 3200
}
```

## Capture

`/tmp/cap/rep/SR_03.vcd`
