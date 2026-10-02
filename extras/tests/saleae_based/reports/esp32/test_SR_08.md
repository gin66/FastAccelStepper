# SR_08 — 4000 steps, QueueFull retry

**Test ID:** SR_08
**Goal:** 4000 steps, QueueFull retry
**Program:** `QSEG 4000 640 1`
**DUT:** 16000000 ticks/s, `MIN_CMD_TICKS` 3200, `QUEUE_LEN` 32, fastest legal 640 ticks
**Configuration:** 1ch on auto (esp32)
**Tag:** `esp32_auto_1ch`
**Captured at:** 24000000 Hz
**Result:** **PASS**

## Measured

| stepper | steps | period us (min–max, spread) | pulse high us (min–max, spread) | duty % |
|---|---|---|---|---|
| A | 4000 | 39.9167–40, 0.0833 | 15.5417–15.625, 0.0833 | 39.06 |

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
    "periods_measured": 3999,
    "short_periods_us": [],
    "tolerance_us": 2.0
  },
  "steps": {
    "extra_steps": 0,
    "missing_steps": 0,
    "ok": true,
    "steps_expected": 4000,
    "steps_measured": 4000
  },
  "ticks": 640
}
```

## Capture

`/tmp/cap/rep/SR_08.vcd`
