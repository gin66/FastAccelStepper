# SR_06 — trailing wait after last step

**Test ID:** SR_06
**Goal:** trailing wait after last step
**Program:** `QSEG 2 1600 1 | QSEG 2 1600 1`
**DUT:** 16000000 ticks/s, `MIN_CMD_TICKS` 3200, `QUEUE_LEN` 32, fastest legal 640 ticks
**Configuration:** 1ch on auto (esp32)
**Tag:** `esp32_auto_1ch`
**Captured at:** 24000000 Hz
**Result:** **PASS**

## Measured

| stepper | steps | period us (min–max, spread) | pulse high us (min–max, spread) | duty % |
|---|---|---|---|---|
| A | 4 | 99.875–99.9167, 0.0417 | 15.625–15.625, 0 | 15.64 |

## Detail

```json
{
  "invariants": {
    "dir_while_step_high": {},
    "n_dir_while_step_high": 0,
    "ok": true
  },
  "period": {
    "expected_period_us": 100.0,
    "long_periods_us": [],
    "n_long": 0,
    "n_short": 0,
    "ok": true,
    "periods_measured": 3,
    "short_periods_us": [],
    "tolerance_us": 5.0
  },
  "steps": {
    "extra_steps": 0,
    "missing_steps": 0,
    "ok": true,
    "steps_expected": 4,
    "steps_measured": 4
  },
  "ticks": 1600
}
```

## Capture

`/tmp/cap/rep/SR_06.vcd`
