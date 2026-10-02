# SR_04 — ticks = 65535 (16-bit max)

**Test ID:** SR_04
**Goal:** ticks = 65535 (16-bit max)
**Program:** `QSEG 4 65535 1`
**DUT:** 16000000 ticks/s, `MIN_CMD_TICKS` 3200, `QUEUE_LEN` 32, fastest legal 640 ticks
**Configuration:** 1ch on auto (esp32)
**Tag:** `esp32_auto_1ch`
**Captured at:** 24000000 Hz
**Result:** **PASS**

## Measured

| stepper | steps | period us (min–max, spread) | pulse high us (min–max, spread) | duty % |
|---|---|---|---|---|
| A | 4 | 4092.4583–4092.5, 0.0417 | 15.5833–15.625, 0.0417 | 0.38 |

## Detail

```json
{
  "invariants": {
    "dir_while_step_high": {},
    "n_dir_while_step_high": 0,
    "ok": true
  },
  "period": {
    "expected_period_us": 4095.9375,
    "long_periods_us": [],
    "n_long": 0,
    "n_short": 0,
    "ok": true,
    "periods_measured": 3,
    "short_periods_us": [],
    "tolerance_us": 204.796875
  },
  "steps": {
    "extra_steps": 0,
    "missing_steps": 0,
    "ok": true,
    "steps_expected": 4,
    "steps_measured": 4
  },
  "ticks": 65535
}
```

## Capture

`/tmp/cap/rep/SR_04.vcd`
