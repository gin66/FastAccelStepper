# SR_10 — dir change -> first step

**Test ID:** SR_10
**Goal:** dir change -> first step
**Program:** `QSEG 20 640 1 | QSEG 20 640 0`
**DUT:** 16000000 ticks/s, `MIN_CMD_TICKS` 3200, `QUEUE_LEN` 32, fastest legal 640 ticks
**Configuration:** 1ch on auto (esp32)
**Tag:** `esp32_auto_1ch`
**Captured at:** 24000000 Hz
**Result:** **PASS**

## Measured

| stepper | steps | period us (min–max, spread) | pulse high us (min–max, spread) | duty % |
|---|---|---|---|---|
| A | 40 | 39.9167–1538.6667, 1498.75 | 15.5833–15.625, 0.0417 | 19.91 |

## Detail

```json
{
  "below_capture_resolution": false,
  "capture_rate_hz": 24000000,
  "dir_edges": 1,
  "dir_to_first_step_min_us": 17.125,
  "dir_to_first_step_us": [
    17.125
  ],
  "invariants": {
    "dir_while_step_high": {},
    "n_dir_while_step_high": 0,
    "ok": true
  },
  "sample_us": 0.0417,
  "steps": {
    "extra_steps": 0,
    "missing_steps": 0,
    "ok": true,
    "steps_expected": 40,
    "steps_measured": 40
  }
}
```

## Capture

`/tmp/cap/rep/SR_10.vcd`
