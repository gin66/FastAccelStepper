# SR_09 — pause command

**Test ID:** SR_09
**Goal:** pause command
**Program:** `QSEG 5 640 1 | QSEG 0 12800 1 | QSEG 5 640 1`
**DUT:** 16000000 ticks/s, `MIN_CMD_TICKS` 3200, `QUEUE_LEN` 32, fastest legal 640 ticks
**Configuration:** 1ch on auto (esp32)
**Tag:** `esp32_auto_1ch`
**Captured at:** 24000000 Hz
**Result:** **PASS**

## Measured

| stepper | steps | period us (min–max, spread) | pulse high us (min–max, spread) | duty % |
|---|---|---|---|---|
| A | 10 | 39.9583–839.2917, 799.3333 | 15.5833–15.625, 0.0417 | 12.13 |

## Detail

```json
{
  "expected_gap_us": 840.0,
  "invariants": {
    "dir_while_step_high": {},
    "n_dir_while_step_high": 0,
    "ok": true
  },
  "measured_gaps_us": [
    39.9583,
    40.0,
    39.9583,
    39.9583,
    839.2917,
    39.9583,
    39.9583,
    40.0
  ],
  "pause_found": true,
  "pause_ticks": 12800,
  "pause_us": 800.0,
  "steps": {
    "extra_steps": 0,
    "missing_steps": 0,
    "ok": true,
    "steps_expected": 10,
    "steps_measured": 10
  },
  "ticks": 640
}
```

## Capture

`/tmp/cap/rep/SR_09.vcd`
