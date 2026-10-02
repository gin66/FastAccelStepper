# SR_12 — forward, reverse, forward: two direction changes

**Test ID:** SR_12
**Goal:** forward, reverse, forward: two direction changes
**Program:** `QSEG 10 640 1 | QSEG 10 640 0 | QSEG 10 640 1`
**DUT:** 16000000 ticks/s, `MIN_CMD_TICKS` 3200, `QUEUE_LEN` 32, fastest legal 640 ticks
**Configuration:** 1ch on auto (esp32)
**Tag:** `esp32_auto_1ch`
**Captured at:** 24000000 Hz
**Result:** **PASS**

## Measured

| stepper | steps | period us (min–max, spread) | pulse high us (min–max, spread) | duty % |
|---|---|---|---|---|
| A | 30 | 39.9583–1538.7083, 1498.75 | 15.5833–15.625, 0.0417 | 10.89 |

## Detail

```json
{
  "dir_edges": 2,
  "expected_final_dir": 1,
  "expected_steps": [
    10,
    10,
    10
  ],
  "final_dir": 1,
  "invariants": {
    "dir_while_step_high": {},
    "n_dir_while_step_high": 0,
    "ok": true
  },
  "phases": 3,
  "phases_expected": 3,
  "steps": {
    "extra_steps": 0,
    "missing_steps": 0,
    "ok": true,
    "steps_expected": 30,
    "steps_measured": 30
  },
  "steps_per_phase": [
    10,
    10,
    10
  ]
}
```

## Capture

`/tmp/cap/rep/SR_12.vcd`
