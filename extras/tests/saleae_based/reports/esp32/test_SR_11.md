# SR_11 — reverse then forward: the drain is symmetric

**Test ID:** SR_11
**Goal:** reverse then forward: the drain is symmetric
**Program:** `QSEG 20 640 0 | QSEG 20 640 1`
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
  "dir_edges": 2,
  "expected_final_dir": 1,
  "expected_steps": [
    20,
    20
  ],
  "final_dir": 1,
  "invariants": {
    "dir_while_step_high": {},
    "n_dir_while_step_high": 0,
    "ok": true
  },
  "phases": 2,
  "phases_expected": 2,
  "steps": {
    "extra_steps": 0,
    "missing_steps": 0,
    "ok": true,
    "steps_expected": 40,
    "steps_measured": 40
  },
  "steps_per_phase": [
    20,
    20
  ]
}
```

## Capture

`/tmp/cap/rep/SR_11.vcd`
