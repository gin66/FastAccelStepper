# SR_20 — 255, pause, 255 (large command on both sides)

**Test ID:** SR_20
**Goal:** 255, pause, 255 (large command on both sides)
**Program:** `QSEG 255 640 1 | QSEG 0 6400 1 | QSEG 255 640 1`
**DUT:** 16000000 ticks/s, `MIN_CMD_TICKS` 3200, `QUEUE_LEN` 32, fastest legal 640 ticks
**Configuration:** mcpwm on mcpwm_pcnt (esp32)
**Tag:** `esp32_mcpwm_pcnt_mcpwm`
**Captured at:** 24000000 Hz
**Result:** **PASS**

## Measured

| stepper | steps | period us (min–max, spread) | pulse high us (min–max, spread) | duty % |
|---|---|---|---|---|
| A | 510 | 39.9167–439.6667, 399.75 | 15.5417–15.625, 0.0833 | 38.31 |

## Detail

```json
{
  "expected_gap_us": 440.0,
  "invariants": {
    "dir_while_step_high": {},
    "n_dir_while_step_high": 0,
    "ok": true
  },
  "pause_found": true,
  "steps": {
    "extra_steps": 0,
    "missing_steps": 0,
    "ok": true,
    "steps_expected": 510,
    "steps_measured": 510
  }
}
```

## Capture

`/tmp/cap/rep/SR_20.vcd`
