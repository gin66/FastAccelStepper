# SR_18 — 255 steps, gap, exactly 1 (PCNT limit re-arm)

**Test ID:** SR_18
**Goal:** 255 steps, gap, exactly 1 (PCNT limit re-arm)
**Program:** `QSEG 255 640 1 | QSEG 0 6400 1 | QSEG 1 3200 1`
**DUT:** 16000000 ticks/s, `MIN_CMD_TICKS` 3200, `QUEUE_LEN` 32, fastest legal 640 ticks
**Configuration:** mcpwm on mcpwm_pcnt (esp32)
**Tag:** `esp32_mcpwm_pcnt_mcpwm`
**Captured at:** 24000000 Hz
**Result:** **PASS**

## Measured

| stepper | steps | period us (min–max, spread) | pulse high us (min–max, spread) | duty % |
|---|---|---|---|---|
| A | 256 | 39.9167–439.6667, 399.75 | 19.875–99.8333, 79.9583 | 48.35 |

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
    "steps_expected": 256,
    "steps_measured": 256
  }
}
```

## Capture

`/tmp/cap/r8/SR_18.vcd`
