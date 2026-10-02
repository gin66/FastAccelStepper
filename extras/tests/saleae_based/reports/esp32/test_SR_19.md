# SR_19 — 200 steps then a single step at the boundary

**Test ID:** SR_19
**Goal:** 200 steps then a single step at the boundary
**Program:** `QSEG 200 640 1 | QSEG 1 3200 1`
**DUT:** 16000000 ticks/s, `MIN_CMD_TICKS` 3200, `QUEUE_LEN` 32, fastest legal 640 ticks
**Configuration:** mcpwm on mcpwm_pcnt (esp32)
**Tag:** `esp32_mcpwm_pcnt_mcpwm`
**Captured at:** 24000000 Hz
**Result:** **PASS**

## Measured

| stepper | steps | period us (min–max, spread) | pulse high us (min–max, spread) | duty % |
|---|---|---|---|---|
| A | 201 | 39.9167–40, 0.0833 | 15.5417–15.625, 0.0833 | 39.06 |

## Detail

```json
{
  "expected_gap_us": null,
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
    "steps_expected": 201,
    "steps_measured": 201
  }
}
```

## Capture

`/tmp/cap/rep/SR_19.vcd`
