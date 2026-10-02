# SR_26 — pause of exactly 65535 ticks (16-bit pause field)

**Test ID:** SR_26
**Goal:** pause of exactly 65535 ticks (16-bit pause field)
**Program:** `QSEG 1 65535 1 | QSEG 0 65535 1 | QSEG 1 65535 1`
**DUT:** 16000000 ticks/s, `MIN_CMD_TICKS` 3200, `QUEUE_LEN` 32, fastest legal 640 ticks
**Configuration:** 1ch on auto (esp32)
**Tag:** `esp32_auto_1ch`
**Captured at:** 24000000 Hz
**Result:** **PASS**

## Measured

| stepper | steps | period us (min–max, spread) | pulse high us (min–max, spread) | duty % |
|---|---|---|---|---|
| A | 2 | 8184.9583–8184.9583, 0 | 15.625–15.625, 0 | 0.19 |

## Detail

```json
{
  "expected_gap_us": 8191.875,
  "invariants": {
    "dir_while_step_high": {},
    "n_dir_while_step_high": 0,
    "ok": true
  },
  "measured_gaps_us": [
    8184.9583
  ],
  "pause_found": true,
  "pause_ticks": 65535,
  "pause_us": 4095.9375,
  "steps": {
    "extra_steps": 0,
    "missing_steps": 0,
    "ok": true,
    "steps_expected": 2,
    "steps_measured": 2
  },
  "ticks": 65535
}
```

## Capture

`/tmp/cap/rep/SR_26.vcd`
