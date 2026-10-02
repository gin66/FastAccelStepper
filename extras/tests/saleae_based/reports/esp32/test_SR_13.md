# SR_13 — a command below MIN_CMD_TICKS must emit nothing

**Test ID:** SR_13
**Goal:** a command below MIN_CMD_TICKS must emit nothing
**Program:** `QSEG 8 399 1`
**DUT:** 16000000 ticks/s, `MIN_CMD_TICKS` 3200, `QUEUE_LEN` 32, fastest legal 640 ticks
**Configuration:** 1ch on auto (esp32)
**Tag:** `esp32_auto_1ch`
**Captured at:** 24000000 Hz
**Result:** **PASS**

## Measured

| stepper | steps | period us (min–max, spread) | pulse high us (min–max, spread) | duty % |
|---|---|---|---|---|
| A | 0 | —–—, — | —–—, — | 0 |

## Detail

```json
{
  "command_rate_ticks": 3192,
  "invariants": {
    "dir_while_step_high": {},
    "n_dir_while_step_high": 0,
    "ok": true
  },
  "min_cmd_ticks": 3200,
  "rejected_ticks": 399,
  "steps_measured": 0,
  "steps_that_would_have_been": 8
}
```

## Capture

`/tmp/cap/rep/SR_13.vcd`
