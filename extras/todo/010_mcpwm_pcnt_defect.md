# 170 MCPWM/PCNT defect — two queues on ESP32

## Priority

**HIGH** — library defect: configuring two MCPWM/PCNT queues causes the
second stepper to emit continuously and never stop.  A user who connects two
steppers on this driver will get runaway motion.

## Finding

`CONFIG 2 mcpwm_pcnt,mcpwm_pcnt dir`, `QSEG 64 160 1`, `QRUN 3`:

- Stepper A emits **exactly 64 steps**.
- Stepper B emits **22 143 rising edges** at exactly the commanded 10 µs
  period (5.0 µs high, 5.0 µs low) and **never stops**.
- Board's own `POS` reads **non-monotonic** — `64 31`, `64 26`, `64 32`,
  `64 50` — all below 64, which says the *position counter* is being re-read
  while the pin runs on rather than that steps were lost.

## Scope

- It is **two MCPWM/PCNT queues**, not MCPWM alone and not `nodir`:
  - `rmt+rmt` → `64 64` every time — stable
  - `rmt+mcpwm_pcnt` → `64 64` every time — stable
  - `mcpwm_pcnt+rmt` (swapped) → `64 64` every time — stable
  - `mcpwm_pcnt+mcpwm_pcnt` → `64 31 / 64 26 / 64 32 / 64 50 / 64 34`
  - Fails at 3200 ticks too — **not a speed limit**

- Reproduced on **unmodified firmware** (stashed and re-flashed), so it is a
  library defect, not an artefact of the mode or the `nodir` change.
- Also fails with only stepper B selected (`QRUN 2`), which rules out
  synchronized kick-off as the trigger — **configuring a second MCPWM/PCNT
  queue is enough**, no `QRUN` needed.

## Where to look

`StepperISR_idf5_esp32_mcpwm_pcnt.cpp` indexes `channel2mapping[NUM_QUEUES]`
and `pcnt_unit_to_queue[QUEUES_MCPWM_PCNT]` by `channel_num` and sets
`pcnt_unit_id = timer_num`, while the ESP32 has **4 MCPWM timers** (2 groups
× 2) against **`QUEUES_MCPWM_PCNT` = 6**.  The second queue's timer/PCNT
assignment is the first thing to check.

## Consequence

`DRIVER_MAXS` records MCPWM/PCNT as 6 queues on IDF 5, from
`pd_config_idf5.h`, and **that number is now known to be wrong** — not in the
count but in what the driver can actually do.  Until it is fixed, any
MCPWM/PCNT row in the R5 tables is a measurement of a defect, not of the
driver.
