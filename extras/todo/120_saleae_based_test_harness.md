# 120 Saleae-based test harness

## Goal

Build a PC-based / hardware test harness that uses a **Saleae Logic Analyzer**
to verify stepper-motor signal integrity at the pin level.  All eight channels
are used persistently, each bound to a fixed test-hardware role.

## Scope

### Core verification targets

| Area | What to check |
|------|---------------|
| **`addQueueEntry()` abrupt tick changes** | Signal does not glitch when the tick interval changes
sharply (e.g. max → min speed).  Verify step/dir edges remain clean,
no spuriously missed or doubled pulses. |
| **Dir/step pin timing** | Measure the exact time between a direction
change and the first step pulse after it.  Verify it matches the
documented `directionDelay` / platform defaults. |
| **Counting correctness** | Feed a known sequence of steps (forward,
reverse, mixed) and compare the Logic-analyzer-reconstructed count
against `getCurrentPosition()`. |
| **Runtime behaviour characterization** | Systematic measurement of: |
| | • Time from `dir` edge to first step pulse (tick-dependent?) |
| | • Step pulse length variation across tick values |
| | • Step high/low duty-cycle symmetry |
| | • Queue-fill latency under burst conditions |
| **Synchronized start (n-axis)** | Verify that multiple steppers start
their first step on the same timer tick (or within a documented
platform tolerance).  Covers every platform tracked in the 050-series
items (AVR, ESP32, Pico, SAM, SAMD51, Teensy).  Measures cross-channel
skew at the synchronized-start release point, and validates the engine
synchronized-start mechanism (050). |

### Channel assignment (example — configurable)

```
CH 0 — Step  (stepper A)
CH 1 — Dir   (stepper A)
CH 2 — Step  (stepper B)
CH 3 — Dir   (stepper B)
CH 4 — Step  (stepper C)
CH 5 — Dir   (stepper C)
CH 6 — Step  (stepper D)
CH 7 — Dir   (stepper D)
```

Additional trigger channels (e.g. queue-start, test-marker) can be added
as needed.

## Implementation plan

1. **Saleae bridge** — Python or C++ library to talk to Saleae Logic
   (either the CLI `saleae` or the C API).  Acquire persistent captures.
2. **Signal parser** — Reconstruct step/dir edges from raw capture data;
   compute timing metrics (pulse width, inter-step gaps, dir→step delay).
3. **Test scenarios** — Reproduce existing PC-based test cases (`test_01`
   through `test_17`) but with Saleae as the oracle.
4. **Synchronized-start tests** — Multi-stepper `synchronizedStart()`
   scenarios: verify first-step alignment across all channels, measure
   per-platform cross-channel skew, and validate the engine-level
   synchronized-start mechanism (see 050-series items).
5. **Reporting** — Generate a human-readable summary (CSV + HTML) with
   timing histograms and pass/fail per metric.

## Status

_idea — not started_
