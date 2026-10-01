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

1. **Saleae bridge** — `scripts/capture.py`: sigrok-cli wrapper, device
   auto-detect, configurable rate/time, output + duration verification.
   **Done.**
2. **Signal parser** — `scripts/signal_parser.py`: edges, pulse widths,
   inter-step period, duty, glitch count, step count, dir→step delay,
   cross-channel skew. `scripts/analyze_csv.py` holds the SR_00 expectations.
   Hardware-free unit tests in `scripts/tests/`. **Done.**
3. **Test scenarios** — Reproduce PC-based `test_01`–`test_17` with the
   analyzer as oracle.
   - [x] SR_00 connection verification (8 pins, 1 Hz, distinct duty) on
     ESP32 Arduino and ESP32 ESP-IDF; proven on hardware.
   - [x] Host→device control channel: newline text protocol (`SR00`, `SR01
     <steps> <speed_us>`, `POS`, `STOP`), FastAccelStepper integrated into both
     saleae pio dirs, host client `scripts/control.py`.
   - [x] SR_01 basic move forward: constant-speed move, step pulses counted on
     the analyzer vs `steps` (passes on ESP32 Arduino). Orchestrated end-to-end
     by `run_tests.py` (serial + capture + analysis).
   - [ ] SR_02–SR_40.
4. **Synchronized-start tests** — Multi-stepper `synchronizedStart()`: first
   step alignment, cross-channel skew, per-platform tolerance (feeds the
   050-series items). _Pending._
5. **Reporting** — CSV + markdown/HTML summary with spec comparison.
   _Pending._

## Orchestration

`scripts/run_tests.py` runs the implemented tests for a hardware tag key
(`{arch}_{driver}_{channel_config}`, white paper §2.3.3). It captures,
evaluates, writes `results/<tag_key>_<test>.json`, and maintains
`results/tag_index.json`. Each test does **one capture** (start capture →
trigger the test over serial → wait for the capture to finish → analyze). Tests
already recorded `passed` for a tag key are skipped unless `--force`; **SR_00 is
the standard pre-check and always runs first**, gating the rest; unimplemented
tests are recorded `skipped`. This makes a hardware matrix resumable.
**Done (SR_00, SR_01).**

## Status

_Prototype._ Shared test code (`common/` + `apps/`, Arduino + ESP-IDF),
reliable capture, signal parser, unit tests, and the orchestrator are in place;
SR_00 verified on ESP32 (Arduino and IDF5.3). SR_01–SR_40 not implemented.
