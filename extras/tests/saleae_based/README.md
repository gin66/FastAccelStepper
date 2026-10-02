# Saleae-Based Test Harness

Hardware-in-the-loop test harness for verifying stepper-motor signals on real
hardware using a Saleae (or sigrok-compatible) logic analyzer.

The harness is intentionally small: one shared, platform-independent test
module, thin Arduino/ESP-IDF entry points, and a capture + analysis script.
Today it implements **SR_00 (connection verification)**; the wider SR_01–SR_40
catalogue exists only as a design/roadmap in the white paper.

## Layout

```
saleae_based/
├── common/                          ← shared, platform-independent code
│   ├── saleae_test.h / .cpp         ← SR_00 test logic (uses the HAL)
│   ├── saleae_hal.h                 ← tiny gpio/millis/delay abstraction
│   ├── saleae_hal_arduino.cpp       ← HAL for Arduino (ESP32, Pico, ...)
│   └── saleae_hal_espidf.cpp        ← HAL for plain ESP-IDF
├── apps/
│   ├── arduino/saleae_app.ino       ← setup() / loop()
│   └── espidf/saleae_app.cpp        ← app_main()
├── scripts/
│   ├── capture.py                   ← reliable sigrok-cli capture (.sr + VCD)
│   └── analyze_csv.py               ← SR_00 evaluation
├── results/                         ← generated JSON results
├── README.md
└── white_paper_saleae_test_harness.md  ← design/roadmap (aspirational)
```

The same `common/saleae_test.cpp` is used by both entry points; only the HAL
and the entry point differ. `build-pio-dirs.sh` assembles the PlatformIO
projects from these sources using symlinks (nothing is copied, and the
generated `pio_dirs/` and `pio_espidf/` are git-ignored):

- `pio_dirs/saleae` — Arduino: `common/*` + `apps/arduino/*`
- `pio_espidf/saleae` — ESP-IDF: `common/*` + `apps/espidf/*`

## Prerequisites

- sigrok CLI (`brew install sigrok`) and a connected logic analyzer
- PlatformIO (`pio`)
- Python 3.8+

```bash
sigrok-cli --scan                 # detect analyzers
sigrok-cli -d fx2lafw --show      # device options / supported sample rates
```

## Running SR_00 (connection verification)

SR_00 toggles all 8 identification pins at exactly 1 Hz, each with a distinct
asymmetric duty (5 %..40 %), so a mis-wired or inverted channel is immediately
visible. Connect the analyzer channels to:

```
D0..D7 → GPIO 2, 0, 4, 16, 17, 5, 18, 19
```

Build and flash (Arduino framework):

```bash
bash extras/scripts/build-pio-dirs.sh
pio run -d pio_dirs/saleae -e esp32 -t upload --upload-port /dev/cu.usbserial-0001
```

Or build the plain ESP-IDF variant (no Arduino component needed):

```bash
pio run -d pio_espidf/saleae -e esp32_idf_V5_3_0 -t upload --upload-port /dev/cu.usbserial-0001
```

Capture and evaluate (single test, by hand):

```bash
cd extras/tests/saleae_based
python3 scripts/capture.py --sample-rate 1000000 --seconds 5 --output capture.sr --vcd
python3 scripts/analyze_csv.py capture.sr results/
```

### Test orchestration

`scripts/run_tests.py` runs the implemented tests for one hardware tag key
(`{arch}_{driver}_{channel_config}`, white paper §2.3.3) and records results:

```bash
python3 scripts/run_tests.py --list
python3 scripts/run_tests.py --tag-key esp32_idf5_conn_8ch_step_only
python3 scripts/run_tests.py --tag-key esp32_idf5_conn_8ch_step_only --force
```

Each test does **one capture** (start capture → trigger the test over serial →
wait for the capture to finish → analyze). It writes
`results/<tag_key>_<test>.json` and updates `results/tag_index.json`. Tests
already recorded `passed` for that tag key are **skipped** (unless `--force`),
so a hardware matrix can be resumed without re-measuring.

**SR_00 is the standard pre-check and always runs first** (even if you only ask
for a later test); it verifies all 8 I/Os are alive and correctly wired. If it
fails, the remaining tests are recorded `skipped`. Tests not yet implemented are
recorded `skipped (not implemented)`.

### Host control channel

The firmware exposes a newline text protocol over the serial console
(`common/saleae_app.cpp`); `scripts/control.py` sends commands:

```bash
python3 scripts/control.py --port /dev/cu.usbserial-0001 --send "STOP"
python3 scripts/control.py --port /dev/cu.usbserial-0001 --send "SR01 400 400" --read 2
```

| Command | Effect |
|---------|--------|
| `SR00` | start the SR_00 self-test (also runs on boot) |
| `SR01 <steps> <speed_us>` | constant-speed move; replies `DONE <pos>` |
| `POS` | reply `POS <position>` |
| `STOP` | stop move / self-test |

`run_tests.py` drives this itself for SR_01 (serial + capture + analysis):

```bash
python3 scripts/run_tests.py --tag-key esp32_conn_8ch_step_only --tests SR_01 \
    --steps 400 --speed-us 400 --seconds 2
```

### Unit tests

The signal parser and SR_00 evaluation are covered by hardware-free tests:

```bash
python3 -m unittest discover -s scripts/tests -v
```

RP2040/RP2350 use the same Arduino entry point. The CI envs are `rpipico` and
`rpipico2` (see `.github/workflows/build_arduino_examples_matrix.yml`); the same
GPIO map (2, 0, 4, 16, 17, 5, 18, 19) is valid on Pico:

```bash
pio run -d pio_dirs/saleae -e rpipico
pio run -d pio_dirs/saleae -e rpipico2
```

## Proven: ESP32 hardware pinning

**Status: verified correct (2026-10-01), re-verified after the shared-code
refactor.** Flashed to an ESP32-DevKitC and captured with a Saleae Logic
(fx2lafw) at 1 MHz. Every pin toggles at exactly 1 Hz with the intended duty,
proving the analyzer channel ↔ GPIO mapping below and that the
capture/analysis chain can be trusted.

| Saleae | GPIO | Expected duty | Measured duty | Freq | Glitches |
|--------|------|---------------|---------------|------|----------|
| D0 | GPIO 2  |  5 % |  5.0 % | 1.00 Hz | 0 |
| D1 | GPIO 0  | 10 % | 10.0 % | 1.00 Hz | 0 |
| D2 | GPIO 4  | 15 % | 15.0 % | 1.00 Hz | 0 |
| D3 | GPIO 16 | 20 % | 20.0 % | 1.00 Hz | 0 |
| D4 | GPIO 17 | 25 % | 25.0 % | 1.00 Hz | 0 |
| D5 | GPIO 5  | 30 % | 30.0 % | 1.00 Hz | 0 |
| D6 | GPIO 18 | 35 % | 35.0 % | 1.00 Hz | 0 |
| D7 | GPIO 19 | 40 % | 40.0 % | 1.00 Hz | 0 |

Result: `sr_00_passed: true` (see `results/`).

## Capture notes (sample-rate restrictions)

`sigrok-cli` takes the sample rate as a device option. Logic analyzers derive
their rates by dividing a fixed master clock, so only a discrete set of rates
is available. Query them with:

```bash
sigrok-cli -d fx2lafw --show      # lists supported samplerates
```

On the original 8-channel Saleae Logic (`fx2lafw`) this is 48 MHz / n:

```
20k 25k 50k 100k 200k 250k 500k 1M 2M 3M 4M 6M 8M 12M 16M 24M 48M
```

Each listed rate is produced exactly, but above a device-dependent rate/duration
the acquisition is truncated. Measured here with `--time` (8 channels, CSV output):

| rate | requested | captured samples | effective duration |
|------|-----------|------------------|--------------------|
| 1 MHz   | 10 s | 10,000,000 | 10.0 s |
| 500 kHz | 20 s | 10,000,000 | 20.0 s |
| 2 MHz   |  4 s |  5,099,520 |  2.55 s |
| 3 MHz   |  2 s |  2,235,392 |  0.75 s |
| 4 MHz   |  2 s |  2,224,640 |  0.56 s |
| 8 MHz   |  1 s |  3,215,360 |  0.40 s |
| 24 MHz  |  1 s |  8,164,352 |  0.34 s |

So low rates stream for the full requested time, while higher rates return a
capped number of samples regardless of `--time`. The cap is on the number of
samples, not on the rate: `--samples` is honoured exactly up to the cap
(e.g. at 4 MHz, 1,000,000 and 2,000,000 are exact; 3,000,000 returns
2,224,640). Use `--samples` for an exact, bounded acquisition.

- `--time <ms>` / `--time 2s` — sample for a duration
- `--samples 3m` — acquire an exact sample count (`k`/`m`/`g` suffixes)

The sigrok-cli docs additionally recommend capturing to a binary format at high
sample rates, because CSV output is expensive (sigrok-cli is single-threaded and
can terminate early). This harness records `.sr` (sigrok srzip — one packed
byte per sample) for that reason; `--format csv` is still available.

A `.sr` capture is evaluated through a VCD, which sigrok-cli derives from it
(`-I srzip -O vcd`, `--vcd` on `capture.py`). The VCD holds **only value
changes**, so it stays small and opens directly in GTKWave; sigrok picks
`$timescale` from the sample rate (1 us at 1 MHz, 100 ps at 48 MHz), so nothing
is lost. A 2 Msample 8-channel capture is 26 KB as `.sr` versus 80 MB as CSV
and 6 KB as VCD.

`scripts/capture.py` verifies the captured length and warns when it is shorter
than requested; pass `--strict` to make that a failure.

Reference: <https://sigrok.org/wiki/Sigrok-cli>

## Hardware Requirements

You need a board and a logic analyzer. The harness reads the step and dir pins,
which are plain MCU outputs, so the analyzer connects directly to them.

> **⚠ Do not connect a stepper, motor, or driver board.** The generated
> commands are synthetic probe patterns, not motor-safe motion: the fast ones
> command 25–40 kHz from standstill with a ~1 µs pulse, which a real stepper
> cannot follow. It would stall and overheat its driver, and the measurement —
> which is the pin signal — would be unchanged. Full reasoning in white paper
> §10.

| Item | Minimum | Recommended |
|------|---------|-------------|
| MCU Board | Any supported arch, powered over USB | ESP32-DevKitC, ESP32-S3-DevKitC |
| Logic Analyzer | 4 channels, 4 MS/s | 8+ channels, 24 MS/s |
| USB cable | For the serial console | — |

Two channels per stepper (step + dir), so one stepper needs 2, two need 4, and
four need all 8. Wiring and the full rationale are in white paper §10.

## Where things are

| | |
|---|---|
| Design reference | `white_paper_saleae_test_harness.md` — what is built, how, why |
| Task list and status | [`extras/todo/120_saleae_based_test_harness.md`](../../../todo/120_saleae_based_test_harness.md) — the only one |
| Test catalogue | white paper §5, `SR_00`–`SR_26` |
