# Saleae-Based Test Harness

Hardware-in-the-loop test harness for verifying stepper-motor signal integrity on real ESP32 (and other) hardware using a Saleae (or sigrok-compatible) logic analyzer.

## Quick Start

### Prerequisites
- Saleae Logic Analyzer (or sigrok-compatible USB logic analyzer)
- ESP32 development board (DevKitC, S3 DevKitC, etc.)
- sigrok CLI tools: `sigrok-cli`
- Python 3.8+ with libsigrok bindings (optional, for advanced analysis)

### Installation

```bash
# Install sigrok CLI (macOS)
brew install sigrok

# Install Python dependencies (optional, for advanced analysis)
pip install python-libsigrok4

# Verify sigrok detects your analyzer
sigrok-cli --driver saleae --list-devices
```

### Running SR_00 (Connection Verification)

SR_00 runs **before** any other test to verify wiring:

```bash
# 1. Build and flash firmware (ESP32 example)
cd extras/tests/saleae_based
./build-saleae.sh espidf

# 2. Connect Saleae channels to ESP32 GPIOs:
#    CH 0–7: Step/Dir pins (GPIO 2, 0, 4, 16, 17, 5, 18, 19)
#    CH 8: Test marker (GPIO 25 — toggled at startQueue)
#    CH 9: Queue empty (GPIO 26)

# 3. Run capture
python3 scripts/capture.py \
  --channels 0,1,2,3,4,5,6,7,8,9 \
  --sample-rate 1000000 \
  --trigger 'channel.8 rising' \
  --seconds 2

# 4. Analyze results
python3 scripts/analyze.py capture_*.sr --channels 0,1,2,3,4,5,6,7,8,9 --output results/
```

## Directory Structure

```
saleae_based/
├── white_paper_saleae_test_harness.md  ← Design document (renamed from 120_...)
├── README.md                           ← This file
├── tag_schema.json                     ← Tag schema definition
├── design_specs.json                   ← Design specs per platform/driver
├── build-saleae.sh                     ← Build script
├── scripts/
│   ├── capture.py                      ← sigrok-cli wrapper
│   ├── analyze.py                      ← Signal analyzer
│   └── config/
│       ├── channel_configs.py          ← Channel config presets
│       └── test_cases.py               ← Test case definitions
├── firmware/
│   ├── platformio.ini                  ← Arduino PlatformIO
│   ├── platformio_idf.ini              ← ESP-IDF PlatformIO
│   └── src/
│       ├── main.cpp                    ← Entry point
│       ├── serial_protocol.cpp         ← Command protocol
│       ├── command_executor.cpp        ← Queue command handler
│       └── serial_reporter.cpp         ← Metrics reporter
├── results/                            ← JSON results (generated)
├── capture/                            ← .sr capture files (generated)
└── reports/                            ← Markdown reports (generated)
```

## Test Cases

41 test cases (SR_00–SR_40) organized by category:

| Category | Tests | Description |
|----------|-------|-------------|
| Connection | SR_00 | Pin toggle sanity check (runs first) |
| Basic Ramp | SR_01–SR_05 | Forward/reverse, acceleration, multi-phase |
| Timing | SR_06–SR_12 | Direction delay, pulse width, duty cycle, boundaries |
| Synchronized | SR_13–SR_17 | Multi-stepper sync (same/delayed/different speeds) |
| Queue | SR_18–SR_24 | Queue full/empty, moveTimed, pause, drift |
| Driver | SR_25–SR_30 | RMT, MCPWM, I2S driver-specific tests |
| Stress | SR_31–SR_35 | Channel config stress tests |
| Edge | SR_36–SR_40 | Pin reuse, interrupt load, overflow, emergency stop |

## Proven: ESP32 hardware pinning

**Status: verified correct (2026-10-01).** The `simple_test.cpp` self-test was
flashed to an ESP32-DevKitC and captured with a Saleae Logic (fx2lafw) at
1 MHz. Every configured pin toggles at exactly 1 Hz with the intended duty,
proving that the analyzer channel ↔ GPIO mapping below is correct and that
`simple_test.cpp` and the capture/analysis chain can be trusted.

Firmware: `firmware/src/simple_test.cpp` (8 pins, all 1 Hz, asymmetric duty so
an inverted channel is detected as the complement duty).

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

Result: `sr_00_passed: true` in `results/20261001_193507_sr00_analysis.json`.

Reproduce:

```bash
# flash the self-test
bash extras/scripts/build-pio-dirs.sh
pio run -d pio_dirs/saleae_simple -e esp32 -t upload --upload-port /dev/cu.usbserial-0001

# capture (configurable rate/time) and evaluate
python3 scripts/capture.py --sample-rate 1000000 --seconds 5 --output capture.csv
python3 scripts/analyze_csv.py capture.csv results/
```

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
the acquisition is truncated. Measured here with `--time` (8 channels, CSV):

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

Two ways to bound an acquisition:

- `--time <ms>` / `--time 2s` — sample for a duration
- `--samples 3m` — acquire an exact sample count (`k`/`m`/`g` suffixes)

The sigrok-cli docs additionally recommend capturing to a binary format at high
sample rates, because CSV output is expensive (sigrok-cli is single-threaded and
can terminate early).

`scripts/capture.py` verifies the captured length and warns when it is shorter
than requested; pass `--strict` to make that a failure.

Reference: <https://sigrok.org/wiki/Sigrok-cli>

## Hardware Requirements

| Item | Minimum | Recommended |
|------|---------|-------------|
| Logic Analyzer | 8 channels, 100 MS/s | 16+ channels, 500 MS/s+ (Saleae Logic 8/Pro 8) |
| ESP32 Board | Any dev kit | ESP32-DevKitC, ESP32-S3-DevKitC |
| Stepper Drivers | A4988 / TMC2209 | TMC5160 (high-speed testing) |
| Power Supply | 12V stepper supply | Regulated, current-limited |

## Integration with Existing Tests

| Layer | Tool | Purpose |
|-------|------|---------|
| Unit tests | PC-based `test_XX` | Algorithm validation |
| Simulation | SimAVR `test_sd_*` | AVR-specific timing |
| Hardware validation | Saleae-based `SR_XX` | Real signal integrity |
| CI | SimAVR stub | Every commit (no hardware) |
| Release | Full Saleae suite | Release candidates |

## See Also

- **Design document**: `white_paper_saleae_test_harness.md` (complete architecture, test catalogue, implementation phases)