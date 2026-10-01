# Saleae-based harness — agent guide

Hardware-in-the-loop verification of **stepper-motor signals at the pin level**,
using a Saleae / sigrok-compatible logic analyzer as the oracle. It complements
the PC-based (`extras/tests/pc_based`) and SimAVR tests: those validate the
algorithm, this validates what actually comes out of the driver on real silicon
(pulse shape, dir→step timing, glitch-free edges, cross-driver skew).

## Motivation / why it exists

Drivers (RMT, MCPWM+PCNT, I2S, PIO, AVR timer) and processors differ, and bugs
there are invisible to PC tests. The only trustworthy check is to capture the
step/dir pins and measure. `extras/todo/120_saleae_based_test_harness.md` is the
tracked backlog item; `white_paper_saleae_test_harness.md` is the full design
(SR_00–SR_40 catalogue, tag schema, reporting).

## Layout

```
common/   platform-independent test logic
  saleae_test.{h,cpp}      SR_00 self-test (8 pins, 1 Hz, asymmetric duty)
  saleae_app.{h,cpp}       host command protocol + channel configs + scenarios
  saleae_hal.h             gpio / millis / delay / serial abstraction
  saleae_hal_arduino.cpp   HAL for Arduino (ESP32, Pico, ...)
  saleae_hal_espidf.cpp    HAL for plain ESP-IDF
apps/     thin entry points (only call saleae_app_setup/loop)
  arduino/saleae_main.ino  setup()/loop()
  espidf/                  saleae_main.cpp (app_main) + CMakeLists.txt
scripts/
  harness.py               high-level front-end (arch/framework/driver -> env/tag)
  run_tests.py             orchestrator: capture + serial + analyze + record/skip
  control.py               send serial commands and read replies
  capture.py               reliable sigrok-cli wrapper (.sr capture, --vcd, rate/time)
  analyze_csv.py           SR_00 evaluation
  signal_parser.py         edges/metrics core (shared)
  tests/                   hardware-free unit tests
results/                   generated JSON results + tag_index.json (git-ignored)
capture.sr, capture.vcd     generated captures (git-ignored)
```

The **same `common/` code** is compiled by both the Arduino and the ESP-IDF
entry points; only the HAL and the entry differ. `build-pio-dirs.sh` assembles
`pio_dirs/saleae` (Arduino) and `pio_espidf/saleae` (ESP-IDF) from `common/` +
`apps/` using symlinks (the generated dirs are git-ignored; **never commit
symlinks**).

## How it runs

1. Build/flash: `build-pio-dirs.sh` then `pio run -d <dir> -e <env>`.
2. **One capture per test** (start capture → trigger the test over serial →
   wait for the capture to finish → analyze). The capture must start before the
   test and outlive it.
3. **SR_00 is the standard pre-check and always runs first**; if it fails the
   rest are recorded `skipped`. Results are keyed by a tag
   `{arch}_{framework}{ver}_{driver}_{channel_config}` so an already-`passed`
   test is skipped (resume a hardware matrix; `--force` re-measures).

## Commands

```bash
# unit tests (no hardware)
python3 -m unittest discover -s scripts/tests -v

# high-level: pick target + test; --flash builds+flashes first
python3 scripts/harness.py --arch esp32 --framework idf --version 5.3 \
    --driver mcpwm_pcnt --channel-config 4ch_rmt --tests SR_01 \
    --steps 4000 --speed-us 5 --flash
python3 scripts/harness.py --arch nanoatmega328 --driver timer \
    --tests SR_01 --speed-us 25 --flash          # 40 kSteps/s AVR
python3 scripts/harness.py --arch rpipico --driver pio \
    --tests SR_01 --speed-us 5 --flash           # 200 kHz Pico

# low-level (firmware already flashed; you supply the tag key)
python3 scripts/run_tests.py --tag-key esp32_idf5_3_0_mcpwm_pcnt_4ch_rmt \
    --tests SR_01 --steps 400 --speed-us 400

# manual serial
python3 scripts/control.py --send "STOP;CONFIG mixed rmt,mcpwm;MOVEALL 300 400" --read 2
```

## Capabilities (today)

- **SR_00** connection self-test: 8 pins, 1 Hz, distinct asymmetric duty so an
  inverted channel reads as the complement duty. Proven on ESP32 (Arduino and
  IDF5.3).
- **SR_01** basic move forward: constant-speed move, step pulses counted vs
  `steps`.
- **SR_17** synchronized start cross-driver: `mixed` config moving RMT + MCPWM
  together; reports per-channel step counts and cross-channel skew.
- Channel configs configurable over serial: `CONFIG 4ch_rmt`, `CONFIG 4ch_mcpwm`,
  `CONFIG mixed <drv0,drv1,...>` (drivers: `rmt`, `mcpwm`, `i2s`, `i2s_mux`,
  `auto`). `MOVEALL` moves all configured steppers.
- Serial protocol (`common/saleae_app.cpp`): `SR00`, `CONFIG`, `SR01`,
  `MOVEALL`, `POS`, `STOP`. Responses are `OK …` / `DONE …` / `ERR …`.

SR_02–SR_40 are not implemented (see the white paper).

## Target/driver notes

- ESP32: `stepperConnectToPin(pin, DRIVER_{RMT,MCPWM_PCNT,I2S_*})`
  (`SUPPORT_SELECT_DRIVER_TYPE`). I2S only on IDF5/6 (`SUPPORT_ESP32_I2S`).
  `i2s_mux` needs `engine.initI2sMux(...)`.
- AVR: Timer-based; the step pin must be timer-capable (Timer1 OC1A = pin 9).
- Pico: PIO; ordinary GPIOs.
- Max achievable speed is **processor- and driver-specific** (SR_11 discovers
  it; see white paper §7.1.1). Never assume a fixed value.

## Capture format

Record `.sr` (sigrok srzip, one packed byte per sample), never CSV: a 2 Msample
8-channel capture is 26 KB as `.sr` and 80 MB as CSV. `capture.py --vcd` derives
a VCD from it via `sigrok-cli -I srzip -O vcd`. A VCD contains only value
changes, so it is the compact, GTKWave-readable evaluation artifact; sigrok
picks `$timescale` from the sample rate (1 us at 1 MHz, 100 ps at 48 MHz — do
not assume a fixed or finer timescale). `signal_parser.load_vcd()` reads it
back, so a VCD may end before the last constant stretch of the capture.

## Sample-rate / capture gotchas

- Supported rates are `48 MHz / n` (`sigrok-cli -d fx2lafw --show`). Low rates
  (≤2 MHz) stream for the full `--time`; higher rates are **truncated** — e.g.
  4 MHz is capped at ~2.22 Msamples regardless of `--time`, so a 1 s request
  yields ~0.56 s. `capture.py` warns on this (`--strict` fails).
- For edge/step counting ~20 samples per step period is enough. For pulse-width
  / duty tests (SR_07/SR_08) the rate must resolve the **pulse width** (a few
  µs or less), i.e. MHz–tens of MHz — not just the period.
- **Acceleration matters**: a small `setAcceleration` makes a move far longer
  than `steps × speed_us`, so the capture window ends before the move does and
  step counts come up short. The app scales acceleration to the target speed
  (~0.1 s ramp) so moves are effectively constant-speed.

## Conventions

- Generated artifacts — `results/`, `capture*`, `pio_dirs/`, `pio_espidf/` — are
  git-ignored. Do not commit them.
- Symlinks are allowed only in the generated `pio_*` dirs; source in `common/`,
  `apps/`, `scripts/` must be real files.
- One capture per test; SR_00 always first.
- `src/` (the library) must not use 64-bit integers; this harness's Python may.
- Builds must keep working across the Arduino CI matrix and the ESP-IDF matrix
  (`pio_dirs/*`, `pio_espidf/*` are built for every env).

## Hardware pin map (ESP32-DevKitC)

```
Saleae D0..D7 -> GPIO 2, 0, 4, 16, 17, 5, 18, 19
4ch steppers : A step/dir 2/0, B 4/16, C 17/5, D 18/19
GPIO0 is a boot-strapping pin (must be HIGH at boot).
```

## References

- Design / roadmap: `white_paper_saleae_test_harness.md`
- Backlog item: `extras/todo/120_saleae_based_test_harness.md`
- User-facing overview: `README.md`
- Repo-wide rules: `AGENTS.md` at the repository root
- sigrok-cli: <https://sigrok.org/wiki/Sigrok-cli>
