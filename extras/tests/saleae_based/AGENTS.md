# Saleae-based harness — agent guide

Hardware-in-the-loop verification of the **step and dir pins** using a Saleae /
sigrok-compatible logic analyzer as the oracle. It complements the PC-based
(`extras/tests/pc_based`) and SimAVR tests: those validate the algorithm, this
validates what actually comes out of the MCU on real silicon (pulse shape,
dir→step timing, spurious or swallowed steps, cross-stepper skew, and how
closely the emitted step rate follows the commanded one).

The analyzer connects straight to the step and dir pins, which are ordinary
push-pull outputs. **⚠ Do not connect a stepper, motor, or driver board**: the
generated commands are synthetic probe patterns (25–40 kHz from standstill,
~1 µs pulses) and are not motor-safe. Reasoning in white paper §10.

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
  report.py                markdown + csv view of a run; mode tables
  analyze_csv.py           SR_00 evaluation
  signal_parser.py         edges/metrics core (shared)
  tests/                   hardware-free unit tests
results/                   generated JSON results + tag_index.json (git-ignored)
capture.sr, capture.vcd     generated captures (git-ignored)
```

The firmware is a thin **addQueueEntry() driver**, not a test library: it holds
a small segment program and feeds it into the queue. See "What is under test"
below.

The **same `common/` code** is compiled by both the Arduino and the ESP-IDF
entry points; only the HAL and the entry differ. Each HAL compiles to nothing on
the platform it does not serve, so both can be linked into every build.

`apps/arduino/platformio.ini` is self-contained. `scripts/link_app.sh` creates
the generated symlinks it needs — the `common/` sources into `src/`, and the
library into `FastAccelStepper/` — so there is one copy of the firmware. Those
symlinks and `.pio/` are git-ignored; **never commit symlinks**.

## How it runs

1. Build/flash:
   ```bash
   bash extras/tests/saleae_based/scripts/link_app.sh
   pio run -d extras/tests/saleae_based/apps/arduino -e saleae_avr
   pio run -d extras/tests/saleae_based/apps/arduino -e saleae_esp32
   ```
   `saleae_avr` (328P) is the interesting one to build first: it is the
   tightest target for RAM, and its step pins are the Timer1 compare outputs, so
   it is where a pin-map mistake shows up.
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
    --driver mcpwm_pcnt --count 2 --tests SR_01 --flash
python3 scripts/harness.py --arch nanoatmega328 --driver timer --count 1 --flash
python3 scripts/harness.py --arch rpipico --driver pio --count 2 --flash

# two steppers on two different drivers
python3 scripts/harness.py --arch esp32 --drivers rmt,mcpwm_pcnt --count 2 \
    --tests SR_17 --flash

# the two generic modes (todo R3). Neither names an architecture; --arch and
# --driver are tags on the run. --dry-run prints the whole plan first.
python3 scripts/harness.py --mode scale --driver rmt_v2 --pin-mode nodir --dry-run
python3 scripts/harness.py --mode scale --driver rmt_v2 --pin-mode nodir --flash
python3 scripts/harness.py --mode sync --arch esp32 --dry-run
python3 scripts/harness.py --mode sync --arch esp32 --speed-us 5 --flash

# the mux, once it is wired: three pins on existing firmware, no rebuild
python3 scripts/harness.py --mode scale --arch esp32 --driver i2s_mux \
    --imux 12,13,14 --pin-mode dir --flash

# low-level (firmware already flashed; you supply the tag key)
python3 scripts/run_tests.py --tag-key esp32_idf5_3_0_mcpwm_pcnt_2ch --tests SR_01

# report: catalogue rows from VCDs, mode tables from result JSON.
# A mode run records a result and NO capture, so --results-dir is where the
# parallel-count and sync tables come from.
python3 scripts/report.py /tmp/cap/hw --results-dir /tmp/results

# manual serial: read the limits, then run a program
python3 scripts/control.py --send "CONFIG 1 timer" --read 2
python3 scripts/control.py --send "QINFO" --read 1
python3 scripts/control.py --send "QCLR;QSEG 255 80 1;QSEG 0 1600 1;QSEG 1 80 1;QRUN 1" --read 6
```

## What is under test

**`addQueueEntry()` and nothing else.** This harness exists to characterize
the queue layer at the pin level: the step/dir waveform that actually comes
out of `addQueueEntry()` plus the driver. It deliberately does **not** use the
ramp generator (`move()`) or `moveTimed()` — those are covered by the PC-based
and SimAVR tests, and a logic analyzer adds nothing to them. Any SR test that
only exercises `move()`/`moveTimed()` belongs in `pc_based`, not here.

What a Saleae capture uniquely adds over the other suites is the **measured
waveform**: pulse high time, dir→first-step delay, and inter-step jitter at
high speed, on real silicon.

### Firmware protocol (the whole surface)

The firmware builds queue commands itself. 115200 baud needs ~0.3 s for a few
hundred commands and the stepper would move long before the last one arrives,
so streaming a plan over serial is not an option; and a large static plan does
not fit in AVR RAM. Instead the host sends one short line per segment and the
firmware keeps a bounded program of at most `QE_MAX_SEG` (8) segments — 48 B,
shared once by all steppers — plus an 8 B cursor per stepper. The whole
scenario cost is ~50 B regardless of length, and sweeping a parameter needs no
recompile.

```
SR00                        SR_00 pin self-test (8 pins, 1 Hz)
CONFIG <n> <drv>[,<drv>..] [dir|nodir]
                            connect <n> steppers, one driver NAMED per stepper.
                            There is no `auto`: an unknown driver, an absent
                            one, a list whose length is not <n>, a count this
                            platform cannot provide and an unimplemented pin
                            mode are all REFUSED, never substituted.
                            The mode is not cosmetic. The analyzer has 8
                            channels; `dir` spends 2 per stepper (step, dir) and
                            `nodir` 1, so `dir` reaches 4 steppers and `nodir`
                            reaches 8. The cap is min(platform stepper queues,
                            channels/stride) and a refusal names both bounds.
                            In `nodir` no direction pin is connected at all, so
                            the QSEG direction argument still parses but is
                            forced true -- there is no pin to toggle for a
                            false, and the queue would refuse it.
MAP                         count, mode, stride, and the GPIO behind each
                            reachable channel. The host MUST read this rather
                            than assume a channel map: in `dir` stepper B is D2,
                            in `nodir` it is D1, and a host that guesses reads a
                            quiet pin and reports a driver that emits nothing.
                            The map is passed to each evaluator as a `Pins`
                            object; there is no module-level channel table. A
                            capture lacking a stepper the board connected is an
                            *incomplete capture* and fails the run -- it is not
                            reported as a quiet stepper, and not passed.
MARK <ch> | none             designate an analyzer channel no stepper owns as
                            the event marker: the firmware flips its level when
                            it processes a stop, putting the stop instant on the
                            waveform instead of leaving it to be inferred from
                            "the pulses ceased". That inference cannot work
                            here -- the capture delivered is not the capture
                            requested (24 MHz truncates) -- so the two cannot be
                            told apart. MAP reports marker=. At 8 steppers in
                            `nodir` there is no free channel and MARK is refused.
                            Send it in the setup phase, never after QRUN: its
                            serial round-trips would otherwise land between the
                            move starting and the stop.
STOP | ESTOP                STOP is `stopMove()` and MUST NOT truncate already
                            queued motion -- a run that keeps stepping after it
                            is the contract holding, not a stop failing. ESTOP is
                            `forceStop()`: nothing further is added, the queue
                            drains. A third API, `forceStopAndNewPosition()`,
                            aborts everything queued. SR_25 and SR_29 send the
                            same program and assert opposite outcomes.
QINFO                       tps, MIN_CMD_TICKS, QUEUE_LEN, maxall (the
                            LARGEST per-stepper speed floor -- the fastest
                            period legal for every connected stepper, and what
                            a shared program is planned against), then
                            maxspeed0..N for each stepper's own. Printed with
                            maxall FIRST so a buffer overrun cannot truncate
                            the one field the host cannot reconstruct. Reading
                            only the first stepper's floor plans too fast
                            whenever a later stepper is slower.
QCLR                        drop the program and stop
QSEG <steps> <ticks> <dir>  append a segment; steps=0 means "pause <ticks>"
QSEG <idx> <steps> <ticks> <dir>
                            the same, into stepper <idx>'s own program, for two
                            steppers at different speeds (SR_15)
QRUN <mask>                 run on the steppers in the bitmask, synced start
POS | STOP
```

Driver names: `rmt` | `rmt_v2` | `mcpwm` | `mcpwm_pcnt` | `i2s` | `i2s_direct` |
`i2s_mux` on the ESP32 family, `timer` on AVR/SAM/SAMD, `pio` on Pico. A driver
the running build has no queues for is refused — `CONFIG 2 rmt,rmt` on a 328P
does not quietly give you two timer queues. The list is explicit on every
architecture precisely because it costs nothing to name a single native driver
and a result that does not record which driver produced it characterizes
nothing. An earlier revision let `1ch`/`2ch` fall back to the library's automatic
choice and tagged 17 of 25 results `auto`, which recorded whatever the firmware
happened to pick.

`ticks` is the **raw queue period in timer ticks, not microseconds**, so the
host can address the 16-bit boundaries exactly (1 and 65535). Always read
`QINFO` first and use its values; never hardcode a tick rate.

`QRUN <mask>` is the multi-stepper selector: `QRUN 1` is one stepper, `QRUN 3`
is steppers A and B together (synchronized start).

Example — 255 steps at max speed, a pause, then a single step, which is the
case that exposes MCPWM/PCNT counter-limit overrun handling:

```
QCLR
QSEG 255 80 1      # ticks = the max-speed floor from QINFO
QSEG 0 1600 1      # pause
QSEG 1 80 1        # single step after the pause
QRUN 1
```

## Characterization scenarios

Every scenario is a handful of `QSEG` lines. The ones that matter, by goal:

| Goal | Program | What the capture must show |
|------|---------|---------------------------|
| Pulse high time / duty vs speed | `QSEG 1 <ticks> 1`, sweep `ticks` | high time shrinks as ticks drop; no pulse merges |
| 1..255 steps per command | `QSEG <n> <ticks> 1`, n = 1..255 | exactly n pulses, inter-step period = ticks, no glitch at the last step |
| `ticks` = 65535 | `QSEG 1 65535 1` | longest period the 16-bit field allows |
| dir change → first step | `QSEG 10 <ticks> 1` then `QSEG 10 <ticks> 0` | dir edge settles before the first step of phase 2; measure the delay |
| Pause command | `QSEG 5 <ticks> 1`, `QSEG 0 <p> 1`, `QSEG 5 <ticks> 1` | gap of exactly p ticks, dir must not flip on a pause |
| MCPWM overrun | `QSEG 255 <max> 1`, `QSEG 0 <p> 1`, `QSEG 1 <max> 1` | 255 pulses, gap, then exactly 1 — no lost or extra step |
| Multi-stepper max speed | `CONFIG 2 <d>,<d> dir`, `QSEG <n> <max> 1`, `QRUN 3` | does the second stepper perturb the first step's timing |
| Sync start | `CONFIG 2 <d>,<d> dir`, `QSEG <n> <ticks> 1`, `QRUN 3` | time from kick-off to each stepper's first step; all within tolerance |

The last two interact on AVR: `max_speed_in_ticks` is
`TICKS_PER_S/50000` with one stepper but 426 with two
(`adjustSpeedToStepperCount`, `src/pd_avr/avr_queue.cpp`), so one and two
steppers (`--count 1` and `--count 2`) must both be characterized on the same
board.

## Capabilities (today)

- **SR_00** connection self-test: 8 pins, 1 Hz, distinct asymmetric duty so an
  inverted channel reads as the complement duty. Proven on ESP32 (Arduino and
  IDF5.3). Runs first; if it fails the rest are skipped.
- The `QSEG`/`QRUN` feeder itself: segment program, prefill, synchronized
  kick-off, DIR-pause retry, pause commands. Implemented in
  `common/saleae_app.cpp` (`qe_pump`/`qe_feed`).
- **The two generic modes** (`--mode scale|sync`, todo R3). `scale` sweeps the
  stepper count 1..`min(driver queues, channel budget)` on one named driver with
  a **shared** program, asserting each stepper's own step count and period;
  `sync` runs **every driver-list combination** with each stepper at **its own**
  period, reporting first-step skew (µs *and* step periods) and asserting
  adherence per stepper. A refused point is recorded and the plan continues.
- Characterization scenarios above are runnable by hand with `control.py`.

### Known defect found by `--mode scale`

**Two MCPWM/PCNT queues on one ESP32 do not run**: the second stepper emits
continuously and never stops (22 143 edges where 64 were commanded, at exactly
the commanded period), and `POS` reads non-monotonic. Reproduces on unmodified
firmware, in both `dir` and `nodir`, at any speed, with only stepper B selected.
`rmt+mcpwm_pcnt` and `rmt+rmt` are both fine. Suspect the `channel_num` /
`pcnt_unit_id` indexing in `src/pd_esp32/StepperISR_idf5_esp32_mcpwm_pcnt.cpp`
against 4 MCPWM timers vs `QUEUES_MCPWM_PCNT` = 6. Until fixed, MCPWM/PCNT
multi-stepper results characterize this bug, not the driver.

## Target/driver notes## Capture format

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
- A scenario's duration is exactly `steps × ticks / TICKS_PER_S` (plus the
  trailing `ticks` of the last command), because the feeder drives
  `addQueueEntry()` directly — there is no ramp to stretch the move. Size the
  capture window from that.

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
