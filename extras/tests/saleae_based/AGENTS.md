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
  i2s_mux_decoder.py       8-channel VCD -> 37-channel VCD (ESP32 I2S mux)
  probe_mux_bits.py        one-off: measures the I2S wire protocol on hardware
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
python3 scripts/harness.py --mode scale --driver rmt --pin-mode nodir --dry-run
python3 scripts/harness.py --mode scale --driver rmt --pin-mode nodir --flash
python3 scripts/harness.py --mode sync --arch esp32 --dry-run
python3 scripts/harness.py --mode sync --arch esp32 --speed-us 5 --flash

# the mux: 32 multiplexed steppers on 3 wires, 8-channel analyzer. --imux is a
# flag, not a pin list -- the bus IS channels D5/D6/D7 (see "The I2S mux" below).
python3 scripts/harness.py --mode scale --arch esp32 --driver i2s_mux \
    --pin-mode nodir --imux --flash

# low-level (firmware already flashed; you supply the tag key)
python3 scripts/run_tests.py --tag-key esp32_idf5_3_0_mcpwm_pcnt_2ch --tests SR_01

# the whole platform-release matrix: 6 firmwares, one flash each, then the
# catalogue + one scale sweep per accepted driver + all 10 sync combinations
# against that flash. Writes reports/esp32_platform_matrix.md.
python3 scripts/run_matrix.py --plan          # the plan, no build/flash/capture
python3 scripts/run_matrix.py                 # all of RELEASE_MATRIX
python3 scripts/run_matrix.py --targets idf-6.13.0   # one row; others are carried

# rebuild the committed report from the results already on disk: no build,
# flash, capture or serial port. The classification below is re-applied to the
# recorded results, so a change to it needs no hardware.
python3 scripts/run_matrix.py --report-only

# Known system limitations. A measured `failed` that is a target working as
# designed is scoped to an architecture family + driver and recognised by shape
# in scripts/known_limitations.py -- the MCPWM/PCNT overrun is the one that
# exists (one extra step on the last command; see
# extras/doc/platforms/esp32.md#mcpwm-pcnt-overrun). `run_matrix.classify()`
# returns `LIMITATION` for it, so it is counted and named under "Known
# limitations" rather than listed as a finding; `report.py` reads the same
# registry. Add a limitation there, not in the report text or in classify().

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
PING                        liveness probe; replies "OK PING". How the host
                            identifies a native-USB board (RP2040/RP2350) whose
                            one-shot READY at boot is gone by the time the host
                            reconnects -- there is no port-open reset there.
RESET                       full chip reset (RP2040/RP2350 only), then reboots;
                            see "Native USB needs a protocol reset" below.
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
MAP                         count, mode, stride, the GPIO behind each reachable
                            channel, plus `bus=` (the three I2S channels, or
                            `-`), `slots=` (one entry per **stepper**: the bit
                            of the 32-bit word that stepper's STEP signal is, or
                            `-` for a GPIO stepper -- verified on hardware, not
                            inferred) and `dslots=` (the same indexing, holding
                            that stepper's DIRECTION bit). Read either as one per
                            channel and every stepper past the first in `dir` is
                            handed a GPIO channel it does not own, which measures
                            0 steps on a quiet pin. `dslots` is reported rather
                            than derived: the host used to compute it as
                            `step_slot + 1`, which held only because CONFIG
                            resets the slot cursor and connects in order, so the
                            pairs came out gapless (0/1, 2/3, ...). A host that
                            infers it is right by coincidence; a reply with no
                            `dslots` field is refused, naming the reflashing.
                            The host MUST read the map rather than assume a
                            channel map: in `dir` stepper B is D2, in `nodir` it
                            is D1, and a host that guesses reads a quiet pin and
                            reports a driver that emits nothing. A multiplexed
                            stepper is on NO channel at all, so its entry is
                            `S<slot>` -- a name that exists only in the decoded
                            capture. The map is passed to each evaluator as a
                            `Pins` object; there is no module-level channel
                            table. A capture lacking a stepper the board
                            connected is an *incomplete capture* and fails the
                            run -- it is not reported as a quiet stepper, and
                            not passed.
MARK <ch> | none             designate an analyzer channel no stepper owns as
                            the event marker: the firmware flips its level when
                            it processes a stop, putting the stop instant on the
                            waveform instead of leaving it to be inferred from
                            "the pulses ceased". That inference cannot work
                            here -- at 48 MS/s the capture delivered is 0.18 ms of
                            the capture requested -- so the two cannot be told
                            apart. MAP reports marker=. At 8 steppers in
                            `nodir` there is no free channel and MARK is refused.
                            Send it in the setup phase, never after QRUN: its
                            serial round-trips would otherwise land between the
                            move starting and the stop.
STOP | XSTOP                The library has three stops, differing in *how*
                            they stop and *what happens to the position*:
                            stopMove() decelerates normally; forceStop() stops
                            abruptly but lets the queue run out, so the
                            position is kept; forceStopAndNewPosition() stops
                            as fast as the hardware allows and empties the
                            queue, so the position is lost and the caller
                            supplies it. They are not three strengths of one
                            operation.
                            STOP is `stopMove()` and MUST NOT truncate already
                            queued motion -- a run that keeps stepping after it
                            is the contract holding, not a stop failing. XSTOP is
                            `forceStopAndNewPosition()`: the queue is emptied, so
                            the queued commands never run. SR_25 and SR_30 send
                            the same program and assert opposite outcomes.
                            Neither of the other two stops has a scenario, and
                            the reason is a limit of this harness rather than a
                            judgement about them: `stopMove()` is a flag the ramp
                            generator reads, and this harness drives
                            addQueueEntry() directly, running no ramp for it to
                            act on. `forceStop()`'s only effect on a
                            harness-filled queue is the admission latch, which
                            refuses *later* addQueueEntry() calls -- and this
                            harness stops feeding once the fill is in, so there
                            are none for it to refuse.
                            That latch is what XSTOP sets too, so `QCLR` rearms
                            it (`resumeCommands()`): without that, a
                            `XSTOP` -> `QCLR` -> `QSEG` -> `QRUN` on one
                            connection queues nothing while reporting
                            success. `CONFIG` also rearms, via the
                            `_initVars()` memset -- which is why every
                            scenario used to pass.
QFILL <mask> [entries]     queue the program with start = false, `entries`
                            deep (default: the whole queue), and stop there;
                            QRUN then starts exactly what was filled. The reply
                            carries the depth REACHED, not the one asked for:
                            QUEUE_LEN is 16 on AVR and 32 on ESP32, and the
                            firmware holds QE_ROOM_RESERVE entries back for a
                            driver's DIR-drain pause. This exists because QRUN
                            alone prefills half a queue and tops it up from the
                            main loop, which leaves the depth a moment into a
                            run to a race between the loop and the drain -- so a
                            stop scenario measured the feeder, not the stop.
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

IMUX                          bring the ESP32 I2S multiplexer up, at runtime.
                              Takes NO arguments. The bus is the last three
                              analyzer CHANNELS and the GPIOs behind them come
                              out of this firmware's own CHAN_PIN table, so the
                              bus wiring has exactly one home and the host has
                              nothing it could name wrongly. Sent inside the same
                              serial session as the CONFIG that needs it --
                              opening the port resets the ESP32 and
                              initI2sMux() cannot run twice or survive a reset,
                              so a mux brought up in an earlier session is gone
                              by the next and the CONFIG is refused with
                              `ERR connect step 0`, which names no cause.
Driver names: `rmt` | `rmt` | `mcpwm` | `mcpwm_pcnt` | `i2s` | `i2s_direct` |
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

### The tick floor is on the command's DURATION, not on `ticks`

`addQueueEntry()` refuses when `ticks × steps < MIN_CMD_TICKS`
(`src/fas_queue/queue_add_entry.cpp`):

```c
uint32_t command_rate_ticks = period;
if (steps > 1) { command_rate_ticks *= steps; }
if (command_rate_ticks < MIN_CMD_TICKS) { return AQE_ERROR_TICKS_TOO_LOW; }
```

`MIN_CMD_TICKS` is `TICKS_PER_S / 5000`, so on ESP32 (16 MHz) it is **3200**
ticks — a command has to occupy 200 µs of the queue's timeline whatever it
contains. Measured on the board:

| `QSEG` | ticks × steps | result |
|---|---|---|
| `40 80 1` | 3200 | accepted |
| `39 80 1` | 3120 | `ERR QE step0 rc=-1` |
| `255 80 1` | 20400 | accepted |
| `1 80 1` | 80 | **refused** |
| `0 1600 1` (pause) | 1600 | **refused** |
| `0 3200 1` (pause) | 3200 | accepted |

Two consequences that cost real time to rediscover:

- **A single step cannot run faster than `MIN_CMD_TICKS`.** One step at 80 ticks
  is 80, well under 3200. To use the advertised `maxspeed0=80` period you must
  put *at least 40 steps* in the command (`40 × 80 = 3200`).
- **`QSEG` does not check this; `QRUN` does.** Appending a too-fast segment
  answers `OK QSEG n/8`, and the refusal arrives as `ERR QE step<u> rc=-1` from
  the feeder — named by *stepper*, not by segment, so it reads like a queue
  problem rather than a tick-range one.

`maxspeed*` in `QINFO` is therefore **not** "the fastest period you may write":
it is the fastest *period*, usable only on a command long enough to clear the
duration floor. Plan shared programs from `mincmd` and treat `maxspeed*` as the
per-stepper period to divide into, not as a `QSEG ticks` value.

Example — 255 steps, a pause, then a single step, which is the case that exposes
MCPWM/PCNT counter-limit overrun handling. Note the pause and the trailing step
both have to clear the duration floor, so the single step runs at `mincmd` and
the example no longer demonstrates "a single step at maximum speed" (nothing can,
on this SDK):

```
QCLR
QSEG 255 80 1      # ticks = the period from QINFO's maxspeed0
QSEG 0 3200 1      # pause: steps==0, so ticks alone must clear mincmd
QSEG 1 3200 1      # single step: ticks alone must clear mincmd
QRUN 1
```

`QRUN <mask>` is the multi-stepper selector: `QRUN 1` is one stepper, `QRUN 3`
is steppers A and B together (synchronized start).

Example — SR_25 / SR_30, the two stop scenarios. The program is four
`QUEUE_FILL_STEPS` segments, the queue is filled to a fixed 16 entries before
the start, and the stop lands a quarter of the way into the fill:

```
QCLR
QSEG 4080 80 1     # 16 entries x 255 steps, at the speed floor from QINFO
QSEG 4080 80 1
QSEG 4080 80 1
QSEG 4080 80 1     # 16320 steps = four queue-fills, of which one is run
QFILL 1 16         # -> OK QFILL q=14 on AVR (QUEUE_LEN 16 - QE_ROOM_RESERVE)
QRUN 1             # starts the fill and adds nothing afterwards
... ~10 ms, a quarter of the 40.8 ms fill ...
XSTOP              # SR_30; SR_25 sends STOP and expects all 4080 steps
```

The delay is a **fraction of the fill's duration**, computed from the DUT's own
tick rate and period (`stop_after_for()`), not a wall-clock constant. It has to
be a fraction of the *fill* rather than of the program — the program is four
fills long, so a quarter of that would land past the end of the run — and it has
to be scaled at all: a fixed 1 ms cleared the run on rmt and mcpwm_pcnt but
not on i2s_direct, whose first step arrives later than that because it streams
from a DMA buffer. Its marker edge then came 16 µs *before* the first pulse and
the scenario measured a stop that interrupted nothing.

Why the fill, and why a scaled delay: the stop has to land while the queue is
still the one `QFILL` put there, or the number of steps after the marker is a
measure of how far the feeder got in the meantime. The earlier form ran 20000
steps and stopped a quarter of the way in, and reported 7464 steps after the
marker against a bound of 8160 — a number that described the host's serial loop,
not any stop.

The measurement both scenarios make is `steps_after_stop`: the pulses from the
marker to the end of the capture. SR_30 requires it to be ~nothing — the queue
was emptied, so only what a driver had already handed to its hardware can still
step — and SR_25 requires it to be the whole remainder. Measured on ESP32 at
160 ticks with a 16-entry fill:

| driver | SR_25 after marker | SR_30 after marker |
|---|---|---|
| rmt | 3999 of 4080 | **0** |
| mcpwm_pcnt | 3988 of 4080 | **0** |
| i2s_direct | 4046 of 4080 | **67** |

MCPWM/PCNT and RMT emit nothing after the abort: both take one queue entry at a
time and have nothing in flight. I2S leaks 67 steps — 0.26 of an entry — because
it streams from a DMA buffer, which is the only hardware buffer in the three.
That difference is invisible to SR_25, where all three produce the full fill, and
it is the reason SR_30 exists as its own scenario.

The upper bound is the fill rather than the queue's capacity, and that is the
other half of the fix: the feeder used to top the queue back up to capacity
within one main-loop pass of the start, so the drain after a stop was always
(QUEUE_LEN − QE_ROOM_RESERVE) × 255 — measured as 7650 steps on all three
drivers, a number with nothing to do with any stop. `QRUN` on a filled queue now
sets `no_topup` and the queue drains exactly what `QFILL` reported.

- The `QSEG`/`QFILL`/`QRUN` feeder itself: segment program, prefill, fill-to-a-
  depth with `start = false`, synchronized kick-off, DIR-pause retry, pause
  commands. Implemented in `common/saleae_app.cpp` (`qe_pump`/`qe_feed`).
- **The two generic modes** (`--mode scale|sync`, todo R3). `scale` sweeps the
  stepper count 1..`min(driver queues, channel budget)` on one named driver with
  a **shared** program, asserting each stepper's own step count and period;
  `sync` runs **every driver-list combination** with each stepper at **its own**
  period, reporting first-step skew (µs *and* step periods) and asserting
  adherence per stepper. A refused point is recorded and the plan continues.
- Characterization scenarios above are runnable by hand with `control.py`.

### The move, not the capture

Every evaluator that asserts a step count measures **the commanded move**
(`run_tests.move_window()`), not the capture. A capture is not the move: it
starts before the test is triggered over serial and outlives it, so it holds
however long the host took to send the command and however long the board idled
afterwards — on a recorded `i2s_mux` `dir` capture, 285 ms of a 500 ms file.

No ramp is in the way (`addQueueEntry()` is driven directly), so the plan fixes
the move's timeline exactly: a segment of `steps` at `ticks` occupies
`steps * ticks`, a pause occupies `ticks`, and steps land at the start of their
own tick. The window is the **first contiguous run of as many steps as the plan
commands whose gaps all match the command** — per gap, so a program with a pause
still places its move; on a grid (the mux) a whole number of frames, off one a
quarter of the commanded gap, which is deliberately looser than the rule that
*judges* periods because this one only has to locate the move.

Four things it deliberately does not do, each a test in `TestMoveWindow`:

- **It cannot hide a dropped step.** Fewer steps than commanded and no window
  exists, so the whole capture is measured and the shortfall is reported.
- **It cannot hide an extra step the driver keeps producing.** The window ends
  at the last commanded pulse *and extends through any pulse whose gap from the
  one before it is one the command could have produced* — the anchor's own rule,
  applied forwards. `mcpwm_pcnt` on IDF 5.5.3 emits one step more than it was
  given, at the commanded period, immediately after the run (the MCPWM/PCNT
  overrun, `extras/doc/platforms/esp32.md`), and at
  24 MS/s that lands on the tick boundary to within a sample: a window cut at the
  last commanded pulse would have set it aside half the time.
- **A driver at the wrong rate cannot place its own move**, so nothing anchors,
  the capture is measured whole and the rate error is reported.
- **It discards nothing.** Every result carries `window`: `anchored`,
  `steps_in_window`, `steps_outside`, and each outside pulse's offset from the
  move's own start. The first-step skew is measured on the move's first step.

`SR_13` (a rejected command must emit nothing at all) and the marker-relative
`SR_25`/`SR_30` deliberately keep the whole capture: there a pulse anywhere is
the measurement.

### SR_31: the maximum stepper count

`SCENARIOS["SR_31"]` is the one entry whose stepper count is **not a literal**.
"How many steppers does this driver drive?" is the one question a driver answers
by refusing, and the answer is a hardware fact rather than a constant in a
header — so the host asks the board rather than believing a table.

`MaxCountProbe` (`scripts/run_tests.py`) sends `CONFIG` **descending** from
`max_stepper_count_bound()` and takes the first count the firmware accepts:

```
CONFIG 8 rmt,... nodir    -> ERR CONFIG n=8 needs 8 channels, max=8   (analyzer)
CONFIG 6 mcpwm_pcnt,...   -> ERR connect step 6 n=7                  (queues)
CONFIG 6 mcpwm_pcnt,...   -> OK CONFIG 6                     <- the maximum
```

Three properties are load-bearing and each has a test:

- **The refusals are the measurement as much as the count.** "stops at 6" and
  "stops at 6 because the analyzer has no channel 7" are different answers, so
  `refused_above` (every count tried, with the board's own words) travels in the
  result record next to `max_stepper_count`.
- **One `CONFIG` per attempt, and the accepted one *is* the run's.** `find()`
  hands back the reply it already has so `measure()` does not send the line
  again; a second `CONFIG` re-arms the drivers through `_initVars()` and would
  discard the very allocation the probe proved.
- **The search starts from the channel budget, not from `QUEUES_*`.** 8 in
  `nodir` (the analyzer), 32 for `i2s_mux` (the 32-bit word, which is what bounds
  a multiplexed stepper since it costs a slot and not a channel). The bound only
  has to be an upper bound — it is the top of a descending search — but never a
  *lower* one, or the probe cannot reach the maximum.

**Why `nodir`:** `dir` spends two channels per stepper and stops at 4, below
`QUEUES_MCPWM_PCNT` = 6, so a `dir` max-count run would report the analyzer's
channel count as the driver's queue count. That is why `SCENARIO_PIN_MODE`
exists and why `scenario_wire()` replaced the hardcoded `dir` in `config_wire()`
— every scenario but this one is `dir`, and a fifth caller of `config_wire()`
would otherwise have handed SR_31 a `dir` CONFIG the firmware happily accepts and
then scored stepper B against D2 instead of D1.

Judged by `eval_scale`, unchanged: **each** stepper's own step count and its own
period, from the capture. Not the board's `POS` tally — that is the queue's
bookkeeping and would pass on a driver emitting complete garbage. The mask is
`(1 << count) - 1` from the probe, so `QRUN` starts every stepper; in the table
the mask column reads `probed` rather than the literal `0`, which would read as a
scenario that starts nothing.

`Pins.for_scenario("SR_31")` builds the **widest** map the pin mode allows (8
steppers, D0…D7) — the fixture and report path only; a hardware run uses the
board's own `MAP`. A 2-stepper map here would score A and B and report the run
done with six channels unread.

**One known reason a mux row may record red,** pre-existing and recorded rather
than worked around:

- the intermittent dropped pulse (`r7_virtual_i2s_mux.md` §5). Observed while
  measuring this item's premise: a forced 32-point `--mode scale --driver i2s_mux
  --pin-mode nodir` sweep passed 31 of 32, with n = 3 reporting **63 of 64 steps**
  and one off-grid period; an immediate re-run of n = 1…4 passed all four. A
  max-count scenario inherits it — at n = 32 the expected cost is one flake per
  few runs, which is a different trade from a scenario that is red every time.
  The 24 MS/s sampling race was the *other* one and is closed: the evaluators
  measure the commanded move rather than the whole capture, and the decoder's
  frame faults now travel with the result (see *The race is not fixed; the
  measurement of it is* below). `nodir` remains the clean case — the data line
  is idle-low and has no transitions to race with — which is one more reason this
  scenario does not use `dir`. A dropped step is still caught by the window: a
  swallowed step leaves no run of gaps for it to anchor on, so the whole capture
  is measured and the shortfall is reported.

**Measured on hardware** (ESP32-DevKitC, `--pin-mode dir`, the catalogue
re-run in full after the watchdog fix below; the `i2s_mux` row on ESP-IDF 5.5.3).
Each stepper's own step count and period, all within tolerance:

| driver | max steppers | probe | notes |
|---|---|---|---|
| `i2s_mux` | **32** | `CONFIG 32 i2s_mux,… nodir` accepted first try | the 32-bit word is the budget and it is not the analyzer's: a multiplexed stepper costs a slot, not a channel. All 32 at 64/64, mean period spread **0.0000 µs** |
| `rmt` | **8** | `CONFIG 8 rmt,… nodir` accepted first try | 8 queues, 8 channels — the analyzer is the binding constraint and the driver happens to match it |
| `mcpwm_pcnt` | **6** | 8 and 7 refused `ERR connect step 6`, 6 accepted | `QUEUES_MCPWM_PCNT` = 6 confirmed by the board |
| `i2s_direct` | **2** | 8 down to 3 refused, 2 accepted | `SOC_I2S_NUM` = 2; the refusals name `i2s_new_channel(): no available channel found` |

Period spread across the steppers of a run: 0.000–0.004 µs.

**Two defects the first run found**, both of which had to be fixed before the
numbers above mean anything:

- **The probe read acceptance off the reply, and the reply lies.** A `CONFIG`
  refused partway through `connect_stepper()` leaves the steppers it did connect
  in place and `slot_count` at that partial count, so the *next* `CONFIG`
  short-circuits to `OK CONFIG n=6 mode=nodir already` — success, for a
  configuration that was never established. Measured, by hand on `control.py`:

  ```
  CONFIG 8 mcpwm_pcnt,… nodir -> ERR connect step 6 n=6
  CONFIG 7 mcpwm_pcnt,… nodir -> OK CONFIG n=6 mode=nodir already
  ```

  The probe recorded **7** for a board running six, and the run *passed*, because
  `eval_scale` judges the six that are really there — so nothing downstream would
  have noticed. Acceptance is now MAP (`read_map`), which is the board's own
  count and cannot be stale, and a CONFIG that answers OK while connecting fewer
  than it was asked for is recorded in `ok_but_short` rather than believed. This
  is a firmware defect as well as a host one, and it is still there: the
  `already` short-circuit reports success for a request it did not honour.
- **The board's own task watchdog reset it mid-capture, and SR_00 called that
  eight dead pins.** See below.

The `i2s_mux` row is green **once**: the intermittent dropped pulse is what
bounds how often it stays that way, and a dropped step is still caught by the
move window (see *The move, not the capture*). The 24 MS/s sampling race is no
longer a reason for it to go red at all — see *The race is not fixed; the
measurement of it is* below.

### The ESP-IDF task watchdog, and why SR_00 failed

Measured on ESP-IDF 5.5.3: **a board doing nothing trips its own watchdog.**
`open_board()`, no command, no capture, and six `task_wdt` lines arrive within
~5.3 s with an IDLE0 backtrace naming `main` as the running task. It is the
harness, not the library: IDLE0 is subscribed to the task WDT by default,
`saleae_hal_serial_read()` is a zero-timeout `uart_read_bytes()`, and at
FreeRTOS's 100 Hz `pdMS_TO_TICKS(1)` is 0, so a pass through the main loop ends
in a sub-tick spin and IDLE0 never gets to run.

Why it reached the report as a *wiring* fault: a panic inside a capture window
truncates the run, and the truncated 1 Hz pattern reads as every channel's first
high time being a fragment of the commanded one — measured D0 19.3 ms against
50 ms, the pairs summing to exactly one 1000 ms period. **SR_00, the check whose
job is to catch a dead cable, reported a firmware panic as eight dead pins.**

Two loops were at fault and **both** had to change — neither alone fixed it:

- `saleae_hal_idle()` was `saleae_hal_delay_ms(1)`, which *is* the sub-tick spin,
  contradicting its own header ("Deliberately not `saleae_hal_delay_ms(1)` …
  Spinning in the idle path starves IDLE, and the task watchdog then resets a
  board that is doing nothing at all"). It now blocks a tick.
- `saleae_test_loop()` spins 1 ms at a time for a whole second-long period, so
  SR_00 starves IDLE0 *even with the idle path fixed* — which is what the second
  attempt showed. The spin cannot simply go: an edge has to land within SR_00's
  2 ms width tolerance. It now sleeps between edges and keeps the spin only for
  the 12 ms before one (`SPIN_WINDOW_MS`, wider than the tolerance and narrower
  than the 50 ms `EDGE_GRID_MS`), which is the only arrangement in which both
  the timing and the watchdog are satisfied.

And the watchdog is fed, in the right order: `esp_task_wdt_add(NULL)` once in
`saleae_app_setup()` **before** `READY`, then `esp_task_wdt_reset()` on every
main-loop pass. Both halves are needed. Feeding without subscribing is worse
than nothing — `esp_task_wdt_reset()` on an unsubscribed task logs
`task not found` *every call*, once per pass, on the UART the host protocol runs
on; that attempt produced a console full of errors and still panicked.

`TestNoRunawaySpin` pins all of it as source checks, since the defect is a timing
property of firmware that no host-side unit test can execute.

Independently, `open_board()` now **raises** if the board does not report
`READY`: opening the port does not guarantee a reset (the DTR/RTS toggle does not
always fire), and the old loop fell through after its timeout and measured
whatever the previous test left behind.

### MCPWM/PCNT: all six queues measured working

`--mode scale --driver mcpwm_pcnt` sweeps n = 1…6 in `nodir` (n = 1…4 in `dir`,
2 channels per stepper) and every point passes: each stepper emits exactly the
commanded step count at the commanded period. n = 7 and 8 are **refused** — the
bound is `QUEUES_MCPWM_PCNT` = 6, which is the ESP32's real hardware
(`SOC_MCPWM_GROUPS=2` × `SOC_MCPWM_TIMERS_PER_GROUP=3`), so the board refusing
the seventh queue is the measurement that the constant is right.

This was not true until the `pcnt_new_unit()` interrupt-enable defect was
fixed; `--mode scale` is what found it, and it is the regression test for it.
The symptom to watch for is one stepper emitting exactly its share while every
later stepper free-runs at exactly the commanded period and never stops (22 143
edges where 64 were commanded), with `POS` reading non-monotonic and *below* the
commanded count. Two invariants in the driver keep that from coming back, both
non-obvious and both load-bearing — see the MCPWM/PCNT section of
`extras/doc/platforms/esp32.md`:

- the driver re-asserts its own PCNT unit's `int_ena` bit on **every** init,
  because `pcnt_new_unit()` clears it;
- MCPWM timer index and PCNT unit index are equal only because the library is
  the first PCNT user, so the driver may not share the chip with an application
  that also allocates PCNT units.

`rmt+mcpwm_pcnt` and `rmt+rmt` are unaffected either way.

## Target/driver notes

### Native USB needs a protocol reset (RP2040/RP2350)

**The harness runs on a Pico 2** (`--arch rpipico2 --driver pio`), but native
USB breaks the reset the whole harness is built on, so it needed a second
mechanism.

Every other board here is a **USB-serial bridge**: opening the port toggles
DTR/RTS and resets the MCU, the boot prints `READY`, and `open_board()` waits
for it. **A queue is allocated once and the engine has no release**, so that
reset is load-bearing: a *different* `CONFIG` on a board that was not reset is
refused (`ERR CONFIG already n=1 ... asked n=2`). Between scenarios the reset is
what returns to an empty engine.

RP2040/RP2350 do not do this. Opening the port does not reset the chip, the
DTR/RTS toggle is ignored, and the only reset the arduino-pico core offers is
the 1200 bps jump to the UF2 bootloader. `READY` is printed once at boot, so by
the time the host reconnects it is gone -- and a *real* reset re-enumerates USB
and loses it again.

The fix is two firmware commands and a host path (`_open_native_usb()` in
`scripts/run_tests.py`):

- `PING` — side-effect-free liveness. `open_board()` identifies the board with
  it instead of `READY`. `native_usb_port()` gates this to `cu.usbmodem*` /
  `ttyACM*`, so a bridge board that did not reset still fails loudly.
- `RESET` — replies `OK RESET` then `rp2040.reboot()`. The host sends it, the
  chip re-enumerates with the **same port name** (measured), and the host waits
  until `PING` answers. This is the stand-in for the port-open reset.

**A scenario naming a driver the build lacks is SKIPPED, not failed.**
`unsupported_scenario_drivers()` uses the board's own `DRIVERS` list: SR_17 asks
for RMT+MCPWM, SR_18–20 for MCPWM/PCNT, SR_23 for I2S; none exist on a Pico, so
they are recorded `skipped` with the driver list, because "this core has no RMT"
is not a defect in the board. A bare `run_tests.py` that did not read `DRIVERS`
first falls back to the `ERR CONFIG no such driver` refusal.

Three defects were fixed to get there, all latent because only ESP32 had ever
been measured:

- **The short `MAP` reply buffer was 32 bytes for a 36-byte reply**
  (`handle_map()` under `!SUPPORT_SELECT_DRIVER_TYPE`), so `MAP` truncated to
  `...stride=2 c` on every AVR/Pico/SAM build and `read_map()` always failed.
  The buffer is sized by a `static_assert` on the widest reply now.
- **That reply also omitted `marker=`**, so SR_25/SR_30 could not find a marker
  channel; and `MAP_RE` did not consume `ch=-`, which stopped the optional
  `marker` group from matching. Both fixed.
- **`strtok()` returns one token too few on the first call after
  `rp2040.reboot()`** (newlib static state), so the first `CONFIG` with 3+
  drivers was refused. Replaced with an explicit splitter that keeps strtok's
  skip-runs-of-commas semantics.

### Which drivers a build has, measured per SDK

`DRIVERS` is the host's list of names; what a build actually accepts is
**asked of the board** (`DRIVERS` command, `read_drivers()`), because only the
firmware knows. Measured over the release matrix, ESP32 classic, 350 runs:

| PlatformIO env | ESP-IDF | drivers the board reports |
|---|---|---|
| `esp32_V4_4_0`, `esp32_V5_3_0`, `esp32_V6_13_0` (arduino) | 4.4.7 | `rmt`, `mcpwm_pcnt` |
| `esp32_idf_V5_3_0` | 4.4.3 | `rmt`, `mcpwm_pcnt` |
| `esp32_idf_V6_13_0` | 5.5.3 | `rmt`, `mcpwm_pcnt`, `i2s_direct`, `i2s_mux` |
| `esp32_idf_V7_1_2` | 6.1.0 | `rmt`, `mcpwm_pcnt`, `i2s_direct`, `i2s_mux` |

**There are no I2S queues below ESP-IDF 5**, and that includes every Arduino
build: `SUPPORT_ESP32_I2S` is defined only in `pd_config_idf5.h` and
`pd_config_idf6.h`, never in `pd_config_idf4.h`, so `QUEUES_I2S_MUX` and
`QUEUES_I2S_DIRECT` are 0 and the library has no i2s driver to name. Do not
read the missing `i2s_*` fields on an Arduino or IDF4 row as a broken build or
a missing build flag — it is the library's own version split, and the env name
carries the *platform* version, not the IDF version (see
`extras/doc/platformio-espressif-versions.md`).

Consequence for reading a matrix report: `SR_23` (the `i2s_direct` scenario)
records **failed** with `ERR CONFIG no such driver` on every row without I2S.
That is the firmware answering a capability question, not a measurement going
wrong, and it is why the report says so in its legend rather than suppressing it.

### ESP-IDF 5.5.3 broke RMT and `i2s_direct` — one cause, in this harness (fixed)

Both symptoms on that row were **the same defect**, and it is not in the library
or in either driver: **the main task overflowed its stack.**
`CONFIG_ESP_MAIN_TASK_STACK_SIZE` is 3584 B, and every `CONFIG` constructs its
drivers from that task, so `stepperConnectToPin()` ran on a stack that was out
of budget before it started.

- **Every RMT `CONFIG` panicked the firmware** (`LoadProhibited` inside the
  allocator, reached from `rmt_new_tx_channel`): 34 measurements, `connect_rmt()`
  never returned. The overflow ran off the *top* of the stack into the DRAM tlsf
  pool, so it presented as a corrupted free list rather than a stack fault.
  → [`extras/doc/implemented/idf55_main_task_stack_overflow.md`](../../../doc/implemented/idf55_main_task_stack_overflow.md)
- **`i2s_direct` was unstable**: the `scale` sweep answered refused / failed /
  stack-overflow for the same point across runs. Here FreeRTOS *did* report it
  (`CHECK_STACKOVERFLOW_CANARY` catches downward writes; the RMT row's
  upward-into-heap write is invisible to it).
  → [`idf55_main_task_stack_overflow.md`](../../../doc/implemented/idf55_main_task_stack_overflow.md)

**What the stack was spent on** (measured with
`uxTaskGetStackHighWaterMark()`, ESP-IDF 5.5.3, peak for `CONFIG 1 rmt dir`):

| | stack |
|---|---|
| reply buffers held as **stack locals** — `handle_config`'s `SALEAE_CFG_REPLY_MAX` alone is 1152 B, and a local array holds its slot for the whole function | **~1900 B** |
| libc `sscanf` (one call) + libc `snprintf` (384 B each) — the protocol only ever formats `%s`/`%u`/`%d` | ~1900 B |
| `StepperTask` (6000 B, 8 % used) — not involved | 508 B |

Peak went **4272 B → 2336 B** of the 3584 B default, so the cliff is gone rather
than moved and no Kconfig change was needed. `CONFIG 1 rmt dir` and
`CONFIG 2 i2s_direct,i2s_direct nodir` both answer `OK` on 5.5.3 now.

The rules that keep it that way are enforced by `TestStackBudget` and
`TestSaleaeFmt` in `scripts/tests/test_saleae.py`: no `char[]` of 64 bytes or
more may be a stack local off AVR, and no format string may use a conversion the
small formatter does not implement. `SAL_REPLY_BUF` in `saleae_app.cpp` is the
one place that says which side of the line a buffer is on, and the reason is
platform-specific — off AVR the task stack is scarce, on AVR the 2 KB part is.

Two things this harness still gets wrong, both unrelated to the stack:

- **`ticks` must be ≥ `MIN_CMD_TICKS` from `QINFO` (3200 on ESP32), so the
  documented `QSEG … 80` example answers `ERR QE step0 rc=-1`
  (`ErrorTicksTooLow`) on a current build.**
- **The catalogue now asks how many steppers a driver drives: SR_31.** See
  "SR_31: the maximum stepper count" below. The other catalogue scenarios are
  still 1–3 steppers, because a *named* case wants a shape to reason about; SR_31
  is the one whose count is the question.

A third claim is **withdrawn**: `i2s_mux` mangling commands from n ≥ 16 does not
reproduce. `CONFIG 16…` and `CONFIG 32…` both parse (768-byte reply, 32 driver
names, 32 `maxspeedN` fields), `MAP` reports `slots=0…31`, n=33 is refused, and
`dir` reaches 16 and refuses 17. Both of the mux's documented claims now hold.
The reported `ERR unknown '2s_mux,i2s_mux'` was head-loss (the `cmd` field is 15
characters wide and that token is exactly 15), and the suspected 256-byte RX
ring was already 1024 before the report was filed. The direction slots on the
wire were unmeasured at the time because of the sampling race below; the
`syncdir_i2s_mux*dirn2` rows re-evaluate as passing with `slots` and `dslots`
agreeing on every one.

### The mux in `dir` mode: a host bug, and the sampling race

`CONFIG 2 i2s_mux,i2s_mux dir` was recorded as an **incomplete capture, S2
missing**, on both I2S SDKs, and read as a lost slot in the driver. It is not.
Measured from the three bus wires with **no slot map in the loop at all** —
group the bclk rising edges into 32s, read the data at each edge, count which
positions are high. That involves no slot numbering, so it cannot be wrong the
way a map can be. Five positions ever carry a bit, counted from the first edge
of the group:

| wire pos | frames high, of 125 255 | spacing | what it must be |
|---|---|---|---|
| 12 | 125 205 | — | a DIRECTION bit, high from t=0 |
| 14 | 125 205 | — | the other DIRECTION bit |
| 15 | **64** | mean **24.931 µs** (24/28 on the 4 µs frame grid) | a step signal, 400 ticks |
| 13 | **64** in the move | mean **49.925 µs** (48/52 on the grid) | a step signal, 800 ticks |
| 11 | 50 | 8177 µs | nothing — a sampling race, see below |

400 ticks at 16 MHz is 25.000 µs and 800 is 50.000 µs, so both step signals sit
on the commanded periods carried on the frame grid. Positions 15 and 13 are high
together in exactly 32 frames and otherwise alternate 2:1, with 13 continuing
alone after 15 stops — two steppers on a shared start at twice the rate, which is
what the plan commands. `POS 64 64` agrees.

The slot *numbers* do need the decoder's half-swap, and that is the one thing
that could be wrong, so it is checked rather than assumed: wire position `p`
maps to slot `15 - p` for `p < 16`, which puts 12 and 14 on slots 3 and 1 and
15 and 13 on slots 0 and 2 — exactly MAP's `dslots=1,3` and `slots=0,2`, in the
right roles. Were the swap wrong, the two bits high in *every* frame would not
land on the two direction slots MAP reported. The one position that lands
nowhere is 11 → slot 4, which MAP never allocated.

What the run actually measured was our own decoder config naming one stepper
instead of two: the decoded VCD said so in its own header (`slots: A=S0`, one
channel), because `_mux_map_with_slots()` built it with `slots[j * stride]` and
`slots=[0,2]` with `stride=2` is *one* stepper. The same one-entry-per-channel
misreading `read_map()` had, fixed with it.

With the map right the run was **still red**, for a different reason: position 13
is high in 51 more frames, 285 ms *before* `QRUN`, and position 11's 50 cannot
be emitted at all. That is the 24 MS/s floor racing itself (3 samples per bclk
period, no margin). `--mode scale --driver i2s_mux --pin-mode nodir` was green
throughout (n=1...8, 64/64) because in `nodir` the data line is idle-low and has
no transitions to race with.

### The race is not fixed; the measurement of it is

Those 51 frames were 51 extra steps in the result, because **every evaluator
counted the whole capture** and a capture is not the move: it starts before the
test is triggered over serial and outlives it, so it holds 285 ms of pre-move
idle on a 500 ms capture. Nothing the driver did is in that stretch. `dir` is
the mode where it shows because the direction bits are held high, so the data
line toggles in **every** frame and every frame is a race.

The evaluators now measure the **commanded move** (`run_tests.move_window()`):
the first contiguous run of as many steps as the plan commands whose gaps all
match the command, with the gaps derived per gap (so a program with a pause
still places its move) and judged on the frame grid in `dir`/mux. Steps outside
the window are **recorded** in the result — `window.steps_outside` with each
pulse's offset from the move — never discarded. The first-step skew is measured
on the move's first step too, which is why `i2s_mux+i2s_mux` reads 0.0 us
against the 242 334 us it reported (44.9 step periods, from a phantom 242 ms
early). Nothing the window can do makes a bad run pass: a dropped step, an extra
step inside the move's span, and a driver at the wrong rate are each still a
failure, and each is a test in `TestMoveWindow`.

`extract_frames()`'s frame-alignment diagnostics travel with the result now
(`decoded_from.frame_faults`): **98–103 on every 24 MS/s `dir` capture**, and
**102 on the `nodir` capture at 32 steppers** as well — so the faults belong to
three samples per bit period and not to `dir`. One capture in 1250 decodes wrong,
and the run says so next to the step count instead of the step count being the
only evidence. Recorded, never gated: a threshold high enough to be meaningful
is one every mux run at this rate fails.

**Measured on hardware**, `--mode sync --pin-mode dir --imux`, one sweep per I2S
SDK — all eight rows that were red:

| SDK | `i2s_direct+i2s_mux` | `mcpwm_pcnt+i2s_mux` | `rmt+i2s_mux` | `i2s_mux+i2s_mux` |
|---|---|---|---|---|
| IDF 5.5.3 | **passed** | **passed** | **passed** | **passed** |
| IDF 6.1.0 | **passed** | **passed** | **passed** | **passed** |

Each stepper 64/64 at its own commanded period, `first_step_skew_us` **0.0** on
both `i2s_mux+i2s_mux` rows against the 242 334 us they reported, and one to
three pulses per capture set aside — the closest to the move 4.5 ms, against a
move 1.575 ms long. Before the hardware was attached, every recorded capture was
re-run through the new code offline, as a check that nothing else moved: **212 SR
catalogue, 16 `scale` and 28 non-mux `sync` verdicts unchanged**, and the seven
then-red mux rows evaluating as passed. Full record, measurements and the
invariants:
[`extras/doc/implemented/i2s_mux_dir_phantom_steps.md`](../../../doc/implemented/i2s_mux_dir_phantom_steps.md).

The one red row in those sweeps is `mcpwm_pcnt+i2s_direct` on IDF 5.5.3, **three
sweeps in three**, and it is the
[MCPWM/PCNT overrun](../../../doc/platforms/esp32.md#mcpwm-pcnt-overrun) — a
scoped known limitation, not a finding. It is also what found this
item's own trailing knife-edge — the extra pulse lands on the tick boundary to
within a sample at 24 MS/s, so the window now extends through a pulse that
continues the move's rhythm rather than being cut at the last commanded pulse.
See "The move, not the capture" above.

## Capture format

Record `.sr` (sigrok srzip, one packed byte per sample), never CSV: a 2 Msample
8-channel capture is 26 KB as `.sr` and 80 MB as CSV. `capture.py --vcd` derives
a VCD from it via `sigrok-cli -I srzip -O vcd`. A VCD contains only value
changes, so it is the compact, GTKWave-readable evaluation artifact; sigrok
picks `$timescale` from the sample rate (1 us at 1 MHz, 100 ps at 48 MHz — do
not assume a fixed or finer timescale). `signal_parser.load_vcd()` reads it
back, so a VCD may end before the last constant stretch of the capture.

## Sample-rate / capture gotchas

- Supported rates are `48 MHz / n` (`sigrok-cli -d fx2lafw --show`). **Measured
  delivered length for a 2 s request**, 8 channels:

  | rate | 4 | 8 | 16 | 24 | 48 MHz |
  |---|---|---|---|---|---|
  | samples | 40 448 | 80 384 | 160 256 | 240 128 | **8 704** |
  | window | 10 ms | 10 ms | 10 ms | 10 ms | **0.18 ms** |

  (Those are *first-member* byte counts; the `.sr` is chunked into 200 members
  and totals 2 s at 4–24 MHz. **48 MHz is the outlier — it truncates hard**, to
  less than a scenario's duration, so a capture there covers a run that has not
  started. Do not pick 48 MHz because it is the highest rate on the list.)

  The 24 MHz VCD writes `$timescale 100 ps`, and `load_vcd()` recovers the sample
  period from `$timescale × $comment rate` — so the two have to agree or every
  timestamp lands on the wrong sample. `i2s_mux_decoder.timescale_for()` emits
  exactly `1e9/rate` ns for this reason.
- The **analyzer channel budget** is `CHANNELS - 3` while the mux bus is up (the
  bus is the last three channels), so 5 physical steppers in `nodir`, 2 in `dir`.
  A *multiplexed* stepper is not bounded by that at all — it costs a bit of the
  32-bit word. See "The I2S mux" above.
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

## Strings and `const` tables: the AVR RAM rule

**On AVR a string literal is an SRAM allocation, and so is a `const` object.**
The linker script copies `.rodata` into RAM to initialise it at reset. This is
counter-intuitive, it is invisible on every other target, and it cost this
harness 51 % of a 328P:

```
before  .data 1136 B =  96 B variables + 1040 B string pool   ->  80 B free
after   .data   20 B                                         -> 1196 B free
```

So, in `common/` and any app built for AVR:

- **Every literal goes in `SAL_PSTR()`** (`common/saleae_str.h`; it is `PSTR`
  on AVR, the identity elsewhere). `TestAvrRamBudget` in
  `scripts/tests/test_saleae.py` fails the build on a bare one — that test is
  the enforcement, this paragraph is the reason.
- **A flash string is never handed to anything that reads RAM.** Reply with
  `reply_p()` (→ `saleae_hal_serial_write_p`), never `reply()`. Getting that
  backwards is silent on ESP32 and prints SRAM garbage on a 328P. A `%s`
  argument must be in RAM, so copy it first with `sal_to_ram()` — which is why
  `driver_name()` and `pin_mode_name()` fill a caller's buffer rather than
  returning a pointer. `%S` is deliberately not used: it would fork the AVR
  reply path from every other target for no gain.
- **`const` tables are `SAL_PROGMEM`** and read with `sal_pgm_read_byte()` /
  `sal_pgm_read_word()`. `static_assert` cannot read those, so a table the
  build asserts on needs a `static constexpr` mirror — `kChanPin` has one,
  generated from the same initializer macro so the two cannot drift.
- Not even a delimiter is free: `strtok(d, ",")` cost 2 bytes, so it is a
  `char[2]` built from char constants.

The library was never affected — it routes everything through `FAS_PSTR`
(`src/fas_arch/result_codes.h`). Measured on `saleae_avr`:

```bash
pio run -d extras/tests/saleae_based/apps/arduino -e saleae_avr
~/.platformio/packages/toolchain-atmelavr/bin/avr-size -A \
    extras/tests/saleae_based/apps/arduino/.pio/build/saleae_avr/firmware.elf
```

`.data` is the number to watch: it holds `.rodata` *and* the stack. PlatformIO's
`RAM:` line only gives the sum. See white paper §4.3.2.

## The I2S mux: 32 steppers on 3 wires

The ESP32 I2S multiplexer carries up to **32 stepper signals as one 32-bit word**,
repeated every I2S frame. That is what lets an eight-channel analyzer measure 32
steppers: the three bus wires are captured and *decoded*, and the other five
channels carry whatever physical steppers are also attached.

```
D0..D4   five stepper channels (step / dir of a physical driver)
D5,D6,D7 I2S data, bclk, ws
```

The bus is the **tail**, deliberately. The stepper channel map stays `stride * i`
from channel 0 with no offset in front of it, and a mux capture is a superset of a
physical one: decode it and D0..D4 keep their names. A mux stepper costs a bit of
the word and **no analyzer channel**, which removes the channel budget as its
limit -- `nodir` reaches 32, `dir` reaches 16 (a mux direction is a second bit of
the *same* word).

`scripts/i2s_mux_decoder.py` is the 8 -> 37 leg: 3 bus channels consumed, 5
passthrough + 32 slots (`S0`..`S31`) written. The result is an ordinary VCD, so
`signal_parser` and every evaluator read it with **no mux-specific code** --
which is the property the whole design rests on, and it is tested
(`TestMuxDecode.test_the_decoded_capture_evaluates_through_the_ordinary_path`).

Three measured facts that the design reference got wrong, and that the
implementation depends on:

- A slot is high for **one bclk period (125 ns)**, not for the frame. The frame
  is the unit of *time*; a slot is one of 32 bits inside it. The decoder widens
  the one-bit pulse back to a frame, because that is the unit the receiving shift
  register latches and the unit a step occupies.
- The sample rate floor is **24 MS/s**, not 8. 24/8 = 3 samples per 8 MHz bclk
  period; 8 MS/s is one sample per period and recovers nothing. **48 MS/s is
  worse** -- this analyzer truncates an eight-channel 48 MS/s capture to 0.18 ms
  and a scenario lasts milliseconds. See the table in
  `extras/doc/implemented/r7_virtual_i2s_mux.md`.
- The 32-bit word goes out as **two 16-bit halves, the low half (slots 0-15)
  first**, each half MSB-first. ws is LOW for the first half and HIGH for the
  second, so a word starts at a ws **falling** edge and a ws rising edge is its
  middle: wire bit k < 16 carries slot 15-k, k >= 16 carries slot 47-k. The
  decoder is a bclk-edge shift register anchored at the first ws fall and swaps
  the two halves back, so slot S is bit S, matching
`i2s_mux_slot_to_bit_pos()`. An earlier version anchored at the ws *rise*
   with whole-word MSB-first order and read every word half-swapped: a 64-step
   run decoded as two phase groups (`0x0000FFFF` / `0x000F0000`) that are really
   the two halves of one word, and the intermittent "16 of 20 slots" sweep
   failure was that misread.

**The wire order is right; the fixture's _phase_ was not.** A separate defect,
[184](../../../todo/184_mux_decoder_fixture_launch_phase.md), had 11 tests in
`test_i2s_mux_decoder.py` failing, and they were correct to:
`Bus._render()` drew each
bit's data cell starting **at** its bclk rising edge, while the peripheral
launches it **half a cell before** the edge that latches it (at 48 MS/s the cell
is 208..213 with rises at 211 and 217 — the edge is at the cell's midpoint). The
decoder samples `edge - 1`, so the fixture was read one cell early and **every
bit landed one position out**: slot 0 decoded as slot 31, uniformly.

It stayed invisible because every test in the file drove all 32 slots or a
symmetric set, and a uniform shift is a consistent relabelling that those
assertions are invariant under. The fixture is now checked asymmetrically — one
slot at a time, asserting that exactly that slot lights up — which is the only
form that catches it. `TestGeometry` covers 24 MS/s separately, because that is
the rate `harness.py` actually selects (3 samples per cell, `bclk_half == 1`) and
it is a different regime rather than the same one at lower resolution.

`i2s_mux_decoder.py` needed no change. Re-verified end-to-end on the real 24 MS/s
captures: `S0 = 64` steps on the n=1 run, `S0 = 64` and `S1 = 64` on n=2, mean
period 24.931 us — the slots and the number the results recorded.

**Sample the data one sample before the bclk rising edge** (`edge - 1`), because
the bit has been stable for half a cell by then. The two plausible alternatives
are both wrong *silently*: the middle of the clock's high time needs a 50 % clock
and the ESP32 is not one (measured 4-in-6 high at 48 MS/s, 1-in-3 at 24 MS/s), and
it turned a clean 64-step run into 37 steps across two slots; the cell's last
sample lands on the *next* cell at three samples per cell and decoded slot 0 to
slot 1. The tests in `test_i2s_mux_decoder.py::TestSamplingPoint` pin all three.

A multiplexed step can only start on a frame boundary, so its period is a **set**:
400 ticks is 6, 6, 6 then 7 frames -- 24, 24, 24, 28 us, averaging exactly 25.
`signal_parser.grid_period_defects()` derives the legal set from the grid, and
`run_tests.check_periods()` is the single place choosing between the grid and the
ordinary +-5 % band (twelve call sites). A period on any other frame count still
fails.

**Known finding:** an intermittent dropped step. With the corrected decoder
(the old sweep numbers were the half-swap misread above), a forced 32-point
`--mode scale` sweep failed exactly once: n=28, slot **16**, 63 of 64 steps,
one 47.96 us inter-step period (two frame periods = one missing pulse mid-run).
Three follow-up n=28 runs were clean, so it stays intermittent. The earlier
"16 of 20 slots" reading does not survive the decoder fix and should not be
chased. Suspected: `i2s_fill_buffer_mux()` does `buf[...] |= bit_mask` on
memory the DMA is reading. Recorded, not worked around. Details in
`extras/doc/implemented/r7_virtual_i2s_mux.md` §5.

## Hardware pin map (ESP32-DevKitC)

```
Saleae D0..D7 -> GPIO 2, 0, 4, 16, 17, 5, 18, 19
4ch steppers : A step/dir 2/0, B 4/16, C 17/5, D 18/19
I2S mux bus  : D5=D5/GPIO5 data, D6=D18 bclk, D7=D19 ws
GPIO0 is a boot-strapping pin (must be HIGH at boot).
```

The GPIO-to-channel table is **per board and per cable** -- the ESP32-S3 map is not
the ESP32-DevKitC one. That is why `IMUX` names no pins and why the mux's channel
allocation is a constant here rather than a host argument.

## References

- Design / roadmap: `white_paper_saleae_test_harness.md`
- Backlog item: `extras/todo/120_saleae_based_test_harness.md`
- User-facing overview: `README.md`
- Repo-wide rules: `AGENTS.md` at the repository root
- sigrok-cli: <https://sigrok.org/wiki/Sigrok-cli>

- PlatformIO ESP32 platform version mapping: \`extras/doc/platformio-espressif-versions.md\`
