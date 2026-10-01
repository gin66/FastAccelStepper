# 120 Saleae-based Test Harness — White Paper

> This document is the design reference: what is built, how it works, and why.
> It deliberately carries no task list and no status. Progress and the
> remaining work live in exactly one place:
> [`extras/todo/120_saleae_based_test_harness.md`](../../../todo/120_saleae_based_test_harness.md).

> **⚠ WARNING — do not connect a stepper, motor, or stepper driver.**
>
> The commands this harness generates are **not intended to drive a motor**.
> They are synthetic probe patterns chosen to make the waveform measurable: raw
> tick periods picked to land inside the 16-bit range, step counts chosen to sit
> on boundaries (1, 8, 200, 255, 4000), and pauses chosen to be long enough to
> see in a capture.
>
> The fast scenarios command **25–40 kHz step rates reached instantly from
> standstill**, with a pulse high time of about **1 µs**. At 1.8° full step that
> is roughly **7,500–9,400 rpm**, which no stepper can follow from rest without
> acceleration. A real motor would stall, lose steps, and sit drawing
> near-standstill current through the driver while the harness ran. On AVR the
> speed floor is even lower still.
>
> Nothing is gained by attaching one: the measurement is the pin signal, taken
> before any driver chip. The results are identical with the pins unconnected
> apart from the analyzer.
>
> The library's normal use of these same queues is `move()`, which ramps up
> within motor limits. This harness deliberately bypasses that and addresses
> `addQueueEntry()` directly, so **no speed limit, acceleration profile, or
> current sense is applied anywhere in this path.**


## 1. Goal

Build a **hardware-in-the-loop test harness** using a **Saleae Logic Analyzer**
(or any sigrok-compatible USB logic analyzer) to **characterize `addQueueEntry()`
at the pin level**, on **any supported architecture** — AVR, the ESP32 family,
RP2040/Pico, SAM/SAMD51, Teensy — and for every pulse driver the library
offers on it.

### 1.1 What is under test — and what is not

The subject is the **queue layer**: `FastAccelStepper::addQueueEntry()`, the
ring queue behind it, and the pulse driver. The question is always the same
shape: *given these exact `stepper_command_s` values, what step/dir waveform
comes out of the pin?*

Explicitly **out of scope**:

| Not tested here | Why |
|-----------------|-----|
| The ramp generator (`move()`, `runForward()`, `setAcceleration`) | Pure integer math over `steps`/`accel`. Fully covered by `extras/tests/pc_based` (test_02, test_05, test_09, test_10) and SimAVR. A logic analyzer cannot see anything the math does not already determine. |
| `moveTimed()` | Nothing but an `addQueueEntry()` loop. Tested in pc_based (test_20, test_24, test_25). |
| The n-axis planner (`FasNAxis`) | Tested in pc_based (test_23–test_26). Its *pin* output is only interesting once the per-stepper queue is characterized. |

A test that only drives `move()` or `moveTimed()` and then counts pulses
belongs in `pc_based`, not here.

What a capture uniquely adds over every other suite is the **measured
waveform on real silicon**: pulse high time, dir→first-step delay, inter-step
period and jitter at high speed, cross-stepper start skew, and whether the
driver's overrun/limit handling is correct. None of that is observable from
`getCurrentPosition()`.

### 1.2 Characterization goals

The outcome of the whole suite is a characterization of the queue layer:

1. **Step pulse high time / duty cycle vs speed.** How wide is the pulse the
   driver emits, and how does it scale with the commanded `ticks`?
2. **Step timing vs speed, and vs stepper count.** Is the inter-step period
   exactly `ticks`, at 1…255 steps per command, at `ticks` = 1 and 65535? Does
   adding a second stepper perturb the first stepper's timing (on AVR it
   deliberately lowers the speed floor)?
3. **Dir change → first step.** How long after the dir edge does the first step
   of the reversed phase appear?
4. **Driver-specific edge behaviour.** MCPWM/PCNT counter-limit overrun,
   pause commands, synchronized start.
5. **Step rate adherence.** How closely does the achieved step rate follow the
   commanded one, and how much does the gap depend on the driver and on uC load?
6. **Synchronized start skew.** How close together do several steppers actually
   begin? Reported, not gated — see §1.3.

The required sample rate is modest: 4 MS/s is the practical minimum for a
16 MHz tick clock (see §2.1), so 200+ MS/s is not a requirement. All captured
signals are recorded with `sigrok-cli`, decoded in Python, and the results are
tagged with architecture, driver, channel configuration, and test metadata.

### 1.3 Measured vs asserted — driver capability limits

**An imperfect result is not automatically a defect.** Some of what the pins do
is limited by what the hardware can do, not by whether the queue is correct. A
pulse driver that steps the pin from an interrupt cannot start as precisely as
one that arms a hardware compare unit. Rejecting a capture for that would
report a platform characteristic as a bug — and, for the interrupt-driven
drivers, would fail those targets on every single run by construction.

So for every quantity this suite records, the pass criterion is decided by
asking whether a wrong value could mean the queue misbehaved:

| Quantity | Verdict | Why |
|----------|---------|-----|
| Step count per command | **asserted** | A swallowed or spurious step is a defect on any platform. |
| Inter-step period | **asserted** | Deviates only if the timer or the ISR is wrong. |
| Step rate adherence | **asserted** (2 %) | ISR cost and uC load shift it, but not by much, and a large sag is a real problem. |
| Pulse high time | **asserted** | Set by the driver logic, not by load. |
| Pause duration | **asserted** | The queue owns the timing. |
| Dir edge → first step | **measured** | The driver sets the minimum and the value depends on the driver, so the number is recorded; only the *existence* of a delay and the step counts are asserted. |
| **Synchronized start skew** | **measured** | Driver capability and uC load, not correctness. |

The distinction is deliberate: an asserted value that drifts is a regression to
chase, while a measured value is a number to compare across architectures,
drivers and uC load conditions. Both go into the result; only one decides
pass/fail.

Concretely, for synchronized start: `synchronizedStart()` asks every stepper to
begin, but the offset between them depends on how each driver starts stepping
and on what the processor is doing at that instant.

| Driver | Start mechanism | What the skew depends on |
|--------|-----------------|-------------------------|
| RMT | hardware compare on the RMT unit | a fixed few µs |
| MCPWM | hardware compare on the timer | a fixed few µs |
| PCNT | pin-change interrupt | ISR latency, so **uC load** |
| AVR | timer compare ISR | ISR latency, so **uC load** |
| I2S / mixed | different mechanisms per stepper | not comparable across steppers |

A stepper that begins a few microseconds late has not malfunctioned. The
harness therefore records the skew, each stepper's first-step timestamp, and
the skew expressed in step periods, and leaves the verdict to the step counts.

#### Global pin invariants

One rule is not tied to any scenario, because it is a property of the pin
protocol rather than of the command under test:

> **The direction pin must never change while the step pin is high.**

A stepper driver latches the direction on the STEP edge. A DIR transition inside
the pulse window can therefore make the driver decode the *new* direction for
that step, and the transition itself can glitch the DIR input while the coil is
being driven. The library avoids this by ordering `Stepper_ToggleDirection()`
before `Stepper_One()` in the same ISR, so the direction has settled before the
step is emitted — any occurrence is a defect.

It is checked across every stepper on **every** test, not only in the
direction-change scenario, and a violation fails the test regardless of what the
scenario's own checks concluded. A DIR edge during a high STEP would be a bug
anywhere in the program; a per-scenario check would only catch it in whichever
scenario happens to change direction.

A DIR change exactly on the rise or fall sample is the boundary, not a
violation — those are the two edges where a direction change belongs.

The golden fixture `bad_dir_during_step_high` exists to keep this honest: it has
a correct step count, a correct inter-step period and correct rate adherence, and
fails **only** on the invariant. If the scenario's own checks were the whole
verdict, it would pass.

#### Dir change → first step

The other measured quantity, and the one where the platforms differ by three
orders of magnitude. The time from the DIR edge to the first step of the
reversed phase is set by *how the driver starts stepping*, not by the queue:

| Platform | Mechanism | Programmed delay | Expected dir → step |
|----------|-----------|------------------|---------------------|
| **Pico** | PIO: the DIR `set` and the STEP `mov` are **adjacent instructions in the same program**, with no delay between them | none | **3–4 PIO cycles ≈ 40–50 ns** at 80 MHz |
| **AVR** | `Stepper_ToggleDirection()` then `Stepper_One()` in the same timer-compare ISR (`avr_queue.cpp:186-190`) | none | a few µs, ISR-latency dependent |
| **SAM / SAMD** | as AVR, plus a blocking `AFTER_SET_DIR_PIN_DELAY_US = 30` when the queue is idle at insert time | 30 µs (idle start only) | ~30 µs idle-start, a few µs while running |
| **Teensy** | as SAM, `AFTER_SET_DIR_PIN_DELAY_US = 5` | 5 µs (idle start only) | ~5 µs idle-start |
| **ESP32 MCPWM/PCNT** | **one `MIN_CMD_TICKS` pause with the OLD direction is inserted *before* the toggle**; the toggle lands at TEA of that pause with STEP already low, and the first step follows at the next compare (`FastAccelStepper.h:119`) | `MIN_CMD_TICKS` = 3200 ticks | **~200 µs** |
| **ESP32 RMT** | DIR and STEP as symbols in one RMT item stream | bounded by RMT symbol resolution | to be measured |
| **ESP32 I2S / mixed** | slot-based; different mechanisms per stepper | slot duration | to be measured |

So `MIN_CMD_TICKS` is the number to look at: it is 3200 ticks (200 µs) on
ESP32, Pico, SAM and SAMD, and 640 ticks (40 µs) on AVR, whose
`MIN_CMD_TICKS` is `TICKS_PER_S / 25000` rather than `/ 5000`.

The Pico row is the interesting one, and the reason this is a measurement: the
PIO sets DIR and STEP from the same instruction stream with no delay at all, so
the two edges are a handful of cycles apart and there is nothing to tune. The
ESP32 MCPWM path is the opposite — it deliberately spends a whole 200 µs pause
getting the direction settled before it dares step, which is a hardware
requirement of the MCPWM/PCNT compare scheme rather than a library choice.

**Resolution matters for this measurement.** One sample at the default 4 MS/s is
250 ns, so the Pico's ~50 ns cannot be resolved at all and would be reported as
"0 or 1 sample". Measuring SR_06/SR_10 on PIO targets needs a higher capture
rate:

| Capture rate | One sample | Resolves Pico's ~50 ns? |
|--------------|-----------|--------------------------|
| 4 MS/s (default) | 250 ns | no |
| 24 MS/s | 42 ns | marginally |
| 100 MS/s | 10 ns | yes |

Reporting is not optional, though: a metric that is never gated on is exactly
the kind that silently degrades to a constant. Every such metric is pinned by a
fixture with a known value, which must produce that value in the result — so the
number cannot quietly become 0.0.

---

## 2. Architecture

```
┌──────────────────────────────────────────────────────────────────────┐
│  Test Runner (Python)                                                │
│  ┌──────────┐  ┌─────────────┐  ┌───────────┐  ┌──────────┐       │
│  │ Build    │→ │ Flash target│→ │ sigrok-   │→ │ Python   │       │
│  │ & Flash  │  │             │   │ CLI       │   │ Analyzer │       │
│  └──────────┘  └─────────────┘   └───────────┘   └──────────┘       │
│       ▲                                              │               │
│       │              ┌─────────────┐                 │               │
│       └──────────────│  Tag DB     │◄────────────────┘               │
│                      │  (JSON)     │                                 │
│                      └─────────────┘                                 │
│                      ┌─────────────┐                                 │
│                      │  HTML Report │                                │
│                      │  (Browser)  │                                 │
│                      └─────────────┘                                 │
└──────────────────────────────────────────────────────────────────────┘
```

### 2.1 Capture Pipeline (`sigrok-cli`)

```bash
sigrok-cli \
  --driver saleae \
  --config channels:0,1,2,3,4,5,6,7,8,9 \
  --config sample-rate:1000000 \
  --trigger 'channel.8 rising' \
  --seconds 2 \
  --output capture_$(date +%Y%m%d_%H%M%S).sr
```

- **Channels**: 8–10 persistent channels (Step/Dir per stepper), plus optional
  trigger channels (e.g. `startQueue`, `queueEmpty`, `testMarker`).
- **Sample rate**: ≥ 1 MS/s (10× the highest step frequency; 16 MHz clock
  → max ~400 kHz step frequency → 4 MHz sample rate is the practical minimum).
- **Trigger**: Saleae CH 8 connected to a GPIO that the firmware toggles at
  `startQueue` — this ensures the capture window starts exactly when the test
  begins, avoiding long pre-trigger recordings.
- **Output format**: `.sr` (sigrok srzip) — lossless and, unlike CSV, compact:
  one packed byte per sample, so a 2 Msample 8-channel capture is 26 KB as
  `.sr` against 80 MB as CSV. High-rate captures are not truncated by the
  output stage. CSV remains available (`capture.py --format csv`) for eyeballing.
- **Evaluation format**: a VCD, derived from the `.sr` by sigrok-cli
  (`-I srzip -O vcd`, or `capture.py --vcd`). A VCD stores **only value
  changes**, so it stays compact and opens directly in GTKWave, and sigrok picks
  `$timescale` from the sample rate (1 us at 1 MHz, 100 ps at 48 MHz) so no
  timing resolution is lost. `signal_parser.load_vcd()` reads it back.
- **Driver selection**: `--driver saleae` is Saleae-specific. Other analyzers
  use different sigrok drivers (`fx2lafw`, `hantek_dso620`, …), so `capture.py`
  should map the detected device to the correct driver rather than hard-coding
  `saleae`.

### 2.2 Python Analyzer

```python
# Pipeline:
# 1. Convert the .sr capture to a change-only VCD (sigrok-cli), or read either
#    directly — signal_parser.load_capture() dispatches on the extension and
#    needs no third-party dependency (the srzip reader is plain zipfile).
# 2. Decode Step/Dir channels: detect rising/falling edges, compute pulse
#    widths, inter-step gaps, dir→step delay
# 3. Validate against the expected command stream (derived from the same
#    QSEG program the firmware ran, so the expectation is not a hand-copied
#    constant)
# 4. Write tagged JSON result + generate HTML report
```

Key metrics per channel:

| Metric | Description |
|--------|-------------|
| **pulse_width_us** | Step high time (high) / low time |
| **inter_step_period_us** | Time between successive step pulses (L→L or H→H) |
| **dir_to_first_step_us** | Time from direction-change edge to first step pulse |
| **duty_cycle** | High time / total period |
| **step_count** | Total step pulses counted from edges |
| **glitch_count** | Spurious edges (width < 1/2 MIN_CMD_TICKS) |
| **cross_channel_skew_us** | Max time difference between any two channels'
  first step pulse (for synchronized-start tests) |

### 2.3 Tag System

Every test result is tagged with a **composite key** that allows filtering,
comparison, and regression tracking across architectures, drivers, and
configurations.

#### 2.3.1 Tag Schema

```json
{
  "test_id": "test_01",
  "timestamp": "2026-10-01T12:00:00Z",
  "hardware": {
    "board": "esp32-devkitc",
    "chip": "ESP32",
    "idf_version": "5.3",
    "arch": "esp32"
  },
  "driver": {
    "type": "rmt_v2",
    "channel": 0,
    "channel_group": 0
  },
  "channel_config": {
    "mode": "8ch_step_only",
    "stepper_count": 4,
    "stepper_A": { "step_pin": 2,  "dir_pin": 0,   "driver": "rmt_v2" },
    "stepper_B": { "step_pin": 4,  "dir_pin": 16,  "driver": "rmt_v2" },
    "stepper_C": { "step_pin": 17, "dir_pin": 5,   "driver": "rmt_v2" },
    "stepper_D": { "step_pin": 18, "dir_pin": 19,  "driver": "rmt_v2" }
  },
  "test_case": {
    "name": "basic_move_forward",
    "type": "ramp",
    "steps": 1000,
    "speed_us": 400,
    "acceleration": 10000
  },
  "results": {
    "passed": true,
    "metrics": {
      "stepper_A": {
        "step_count": 1000,
        "expected_count": 1000,
        "dir_to_first_step_us": 12.5,
        "avg_inter_step_us": 0.8,
        "max_pulse_width_us": 0.4,
        "glitch_count": 0
      }
    },
    "errors": []
  },
  "tags": ["esp32", "rmt_v2", "8ch_step_only", "idf5", "ramp", "passed"]
}
```

#### 2.3.2 Tag Index

Tags are stored in a JSON index file (`tag_index.json`) for fast lookup:

```json
{
  "esp32_rmt_v2_8ch_step_only": [
    "2026-10-01_test_01_esp32_rmt_v2_8ch_step_only.json",
    "2026-10-02_test_05_esp32_rmt_v2_8ch_step_only.json"
  ],
  "esp32_mcpwm_pcnt_4ch_rmt": [
    "2026-10-01_test_01_esp32_mcpwm_pcnt_4ch_rmt.json"
  ],
  "esp32c3_rmt_v2_2rmt_2i2s": [
    "2026-10-03_test_09_esp32c3_rmt_v2_2rmt_2i2s.json"
  ]
}
```

#### 2.3.3 Pre-defined Tag Categories

| Category | Values |
|----------|--------|
| **arch** | `esp32`, `esp32s2`, `esp32s3`, `esp32c3`, `esp32c6`, `esp32h2`, `esp32p4`, `avr`, `pico`, `sam`, `samd51`, `teensy` |
| **driver** | `mcpwm_pcnt`, `rmt`, `rmt_v2`, `i2s_direct`, `i2s_mux` |
| **idf_version** | `idf4`, `idf5`, `idf6` |
| **channel_config** | `8ch_step_only`, `7ch_shared_dir`, `4ch_rmt`, `4ch_mcpwm`, `2rmt_2i2s`, `4ch_i2s_extender`, `6ch_i2s_mux`, `mixed` |
| **test_type** | `ramp`, `sync_start`, `dir_change`, `abrupt_speed`, `queue_full`, `move_timed`, `pause`, `overflow`, `speed_limit`, `queue_fill_latency` |
| **result** | `passed`, `failed`, `skipped`, `error` |

Composite tag key format: `{arch}_{driver}_{channel_config}` — used as the
primary index key.

---

## 3. Driver Families and Channel Configuration Model

The harness is architecture-agnostic. Every architecture gets the same
characterization run against **its own** drivers, because the whole point is
that the numbers differ per architecture and per driver and must be measured
rather than assumed.

### 3.1 Driver Families

| Architecture | Pulse driver | Driver families | Max steppers |
|---|---|---|---|
| AVR (`pd_avr`) | hardware timer compare (OC1A/OC1B, …) | `timer` only | 2 (328P) / 3 (2560, 32U4) |
| ESP32 family (`pd_esp32`) | RMT / MCPWM+PCNT / I2S | `mcpwm_pcnt`, `rmt` (V1), `rmt_v2`, `i2s_direct`, `i2s_mux` | up to 49 (see below) |
| Pico / RP2040 (`pd_pico`) | PIO state machine | `pio` | `4 × NUM_PIOS` |
| SAM (`pd_sam`) | TC timer compare | `timer` | `NUM_QUEUES` (6 on the tested parts) |
| SAMD51 (`pd_samd`) | TC timer compare | `timer` | `TCC_INST_NUM` (chip dependent) |
| Teensy 4.x (`pd_teensy`) | interval timer + FlexPWM | `flexpwm`, `interval` | 16 |

`SUPPORT_SELECT_DRIVER_TYPE` only exists on the ESP32 family, so `CONFIG mixed
<drivers>` is an ESP32-only concept; elsewhere the single native driver is used
and `1ch` / `2ch` are the meaningful configuration axes.

#### ESP32 sub-types, in detail

| Family | Sub-type | Supported IDF | Queues (typical) | Description |
|--------|----------|---------------|------------------|-------------|
| **MCPWM/PCNT** | `mcpwm_pcnt` | IDF4, IDF5, IDF6 | 2–8 | Uses MCPWM timer for step pulses + PCNT for direction counting. Classic ESP32/ESP32-S3 approach. |
| **RMT** (V1) | `rmt` | IDF4 | 2–8 | Raw RMT memory buffer, two-part split. Older driver. |
| **RMT** (V2) | `rmt_v2` | IDF5.3+, IDF6 | 2–8 | New RMT TX driver API with fill encoder. Supports sync manager on targets with `SOC_RMT_SUPPORT_TX_SYNCHRO`. |
| **I2S Direct** | `i2s_direct` | IDF5+, IDF6 | 0–3 | I2S bus used as a parallel step output (16-bit per sample). Direct GPIO mapping. |
| **I2S Mux** | `i2s_mux` | IDF5+, IDF6 | 0–32 | I2S bus with GPIO matrix mux for more channels (up to 32). |

#### Per-chip queue allocation (IDF5/6 example):

| Chip | MCPWM/PCNT | RMT | I2S Direct | I2S Mux | Total |
|------|-----------|-----|------------|---------|-------|
| ESP32 | 6 | 8 | 3 | 32 | 49 |
| ESP32-S2 | 0 | 4 | 3 | 32 | 39 |
| ESP32-S3 | 4 | 4 | 3 | 32 | 43 |
| ESP32-C3 | 0 | 2 | 3 | 32 | 37 |
| ESP32-C6 | 2 | 2 | 3 | 32 | 39 |
| ESP32-H2 | 2 | 2 | 3 | 32 | 39 |
| ESP32-P4 | 0 | CONFIG | 3 | 32 | variable |

**IDF6 note:** ESP32-C6 and ESP32-H2 report `QUEUES_MCPWM_PCNT 0` under IDF6
(`pd_config_idf6.h`), so their MCPWM/PCNT count of 2 applies to IDF5 only —
those chips have **no** MCPWM/PCNT driver in IDF6.

The I2S driver families (`i2s_direct`, `i2s_mux`) are **ESP32-only**
(`SUPPORT_ESP32_I2S`), so the tests that target them are simply not run
elsewhere. Everything else in the catalogue is architecture-independent and must
be run on **every** architecture — that comparison is where the value is.

Two architecture facts shape the test plan itself:

- **AVR speed floor depends on the stepper count.**
  `StepperQueue::adjustSpeedToStepperCount()` (`src/pd_avr/avr_queue.cpp`) sets
  `max_speed_in_ticks` to `TICKS_PER_S/50000` with one stepper but **426** with
  two, because the ISR needs ~14 us. So `1ch` and `2ch` must both be
  characterized on the same board.
- **AVR step pin is not a free choice.** It must be the pin the library maps to
  the timer compare output (`stepPinStepperA`/`stepPinStepperB` in
  `src/AVRStepperPins.h`), and *which* physical pin that is depends on
  `FAS_TIMER_MODULE`. Never hardcode it.

### 3.2 Channel Configuration Modes

The test app shall support the following **channel configuration presets**,
selectable at build time or via runtime command:

| Config ID | Description | Steppers | Driver Assignment |
|-----------|-------------|----------|-------------------|
| **`8ch_step_only`** | 8 channels, Step only (no Dir). Each stepper has its own Step channel. | 8 | All RMT V2 (or all MCPWM) |
| **`7ch_shared_dir`** | 7 channels, shared Dir line. One Dir drives all steppers' direction. | 7 | Mixed (4 RMT + 3 MCPWM) |
| **`4ch_rmt`** | 4 steppers, all using RMT driver (Step + Dir each). | 4 | All RMT V2 |
| **`4ch_mcpwm`** | 4 steppers, all using MCPWM/PCNT driver. | 4 | All MCPWM/PCNT |
| **`2rmt_2i2s`** | 2 steppers on RMT + 2 steppers on I2S Direct. | 4 | 2 RMT V2 + 2 I2S Direct |
| **`4ch_i2s_extender`** | 4–8 steppers on I2S Extender (GPIO matrix mux). | 4–8 | I2S Mux |
| **`6ch_i2s_mux`** | 6 steppers on I2S Mux (maximizing mux channels). | 6 | I2S Mux |
| **`mixed`** | Arbitrary mix of RMT + MCPWM + I2S. Configurable per-stepper. | 1–8 | Per-stepper driver selection |

#### Runtime Configuration Interface

```c
// In the test firmware:
typedef enum {
  CH_CONFIG_8CH_STEP_ONLY,
  CH_CONFIG_7CH_SHARED_DIR,
  CH_CONFIG_4CH_RMT,
  CH_CONFIG_4CH_MCPWM,
  CH_CONFIG_2RMT_2I2S,
  CH_CONFIG_4CH_I2S_EXTENDER,
  CH_CONFIG_6CH_I2S_MUX,
  CH_CONFIG_MIXED
} channel_config_t;

// Configure channels before initializing steppers:
void configure_channels(channel_config_t config);

// Per-stepper driver override (for mixed mode):
void set_stepper_driver(uint8_t stepper_idx, FasDriver driver);
```

### 3.3 Channel Pin Mapping

The firmware derives its pins per architecture rather than from a table (see
`common/saleae_app.cpp`); AVR uses the library's `stepPinStepperA/B` macros.
The ESP32-DevKitC mapping is the worked example:

```
Config: 4ch_rmt (default for most testing)

Stepper A:  Step=GPIO2,  Dir=GPIO0   (RMT channel 0)
Stepper B:  Step=GPIO4,  Dir=GPIO16  (RMT channel 1)
Stepper C:  Step=GPIO17, Dir=GPIO5   (RMT channel 2)
Stepper D:  Step=GPIO18, Dir=GPIO19  (RMT channel 3)

Saleae CH 0: Step A  (GPIO2)
Saleae CH 1: Dir A   (GPIO0)
Saleae CH 2: Step B  (GPIO4)
Saleae CH 3: Dir B   (GPIO16)
Saleae CH 4: Step C  (GPIO17)
Saleae CH 5: Dir C   (GPIO5)
Saleae CH 6: Step D  (GPIO18)
Saleae CH 7: Dir D   (GPIO19)
Saleae CH 8: Test marker (GPIO25 — toggled at startQueue)
Saleae CH 9: Queue empty (GPIO26 — toggled when queue empties)
```

**Hardware notes:**
- **GPIO0**: Must be **HIGH** at boot (otherwise ESP32 enters download mode).
  Use an external pull-up resistor (4.7 kΩ to 3.3 V) if the board does not
  provide one.
- **GPIO2**: Often connected to an onboard LED on ESP32 dev kits. Disconnect
  the LED (or desolder Rxx) if it interferes with stepper signals.
- All pins (2, 0, 4, 16, 17, 5, 18, 19, 25, 26) are **output-capable** on the
  ESP32. No input-only pins are used, and no GPIO is assigned to two roles.
- The trigger/marker pins (25, 26) must be **distinct** from every Step/Dir
  pin. Do not reuse a stepper GPIO as a trigger.

**Reuse the built-in test probes:** the library already ships probe macros in
`src/pd_esp32/test_probe.h` that toggle a GPIO directly from the RMT ISR —
`PROBE_1` at `startQueue`/queue stop (double-toggle at start), `PROBE_2` at the
end interrupt, `PROBE_3` at the threshold interrupt, `PROBE_4` on command
completion. `PROBE_1` can serve directly as the Saleae trigger, giving
ISR-accurate timing instead of application-level GPIO toggling. Caveats: the
probes are **disabled by default** (enable `ESP32_TEST_PROBE` /
`ESP32C3_TEST_PROBE`), are currently defined **only for ESP32 and ESP32-C3**
(not S3/C6/H2/P4), and exist only in the RMT drivers (not MCPWM/PCNT or I2S).

---

## 4. Firmware Architecture — Generic App Driving `addQueueEntry()` Directly

### 4.1 Design Philosophy

The test firmware is a **single generic application** that feeds
`addQueueEntry()` directly. It avoids maintaining separate firmware binaries per
test case, and it runs unchanged on every platform (AVR, ESP32 variants, Pico,
SAM) with platform differences handled by conditional compilation.

It contains **no ramp generator usage at all** — no `move()`, no `moveTimed()`,
no `setAcceleration()`. Everything the analyzer sees is the consequence of
explicit `stepper_command_s` values that the host chose.

### 4.2 Command Format

`addQueueEntry()` takes the *public* command struct, not the internal queue
entry (`src/fas_arch/common.h`):

```c
struct stepper_command_s {
  uint16_t ticks;    // ticks between steps; TICKS_PER_S ticks per second
  uint8_t  steps;    // 1..255 steps, or 0 = pause for `ticks` ticks
  bool     count_up; // direction pin high (true) / low (false)
};
```

Rules a caller must respect, all enforced by `addQueueEntry()` itself:

- `ticks >= getMaxSpeedInTicks()` → else `AQE_ERROR_TICKS_TOO_LOW`
- `ticks * steps >= MIN_CMD_TICKS` → else `AQE_ERROR_TICKS_TOO_LOW`
- `ticks` is `uint16_t`, so a period above 65535 ticks needs `steps=1`
  followed by `steps=0` pause entries
- `steps` is `uint8_t`, so a long run needs repeated commands
- `count_up=false` with no dir pin set → `AQE_ERROR_NO_DIR_PIN_TO_TOGGLE`
- A retriable code is any **positive** value (`aqeRetry()`):
  `QueueFull`, `DirPinIsBusy`, `WaitForEnablePinActive`, `DeviceNotReady`,
  `DirPin2msPauseAdded`, `DirChangePauseInjected`. The last one means the
  driver inserted a DIR drain pause and did **not** take the command, so the
  identical command must be resubmitted **immediately** (`aqeRetryImmediately()`)
  with no delay.
- Kick-off is `addQueueEntry(NULL, true)`, or
  `FastAccelStepperEngine::synchronizedStart()` for several steppers on one
  shared timer compare.

### 4.3 Why the Plan Lives in the Firmware, Not on the Wire

Both obvious alternatives were rejected:

**Streaming `queue_entry` structs over serial does not work.** At 115200 baud,
transferring a few hundred entries takes ~0.3–0.6 s. A stepper commanded at
200 kSteps/s emits ~60 000 steps in that time, so the plan would be consumed
long before the last entry arrived — the capture would show a burst, a long
silence, then a burst. Raising the baud to 921600 helps the wire but not the
semantics: the firmware still cannot start the queue until the whole plan has
landed, which still wastes the capture window. (A held stepper with auto-enable
off makes it worse, not better: the *first* step is then gated on the
*last* byte.)

**Hard-coding every scenario in flash does not work either.** The whole point is
to *sweep* `ticks` and `steps` to find where behaviour breaks — the speed floor,
the 1…255 steps-per-command range, the 16-bit period boundary. Each value is a
compile-time constant, so a sweep means a rebuild per point.

**What the firmware does instead.** The host sends one short line per segment
(`QSEG <steps> <ticks> <dir>`); the firmware keeps a bounded program of at most
`QE_MAX_SEG = 8` segments and generates the queue entries itself, from
`saleae_app_loop()`, exactly when the queue has room. So:

- the wire carries ~15 bytes per segment, not a 6-byte struct per queue entry;
- the RAM is `8 × sizeof(segment) = 48` bytes, **one shared copy** for all
  steppers, plus an 8-byte cursor per stepper;
- a parameter sweep is a new serial line, not a new binary;
- a whole scenario is a handful of commands, which is all the host needs to
  express anyway.

### 4.3.1 Platform RAM Analysis

The harness's own footprint is negligible, which is the point: the queue itself
is what consumes RAM, and it is already accounted for.

| Platform | SRAM | `QUEUE_LEN` | Queue RAM | Harness program | Per-stepper cursor |
|----------|------|-------------|-----------|-----------------|--------------------|
| ATmega328P | 2 KB | 16 | 96 B | 48 B | 8 B × 2 = 16 B |
| ATmega2560 | 8 KB | 16 | 96 B | 48 B | 8 B × 3 = 24 B |
| ESP32 (all) | 512 KB | 32 | 192 B | 48 B | 8 B × N |
| RP2040 | 264 KB | 32 | 128 B | 48 B | 8 B × N |

**AVR is the binding constraint** and is the reason for §4.3: with 2 KB of SRAM
on a 328P, of which the engine and two stepper objects already claim a large
part, a download buffer of even a few hundred bytes would have been the single
largest allocation in the firmware. `SALEAE_MAX_STEPPERS` is derived from
`MAX_STEPPER` for the same reason (2 on a 328P), so `CONFIG 4ch_*` cannot
work there and `1ch`/`2ch` exist to characterize both cases.

### 4.4 Firmware Structure

The firmware is small on purpose. It holds a segment program, feeds it into
`addQueueEntry()`, and reports back — there is no test-case library, no
per-driver adapter layer and no scenario code, because a scenario is just a few
`QSEG` lines the host sends.

```
extras/tests/saleae_based/
├── common/                       ← compiled for every target
│   ├── saleae_app.{h,cpp}          command parser + the QSEG/QRUN feeder
│   ├── saleae_test.{h,cpp}         SR_00 pin self-test
│   └── saleae_hal.h                gpio / millis / delay / serial
│   ├── saleae_hal_arduino.cpp      HAL for Arduino (AVR, ESP32, Pico, SAM)
│   └── saleae_hal_espidf.cpp       HAL for plain ESP-IDF
├── apps/                         ← thin entry points, no logic
│   ├── arduino/saleae_main.ino     setup() / loop()
│   └── espidf/saleae_main.cpp      app_main() + CMakeLists.txt
└── scripts/                      ← host side
    ├── harness.py                 arch/framework/driver -> env + tag key
    ├── run_tests.py               program, capture, evaluate, record
    ├── control.py                 manual serial
    ├── capture.py                 sigrok-cli wrapper (.sr, --vcd)
    ├── analyze_csv.py             SR_00 evaluation
    └── signal_parser.py           edges/metrics core
```

Only `apps/` differs per framework; everything else is shared, so an AVR build
and an ESP-IDF build exercise the same feeder.

The firmware is assembled into `pio_dirs/saleae` (Arduino) and
`pio_espidf/saleae` (ESP-IDF) by `extras/scripts/build-pio-dirs.sh` using
symlinks; those generated dirs are git-ignored and the symlinks must never be
committed.

#### 4.4.1 Targets

| Framework | Targets | Entry point |
|-----------|---------|-------------|
| Arduino | ATmega168/328P/2560/32U4, RP2040/Pico, SAM/SAMD51, Teensy, ESP32 | `apps/arduino/saleae_main.ino` |
| ESP-IDF | ESP32, ESP32-S2/S3, C3, C6, H2, P4 | `apps/espidf/saleae_main.cpp` |

### 4.5 Serial Command Protocol

The whole surface. Note how small it is: everything else is assembled from
`QSEG` lines.

| Command | Response | Description |
|---------|----------|-------------|
| `SR00` | `OK SR00` | SR_00 pin self-test (§5.0) |
| `CONFIG <name> [d0,d1,…]` | `OK CONFIG <name> n=<N> maxspeed<i>=<ticks> …` | Connect N steppers. `1ch`, `2ch`, `4ch_rmt`, `4ch_mcpwm`, `mixed <drv,drv,…>` (drivers `rmt`, `mcpwm`, `i2s`, `i2s_mux`, `auto`) |
| `QINFO` | `QINFO tps=… mincmd=… qlen=… maxspeed=…` | Platform limits the host must respect |
| `QCLR` | `OK QCLR` | Drop the program, stop everything |
| `QSEG <steps> <ticks> <dir>` | `OK QSEG <n>/8` | Append a segment. `steps=0` means "pause for `<ticks>` ticks". `dir` is 0 or 1 |
| `QRUN <mask>` | `OK QRUN` … later `DONE <pos…>` | Run the program on the steppers in the bitmask, synchronized start |
| `POS` | `POS <pos…>` | Current position of every connected stepper |
| `STOP` | `OK STOP` | Stop, clear the program and the self-test |

Responses are `OK …` / `DONE …` / `ERR <code>` so the host can always detect a
failure.

**Why `ticks` and not microseconds.** `ticks` is the raw `uint16_t` queue
period. Exposing it directly is what makes the interesting boundaries
addressable: `ticks` = 1, `ticks` = 65535 (the 16-bit maximum), and
`steps` = 1…255 (the `uint8_t` maximum). A microsecond interface cannot
express those exactly. The host reads `TICKS_PER_S` from `QINFO` and converts.

**Why `QRUN` takes a mask.** The multi-stepper tests need to select which
steppers participate — `QRUN 1` for one, `QRUN 3` for A+B. This is also how the
AVR speed-floor comparison works: `adjustSpeedToStepperCount()`
(`src/pd_avr/avr_queue.cpp`) sets `max_speed_in_ticks` to `TICKS_PER_S/50000`
with one stepper but **426** with two, so the same board has to be measured
under both configurations.

**`CONFIG` cannot be re-applied.** Each queue can only be allocated once per
boot (`stepper_allocated_mask` on AVR), so a second `CONFIG` reports the
existing setup rather than silently running with a different pin map than the
host believes.

**Prefill and kick-off.** `qe_pump()` fills every selected queue to half its
depth, then calls `synchronizedStart()` once all participants are ready, then
keeps topping up as the queue drains — the same prefill/kick-off protocol as
`FasNAxis`. An empty queue after kick-off while the program still has steps
would be an underrun and is a test failure.

**Example — the MCPWM overrun case.** 255 steps at the speed floor, a pause,
then a single step. The PCNT high limit is re-armed from the running counter
value on every command (`StepperISR_idf5_esp32_mcpwm_pcnt.cpp`), and a
`steps=1` command immediately after a 255-step run is exactly where a stale or
mis-computed limit shows up as a lost or extra pulse:

```
QCLR
QSEG 255 80 1      # 80 = the ESP32 max-speed floor (from QINFO)
QSEG 0 1600 1      # pause
QSEG 1 80 1        # single step after the pause
QRUN 1
```

## 5. Test Case Catalogue

**Every SR test is a `QSEG` program, and every metric is a measured waveform.**
The catalogue is organised by *characterization goal*, not by which PC test it
resembles. Tests whose subject is the ramp generator, `moveTimed()`, or the
n-axis planner were removed (§1.1) — they belong in `pc_based`.

Each entry lists the program and what the capture must show. `ticks` values are
examples; the host must substitute the value `QINFO` reports for the target.

### 5.0 Category: Connection Verification

Runs **before** anything else. It proves every analyzer channel is electrically
connected to the right GPIO and the firmware can toggle it. Not a queue test,
but a broken channel invalidates every measurement below it, so it gates the
suite.

| Test ID | Name | Program | Saleae Check |
|---------|------|---------|--------------|
| **SR_00** | `port_toggle` | 8 pins, 1 Hz, high times 50/100/…/400 ms | Clean square wave on all 8 channels, correct frequency. Duties are all distinct and none is 50 %, so an inverted channel reads as the complement duty (95/…/60 %) and is immediately recognisable. |

### 5.1 Category: Step Timing — the core characterization

This is the category that justifies the harness: at the pin, on real silicon,
across the full range the 16-bit `ticks` and 8-bit `steps` fields allow.

| Test ID | Name | Program | Saleae Check |
|---------|------|---------|--------------|
| **SR_01** | `period_exact` | `QSEG 8 <ticks> 1` | Inter-step period equals `ticks` within one sample. Proves the commanded period is what reaches the pin. |
| **SR_02** | `steps_per_command_1_255` | `QSEG <n> <ticks> 1`, `n` = 1…255 | Exactly `n` pulses; period = `ticks` for all of them; nothing merges or doubles at `n = 1` or `n = 255`. Sweeps the whole `uint8_t` range. |
| **SR_27** | `single_step` | `QSEG 1 <ticks> 1` | One step in one command. Not just the low end of the SR_02 sweep: `steps == 1` takes the other ISR branch (`e->steps > 1` is false, so the read pointer advances and the next entry is stepped in the same interrupt), and it is the only command that yields no inter-step period to measure. |
| **SR_03** | `ticks_min` | `QSEG 1 <max_speed> 1` | The speed floor itself: clean pulses at `getMaxSpeedInTicks()`, and `QRUN` refused below it (`ERR QE ticks … < maxspeed …`). |
| **SR_04** | `ticks_max_16bit` | `QSEG 1 65535 1` | The longest period the 16-bit field allows. Confirms no wrap to a short period. |
| **SR_05** | `pulse_high_time` | `QSEG 16 <ticks> 1`, sweep `ticks` from the floor to 65535 | **Primary characterization output**: step pulse high time (and low time) vs speed, i.e. duty cycle. This is the number that has no counterpart in `getCurrentPosition()`. |
| **SR_06** | `trailing_wait` | `QSEG 2 <ticks> 1`, `QSEG 2 <ticks> 1` | A command with `steps=n` occupies `n × ticks`, including a trailing `ticks` wait after the last step. So the gap between command *k*'s last step and command *k+1*'s first step is exactly `ticks`, not `2 × ticks`. Catches off-by-one in the tick accounting. |
| **SR_07** | `long_run_no_underrun` | `QSEG 2000 <ticks> 1` | 2000 pulses with no gap larger than `ticks`, i.e. `qe_pump()` kept the queue fed without underrunning. Also checks position: `POS` must read 2000. |
| **SR_08** | `queue_full_no_loss` | `QSEG 4000 <ticks> 1` at a speed that outruns the feeder | Exactly 4000 pulses. A `QueueFull` retry must never drop or duplicate a command. |
| **SR_09** | `pause_command` | `QSEG 5 <ticks> 1`, `QSEG 0 <p> 1`, `QSEG 5 <ticks> 1` | Gap of exactly `p` ticks with no pulses in it, and **the dir pin must not change** on a pause (`steps == 0` still carries `count_up`, but it emits no step and no DIR toggle). |
| **SR_10** | `dir_change_first_step` | `QSEG 20 <ticks> 1`, `QSEG 20 <ticks> 0` | **Primary characterization output**: time from the dir edge to the first step of the reversed phase. Must be ≥ the driver's DIR drain pause (`MIN_DIR_DELAY_US`), and steps must not be emitted while dir is still settling. Position after: 0. |
| **SR_11** | `dir_change_both_ways` | `QSEG 20 <ticks> 0`, `QSEG 20 <ticks> 1` | Same in the other direction; confirms the pause is symmetric and not dependent on which way the pin goes. |
| **SR_12** | `multi_step_direction` | `QSEG 10 <ticks> 1`, `QSEG 10 <ticks> 0`, `QSEG 10 <ticks> 1` | Three direction changes in one program; cumulative position returns to +10 and the dir pin tracks every phase. |
| **SR_13** | `ticks_error_rejected` | `QSEG 8 <max_speed - 1> 1` | The firmware refuses it up front (`ERR QE ticks … < maxspeed …`) and **no pulse is emitted**. A rejection that still steps would be a serious bug. |

### 5.2 Category: Multi-Stepper — timing and synchronization

Synchronized start skew is **measured, not asserted** — the rule and the
per-driver reasoning are in §1.3. What is asserted here is the step count per
stepper, since a swallowed or spurious step is a real defect regardless of what
the driver supports.

The uC-load dependence is worth measuring rather than assuming away: the same
board can show a different skew with the steppers idle than with a timer or
UART competing for the same core, which is why the value is recorded per tag
key and compared across runs instead of being reduced to a pass or fail.



| Test ID | Name | Program | Saleae Check |
|---------|------|---------|--------------|
| **SR_14** | `sync_start_skew` | `QRUN 0b11`, then 2000 steps at `<ticks>` | **Measures** the offset between the first step of each stepper. Not gated on: see §5.2. Step counts are still checked. |

| **SR_15** | `sync_start_diff_speed` | `QSEG <n> <ticks_a> 1`, `QRUN 3` at a speed valid for both | Same first-step instant, then each stepper runs at its own period — proves the arm is aligned but the periods stay independent. |
| **SR_16** | `multi_stepper_timing_impact` | `CONFIG 1ch`, `QSEG 64 <ticks> 1`, `QRUN 1`; then `CONFIG 2ch`, same, `QRUN 3` | **Does the second stepper perturb the first?** Compare SR_01's period from the `1ch` run against the same channel's period in the `2ch` run. On AVR the answer is structural: the floor rises from `TICKS_PER_S/50000` to 426 ticks, so a sweep done only at `1ch` would report a speed the board cannot sustain with two steppers connected. |
| **SR_17** | `sync_cross_driver` | `CONFIG mixed rmt,mcpwm`, `QSEG <n> <ticks> 1`, `QRUN 3` | Cross-driver start skew — the hardest case, since RMT and MCPWM+PCNT arm through entirely different hardware. ESP32 only. |

### 5.3 Category: Driver Edge Behaviour

The cases where a driver's own limit/overrun handling is wrong. These are the
highest-value tests in the catalogue, because the symptom is *silent* — the
step count still looks plausible.

| Test ID | Name | Program | Saleae Check |
|---------|------|---------|--------------|
| **SR_18** | `mcpwm_overrun_after_255` | `QSEG 255 <max> 1`, `QSEG 0 <p> 1`, `QSEG 1 <max> 1` | Exactly 255 pulses, then a gap of `p`, then **exactly 1**. The PCNT high limit is re-armed from the live counter value on every command (`StepperISR_idf5_esp32_mcpwm_pcnt.cpp`); a `steps=1` command straight after a 255-step run is precisely where a stale or mis-computed limit produces a lost or extra pulse. |
| **SR_19** | `mcpwm_overrun_boundary` | `QSEG <n> <max> 1` for `n` = 200…255, each followed by `QSEG 1 <max> 1` | Sweeps the suspicious band. The 8-bit counter and the MCPWM timer period interact differently at each `n`; one `n` is enough to lose the trailing step. |
| **SR_20** | `pause_after_full_command` | `QSEG 255 <ticks> 1`, `QSEG 0 <p> 1`, `QSEG 255 <ticks> 1` | A full 255-step run, a pause, then another full run: 255 / gap / 255. Complements SR_18 by making the *second* command the large one. |
| **SR_21** | `rmt_buffer_split` | `QSEG 200 <ticks> 1` | RMT V1 splits its hardware buffer at a command boundary. A split at the wrong step shows up as one irregular inter-step gap. |
| **SR_22** | `rmt_v2_encoder` | `QSEG 200 <ticks> 1` | RMT V2 fill-encoder output has no irregular gap. |
| **SR_23** | `i2s_timing` | `QSEG 64 <ticks> 1` | I2S direct/mux step output at the correct intervals. ESP32 only. |
| **SR_24** | `avr_timer_timings` | `QSEG 64 <ticks> 1` on each of Timer1/3/4/5 | Each AVR timer channel produces the commanded period on its OC pin. |

### 5.4 Category: Limits and Error Conditions

| Test ID | Name | Program | Saleae Check |
|---------|------|---------|--------------|
| **SR_25** | `emergency_stop` | long `QSEG 2000 …`, then `STOP` mid-run | Pulses cease immediately; position frozen at whatever was completed; no partial pulse. |
| **SR_26** | `pause_ticks_max` | `QSEG 1 65535 1`, `QSEG 0 65535 1` | A pause of exactly 65535 ticks — the 16-bit boundary on the pause path too, which is a separate field from `ticks*steps`. |

The catalogue is intentionally small — 26 ids of which 11 are automated today
(§14) — and every one is either a measured waveform property or a driver limit.
Nothing in it re-verifies arithmetic, and nothing in it needs more than a
handful of `QSEG` lines.

---

## 6. Relationship to the PC-Based and SimAVR Suites

The Saleae suite does **not** mirror the PC-based test list; the two are
related by **layer**, not one-to-one:

| Layer | Suite | Subject |
|-------|-------|---------|
| Algorithm | `pc_based` test_01–test_30 | Ramp calculator, `moveTimed()`, n-axis planner, position bookkeeping — deterministic, no hardware |
| Simulated timing | `simavr_based` `test_sd_*` | AVR ISR timing, timer resources, VCD traces from `run_avr` |
| **Pin-level characterization** | **`saleae_based` `SR_*`** | **What `addQueueEntry()` + the driver actually emit on the wire** |

Two consequences:

1. A PC test whose subject is arithmetic gets **no** Saleae twin. The only way
   to earn one is to show that the pin waveform carries information the
   arithmetic does not already determine.
2. `SR_*` results are keyed per architecture *and* driver (§2.3), because the
   whole purpose is to record that e.g. the AVR speed floor is 320 ticks with
   one stepper and 426 with two, and the ESP32 MCPWM/PCNT floor is 80 ticks.
   Those numbers are properties of the silicon, and the tag system exists so
   they can be compared across targets rather than assumed.

---

## 7. Design-Spec Comparison

Every test result is compared against **design specification values** defined
per platform/driver combination. The design specs are stored in
`design_specs.json` and define the expected performance bounds for each metric.

### 7.1 Design Spec Schema

```json
{
  "esp32_rmt_v2": {
    "SR_06_direction_to_step_delay": {
      "expected_us": 12.5,
      "tolerance_us": 2.0,
      "description": "Time from DIR edge to first step pulse at 400us speed"
    },
    "SR_07_step_pulse_width_at_400us_speed": {
      "expected_us": 0.4,
      "tolerance_us": 0.1,
      "description": "Step pulse width at 400us inter-step period"
    },
    "SR_08_duty_cycle_deviation": {
      "max_percent": 5.0,
      "description": "Maximum duty cycle deviation from 50%"
    },
    "SR_13_sync_start_skew_us": {
      "max_us": 0.0625,
      "description": "Max cross-channel skew at synchronized start (1 tick at 16MHz)"
    }
  },
  "esp32_mcpwm_pcnt": {
    "SR_06_direction_to_step_delay": {
      "expected_us": 15.0,
      "tolerance_us": 3.0,
      "description": "MCPWM has additional setup latency"
    }
  }
}
```

### 7.1.1 Baseline-only metrics (no fixed expected value)

Some metrics have **no absolute design bound** because they are properties of
the platform/driver rather than of the library. These are recorded as
*measured baselines* and compared against the stored baseline for the same
platform/driver, not against a fixed number. `SR_11` (driver max speed / min
inter-step period) is the canonical case:

```json
{
  "esp32_rmt_v2": {
    "SR_11_max_speed": {
      "baseline_min_inter_step_us": 0.4,
      "baseline_max_frequency_khz": 2500,
      "description": "Measured clean-pulse limit; processor- and driver-specific (MCU clock + driver engine/divider + active channels). Baseline only, re-measured per processor/driver."
    }
  }
}
```

The max speed is limited by, among others:

- the **processor** — MCU clock frequency and the resolution of the timer/tick
  source it provides (e.g. 16 MHz timer tick vs. 240 MHz SoC clock),
- the **driver engine** — RMT symbol rate, MCPWM/PCNT timer clock, I2S bit
  clock, PIO SM clock, or a CPU timer — and its divider,
- the number of simultaneously active channels sharing that clock,
- the minimum command tick (`MIN_CMD_TICKS`) and the step high/low encoding.

Because the limit is **processor- and driver-specific**, SR_11 is measured and
stored per `{processor (arch), driver}` — the same tag key used for all results
(`{arch}_{driver}_{channel_config}`). It reports the discovered limit and flags
a **regression** only if it drops below the stored baseline by more than a
tolerance.

### 7.2 Per-Test Spec Comparison

Each test result includes a `spec_comparison` section that maps measured values
to their design spec bounds:

```json
{
  "test_id": "SR_05",
  "segments": [[16, 1600, 1]],
  "dut": {
    "ticks_per_s": 16000000,
    "min_cmd_ticks": 3200,
    "queue_len": 32,
    "max_speed_ticks": 80
  },
  "detail": {
    "expected_period_us": 100.0,
    "avg_high_us": 0.44,
    "avg_low_us": 99.56,
    "duty_percent": 0.44,
    "frequency_hz": 10000.0,
    "glitches": 0
  }
}
```

Note the `dut` block: **`ticks_per_s` is read back from the firmware with
`QINFO`, never assumed.** `expected_period_us` is `ticks × 1e6 / ticks_per_s`,
so the same segment list is evaluated correctly on a 16 MHz target, a prescaled
Teensy, or an AVR whose `F_CPU` differs from the default. The same applies to
`max_speed_ticks`, which on AVR is a function of the connected stepper count and
therefore differs between the `1ch` and `2ch` runs of the same test.

### 7.3 Design Spec File

The `design_specs.json` file is organized by platform/driver:

```
extras/tests/saleae_based/
├── design_specs.json          ← all design specs per platform/driver
├── design_specs/              ← per-platform spec files (for review)
│   ├── esp32_rmt_v2.json
│   ├── esp32_mcpwm_pcnt.json
│   ├── esp32s3_rmt_v2.json
│   ├── esp32c3_rmt.json
│   ├── avr_328p.json
│   └── pico.json
└── spec_baseline/             ← measured baselines from golden hardware
    └── 2026-10-01_esp32_rmt_v2_baseline.json
```

### 7.4 Spec Comparison in Reports

The markdown report includes a **spec compliance table** for each test:

```
## SR_06 — Direction-to-Step Delay

| Stepper | Measured (us) | Expected (us) | Tolerance (us) | Deviation | Spec Pass | Actual Pass |
|---------|--------------|---------------|----------------|-----------|-----------|-------------|
| A       | 12.3         | 12.5          | 2.0            | 0.2       | ✓         | ✓           |
| B       | 12.8         | 12.5          | 2.0            | 0.3       | ✓         | ✓           |
| C       | 14.1         | 12.5          | 2.0            | 1.6       | ✓         | ✓           |
| D       | 15.2         | 12.5          | 2.0            | 2.7       | ✗         | ✗           |

Result: FAIL (stepper D exceeds spec tolerance)
```

---

## 8. Markdown Report Generation

### 8.1 Report Pipeline

```
Python analyzer
    │
    ├──► JSON result files (one per test run)
    │
    ├──► tag_index.json (index by composite key)
    │
    ├──► design_specs.json (design spec values)
    │
    └──► generate_report.py
            │
            ├──► results/
            │       └── 2026-10-01_test_01_esp32_rmt_v2_8ch_step_only.json
            │
            ├──► capture/
            │       └── 2026-10-01_test_01_esp32_rmt_v2_8ch_step_only.sr
            │
            └──► reports/
                    ├── index.md                ← main report
                    ├── test_SR_01.md           ← per-test detail
                    ├── test_SR_04.md
                    ├── spec_compliance.md      ← all spec comparisons
                    ├── regression.md           ← regression tracking
                    ├── comparison_esp32_vs_esp32s3.md
                    ├── all_results.csv         ← spreadsheet export
                    └── tag_summary/            ← per-tag summaries
                        ├── esp32_rmt_v2.md
                        └── esp32s3_mcpwm_pcnt.md
```

### 8.2 Report Format

Markdown reports are **plain text, human-readable, git-diffable**, and
generated by `generate_report.py`. They include:

1. **Summary dashboard**
   - Total tests run / passed / failed / skipped
   - Pass rate by tag (arch, driver, channel_config)
   - Latest run timestamp

2. **Per-test detail** (one file per test: `test_SR_XX.md`)
   - Test case name, type, and parameters
   - Per-stepper metrics with design-spec comparison (measured vs expected)
   - Pass/fail status with pass criteria
   - Raw signal summary (step count, pulse widths, inter-step periods)

3. **Spec compliance report** (`spec_compliance.md`)
   - Table of all measured metrics vs design specs
   - Pass/fail per metric per platform/driver
   - Highlight regressions (metric exceeded tolerance)

4. **Regression tracking** (`regression.md`)
   - Compare current results against a baseline (selected by tag)
   - Highlight regressions (metric exceeded threshold)
   - Show pass/fail change between runs

5. **Cross-platform comparison** (`comparison_*.md`)
   - Side-by-side metrics for same test across different chips/drivers
   - Example: `SR_16 (multi_stepper_timing_impact)` on ESP32 vs AVR-328P
   - Highlight platform-specific differences — e.g. the AVR speed floor being
     `TICKS_PER_S/50000` with one stepper but 426 with two

### 8.3 Sample Markdown Report

```
# SR_05 — Pulse High Time

**Test ID:** SR_05
**Goal:** step pulse high time / duty vs commanded period
**Program:** `QSEG 16 1600 1`  (16 steps, 1600 ticks apart, dir high)
**DUT:** 16000000 ticks/s, `MIN_CMD_TICKS` 3200, `QUEUE_LEN` 32,
       `max_speed_in_ticks` 80
**Date:** 2026-10-01
**Tag:** esp32_idf5_3_0_mcpwm_pcnt_1ch

## Measured

| Metric | Expected | Measured | Verdict |
|--------|----------|----------|---------|
| Commanded period | 100.00 us (1600 ticks) | 100.02 us | ✓ |
| Step count | 16 | 16 | ✓ |
| Pulse high time | — | 0.44 us | recorded |
| Pulse low time | — | 99.58 us | recorded |
| Duty cycle | — | 0.44 % | recorded |
| Glitches | 0 | 0 | ✓ |

## Notes

High/low time and duty are **recorded, not asserted**: the driver sets the pulse
width, and its value is a property of the silicon rather than something the
library promises. They become the baseline that a regression is judged against
(§7.1.1). The assertions in this test are the things the queue layer *does*
promise: the period is exactly what was commanded, the step count is exact, and
no pulse is narrower than the glitch filter.
```

### 8.4 CSV Export

A `all_results.csv` is generated for spreadsheet analysis:

```csv
timestamp,test_id,arch,driver,channel_config,stepper,ticks,ticks_per_s,
step_count,expected_count,period_us,avg_high_us,avg_low_us,duty_percent,
glitch_count,pass
2026-10-01T12:00:00Z,SR_05,esp32,rmt_v2,1ch,A,1600,16000000,16,16,100.02,0.44,99.58,0.44,0,true
2026-10-01T12:00:01Z,SR_05,esp32,rmt_v2,1ch,B,1600,16000000,16,16,100.03,0.45,99.58,0.45,0,true
2026-10-01T12:01:00Z,SR_03,esp32s3,mcpwm_pcnt,1ch,A,80,16000000,8,8,5.00,0.21,4.79,4.20,0,true
2026-10-01T12:02:00Z,SR_16,avr328,timer,2ch,A,426,16000000,64,64,26.63,0.48,26.15,1.80,0,true
```

---

## 9. Directory Structure

```
extras/tests/saleae_based/
├── white_paper_saleae_test_harness.md   # This white paper
├── README.md                            # Quick start guide
├── AGENTS.md                            # Agent guide
├── capture/                             # Generated .sr / .vcd captures
├── scripts/
│   ├── harness.py                       # arch/framework/driver -> env + tag
│   ├── run_tests.py                     # program, capture, evaluate, record
│   ├── control.py                       # manual serial
│   ├── capture.py                       # sigrok-cli capture (.sr, --vcd)
│   ├── analyze_csv.py                   # SR_00 evaluation
│   ├── signal_parser.py                 # edges/metrics core
│   └── tests/                           # hardware-free unit tests
├── common/                              # shared firmware (see §4.4)
├── apps/                                # thin entry points
└── capture/                             # generated captures (git-ignored)
```

**Note:** there is no separate `build-saleae.sh`. The firmware is assembled by
`extras/scripts/build-pio-dirs.sh` into `pio_dirs/saleae` (Arduino) and
`pio_espidf/saleae` (ESP-IDF) with symlinks to `common/` and `apps/`, so it
builds in the same CI matrix as every other target. The generated dirs are
git-ignored; never commit the symlinks.

---

## 10. Hardware Requirements

The harness measures the **step and direction pins at the MCU**. Those are
ordinary push-pull outputs, and a logic analyzer input is high impedance, so
the analyzer can be connected straight to them. There is no motor in the
circuit and nothing to power but the board.

| Item | Minimum | Recommended |
|------|---------|-------------|
| Target board | Any supported architecture, powered over USB (AVR / ESP32 / Pico / SAM / Teensy) | ESP32-DevKitC, ESP32-S3-DevKitC, ATmega328P, RP2040 |
| Logic analyzer | 4 channels, 4 MS/s | 8+ channels, 24 MS/s+ (Saleae Logic 8 / Logic Pro 8) |
| USB cable | For the serial console | — |

**Not required:** stepper motors, stepper driver boards (A4988 / TMC2209 /
TMC5160), a 12 V motor supply, a motor power rail, or an oscilloscope. The
driver chip is not in the measurement path at all, so nothing about motor
current, microstepping or driver-side step shaping is exercised — the subject
is what the MCU emits.

**⚠ Do not attach one anyway.** These are not motor-safe commands; see the
warning at the top of this document. The fast scenarios exceed what a stepper
can follow from standstill, so a connected motor would stall and overheat its
driver while producing nothing the analyzer would use.

### 10.1 Channel count and sample rate

Two channels per stepper, one for step and one for direction:

| Config | Steppers | Channels needed |
|--------|----------|-----------------|
| `1ch` | 1 | 2 |
| `2ch` | 2 | 4 |
| `4ch_mcpwm`, `4ch_rmt`, `mixed` | up to 4 | 8 |

The sample rate has to resolve the pulse high time, which is a few
microseconds. At 16 MHz one tick is 62.5 ns, so a 16-tick pulse is 1 us wide and
4 MS/s gives four samples across it. That is the floor; 24 MS/s is enough
headroom for the narrowest pulse worth resolving without needing the analyzer's
full bandwidth.

### 10.2 Wiring

| Analyzer | Board |
|----------|-------|
| GND | any GND pin |
| D0, D1 | stepper A step, dir |
| D2, D3 | stepper B step, dir |
| D4, D5 | stepper C step, dir |
| D6, D7 | stepper D step, dir |

A common ground between analyzer and board is the only connection required
beyond the signal lines. Note that on AVR the step pin must be one of the
timer-capable pins listed in §3.3, which is why the firmware reads
`stepPinStepperA/B` rather than hardcoding pin numbers.

---

## 11. Integration with Existing Test Infrastructure

The Saleae-based tests complement (not replace) the existing PC-based and
SimAVR-based tests:

| Layer | Tool | Purpose |
|-------|------|---------|
| **Unit tests** | PC-based `test_XX` | Algorithm validation (ramp calculator, queue management) |
| **Simulation** | SimAVR `test_sd_*` | AVR-specific timing validation |
| **Pin-level characterization** | Saleae-based `SR_XX` | Measured `addQueueEntry()` waveform on real hardware, per architecture and driver |
| **CI** | SimAVR stub | Every commit (no hardware needed) |
| **Release** | Full Saleae suite | Release candidates only |

The SimAVR stub (`saleae_stub.cpp`) logs expected signal patterns to a file
that the Python analyzer can read, allowing the Saleae analysis pipeline to
run in CI without hardware.