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

The outcome of the whole suite is a characterization of the queue layer.

It is organized as **two generic test modes**. Neither mode names an
architecture: the architecture, SDK version and driver are *tags on a run*, not
modes. That is what makes the same two commands the whole cross-architecture
matrix, and what lets a driver nobody has connected yet be added by naming it.

| Mode | Question | Varies over | Applies to |
|------|----------|--------------|-----------|
| **`scale`** | How does a single driver behave from 1 up to its maximum steppers in parallel? | driver × stepper count `1…driver-max` | every architecture |
| **`sync`** | On an architecture with more than one driver: do they start together, and does each stepper keep the speed it was given? | every driver-list combination | every architecture with >1 driver |

`scale` is the single run an AVR board needs: one driver, two steppers.
`sync` applies today to the ESP32 family, because
`SUPPORT_SELECT_DRIVER_TYPE` exists only there — but the *mode* is generic, and
an architecture that grows a second driver needs no new code path, only a
second name in its driver list.

**The two modes are deliberately different measurements.** `scale` gives every
stepper *one shared program*, so a run answers "does this driver still emit the
commanded period with N attached, and does each stepper get every step".
`sync` gives each stepper *its own period*, on a 1:2:3 ratio ladder, because
adherence is only checkable when the steppers were given **different**
expectations: a synchronized start that dragged them all onto one speed would
satisfy a first-step test *and* a shared-period test, and be reported as a
perfect sync. Each stepper is then judged against **its own** commanded period,
so a collapse is observed rather than inferred from its absence.

**`scale` stops at `min(driver-max, channel budget)` and says which bound it
hit.** These are different findings — "RMT reaches 8 steppers step-only" and
"RMT reaches 4 with direction pins" — and only the second is a fact about the
analyzer rather than about RMT. The driver bound comes from the library's own
`QUEUES_*` values, which the host cross-checks against
`src/pd_esp32/pd_config_idf5.h`; an unknown entry is **refused**, never guessed,
since a guessed bound either truncates the sweep or runs past what the board can
connect.

**`sync` enumerates over driver *identities*, not spellings.** `rmt` and
`rmt_v2` are one driver — the firmware maps both to `SA_RMT` and reports both as
`rmt` — so enumerating names would file `rmt_v2+rmt` as a *cross-driver*
combination under a name claiming they differ. That is exactly the failure §5.5
records having produced once already, and the reason the plan contains both
same-driver and cross-driver pairs is that the same-driver row is the only
baseline the cross-driver skew is interpretable against.

A **refused** point is recorded and the plan continues: for `scale` the point
where the board says no *is* the answer, and for `sync` a driver this build
cannot connect is a fact worth recording beside the combinations that did
measure. That is what makes `i2s_mux` — never run, see §5.8 — one flag away with
no new code path: it is already in the driver list, so the run attempts it and
records the refusal.

**Driver maximum is a property of the driver, bounded by the analyzer.** The
loop stops at `min(driver-max, channel budget)` and says which bound it hit:
the `dir` mapping spends two channels per stepper and so caps at 4, while
step-only spends one and caps at 8. Reporting "8 steppers" on a configuration
where the analyzer ran out of channels would be the same category of error as
letting the firmware choose the driver.

Underneath the two modes sit the original targets:

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
| Pulse high time | **measured** | Set by the driver logic, not by load — so it is *recorded* and becomes the baseline a regression is judged against, but no threshold is asserted on it. See §1.4. |
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

#### 1.4 Pulse width is measured, not asserted — with the measurement

This table row was originally "asserted" and the hardware disagreed with the
reasoning, so the reasoning changed.

Measured on ESP32, sweeping the commanded period over 200 µs, 500 µs, 1 ms, 2 ms,
2.048 ms and 4.096 ms, the **pulse high time is a constant 15.625 µs (250
ticks) in every one of them**. The driver emits a fixed-width pulse and varies
the silence between pulses; high time does not scale with `ticks` at all.

So the pulse width is a property of the silicon, not a promise the queue makes,
and asserting it would mean asserting a number with no source behind it. It is
recorded — as a *distribution*, min/max/median/spread, because a driver holding
a fixed width shows `min == max` and that is itself the result — and it becomes
the baseline a future regression is measured against.

This is also why there is no glitch counter. The CSV schema originally carried a
`glitch_count` column and filling it in was the only reason to want one; a glitch
count needs a threshold invented for it, and it collapses a distribution into a
single number. The statistics have a source and the counter does not.

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

- **Channels**: exactly **8**. Four steppers with step+dir, or eight step-only.
  There is no spare channel for a trigger marker — see §3.4 for why that is a
  design outcome rather than an omission.
- **Sample rate**: ≥ 1 MS/s (10× the highest step frequency; 16 MHz clock
  → max ~400 kHz step frequency → 4 MHz sample rate is the practical minimum).
- **Trigger**: **unused**, and not optional. All eight channels carry step and/or
  dir pins, so there is nothing left to trigger on. Captures are started, armed
  for a fixed interval, and only then is `QRUN` issued. `capture.py` still
  supports `--wait-trigger` for setups with a spare channel, but the committed
  suite never uses it, because triggering on the first STEP edge makes that edge
  sample 0 and undercounts by one — see §3.4.
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
| ~~**glitch_count**~~ | *Removed* — needs an invented threshold and collapses a distribution; see §1.4 and §8.4 |
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
        "pulse_high_us": {"min": 15.5417, "max": 15.625, "mean": 15.6116}
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
| **channel_config** | derived, not chosen: `<count>` steppers, `dir` or `nodir`, plus the driver list — e.g. `4/dir`, `8/nodir` |
| **test_type** | `ramp`, `sync_start`, `dir_change`, `abrupt_speed`, `queue_full`, `move_timed`, `pause`, `overflow`, `speed_limit`, `queue_fill_latency` |
| **result** | `passed`, `failed`, `skipped`, `error` |

Composite tag key format: `{arch}_{sdk}_{driver_list}_{count}_{pin_mode}` —
used as the primary index key. `driver_list` is the per-stepper list joined with
`+` (`rmt`, `rmt+rmt`, `rmt+mcpwm`, `i2s_mux+i2s_mux+i2s_mux`). A key that
cannot name its driver is not a valid key.

Note there is no `tag_index.json` in the shipped design: results are one JSON
file per test under `results/`, and the index is derived from them by
`generate_report.py`. An index file kept in step with the results is a second
thing that can disagree with them.

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

`SUPPORT_SELECT_DRIVER_TYPE` only exists on the ESP32 family, so only there can
one run mix drivers. Everywhere else there is a single native driver, and the
driver list simply repeats it — `timer` for AVR, `pio` for Pico. The list is
**always explicit**, on every architecture, for one reason: a result that does
not record which driver produced it characterizes nothing.

> **There is no automatic driver selection.** Not as a convenience and not as a
> default. The harness never sends an unspecified driver, and the firmware
> refuses one rather than falling back. An earlier version of this harness drove
> most scenarios with `CONFIG 1ch`/`2ch`, which resolved to the library's
> automatic choice; seventeen of twenty-five recorded results were tagged
> `auto`, which meant they recorded whatever the firmware happened to pick and
> could say nothing about any driver. See the todo for the correction work.

**The analyzer bounds the stepper count, and the driver bounds it too.** The
queue totals above (49 on ESP32) are the library's capacity; the harness is
limited by 8 analyzer channels, which is 4 steppers with a direction pin and 8
without. The `scale` mode therefore stops at the smaller of the two bounds and
reports which bound it hit.

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
  two, because the ISR needs ~14 us. So one and two steppers must both be
  characterized on the same board — which is mode `scale` at count 1 and 2.
- **AVR step pin is not a free choice.** It must be the pin the library maps to
  the timer compare output (`stepPinStepperA`/`stepPinStepperB` in
  `src/AVRStepperPins.h`), and *which* physical pin that is depends on
  `FAS_TIMER_MODULE`. Never hardcode it.

### 3.2 Channel Configuration Modes

Originally this section listed eight named presets (`8ch_step_only`,
`7ch_shared_dir`, `4ch_rmt`, `2rmt_2i2s`, …). That was the wrong shape: every
preset is just *(a count, a list of drivers, a pin mode)*, and enumerating named
combinations means a new name for every one. The harness now expresses all of
them with one generic command:

```
CONFIG <count> <driver>[,<driver>…] [dir|nodir]
```

- `<count>` — number of steppers to connect, 1…8.
- `<driver>` — one name per stepper, comma separated, **no shorthand, no
  default**. A count that does not match the list length is refused.
- `dir` (default) — two channels per stepper, direction pin present, max 4.
- `nodir` — one channel per stepper, step-only, max 8. **No direction pin is
  connected at all**, so `setDirectionPin()` is not called. The `QSEG` direction
  argument still parses — a scenario's program is therefore identical in both
  modes — but is forced true, because there is no pin to toggle for a false and
  the queue would refuse the command with `ErrorNoDirPinToToggle`. That is the
  only semantic difference between the modes, and it is why the
  direction-observing scenarios are `dir`-mode by construction.

The count cap is `min(platform stepper queues, channels / stride)` and a
refusal reports **both** bounds — `ERR CONFIG n=5 max=4/8/8/2` for
`n / cap / stepper queues / channels / stride` — because "too many steppers"
cannot say which one bit: the channel budget running out at 4 and a driver
running out of queues (MCPWM/PCNT has 6 on IDF 5) are different facts.

**One pin table serves both modes; the stride is what selects between them.**
`kChanPin[8]` maps analyzer channel to GPIO, and in `dir` stepper *j* owns
channels 2*j* (step) and 2*j*+1 (dir) while in `nodir` it owns channel *j*. So
the mode cannot drift from the map — there is nothing to keep in step but the
stride, and the table does not even have to know the count. It is also the same
pins in the same order as SR_00's eight (§5.0), so a channel the self-test
proved is a channel a scenario measures.

Every preset in the old list is expressible:

| Old preset | Now |
|------------|-----|
| `8ch_step_only` | `CONFIG 8 rmt,rmt,rmt,rmt,rmt,rmt,rmt,rmt nodir` |
| `4ch_rmt` | `CONFIG 4 rmt,rmt,rmt,rmt` |
| `4ch_mcpwm` | `CONFIG 4 mcpwm,mcpwm,mcpwm,mcpwm` |
| `6ch_i2s_mux` | `CONFIG 6 i2s_mux,i2s_mux,… nodir` |
| `2rmt_2i2s` | `CONFIG 4 rmt,rmt,i2s_direct,i2s_direct` |
| `mixed` | `CONFIG <n> <any list> [dir|nodir]` |

The superseded preset table, kept only for the record:

<details><summary>Superseded preset list</summary>

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

</details>

All of the above are superseded by the single generic `CONFIG` grammar.

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
```

**The analyzer has 8 channels, so there are no CH 8 / CH 9.** An earlier draft
of this section reserved a 9th channel for a start marker and a 10th for
queue-empty; both do not exist. Worse, there is no spare channel to move them
to: **all eight are spoken for by four steppers' step and dir pins.** A marker
channel is therefore not available on a 4-stepper `dir` run at all, and on a
step-only run every channel is a step pin. See §3.4.

**Step-only mapping** (8 steppers, no direction pin). Same GPIO per channel as
the `dir` table above — the pins do not move, only which of them a stepper owns:

```
CH 0: Step A   CH 1: Step B   CH 2: Step C   CH 3: Step D
CH 4: Step E   CH 5: Step F   CH 6: Step G   CH 7: Step H
```

**Verified on hardware**, not just derived: `CONFIG 8 rmt×8 nodir` gives 8 steps
on every one of `D0`…`D7` at 40.00 µs with a 15.50 µs high time, and
`CONFIG 4 rmt×4 dir` gives steps on `D0`, `D2`, `D4`, `D6` with `D1`, `D3`,
`D5`, `D7` quiet. A host that assumed the `dir` map while running the 8-stepper
case would find `E`–`H` unreadable and report working drivers as dead; one that
assumed it for a 2-stepper `nodir` run reads `B` as 0 steps when it is on `D1`.
That is why `MAP` is in the protocol and the host parses it.

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

#### 3.4 There is no universally safe start marker

A trigger marker is attractive: it makes the start instant exact instead of
inferred from when the host sent `QRUN`. It was implemented, measured, and
removed. The reasons are worth recording, because they will recur.

- **No channel is free.** Four steppers need all eight channels. A marker would
  have to displace a step or dir pin, which changes the thing being measured.
- **The library's own probes cannot cover the suite.** `PROBE_1` and friends are
  disabled by default (`ESP32_TEST_PROBE` / `ESP32C3_TEST_PROBE`), are defined
  only for ESP32 and ESP32-C3 — not S3/C6/H2/P4 — and exist **only in the RMT
  drivers**, not MCPWM/PCNT or I2S. A suite whose whole point is comparing
  drivers cannot have its start instant defined by one of them.
- **Triggering costs a step.** Capturing triggered on the first STEP edge makes
  that edge sample 0 and undercounts the run by exactly one. Every scenario that
  asserts a step count therefore captures *untriggered*.

The arming is instead done by starting the capture, sleeping a fixed interval,
then issuing `QRUN` — and the capture length is chosen so the whole program and
a quiet tail both fit inside it. The `scale` and `sync` modes inherit this.

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
work there, and mode `scale` covers both counts on that driver.

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
| `CONFIG <count> <drv>[,<drv>…] [dir\|nodir]` | `OK CONFIG n=<N> mode=<pinmode> maxspeed<i>=<ticks> …` | Connect `<count>` steppers, one driver named per stepper. Pin mode `dir` (default) or `nodir`. **There is no `auto`.** |
| `MAP` | `MAP count=<n> mode=<pinmode> stride=<s> ch=<pin,…>` | Which analyzer channel carries which stepper, plus the GPIO behind each reachable channel, so host and firmware cannot disagree (§3.3). **The host must read this rather than assume a map**: in `dir` stepper B is `D2`, in `nodir` it is `D1`, and a host that guesses reads a quiet pin and reports a driver that emits nothing |
| `QINFO` | `QINFO tps=… mincmd=… qlen=… maxall=… maxspeed0=… [maxspeed1=…]` | Platform limits the host must respect. **`maxall` is the largest per-stepper speed floor** — the fastest period legal for *every* connected stepper, and what a shared program is planned against. The indexed `maxspeedN` fields are each stepper's own, for a program that addresses one stepper specifically. `maxall` is printed **first** so a buffer overrun cannot truncate the field the host cannot reconstruct |
| `QCLR` | `OK QCLR` | Drop the program, stop everything |
| `QSEG <steps> <ticks> <dir>` | `OK QSEG <n>/8` | Append a segment to the **shared** program. `steps=0` means "pause for `<ticks>` ticks". `dir` is 0 or 1 |
| `QSEG <idx> <steps> <ticks> <dir>` | `OK QSEG <n>/8` | Append a segment to **stepper `<idx>`'s own** program, ignoring the shared one. Needed for two steppers at *different* periods (SR_15, mode `sync`) |
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

**Why two `QSEG` forms.** The forms differ in argument count, so no command can
be misread as the other. The three-argument form means the shared program,
which every stepper walks unless it has one of its own — that keeps every
single-program scenario unchanged. The four-argument form exists because a
shared program cannot express two steppers at different periods, which is
exactly what mode `sync` has to measure. The cost is a per-stepper segment
matrix in RAM: **+110 bytes on AVR** (1481 → 1591 of 2048). That is the price of
the capability and it fits.

**`CONFIG` refuses rather than clamps.** If the count exceeds what the platform
or the channel budget allows, or the driver list length does not match the
count, it returns `ERR` rather than quietly connecting fewer steppers — a
silently reduced count makes the capture look like a driver problem.

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
| **SR_13** | `ticks_error_rejected` | `QSEG 8 399 1` | The firmware refuses it and **no pulse is emitted**. A rejection that still steps would move the motor by steps nobody asked for. 8 steps at 399 ticks is 3192 ticks of motion against a floor of 3200, so it is refused with **`ERR QE step0 rc=-1`** (`ErrorTicksTooLow`) — *not* the `ERR QE ticks … < maxspeed …` this table used to predict. Measured: zero pulses, position 0. |

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
| **SR_16** | `multi_stepper_timing_impact` | `CONFIG 1 <drv>`, `QSEG 64 <ticks> 1`, `QRUN 1`; then `CONFIG 2 <drv>,<drv>`, same, `QRUN 3` | **Does the second stepper perturb the first?** Compare SR_01's period from the `1ch` run against the same channel's period in the `2ch` run. On AVR the answer is structural: the floor rises from `TICKS_PER_S/50000` to 426 ticks, so a sweep done only at `1ch` would report a speed the board cannot sustain with two steppers connected. |
| **SR_17** | `sync_cross_driver` | `CONFIG 2 rmt,mcpwm_pcnt dir`, `QSEG <n> <ticks> 1`, `QRUN 3` | Cross-driver start skew. **Measured: 49.0 µs = 1.225 step periods, against 29.5 µs = 0.738 for two steppers on one RMT.** So the premise that RMT and MCPWM+PCNT "arm through entirely different hardware and must therefore diverge" holds after all — see §5.5, including how an earlier run of this very test appeared to refute it and did not. |

### 5.5 The `sync` mode, and what the permutations showed

`sync` runs every driver-list combination on an architecture with more than one
driver, and records two things per combination: **sync start** (first-step skew,
in µs *and* in step periods) and **adherence** (whether each stepper kept the
period it was individually given — SR_15). Skew is reported, never gated.

The permutations matter more than any single pair, because the interesting
question is whether skew tracks *driver heterogeneity* at all:

| Driver list | First-step skew | In step periods |
|-------------|-----------------|------------------|
| `rmt+rmt` (same driver, SR_14) | 29.5417 µs | 0.7385 |
| `rmt+mcpwm_pcnt` (cross-driver, SR_17) | **49.0 µs** | **1.2250** |
| `rmt+rmt` at 2:1 speeds (SR_15) | 27.0417 µs | 0.6760 |

**Cross-driver skew is ~66 % worse than same-driver** — 1.22 step periods against
0.74. The two drivers do arm through unrelated hardware and they do diverge; §5.2's
original premise was right and the correction it used to be was wrong.

**Re-measured by `--mode sync`, which enumerates every combination rather than
the three above** (ESP32, 5 µs step, 4 MS/s; each stepper given its own period,
so adherence is checked per stepper):

| Driver list | Skew µs | In step periods | Adherence |
|-------------|---------|-----------------|-----------|
| `rmt+rmt` | 37.5 | 3.75 | both kept 160 t / 320 t |
| `rmt+mcpwm_pcnt` | 62.25 | 6.23 | both kept theirs |
| `mcpwm_pcnt+i2s_direct` | 858.75 | 85.88 | both kept theirs |
| `rmt+i2s_direct` | **990.0** | **99.0** | both kept theirs |
| `i2s_direct+i2s_direct` | 52.25 | 5.23 | both kept theirs |
| `mcpwm_pcnt+mcpwm_pcnt` | 6.5 | 0.65 | **B emitted 11053 steps, not 64** |

**The I2S rows change the conclusion, and only the enumeration exposed them.**
RMT and MCPWM are both *armed timers*, and comparing them says cross-driver skew
is modestly worse than same-driver. I2S is a third thing: it emits from a DMA
callback, so it cannot be armed alongside the others at all, and the gap is
**99 step periods** rather than 6.2 — reproducible to a sample across three runs
(990.0 / 990.25 / 1068.25 µs, with `rmt+rmt` at 37.5 / 37.5 / 37.75 over the same
three). A millisecond there is the time for the DMA pipeline to produce anything
at all, not scheduling jitter.

So "skew tracks driver heterogeneity" is right but undersold: what it tracks is
**how the driver is started**, and among armed timers the spread is tens of µs
while a DMA-fed driver is three orders of magnitude away. The skew *column* is
what makes that legible — 99.0 against 6.23 is a different kind of number from
990 µs against 62 µs, and one figure in µs alone would hide it.

> The `mcpwm_pcnt+mcpwm_pcnt` row is **a defect, not a measurement** — see
> §5.8. Two MCPWM/PCNT queues on one ESP32 do not run: the second stepper emits
> continuously and never stops, on the unmodified firmware too. Until that is
> fixed this row characterizes the bug.

> **This table was wrong once, and the reason is worth recording.** An earlier
> revision of this section reported `rmt+mcpwm` at *exactly* the same 29.5417 µs
> as `rmt+rmt`, four decimal places, and concluded from the identity that skew
> tracks nothing but uC load. The identity was the tell: two independent
> measurements do not agree to a sample at 24 MS/s. The firmware's `mixed`
> channel config parsed its per-stepper driver list and then **overwrote it with
> the automatic driver choice** before connecting, so the "cross-driver" run was
> RMT+RMT — the same run twice. Removing the automatic driver (todo R1) is what
> exposed it: with drivers named explicitly, the cross-driver row is a
> genuinely different measurement and it is ~0.49 periods larger.
>
> The general lesson is the one this harness is built around: a result that
> records what the firmware happened to do is not a measurement of what was
> asked for. Fixing it required re-running SR_17 on hardware, not reasoning
> about it.

A skew of 29.5 µs — or of 49 µs — is meaningless alone: it is three quarters of
a period at 640 ticks and almost nothing at 65535. Hence the period column.

**Adherence is the asserted half.** Each stepper is given its own period and
checked against *its own*, not a common one: a synchronized start that dragged
both onto one speed would satisfy a first-step test and fail this one.

### 5.3 Category: Driver Edge Behaviour

The cases where a driver's own limit/overrun handling is wrong. These are the
highest-value tests in the catalogue, because the symptom is *silent* — the
step count still looks plausible.

| Test ID | Name | Program | Saleae Check |
|---------|------|---------|--------------|
| **SR_18** | `mcpwm_overrun_after_255` | `QSEG 255 <max> 1`, `QSEG 0 <p> 1`, `QSEG 1 <max> 1` | Exactly 255 pulses, then a gap of `p`, then **exactly 1**. The PCNT high limit is re-armed from the live counter value on every command; a `steps=1` command straight after a 255-step run is precisely where a stale or mis-computed limit produces a lost or extra pulse. **The trailing single step must use at least 3200 ticks**, not `<max>`: a `steps=1` command is bounded by its ticks alone, so `<max>` = 640 is under `MIN_CMD_TICKS` and the firmware refuses it with `ErrorTicksTooLow`. The last phase therefore runs at a different period from the one before it — the corrected program uses 3200 ticks (200 µs) for the single step. Measured 256/256 with the expected 439.67 µs boundary gap. |
| **SR_19** | `mcpwm_overrun_boundary` | `QSEG <n> <max> 1` for `n` = 200…255, each followed by `QSEG 1 <max> 1` | Sweeps the suspicious band. The 8-bit counter and the MCPWM timer period interact differently at each `n`; one `n` is enough to lose the trailing step. |
| **SR_20** | `pause_after_full_command` | `QSEG 255 <ticks> 1`, `QSEG 0 <p> 1`, `QSEG 255 <ticks> 1` | A full 255-step run, a pause, then another full run: 255 / gap / 255. Complements SR_18 by making the *second* command the large one. |
| **SR_21** | `rmt_buffer_split` | `QSEG 200 <ticks> 1` | RMT V1 splits its hardware buffer at a command boundary. A split at the wrong step shows up as one irregular inter-step gap. |
| **SR_22** | `rmt_v2_encoder` | `QSEG 200 <ticks> 1` | RMT V2 fill-encoder output has no irregular gap. |
| **SR_23** | `i2s_timing` | `QSEG 64 <ticks> 1` | I2S direct/mux step output at the correct intervals. ESP32 only. |
| **SR_24** | `avr_timer_timings` | `QSEG 64 <ticks> 1` on each of Timer1/3/4/5 | Each AVR timer channel produces the commanded period on its OC pin. |

### 5.4 Category: Limits and Error Conditions

| Test ID | Name | Program | Saleae Check |
|---------|------|---------|--------------|
| **SR_25** | `emergency_stop` | `QSEG 20000 <ticks> 1`, then `STOP` at 0.15 s | No partial pulse and the capture goes quiet after the stop. **Measured: truncated at 11475 of 20000, pin still for the remaining 2.823 s.** The move must exceed the pulse queue — see §5.6. |
| **SR_26** | `pause_ticks_max` | `QSEG 1 65535 1`, `QSEG 0 65535 1`, `QSEG 1 65535 1` | A pause of exactly 65535 ticks on the pause path, which is a separate field from `ticks*steps`. **Measured gap 8185 µs against 8191.875 expected.** The trailing step is not in the original two-segment form: silence can only be measured between two pulses, so without a step after the pause the gap is unobservable rather than wrong. |

### 5.6 `stopMove()` does not stop a move that is already queued

Found by SR_25, and the reason the scenario's program is larger than it looks.

`stopMove()` sets a flag that the ramp generator consults when it is asked for
its *next* command. A move already sitting in the pulse queue never asks again,
so it runs to completion:

| Move | `STOP` issued at | Result |
|------|------------------|--------|
| 2000 steps @ 4000 ticks | position 510 | finished at **2000** — no effect at all |
| 20000 steps @ 640 ticks | position 5825 | stopped at **14240**, truncating only once the queue refilled |

So the stop is honoured, but later than anyone issuing an emergency stop would
expect, and the delay is whatever the queue happened to hold. SR_25 therefore
uses a move far larger than the queue, which makes it a test of *stopping*
rather than of queue drain. Anyone relying on `stopMove()` as an emergency stop
should know this.

### 5.8 What `--mode scale` found that no scenario was looking for

**Two MCPWM/PCNT queues on one ESP32 do not run.** The second stepper emits
continuously and never stops.

`CONFIG 2 mcpwm_pcnt,mcpwm_pcnt dir`, `QSEG 64 160 1`, `QRUN 3`: stepper A emits
exactly 64 steps; stepper B emits **22 143 rising edges at exactly the commanded
10 µs period** (5.0 µs high, 5.0 µs low) straight through to the end of the
capture window. The board's own `POS` reads **non-monotonic** — `64 31`, then
`64 26`, then `64 32`, all below 64 — which says the position counter is being
re-read while the pin runs on, not that steps were lost. The capture and the
firmware disagree about what happened, and the capture is right.

**It is two MCPWM/PCNT queues, not MCPWM, and not the pin mode:**

| Configuration | `POS` sampled over 1.5 s |
|---------------|--------------------------|
| `rmt+rmt` | `64 64` every time |
| `rmt+mcpwm_pcnt` | `64 64` every time |
| `mcpwm_pcnt+rmt` (order swapped) | `64 64` every time |
| `mcpwm_pcnt+mcpwm_pcnt` | `64 31 / 64 26 / 64 32 / 64 50 / 64 34` |
| `mcpwm_pcnt+mcpwm_pcnt` at 3200 ticks (200 µs) | fails too — not a speed limit |

It reproduces with only stepper B selected (`QRUN 2`), so the synchronized
kick-off is not the trigger: **merely configuring a second MCPWM/PCNT queue is
enough.** And it reproduces on the unmodified firmware, so it is a library
defect rather than an artefact of the harness.

*Where to look:* `StepperISR_idf5_esp32_mcpwm_pcnt.cpp` indexes
`channel2mapping[NUM_QUEUES]` and `pcnt_unit_to_queue[QUEUES_MCPWM_PCNT]` by
`channel_num` with `pcnt_unit_id = timer_num`, while the ESP32 has **4 MCPWM
timers** (2 groups × 2) against `QUEUES_MCPWM_PCNT` = **6**. The second queue's
timer/PCNT assignment is the first thing to check.

**This is the argument for `scale` as a mode.** SR_16 asks whether a second
stepper *perturbs* the first, and its answer to a runaway is "perturbed" — it
compares periods on two channels and reports a difference. `scale` sweeps the
count and asserts each stepper's step count, so a stepper that never stops is a
failure rather than a larger number. Coverage that scales with the hardware
finds things a hand-picked case cannot, because the hand-picked case was written
by someone who did not know to look.

Until it is fixed, **any MCPWM/PCNT row in the report tables characterizes this
bug, not the driver.**

The catalogue is intentionally small, and every entry is either a measured
waveform property or a driver limit. Nothing in it re-verifies arithmetic, and
nothing needs more than a handful of `QSEG` lines.

**Status: 25 of 28 ids are implemented, fixtured, and verified on hardware
(ESP32 — 25/25 pass).** Three are documented as not applicable to that board:
**SR_22** needs RMT V2 (this ESP32 has V1), **SR_24** needs an AVR board, and
**SR_00** is the opt-in pin self-test. A committed baseline lives in
`reports/esp32/`. Modes `scale` and `sync` generate their runs from this
catalogue rather than duplicating it.

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
therefore differs between the 1-stepper and 2-stepper runs of the same test.

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
run_hardware.py --results DIR          sweep.py --run
    │
    ├──► JSON result files (one per test)
    │
    └──► generate_report.py
            │
            ├──► results/
            │       └── SR_01.json
            │
            ├──► capture/
            │       └── SR_01.vcd  (+ .meta sidecar)
            │
            └──► reports/
                    ├── index.md                ← dashboard + results table
                    ├── test_SR_01.md           ← per-test detail
                    ├── spec_compliance.md      ← all spec comparisons
                    ├── regression.md           ← vs a baseline run
                    ├── all_results.csv         ← spreadsheet export
                    └── tag_summary/            ← per-tag summaries
                        ├── esp32_rmt_4_dir.md
                        └── esp32_rmt_mcpwm_2_dir.md
```

Two things this pipeline deliberately does **not** have:

- **No `tag_index.json`.** Results are one JSON file per test and the index is
  derived from them. An index file kept in step with the results is a second
  thing that can disagree with them.
- **No `design_specs.json`.** `spec_compliance.md` derives its expectations from
  the `QINFO` limits the DUT reports (`ticks_per_s`, `min_cmd_ticks`,
  `queue_len`, fastest legal ticks) plus the commanded program. That covers what
  the library promises — the commanded period and the step count — and avoids a
  hand-maintained spec file that can drift from the firmware. What it does not
  do is reproduce a per-driver spec table for quantities the library does not
  promise (§1.4).

The generator **formats only**. It never re-parses a capture, so it cannot
disagree with the run that measured it — which matters because a 24 MS/s capture
is ~96M samples of pure-Python waveform, and re-measuring would be free to reach
different numbers than the test just passed on.

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

Real output from `generate_report.py`, for the run in `reports/esp32/`. This is
the per-test detail page shape; `index.md` adds the dashboard and the full
results table on top.

```
# SR_15 — aligned start, then each stepper at its own period

**Test ID:** SR_15
**Goal:** aligned start, then each stepper at its own period
**Program:** `stepper 0: QSEG 0 200 640 1 ; stepper 1: QSEG 1 200 1280 1`
**DUT:** 16000000 ticks/s, `MIN_CMD_TICKS` 3200, `QUEUE_LEN` 32,
       fastest legal 640 ticks
**Configuration:** 2ch on rmt+mcpwm (esp32)
**Tag:** `esp32_rmt_mcpwm_2_dir`
**Captured at:** 24000000 Hz
**Result:** **PASS**

## Measured

| stepper | steps | period us (min–max, spread) | pulse high us (min–max, spread) | duty % |
|---|---|---|---|---|
| A | 200 | 39.9167–40, 0.0833 | 15.5417–15.625, 0.0833 | 39.06 |
| B | 200 | 79.875–79.9583, 0.0833 | 15.5417–15.625, 0.0833 | 19.53 |

**First-step skew between steppers:** 27.1667 us (0.6792 step periods). Both
steppers' steady-state periods matching above does *not* mean they started
together -- this is the number that says when their first steps landed.
Recorded, not asserted.

## Detail

(evaluator JSON: per-stepper expected vs measured period, step-count defects,
first-step skew, skew in periods, speed ratio)

## Capture

`/tmp/.../SR_15.vcd`
```

Three details of that page are load-bearing:

- **`Program` shows one QSEG per stepper**, because SR_15 genuinely sends one
  list per stepper. Printing only the first would misrepresent what the board
  was told to do.
- **The skew sits inside `## Measured`, not only in the JSON blob.** An earlier
  version of this table showed both steppers at the same period and never
  mentioned the skew — which reads as a perfect simultaneous start and is
  exactly the wrong conclusion.
- **Durations are ranges.** B's 79.875–79.9583 µs against its commanded 80 µs is
  the assertion; the 0.0833 µs spread is what makes it meaningful.

### 8.4 CSV Export

A single `all_results.csv` is generated for spreadsheet analysis, one row per
**test and stepper**:

**There is no `glitch_count` column, and `period_us` is not a single number.**
Both were in the original schema and both were wrong.

A glitch count needs a threshold invented for it, and it collapses a
distribution into one number: it would have reported "0" for all six of the
pulse-width sweep points that established a constant 15.625 µs width, and
communicated nothing. The width columns are `min`/`max` instead, so a driver
holding a fixed width shows as `min == max` and one short pulse in ten thousand
moves only the minimum (§1.4).

`period_us` is split into mean/min/max/spread for the same reason, and because
the single mean is what hid the cross-driver result: 29.5417 µs against 40 µs
nominal looks like a rounding detail until you see it is 0.74 of a step period
and *identical* to the same-driver case (§5.5).

The CSV is LF-only. CRLF would make every regenerated export show as a
whole-file diff, which defeats the point of a git-committed baseline.

## 9. Directory Structure

```
extras/tests/saleae_based/
├── white_paper_saleae_test_harness.md   # This white paper
├── README.md                            # Quick start guide
├── AGENTS.md                            # Agent guide
├── common/                              # Shared firmware, all targets (§4.4)
├── apps/                                # Thin entry points (arduino/, esp-idf/)
├── reports/                             # Committed baseline: reports/esp32/
└── scripts/
    ├── capture.py                       # sigrok-cli capture (.sr, --vcd, .meta)
    ├── signal_parser.py                 # edges, metrics, distributions
    ├── run_tests.py                     # scenario table + evaluators
    ├── run_hardware.py                  # run scenarios on a real board
    ├── sweep.py                         # parameter sweeps, --list / --run
    ├── generate_report.py               # the §8 artefacts from JSON results
    ├── report.py                        # quick console view of a run
    ├── control.py                       # manual serial
    ├── analyze_csv.py                   # SR_00 evaluation
    └── tests/                           # hardware-free tests + golden fixtures
        ├── fixtures/                    # generated .vcd, committed
        └── make_fixtures.py             # regenerate them; drift-checked
```

`scripts/` is the split that matters: **`capture.py` and `signal_parser.py` are
the instruments** (they know about volts and sample clocks), and
`run_tests.py`/`run_hardware.py`/`generate_report.py` are the harness (they know
about queues and scenarios). Keeping measurement and judgement apart is what
lets the fixtures test the judgement without hardware.

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

Channels per stepper depends on the pin mode: two with a direction pin, one
without.

| Pin mode | Channels per stepper | Max steppers on 8 channels |
|----------|----------------------|------------------------------|
| `dir` | 2 (step + dir) | **4** |
| `nodir` | 1 (step only) | **8** |

Step-only is what makes 8 parallel steppers reachable. The driver queue limits
on this ESP32 are RMT 8, MCPWM/PCNT 6, I2S mux 32, I2S direct 3, so `dir` is the
binding constraint for every driver at 4 and `nodir` only becomes the binding one
for RMT (and I2S mux, if connected). The `scale` mode reports which bound it hit.

The sample rate has to resolve the pulse high time, which is a few
microseconds. At 16 MHz one tick is 62.5 ns, so a 16-tick pulse is 1 us wide and
4 MS/s gives four samples across it. That is the floor; 24 MS/s is enough
headroom for the narrowest pulse worth resolving without needing the analyzer's
full bandwidth.

**Buffer limit.** The Saleae clone used for the committed results holds **64
MSamples**, which is **2.66 s at 24 MS/s**. A capture that hits that limit ends
mid-run, and an ended capture looks identical to a run that stopped on its own —
which is why `capture.py` writes the true sample count to a `.meta` sidecar and
the VCD loader pads to it (§2.1). Anything needing a longer window must lower
the sample rate or shorten the program.

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