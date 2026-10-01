# 120 Saleae-based Test Harness — White Paper

## 1. Goal

Build a **hardware-in-the-loop test harness** using a **Saleae Logic Analyzer**
(or any sigrok-compatible USB logic analyzer) to verify stepper-motor signal
integrity at the pin level on real ESP32 hardware.  The required sample rate is
modest: 4 MS/s is the practical minimum for a 16 MHz tick clock (see §2.1), so
200+ MS/s is not a requirement.  All captured signals are
recorded with `sigrok-cli`, decoded in Python, and the results are tagged with
architecture, driver, channel configuration, and test metadata.

---

## 2. Architecture

```
┌──────────────────────────────────────────────────────────────────────┐
│  Test Runner (Python)                                                │
│  ┌──────────┐  ┌─────────────┐  ┌───────────┐  ┌──────────┐       │
│  │ Build    │→ │ Flash ESP32 │→ │ sigrok-   │→ │ Python   │       │
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
- **Output format**: `.sr` (sigrok native) — lossless, timestamped, channel-
  interleaved.
- **Driver selection**: `--driver saleae` is Saleae-specific. Other analyzers
  use different sigrok drivers (`fx2lafw`, `hantek_dso620`, …), so `capture.py`
  should map the detected device to the correct driver rather than hard-coding
  `saleae`.

### 2.2 Python Analyzer

```python
# Pipeline:
# 1. Load .sr file with libsigrok (Python bindings)
# 2. Decode Step/Dir channels: detect rising/falling edges, compute pulse
#    widths, inter-step gaps, dir→step delay
# 3. Validate against expected command stream (from test case definition)
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

## 3. ESP32 Driver Types and Channel Configuration Model

### 3.1 Three Driver Families

The ESP32 pulse driver (`pd_esp32`) supports **three driver families**, each
with sub-types:

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

**Non-ESP32 platforms** (not shown above): AVR 2 steppers (ATmega328P) / 4
(ATmega2560), Pico `4 × NUM_PIOS`, SAM 6, SAMD51 `TCC_INST_NUM` (variable by
chip), Teensy 4.x **16**. The I2S driver families (`i2s_direct`, `i2s_mux`) are
**ESP32-only** (`SUPPORT_ESP32_I2S`); tests SR_27/SR_28/SR_34/SR_35 therefore
run on ESP32 chips only.

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

### 3.3 Channel Pin Mapping (Example — ESP32-DevKitC)

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

## 4. Firmware Architecture — Generic App with Serial-Downloaded Commands

### 4.1 Design Philosophy

The test firmware is a **single generic application** that downloads queue
commands via serial and executes them. This avoids maintaining separate
firmware binaries for each test case. The same firmware runs on all platforms
(AVR, ESP32 variants, Pico, SAM), with platform-specific behavior handled by
conditional compilation.

### 4.2 Queue Command Format

Queue commands are downloaded via serial in a compact binary format that maps
directly to the `queue_entry` struct:

```c
// From src/fas_queue/base.h:
struct queue_entry {
  uint8_t steps;       // 1 byte: if 0, pure delay (no step pulses)
  uint8_t toggle_dir : 1;   // 1 bit: toggle direction
  uint8_t countUp    : 1;   // 1 bit: direction (true=forward)
  uint8_t hasSteps   : 1;   // 1 bit: whether this entry has steps
  uint8_t dirPinState: 1;   // 1 bit: direction pin state
  uint16_t ticks;           // 2 bytes: tick count for delay/speed
#if defined(SUPPORT_QUEUE_ENTRY_END_POS_U16)
  uint16_t end_pos_last16;  // 2 bytes: optional end position
#endif
#if defined(SUPPORT_QUEUE_ENTRY_START_POS_U16)
  uint16_t start_pos_last16;// 2 bytes: optional start position
#endif
};
// Base size: 4 bytes (no optional fields)
// With SUPPORT_QUEUE_ENTRY_END_POS_U16 or ..._START_POS_U16: 6 bytes
// With both optional fields: 8 bytes (no current platform defines both)
```

**Download protocol** (serial, 115200 baud):

```
Header:  [0xAA 0x55] (2 bytes)
Length:  [uint16_t] (2 bytes) — number of queue_entry structs
Entries: [steps, flags, ticks, ...] (4, 6, or 8 bytes each)
Checksum: [uint16_t] (2 bytes) — XOR of all data bytes
End:     [0x55 0xAA] (2 bytes)
```

Maximum download size per transfer: 512 entries (2–4 KB, depending on the
platform's `queue_entry` size). This fits comfortably within the RAM of all
supported platforms:

| Platform | SRAM | Max queue entries (QUEUE_LEN) | Entry size | Download buffer |
|----------|------|-------------------------------|------------|-----------------|
| ATmega328P | 2 KB | 16 | 6 B | 96 bytes |
| ATmega2560 | 8 KB | 16 | 6 B | 96 bytes |
| ESP32 (all) | 512 KB | 32 | 6 B | 192 bytes |
| ESP32-S3 | 512 KB | 32 | 6 B | 192 bytes |
| RP2040 | 264 KB | 32 | 4 B | 128 bytes |
| SAMD51 | 192 KB | 32 | 6 B | 192 bytes |
| Teensy 4.x | 1 MB | 32 | 6 B | 192 bytes |

### 4.3 Platform RAM Analysis

The queue_entry struct is the critical size factor:

| Platform | Queue Entry Size | QUEUE_LEN | Queue RAM (entry array only) |
|----------|-----------------|-----------|------------------------------|
| AVR (328P/2560) | 6 bytes (end_pos) | 16 | 96 bytes |
| ESP32 (all IDF) | 6 bytes (start_pos) | 32 | 192 bytes |
| Pico (Arduino/IDF) | 4 bytes (neither) | 32 | 128 bytes |
| SAM/SAMD51 | 6 bytes (end_pos) | 32 | 192 bytes |
| Teensy 4.x | 6 bytes (end_pos) | 32 | 192 bytes |

All well within platform SRAM limits. Even the largest case (192 bytes) is
< 0.1% of available RAM. Sizes are verified against `src/fas_queue/base.h` and
the per-platform `pd_*/pd_config.h` feature flags.

### 4.4 Firmware Structure

The saleae firmware integrates with the existing `build-pio-dirs.sh` and
`build-idf-platformio.sh` scripts. It produces **two build variants**:

1. **Arduino PlatformIO** — for AVR, Pico (Arduino framework), SAM (Arduino)
2. **ESP-IDF PlatformIO** — for ESP32 chips with native ESP-IDF framework

```
extras/tests/saleae_based/firmware/
├── platformio.ini                    ← Arduino PlatformIO (AVR, Pico, SAM)
├── platformio_idf.ini                ← ESP-IDF PlatformIO (ESP32 chips)
├── CMakeLists.txt                    ← ESP-IDF root (referenced by build-idf-platformio.sh)
├── src/
│   ├── main.cpp                      ← serial command parser (entry point)
│   ├── serial_protocol.cpp           ← command/response protocol
│   ├── command_executor.cpp          ← downloads queue commands, enqueues to stepper
│   └── serial_reporter.cpp           ← sends metrics back to host
├── adapters/                         ← per-platform modules (selected by #ifdef)
│   ├── esp32_adapter.cpp             ← ESP32-specific init (IDF4/5/6)
│   ├── esp32s3_adapter.cpp           ← ESP32-S3
│   ├── esp32c3_adapter.cpp           ← ESP32-C3
│   ├── esp32c6_adapter.cpp           ← ESP32-C6
│   ├── esp32h2_adapter.cpp           ← ESP32-H2
│   ├── avr_adapter.cpp               ← ATmega328P/2560
│   ├── pico_adapter.cpp              ← RP2040
│   └── sam_adapter.cpp               ← SAM/SAMD51
├── lib/
│   └── saleae_test_cases/            ← shared test case library (same for all platforms)
│       ├── include/test_cases.h      ← generic test case definitions
│       └── src/
│           ├── test_connections.cpp  ← SR_00 (port-toggle sanity check)
│           ├── test_basic.cpp        ← SR_01–SR_05
│           ├── test_timing.cpp       ← SR_06–SR_12
│           ├── test_sync.cpp         ← SR_13–SR_17
│           ├── test_queue.cpp        ← SR_18–SR_24
│           ├── test_driver.cpp       ← SR_25–SR_30
│           └── test_stress.cpp       ← SR_31–SR_40
└── simavr_stub/                      ← SimAVR stub for CI (no hardware)
    └── saleae_stub.cpp               ← Stub that logs expected signals
```

#### 4.4.1 Arduino PlatformIO Build

```ini
; platformio.ini — Arduino framework (AVR, Pico, SAM)
[platformio]
default_envs = nanoatmega328, nanoatmega168, atmega2560, atmega32u4, rp2040, atmelsam, samd51

[env:nanoatmega328]
platform = atmelavr
board = nanoatmega328
framework = arduino
build_flags = -Werror -Wall
lib_extra_dirs = .
```

#### 4.4.2 ESP-IDF PlatformIO Build

```ini
; platformio_idf.ini — ESP-IDF framework (ESP32 chips)
[platformio]
default_envs = esp32_idf5, esp32s3_idf5, esp32c3_idf5, esp32c6_idf5, esp32h2_idf5

[env:esp32_idf5]
platform = https://github.com/pioarduino/platform-espressif32/releases/download/53.03.11/platform-espressif32.zip
board = esp32dev
framework = espidf
build_flags = -Wall -D ESP_IDF_VERSION_MAJOR=5
board_build.f_cpu = 240000000L
lib_extra_dirs = .
```

**Key difference:** ESP-IDF builds use `framework = espidf` instead of
`framework = arduino`. The ESP-IDF framework requires a `CMakeLists.txt`
and uses ESP-IDF's component-based build system. The Arduino framework uses
`platformio.ini` with `lib_extra_dirs`.

#### 4.4.3 Integration with `build-pio-dirs.sh`

The saleae firmware integrates with the existing build system by producing
build directories that `build-pio-dirs.sh` and `build-idf-platformio.sh`
recognize. Instead of symbolic links (which are not allowed in the saleae
harness), the build script **copies** files into build directories:

```bash
#!/bin/sh
# extras/tests/saleae_based/build-saleae.sh
# Usage: ./build-saleae.sh [targets]
# Targets: arduino (default), espidf, all

ROOT=`git rev-parse --show-toplevel`
TARGETS=${1:-arduino}

# Create build directory (no symlinks — copy files)
rm -fR saleae_build
mkdir -p saleae_build

if [ "$TARGETS" = "arduino" ] || [ "$TARGETS" = "all" ]; then
    # Arduino PlatformIO: copy firmware into pio_dirs/saleae/
    mkdir -p saleae_build/pio_dirs/saleae/src
    cp firmware/platformio.ini saleae_build/pio_dirs/saleae/
    cp firmware/src/*.cpp saleae_build/pio_dirs/saleae/src/
    cp firmware/adapters/*.cpp saleae_build/pio_dirs/saleae/src/
    cp -r firmware/lib/saleae_test_cases saleae_build/pio_dirs/saleae/lib/
    # Copy library source (no symlinks)
    cp -r $ROOT/src saleae_build/pio_dirs/saleae/FastAccelStepper
fi

if [ "$TARGETS" = "espidf" ] || [ "$TARGETS" = "all" ]; then
    # ESP-IDF PlatformIO: copy firmware into pio_espidf/saleae/
    mkdir -p saleae_build/pio_espidf/saleae/src
    cp firmware/platformio_idf.ini saleae_build/pio_espidf/saleae/
    cp firmware/CMakeLists.txt saleae_build/pio_espidf/saleae/
    cp firmware/src/*.cpp saleae_build/pio_espidf/saleae/src/
    cp firmware/adapters/*.cpp saleae_build/pio_espidf/saleae/src/
    cp -r firmware/lib/saleae_test_cases saleae_build/pio_espidf/saleae/lib/
    # Copy library source (no symlinks)
    cp -r $ROOT/src saleae_build/pio_espidf/saleae/FastAccelStepper
    cp $ROOT/CMakeLists.txt saleae_build/pio_espidf/saleae/FastAccelStepper
fi

echo "Build directories created in saleae_build/"
ls -al saleae_build/
```

This script is called by the existing CI infrastructure:

```bash
# Arduino builds (existing build-pio-dirs.sh pattern)
./extras/tests/saleae_based/build-saleae.sh arduino
for i in saleae_build/pio_dirs/*; do
    (cd $i; pio run -s -e nanoatmega328)
done

# ESP-IDF builds (existing build-idf-platformio.sh pattern)
./extras/tests/saleae_based/build-saleae.sh espidf
for i in saleae_build/pio_espidf/*; do
    (cd $i; pio run -s -e esp32_idf5)
done
```

**Key design: `command_executor.cpp`** downloads queue_entry structs via serial,
then calls `stepper.addQueueEntry()` for each one. The ISR handles the rest.
No autonomous long recordings — the host triggers capture via serial command,
and the firmware toggles a GPIO marker at `startQueue` to trigger the Saleae
capture window.

### 4.5 Serial Command Protocol

| Command | Response | Description |
|---------|----------|-------------|
| `LIST` | `OK: SR_00,SR_01,...,SR_40` | List available test cases with tags |
| `CONFIG <name>` | `OK: config=<name>` | Apply channel config |
| `PORT_TOGGLE <pin> <hz>` | `OK: pin=<pin> freq=<hz>Hz` | Toggle pin at frequency (SR_00) |
| `PORT_TOGGLE_STOP` | `OK: stopped` | Stop all port toggles (end SR_00) |
| `DOWNLOAD <count>` | `OK: <count> entries queued` | Download queue commands (binary) |
| `RUN <test_id>` | `OK: test started` | Execute downloaded commands |
| `STATUS` | `OK: pos=1234, running=1, queue=8/32` | Query current state |
| `METRICS <stepper>` | `OK: steps=1000, dt=12.5us, glitches=0` | Get per-stepper metrics |
| `TAG <key>` | `OK: tagged` | Tag current result |
| `STOP` | `OK: stopped` | Emergency stop |
| `RESET` | `OK: reset` | Reinitialize steppers/queue to a known state |

**Error handling.** Every response must carry a status code (`OK` / `ERR <code>`)
so the host can detect failures. The binary `DOWNLOAD` payload must be verified
with a checksum that covers the header, length, and data (the byte-XOR field
itself included), and a corrupt download must be rejected without enqueuing
anything. A dropped or truncated download must leave the queue unchanged, and
`DOWNLOAD` must implicitly stop/clear the current move before enqueuing so new
commands cannot interleave with an in-flight test. `RESET` is required to
recover from an inconsistent state without a physical power cycle.

**Baud rate.** 115200 baud takes ~0.3–0.6 s to transfer a full download, which
is long enough for a previously running stepper to move before all entries
arrive. Use ≥ 921600 baud, and/or have `DOWNLOAD` hold the stepper (auto-enable
off) until the transfer completes.

### 4.6 Capture Trigger Workflow

```
0. Host sends: CONFIG 4ch_rmt
1. Host sends: PORT_TOGGLE 2  1  (Stepper A Step  = GPIO2)
2. Host sends: PORT_TOGGLE 0  1  (Stepper A Dir   = GPIO0)
3. Host sends: PORT_TOGGLE 4  1  (Stepper B Step  = GPIO4)
4. Host sends: PORT_TOGGLE 16 1  (Stepper B Dir   = GPIO16)
5. Host sends: PORT_TOGGLE 17 1  (Stepper C Step  = GPIO17)
6. Host sends: PORT_TOGGLE 5  1  (Stepper C Dir   = GPIO5)
7. Host sends: PORT_TOGGLE 18 1  (Stepper D Step  = GPIO18)
8. Host sends: PORT_TOGGLE 19 1  (Stepper D Dir   = GPIO19)
9. Host captures 2 s on Saleae → verifies clean square wave on all channels
10. Host sends: PORT_TOGGLE_STOP  (stop all toggles)
11. Host sends: DOWNLOAD 500  (binary: 500 queue_entry structs)
12. Host sends: RUN SR_04
13. Firmware: toggles GPIO25 HIGH at startQueue (or RMT `PROBE_1` toggles
    automatically when probes are enabled)
14. Firmware: toggles GPIO25 LOW when queue empties
15. Saleae CH 8 connected to GPIO25 → triggers capture window
16. sigrok-cli captures 2-second window centered on trigger
17. Firmware sends metrics back via serial
18. Host saves .sr file and JSON result with tags
```

**Note:** Steps 0–10 are the SR_00 connection verification. They run once
before any stepper-motion test (SR_01–SR_40).  If SR_00 fails (missing channel,
no signal, wrong frequency), the remaining tests are skipped — the harness
does not proceed with stepper motion until the wiring is corrected.

---

## 5. Test Case Catalogue

### 5.0 Category: Connection Verification (no PC-based equivalent)

This category runs **before** any stepper-motion test. It verifies that every
Saleae channel is electrically connected to the correct GPIO and that the
firmware can toggle pins without loading the stepper driver.

| Test ID | Name | Description | Saleae Check |
|---------|------|-------------|--------------|
| **SR_00** | `port_toggle` | Each Step/Dir pin toggles at a known frequency (e.g. 1 Hz square
wave). No stepper motion. | Saleae sees a clean square wave on every
channel. Frequency matches expected value. No missing edges. |

**Protocol:** Host sends `CONFIG <name>` → firmware sets all configured
Step/Dir pins as outputs → firmware toggles each pin at 1 Hz (50 % duty)
indefinitely until a `STOP` command is received. The host captures a few
seconds, verifies the waveform on every channel, then sends `STOP`.

### 5.1 Category: Basic Ramp Tests (mapped from PC-based test_01–test_05)

| Test ID | Name | Description | Saleae Check |
|---------|------|-------------|--------------|
| **SR_01** | `basic_move_forward` | Move N steps forward at constant speed. | Step count matches expected. No glitches. |
| **SR_02** | `basic_move_reverse` | Move N steps backward. | Direction pin toggles. Step count matches (negative). |
| **SR_03** | `mixed_direction_ramp` | Alternating forward/reverse moves. | Dir pin follows commands. Step count = sum of absolute steps. |
| **SR_04** | `acceleration_ramp` | Accelerate from rest to max speed and decelerate. | Inter-step period decreases/increases as expected. |
| **SR_05** | `speed_profile_multi_phase` | Multi-phase speed: slow → fast → slow. | Each phase has correct inter-step period range. |

### 5.2 Category: Timing Precision Tests (mapped from PC-based test_06–test_12)

| Test ID | Name | Description | Saleae Check |
|---------|------|-------------|--------------|
| **SR_06** | `direction_to_step_delay` | Measure dir→first-step delay at various speeds. | Delay matches platform documentation within ±1 tick. |
| **SR_07** | `step_pulse_width_at_speed` | Measure pulse width at various speeds. | Pulse width scales inversely with speed. |
| **SR_08** | `duty_cycle_symmetry` | Check high/low symmetry across speeds. | Duty cycle deviation < 5%. |
| **SR_09** | `abrupt_speed_change` | Jump from max speed to min speed (and back). | No glitches. No missed/doubled pulses. |
| **SR_10** | `min_tick_boundary` | Test at MIN_CMD_TICKS boundary. | No underflow. Correct minimum pulse width. |
| **SR_11** | `max_speed_discovery` | **Find the driver's maximum achievable speed** (minimum inter-step period). There is no fixed value: the limit is **both processor- and driver-specific** — it depends on the processor (MCU clock and timer/tick source), the driver engine (RMT / MCPWM+PCNT / I2S / PIO / timer) and its divider, and the number of active channels. The host sweeps the commanded speed from slow to fast until pulses drop, merge or glitch. | The fastest speed at which every commanded pulse is still emitted cleanly. Reported as the *measured* max speed / min inter-step period, keyed per {processor, driver} (a baseline, **not** a fixed pass/fail bound). |
| **SR_12** | `queue_fill_latency` | Measure time from enqueue to first step pulse during burst. | Latency < documented maximum. |

### 5.3 Category: Synchronized Start Tests (mapped from PC-based test_13–test_17)

| Test ID | Name | Description | Saleae Check |
|---------|------|-------------|--------------|
| **SR_13** | `sync_start_same_tick` | 4 steppers start simultaneously. | All first step edges within 1 tick (≤ 62.5 ns). |
| **SR_14** | `sync_start_delayed_start` | Steppers start with staggered delays. | Each stepper's first step at correct relative delay. |
| **SR_15** | `sync_start_different_speeds` | Steppers start together but at different speeds. | First step alignment + per-stepper speed correct. |
| **SR_16** | `sync_start_n_axis` | Multi-axis synchronized move to different targets. | All steppers reach target position simultaneously. |
| **SR_17** | `sync_start_cross_driver` | Steppers use different drivers (RMT + MCPWM). | Cross-driver synchronization within tolerance. |

**Resolution caveat:** the 1-tick (62.5 ns at 16 MHz) figure for SR_13 is
below the resolution of a 1–4 MS/s capture. Verifying it requires a fast
analyzer (≥ 100 MS/s) or an on-chip method (e.g. a timer-captured timestamp);
at 4 MS/s the measurable tolerance is ~250 ns.

### 5.4 Category: Queue Management Tests (mapped from PC-based test_18–test_24)

| Test ID | Name | Description | Saleae Check |
|---------|------|-------------|--------------|
| **SR_18** | `queue_full_behavior` | Fill queue to capacity, then try to enqueue more. | No corruption. Stepper continues from queue. |
| **SR_19** | `queue_empty_prevention` | Ensure queue never empties during move. | Continuous step pulses (no gaps > max inter-step). |
| **SR_20** | `moveTimed_accuracy` | Move to position in specified time. | Total time matches commanded duration ± tolerance. |
| **SR_21** | `moveTimed_direction_change` | moveTimed with direction change mid-move. | Direction change at correct position. |
| **SR_22** | `pause_command_insertion` | Pause commands (steps=0) between step commands. | Pause duration matches commanded ticks. |
| **SR_23** | `queue_overflow_overflow` | Rapid enqueue/dequeue cycling. | No lost commands. Position tracking correct. |
| **SR_24** | `moveTimed_drift_check` | Repeated moveTimed cycles (Issue #370 regression). | Net position drift = 0 after each cycle. |

### 5.5 Category: Driver-Specific Tests

| Test ID | Name | Description | Saleae Check |
|---------|------|-------------|--------------|
| **SR_25** | `rmt_buffer_split` | Verify RMT V1 two-part buffer split at command boundary. | Buffer split occurs at correct command boundary. |
| **SR_26** | `rmt_v2_fill_encoder` | Verify RMT V2 fill encoder output. | Encoded symbols match expected pattern. |
| **SR_27** | `i2s_direct_timing` | Verify I2S direct output timing. | Step pulses on I2S data line at correct intervals. |
| **SR_28** | `i2s_mux_timing` | Verify I2S mux output timing. | Step pulses on muxed I2S line at correct intervals. |
| **SR_29** | `mcpwm_pcnt_sync` | Verify MCPWM timer sync across units. | Timer edges aligned across MCPWM units. |
| **SR_30** | `rmt_sync_manager` | Verify RMT TX sync manager (ESP32/ESP32-S3). | All RMT channels start within 1 tick. |

### 5.6 Category: Channel Configuration Stress Tests

| Test ID | Name | Description | Saleae Check |
|---------|------|-------------|--------------|
| **SR_31** | `8ch_step_only_max_speed` | 8 steppers, all at the SR_11 discovered max speed (the per-driver limit is lower with 8 channels active). | All 8 step channels active. No cross-talk; the measured max speed may be below the single-channel SR_11 baseline. |
| **SR_32** | `7ch_shared_dir_consistency` | 7 steppers sharing one Dir line. | All steppers see same Dir edge. |
| **SR_33** | `mixed_driver_interference` | RMT + MCPWM + I2S running simultaneously. | No signal corruption between driver families. |
| **SR_34** | `i2s_extender_scaling` | 4→8 steppers on I2S extender. | Each stepper gets correct step pulses. |
| **SR_35** | `channel_reassignment` | Reassign steppers to different drivers at runtime. | New driver takes over cleanly. No glitch. |

### 5.7 Category: Edge Cases and Error Conditions

| Test ID | Name | Description | Saleae Check |
|---------|------|-------------|--------------|
| **SR_36** | `gpio_pin_reuse` | Step/Dir pins shared with other peripherals. | No interference from shared pin usage. |
| **SR_37** | `interrupt_load` | High interrupt load from other peripherals. | Step timing unaffected. |
| **SR_38** | `power_sag_recovery` | Simulate power sag during move. *(Equipment-dependent: requires a programmable/current-limited supply.)* | Stepper resumes correctly after recovery. |
| **SR_39** | `emergency_stop` | Force stop during active move. | Step pulses cease immediately. Position frozen. |
| **SR_40** | `overflow_wraparound` | 32-bit position counter overflow. | Position wraps correctly. No discontinuity. |

---

## 6. Mapping Existing PC-Based Tests to Saleae-Based Tests

| PC-Based Test | Saleae Equivalent | Notes |
|---------------|-------------------|-------|
| — | **SR_00** `port_toggle` | **No PC equivalent.** Runs before any other test to verify wiring. |
| `test_01` (basic) | SR_01 | Same basic move, but with Saleae as oracle |
| `test_02` (speed profile) | SR_05 | Multi-phase speed profile |
| `test_04` (queue full) | SR_18 | Queue full behavior |
| `test_05` (ramp) | SR_04 | Acceleration ramp |
| `test_06` (direction delay) | SR_06 | Direction-to-step delay measurement |
| `test_07` (pulse width) | SR_07 | Step pulse width at speed |
| `test_08` (duty cycle) | SR_08 | Duty cycle symmetry |
| `test_09` (ramp plot) | SR_05 | Multi-phase speed (with gnuplot equivalent) |
| `test_10` (abrupt change) | SR_09 | Abrupt speed change |
| `test_11` (min tick) | SR_10 | Minimum tick boundary |
| `test_12` (max speed) | SR_11 | Discover the driver's max speed — not fixed, platform/driver dependent |
| `test_13` (sync start) | SR_13 | Synchronized start |
| `test_14` (sync delayed) | SR_14 | Synchronized delayed start |
| `test_15` (sync different speeds) | SR_15 | Synchronized different speeds |
| `test_16` (sync n-axis) | SR_16 | N-axis synchronized move |
| `test_17` (sync cross-driver) | SR_17 | Synchronized cross-driver |
| `test_18` (queue full) | SR_18 | Queue full behavior |
| `test_19` (queue empty) | SR_19 | Queue empty prevention |
| `test_20` (moveTimed) | SR_20 | moveTimed accuracy |
| `test_21` (I2S direct) | SR_27 | I2S direct timing |
| `test_22` (I2S mux) | SR_28 | I2S mux timing |
| `test_24` (moveTimed drift) | SR_24 | moveTimed drift check (Issue #370) |
| `test_25` (moveTimed pause) | SR_22 | Pause command insertion |
| `test_26` (pause reporting) | SR_22 | Pause command reporting |
| `test_27` (overflow) | SR_40 | Overflow wraparound |
| `test_28` (speed limit) | SR_11 | Speed limit — see SR_11 (driver max speed is measured, not fixed) |
| `test_29` (queue fill latency) | SR_12 | Queue fill latency |
| `test_30` (mixed direction) | SR_03 | Mixed direction ramp |

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
  "test_id": "SR_06",
  "spec_comparison": {
    "dir_to_first_step_us": {
      "measured": 12.3,
      "expected": 12.5,
      "tolerance": 2.0,
      "deviation": 0.2,
      "passed": true,
      "spec_key": "SR_06_direction_to_step_delay"
    },
    "step_count": {
      "measured": 1000,
      "expected": 1000,
      "tolerance": 0,
      "deviation": 0,
      "passed": true,
      "spec_key": null
    }
  }
}
```

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
   - Example: `SR_04 (acceleration_ramp)` on ESP32-RMT vs ESP32-S3-MCPWM
   - Highlight platform-specific differences

### 8.3 Sample Markdown Report

```
# SR_04 — Acceleration Ramp

**Test ID:** SR_04
**Type:** ramp
**Steppers:** 4 (A, B, C, D)
**Steps:** 5000 per stepper
**Speed:** 400us → 40us → 400us
**Acceleration:** 10000 steps/s²
**Date:** 2026-10-01
**Platform:** ESP32, RMT V2, 4ch_rmt

## Spec Compliance

| Stepper | Step Count | Dir→Step (us) | Avg Inter-Step (us) | Max Pulse (us) | Glitches | Spec Pass |
|---------|-----------|---------------|--------------------|----------------|----------|-----------|
| A       | 5000/5000  | 12.3          | 0.80               | 0.40           | 0        | ✓         |
| B       | 5000/5000  | 12.8          | 0.81               | 0.41           | 0        | ✓         |
| C       | 5000/5000  | 14.1          | 0.79               | 0.39           | 0        | ✓         |
| D       | 5000/5000  | 15.2          | 0.82               | 0.42           | 1        | ✗         |

## Summary

- **Result:** FAIL (stepper D: dir→step delay exceeds spec, 1 glitch detected)
- **Stepper D dir→step delay:** 15.2us vs spec 12.5±2.0us (exceeded by 0.7us)
- **Stepper D glitches:** 1 glitch detected (width < 125ns)
```

### 8.4 CSV Export

A `all_results.csv` is generated for spreadsheet analysis:

```csv
timestamp,test_id,chip,arch,driver,channel_config,stepper,step_count,expected_count,
dir_to_first_step_us,avg_inter_step_us,max_pulse_width_us,glitch_count,spec_pass,actual_pass
2026-10-01T12:00:00Z,SR_01,ESP32,esp32,rmt_v2,4ch_rmt,stepper_A,1000,1000,12.5,0.8,0.4,0,true,true
2026-10-01T12:00:01Z,SR_01,ESP32,esp32,rmt_v2,4ch_rmt,stepper_B,1000,1000,12.5,0.8,0.4,0,true,true
2026-10-01T12:01:00Z,SR_04,ESP32-S3,esp32s3,mcpwm_pcnt,4ch_mcpwm,stepper_A,5000,5000,8.2,0.3,0.15,0,true,true
2026-10-01T12:02:00Z,SR_06,ESP32,esp32,rmt_v2,4ch_rmt,stepper_D,1000,1000,15.2,0.8,0.4,1,false,false
```

---

## 9. Directory Structure

```
extras/tests/saleae_based/
├── 120_saleae_based_test_harness.md    # This white paper
├── README.md                            # Quick start guide
├── tag_schema.json                      # Tag schema definition
├── tag_index.json                       # Tag index (generated)
├── capture/                             # Raw .sr capture files
│   └── 2026-10-01_test_01_esp32_rmt_v2_8ch_step_only.sr
├── results/                             # Tagged JSON results
│   └── 2026-10-01_test_01_esp32_rmt_v2_8ch_step_only.json
├── build-saleae.sh                      # Build script (copies firmware → build dirs)
├── design_specs.json                    # All design specs per platform/driver
├── design_specs/                        # Per-platform spec files (for review)
│   ├── esp32_rmt_v2.json
│   ├── esp32_mcpwm_pcnt.json
│   ├── esp32s3_rmt_v2.json
│   ├── esp32c3_rmt.json
│   ├── avr_328p.json
│   └── pico.json
├── spec_baseline/                       # Measured baselines from golden hardware
│   └── 2026-10-01_esp32_rmt_v2_baseline.json
├── reports/                             # Generated markdown reports
│   ├── index.md                         # Main report
│   ├── test_SR_01.md                    # Per-test detail
│   ├── spec_compliance.md               # All spec comparisons
│   ├── regression.md                    # Regression tracking
│   ├── all_results.csv                  # Spreadsheet export
│   └── tag_summary/                     # Per-tag summaries
│       ├── esp32_rmt_v2.md
│       └── esp32s3_mcpwm_pcnt.md
├── scripts/
│   ├── capture.py                       # sigrok-cli capture script
│   ├── analyze.py                       # Python signal analyzer
│   ├── tag_db.py                        # Tag database management
│   ├── run_test_suite.py                # Test runner orchestrator
│   ├── generate_report.py               # Markdown/CSV report generator
│   └── config/
│       ├── channel_configs.py           # Channel config presets
│       └── test_cases.py                # Test case definitions
└── firmware/
    ├── platformio.ini                    ← Arduino PlatformIO (AVR, Pico, SAM)
    ├── platformio_idf.ini                ← ESP-IDF PlatformIO (ESP32 chips)
    ├── CMakeLists.txt                    ← ESP-IDF root (for build-idf-platformio.sh)
    ├── src/
    │   ├── main.cpp                      ← serial command parser (entry point)
    │   ├── serial_protocol.cpp           ← command/response protocol
    │   ├── command_executor.cpp          ← downloads queue commands, enqueues
    │   └── serial_reporter.cpp           ← sends metrics back to host
    ├── adapters/                         ← per-platform modules (selected by #ifdef)
    │   ├── esp32_adapter.cpp             ← ESP32-specific init (IDF4/5/6)
    │   ├── esp32s3_adapter.cpp           ← ESP32-S3
    │   ├── esp32c3_adapter.cpp           ← ESP32-C3
    │   ├── esp32c6_adapter.cpp           ← ESP32-C6
    │   ├── esp32h2_adapter.cpp           ← ESP32-H2
    │   ├── avr_adapter.cpp               ← ATmega328P/2560
    │   ├── pico_adapter.cpp              ← RP2040
    │   └── sam_adapter.cpp               ← SAM/SAMD51
    ├── lib/
    │   └── saleae_test_cases/            ← shared test case library (same for all envs)
    │       ├── include/test_cases.h      ← generic test case definitions
    │       └── src/
    │           ├── test_connections.cpp  ← SR_00 (port-toggle sanity check)
    │           ├── test_basic.cpp        ← SR_01–SR_05
    │           ├── test_timing.cpp       ← SR_06–SR_12
    │           ├── test_sync.cpp         ← SR_13–SR_17
    │           ├── test_queue.cpp        ← SR_18–SR_24
    │           ├── test_driver.cpp       ← SR_25–SR_30
    │           └── test_stress.cpp       ← SR_31–SR_40
    └── simavr_stub/                      ← SimAVR stub for CI (no hardware)
        └── saleae_stub.cpp               ← Stub that logs expected signals
```

**Note:** The `build-saleae.sh` script copies files into build directories
instead of using symbolic links. This is required because symbolic links are
not allowed in the saleae test harness (unlike the existing `pio_dirs/` and
`pio_espidf/` directories used by `build-pio-dirs.sh`). The build script
produces directories that integrate with the existing `build-pio-dirs.sh` and
`build-idf-platformio.sh` CI infrastructure.

---

## 10. Implementation Phases

### Phase 1: Foundation (Week 1–2)
- [ ] Create directory structure and tag schema
- [ ] Write `capture.py` — sigrok-cli wrapper for automated captures
- [ ] Write basic `analyze.py` — edge detection and pulse counting
- [ ] Create firmware skeleton with serial command parser
- [ ] Implement queue_entry download protocol (binary format)
- [ ] **Implement SR_00 `port_toggle`** — per-pin `PORT_TOGGLE` / `PORT_TOGGLE_STOP`
  commands, square-wave output, Saleae verification

### Phase 2: Analysis Engine (Week 3–4)
- [ ] Implement full signal analyzer (dir→step delay, pulse width, inter-step)
- [ ] Implement tag database (`tag_db.py`) with JSON index
- [ ] Create `run_test_suite.py` — orchestrator for running tests
- [ ] Implement markdown report generator (`generate_report.py`)
- [ ] Create `design_specs.json` with per-platform/driver spec values

### Phase 3: Test Case Implementation (Week 5–8)
- [ ] **Implement SR_00** `port_toggle` (connection verification)
- [ ] Implement SR_01–SR_12 (basic + timing tests)
- [ ] Implement SR_13–SR_17 (synchronized start tests)
- [ ] Implement SR_18–SR_24 (queue management tests)
- [ ] Implement SR_25–SR_30 (driver-specific tests)
- [ ] Implement SR_31–SR_40 (channel config stress + edge cases)

### Phase 4: Reporting and CI (Week 9–10)
- [ ] Markdown report with spec compliance tables, cross-platform comparison
- [ ] CSV export for spreadsheet analysis
- [ ] CI integration (run on every PR with SimAVR stub)
- [ ] Regression tracking (compare results across tag keys)
- [ ] Integrate `build-saleae.sh` with existing `build-pio-dirs.sh` / `build-idf-platformio.sh`

---

## 11. Hardware Requirements

| Item | Minimum | Recommended |
|------|---------|-------------|
| Logic Analyzer | 8 channels, 100 MS/s | 16+ channels, 500 MS/s+ (Saleae Logic 8/Logic Pro 8) |
| ESP32 Board | Any dev kit | ESP32-DevKitC, ESP32-S3-DevKitC |
| Stepper Drivers | A4988 / TMC2209 | TMC5160 (for high-speed testing) |
| Power Supply | 12V stepper supply | Regulated, current-limited |
| Oscilloscope (optional) | — | For cross-validation of Saleae measurements |

---

## 12. Integration with Existing Test Infrastructure

The Saleae-based tests complement (not replace) the existing PC-based and
SimAVR-based tests:

| Layer | Tool | Purpose |
|-------|------|---------|
| **Unit tests** | PC-based `test_XX` | Algorithm validation (ramp calculator, queue management) |
| **Simulation** | SimAVR `test_sd_*` | AVR-specific timing validation |
| **Hardware validation** | Saleae-based `SR_XX` | Real signal integrity on actual hardware |
| **CI** | SimAVR stub | Every commit (no hardware needed) |
| **Release** | Full Saleae suite | Release candidates only |

The SimAVR stub (`saleae_stub.cpp`) logs expected signal patterns to a file
that the Python analyzer can read, allowing the Saleae analysis pipeline to
run in CI without hardware.