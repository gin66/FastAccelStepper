# 180 R7 — Virtual I2S Mux: 37-Channel Test Harness Extension

> This document is the design reference: what is built, how it works, and why.
> It deliberately carries no task list and no status. Progress and the
> remaining work live in exactly one place:
> [`extras/todo/180_r7_virtual_i2s_mux.md`](../../../todo/180_r7_virtual_i2s_mux.md).

> **⚠ WARNING — do not connect a stepper, motor, or stepper driver.**
>
> The commands this harness generates are **not intended to drive a motor**.
> They are synthetic probe patterns chosen to make the waveform measurable.
> The fast scenarios command **25–40 kHz step rates reached instantly from
> standstill**, with a mux pulse high time of **4 µs** (one I2S frame). At
> 1.8° full step that is **7,500–12,000 rpm**, which no stepper can follow
> from rest without acceleration. A real motor would stall, lose steps, and
> sit drawing near-standstill current through the driver while the harness ran.
>
> Nothing is gained by attaching one: the measurement is the pin signal,
> taken before any driver chip.

---

## 1. Goal

Extend the Saleae-based test harness to characterize the **ESP32 I2S mux
driver** with up to **32 steppers** using a **8-channel logic analyzer**.

The ESP32 I2S mux driver multiplexes up to 32 stepper step signals onto
3 physical I2S bus lines (data, bclk, ws). The Saleae captures these 3 bus
lines plus 5 additional stepper pins from other drivers (RMT, MCPWM, etc.),
totaling **8 physical channels**.

The harness must:

1. **Capture the 8 physical channels** — 3 I2S bus + 5 stepper pins — at a
   sample rate sufficient to resolve the 8 MHz I2S bit clock.
2. **Decode the I2S bus signals** from the 8-channel capture into 32
   mux-slot channels (S0–S31), producing a **37-channel VCD** (8 − 3 + 32).
3. **Evaluate the 37-channel VCD** using the existing signal parser, treating
   mux-slot channels (S0–S31) as first-class stepper step/dir signals.
4. **Generalize the analysis** to handle any number of channels, any stepper
   count, and any mux-slot-to-stepper mapping.

---

## 2. Architecture

```
┌──────────────────────────────────────────────────────────────────────┐
│  Test Runner (Python)                                                │
│  ┌──────────┐  ┌─────────────┐  ┌───────────────────┐  ┌──────────┐ │
│  │ Build    │→ │ Flash target│→ │ sigrok-           │→ │ Python   │ │
│  │ & Flash  │  │             │   │ CLI (8 ch)        │  │ Analyzer │ │
│  └──────────┘  └─────────────┘   └───────────────────┘   └──────────┘ │
│       ▲                                              │               │
│       │              ┌─────────────┐                 │               │
│       └──────────────│  Tag DB     │◄────────────────┘               │
│                      │  (JSON)     │                                 │
│                      └─────────────┘                                 │
│                      ┌───────────────────────────┐                   │
│                      │  VCD-to-VCD Decoder       │                   │
│                      │  (8 ch → 37 ch)           │                   │
│                      └───────────────────────────┘                   │
└──────────────────────────────────────────────────────────────────────┘
```

**Pipeline:**

```
8-channel VCD (Saleae capture)
    │
    ├── 3 channels: I2S bus (data, bclk, ws) — consumed by decoder
    ├── 5 channels: stepper step/dir from other drivers — passed through
    │
    ├──► Decode I2S bus: extract 32-bit mux state per frame
    │
    ├──► Synthesize 32 mux-slot channels (S0–S31) from decoded mux state
    │
    └──► Write 37-channel VCD (5 physical + 32 mux slots)
```

The 3 I2S bus channels are **consumed** by the decoder — they do not appear
in the output. The output has 8 − 3 + 32 = **37 channels**.

---

## 3. I2S Mux Protocol — Background

### 3.1 I2S Bus Parameters

| Parameter | Value |
|-----------|-------|
| Sample rate | 250 kHz (`I2S_SAMPLE_RATE_HZ`) |
| Bits per frame | 16-bit stereo (32 bits total) |
| BCLK | 250 kHz × 32 bits = **8 MHz** |
| Frame duration | 32 bits / 8 MHz = **4 µs** |
| At 16 MHz stepper reference | **64 ticks per frame** |
| Block duration | 500 µs (`I2S_BLOCK_DURATION_US`) |
| Blocks per cycle | 2 (`I2S_BLOCK_COUNT`) |
| Frames per block | 250,000 × 500 / 1,000,000 = **125** |
| Ticks per block | 125 × 64 = **8,000 ticks** |
| Bytes per block | 125 × 4 = **500 bytes** |
| Total frames (2 blocks) | 250 |
| Total bytes (2 blocks) | 1,000 |

### 3.2 Mux Slot Encoding

Each I2S frame carries 32 bits. In mux mode, **one bit per frame represents
one stepper's step signal**. The mapping is:

```
slot S → byte S/8, bit S%8

slot 0 → bit 0  (byte 0, bit 0)
slot 1 → bit 1  (byte 0, bit 1)
...
slot 7 → bit 7  (byte 0, bit 7)
slot 8 → bit 8  (byte 1, bit 0)
...
slot 31 → bit 31 (byte 3, bit 7)
```

The mux state is a 32-bit word (`_mux_state`), and each frame in the I2S
buffer carries this word. The `i2s_fill_buffer_mux()` function sets the
appropriate bit in the buffer based on whether the stepper is emitting a
step pulse (high) or not (low).

### 3.3 Frame-by-Frame Mux Protocol

For each I2S frame:

1. Read the 32-bit mux state word.
2. For each slot S (0–31), the S-th bit of the word represents the step
   signal for that stepper.
3. The bit is placed at `byte[S/8]` with mask `1 << (S % 8)`.
4. When a stepper's queue entry has `steps > 0`, the corresponding bit is
   set to 1 for `I2S_TICKS_PER_FRAME` (64 ticks = 4 µs), producing a
   **high pulse**.
5. When the stepper is in a low period (between steps), the bit is 0.

### 3.4 Minimum Speed Limits

| Mode | Min speed (ticks) | Description |
|------|-------------------|-------------|
| `I2S_DIRECT_MIN_SPEED_TICKS` | 80 | Direct mode minimum |
| `I2S_MUX_MIN_SPEED_TICKS` | 400 | Mux mode minimum (5× slower) |

400 ticks is 5× `I2S_DIRECT_MIN_SPEED_TICKS` (80). The ratio is not
time-sharing among the 32 slots. Every frame holds the whole 32-bit
`_mux_state`, so each slot is updated every frame (4 µs), not once per 32
frames. The limit is the frame grid. A step pulse is one frame high (64
ticks). Rates are whole frames apart (2 frames = 125 kHz, 6 frames ≈ 42 kHz),
and 40 kHz is the practical maximum. A 400-tick command is not one steady
25 µs period on the wire. The 64-tick high is one frame, and the 336 low
ticks are 5 frames plus 16 ticks. `off_ticks` carries that remainder, so the
step period repeats as 6, 6, 6, then 7 frames (24, 24, 24, 28 µs) and the
low time is 5, 5, 5, then 6 frames. The average period is 25 µs (40 kHz).
The average duty is 4 µs / 25 µs = 16%; a single pulse is 4/24 or 4/28.

---

## 4. Saleae Capture — 8 Physical Channels

### 4.1 Channel Allocation

The Saleae captures **8 physical pins**, allocated as:

| Channel | Purpose | Count |
|---------|---------|-------|
| I2S data | Multiplexed step data from ESP32 I2S mux | 1 |
| I2S bit clock | 8 MHz bit clock | 1 |
| I2S word select | Word select (frame sync) | 1 |
| Stepper pins | Step/dir from other drivers (RMT, MCPWM, etc.) | 5 |
| **Total** | | **8** |

The 5 stepper pins carry step/dir signals from steppers using other drivers
(RMT, MCPWM/PCNT, etc.). These are passed through to the output unchanged.

### 4.2 Sample Rate Considerations

The I2S bit clock is **8 MHz**. To resolve the I2S protocol correctly, the
Saleae sample rate must be high enough to capture the bclk edges.

| Sample Rate | Samples per BCLK period | Resolves 8 MHz bclk? |
|-------------|------------------------|----------------------|
| 4 MS/s | 2 | marginal (Nyquist) |
| 8 MS/s | 4 | adequate |
| 16 MS/s | 8 | good |
| 24 MS/s | 16 | excellent |

**Recommendation:** Use **≥ 8 MS/s** to reliably decode the I2S bus signals.
At 4 MS/s, the bclk edges are sampled at exactly 2 samples per period, which
is the Nyquist limit and prone to aliasing. At 8 MS/s, there are 4 samples
per bclk period, which is sufficient for edge detection.

The 5 stepper pins can be analyzed at whatever sample rate is needed for
the stepper signals (typically 1–4 MS/s is sufficient for stepper step/dir
waveforms). Since all 8 channels are captured at the same sample rate, the
I2S bus resolution drives the choice.

### 4.3 Capture Format

The capture is saved as `.sr` (sigrok srzip) and converted to VCD:

```bash
sigrok-cli --driver saleae \
    --config channels:0,1,2,3,4,5,6,7 \
    --config sample-rate:8000000 \
    --output capture.sr

sigrok-cli -I srzip -O vcd -i capture.sr -o capture_8ch.vcd
```

The 8-channel VCD contains:
- 3 I2S bus channels (data, bclk, ws) — to be decoded
- 5 stepper channels (D0–D4 or named) — to be passed through

---

## 5. VCD-to-VCD Decoder Design

### 5.1 Decoder Pipeline

```
8-channel VCD (Saleae capture)
    │
    ├── Identify I2S bus channels (data, bclk, ws) from config
    ├── Identify passthrough channels (stepper pins) from config
    │
    ├── Extract I2S frames from bus signals:
    │   1. Detect ws transitions (frame boundaries)
    │   2. For each frame, read 32 bits sampled on bclk rising edges
    │
    ├── For each frame, extract 32-bit mux state word
    │
    ├── Synthesize 32 mux-slot channels (S0–S31):
    │   For each frame, for each slot S (0–31):
    │       bit = (mux_state >> S) & 1
    │       Append (frame_time, bit) to S's channel
    │
    ├── Pass through 5 stepper channels unchanged
    │
    └── Write 37-channel VCD (5 passthrough + 32 mux slots)
```

### 5.2 I2S Bus Signal Identification

The decoder is configured by a JSON file specifying which of the 8 channels
are the I2S bus signals:

```json
{
  "source_vcd": "capture_8ch.vcd",
  "output_vcd": "decoded_37ch.vcd",
  "i2s_channels": {
    "data": "D0",
    "bclk": "D1",
    "ws": "D2"
  },
  "passthrough_channels": ["D3", "D4", "D5", "D6", "D7"],
  "mux_slot_map": {
    "STEPPER_A": {"slot": 0},
    "STEPPER_B": {"slot": 1},
    "STEPPER_C": {"slot": 2},
    "STEPPER_D": {"slot": 3}
  }
}
```

The `i2s_channels` map specifies which of the 8 source channels carry the
I2S bus signals. The `passthrough_channels` list specifies the remaining
channels that carry stepper step/dir signals from other drivers. The
`mux_slot_map` maps each stepper to its mux slot (0–31).

### 5.3 Frame Extraction

The decoder extracts I2S frames by detecting ws transitions:

```python
def extract_i2s_frames(vcd_channels, i2s_config):
    """Extract I2S frames from bus signals.

    Args:
        vcd_channels: Dict of channel name → signal samples (from load_vcd)
        i2s_config: Dict with 'data', 'bclk', 'ws' channel names

    Returns:
        List of (frame_index, 32_bit_word) tuples
    """
    data = vcd_channels[i2s_config["data"]]
    bclk = vcd_channels[i2s_config["bclk"]]
    ws = vcd_channels[i2s_config["ws"]]

    # Find frame boundaries (ws transitions)
    frame_boundaries = find_ws_transitions(ws)

    frames = []
    for frame_idx, (start, end) in enumerate(frame_boundaries):
        # Read 32 bits sampled on bclk rising edges within this frame
        bits = 0
        for bit_pos in range(31, -1, -1):
            bclk_edge = find_next_bclk_rising_edge(bclk, start)
            if bclk_edge < end:
                bit_value = data[bclk_edge]
                bits = (bits << 1) | bit_value
            start = bclk_edge + 1
        frames.append((frame_idx, bits))

    return frames
```

### 5.4 Mux Slot Synthesis

For each extracted frame, the decoder synthesizes 32 mux-slot channels:

```python
def synthesize_mux_slots(frames, passthrough_channels):
    """Synthesize 32 mux-slot channels from decoded I2S frames.

    Args:
        frames: List of (frame_index, 32_bit_word) tuples
        passthrough_channels: Dict of passthrough channel name → samples

    Returns:
        Dict of channel name → samples (37 channels total)
    """
    channels = {}

    # Pass through 5 stepper channels unchanged
    for name, samples in passthrough_channels.items():
        channels[name] = samples

    # Synthesize 32 mux-slot channels
    for slot in range(32):
        channel_name = f"S{slot}"
        series = bytearray()

        for frame_idx, mux_state in frames:
            bit = (mux_state >> slot) & 1
            # Each frame is 4 µs = 64 ticks at 16 MHz
            # At the capture sample rate, compute the number of samples
            # per frame and append the bit value for the duration
            samples_per_frame = compute_samples_per_frame(frame_idx)
            series.extend(bytes([bit]) * samples_per_frame)

        channels[channel_name] = series

    return channels
```

### 5.5 Decoded VCD Format

The decoded VCD has 37 channels:

- 5 passthrough channels (unchanged from source): `D3`, `D4`, `D5`, `D6`, `D7`
- 32 mux-slot channels: `S0`, `S1`, …, `S31`

```
$date Sat Oct  3 14:20:20 2026 $end
$version libsigrok 0.5.2 $end
$timescale 125 ns $end   ← 8 MS/s → 125 ns per sample
$scope module decoded_i2s_mux $end
$var wire 1 D3 $end
$var wire 1 D4 $end
$var wire 1 D5 $end
$var wire 1 D6 $end
$var wire 1 D7 $end
$var wire 1 ! S0 $end
$var wire 1 " S1 $end
...
$var wire 1 ( S31 $end
$upscope $end
$enddefinitions $end
#0 0D3 0D4 ... 0S0 0S1 ... 0S31
#100 1D3 ... 1S0 ...
...
```

The decoded VCD uses the same `$timescale` as the source VCD (determined by
the capture sample rate). At 8 MS/s, `$timescale` is 125 ns.

### 5.6 Key Design Decision: I2S Signals Are Consumed

The 3 I2S bus channels (data, bclk, ws) are **consumed** by the decoder —
they do not appear in the output VCD. The output has 8 − 3 + 32 = **37
channels**. This is by design: the mux-slot channels (S0–S31) are the
**decoded representation** of the I2S bus signals, just as the passthrough
channels are the unchanged representation of the 5 stepper pins.

After VCD→VCD processing, there is **no I2S bus information** in the
decoded VCD. The existing signal parser evaluates the 37 channels directly,
treating mux-slot channels (S0–S31) as stepper step signals.

---

## 6. Configuration Management

### 6.1 JSON Configuration File

The decoder is configured by a JSON file:

```json
{
  "source_vcd": "capture_8ch.vcd",
  "output_vcd": "decoded_37ch.vcd",
  "i2s_channels": {
    "data": "D0",
    "bclk": "D1",
    "ws": "D2"
  },
  "passthrough_channels": ["D3", "D4", "D5", "D6", "D7"],
  "mux_slot_map": {
    "STEPPER_A": {"slot": 0, "step_channel": "S0", "dir_channel": null},
    "STEPPER_B": {"slot": 1, "step_channel": "S1", "dir_channel": null},
    "STEPPER_C": {"slot": 2, "step_channel": "S2", "dir_channel": null},
    "STEPPER_D": {"slot": 3, "step_channel": "S3", "dir_channel": null}
  },
  "stepper_count": 4,
  "pin_mode": "nodir"
}
```

The configuration specifies:

1. **`i2s_channels`** — which of the 8 source channels carry the I2S bus
   signals (data, bclk, ws).
2. **`passthrough_channels`** — which of the 8 source channels carry stepper
   step/dir signals from other drivers (passed through unchanged).
3. **`mux_slot_map`** — maps each stepper to its mux slot (0–31) and
   specifies which mux-slot channel carries its step signal.
4. **`stepper_count`** — number of steppers on the I2S mux (1–32).
5. **`pin_mode`** — `nodir` (step only, one mux slot) or `dir`. Direction
   is a second slot in the same 32-bit word (another of S0–S31), or a GPIO
   on a passthrough channel. There is no slot S+32.

### 6.2 VCD-Embedded Metadata (Alternative)

Instead of (or in addition to) a separate JSON file, the configuration can
be embedded in the VCD's `$comment` block:

```
$comment
  Acquisition with 8/8 channels at 8 MHz
  I2S_MUX_CONFIG: data=D0, bclk=D1, ws=D2, passthrough=D3,D4,D5,D6,D7
  MUX_SLOT_MAP: STEPPER_A=0, STEPPER_B=1, STEPPER_C=2, STEPPER_D=3
  STEPPER_COUNT: 4
  PIN_MODE: nodir
$end
```

The decoder parses this metadata from the VCD header, eliminating the need
for a separate JSON file. The metadata is written by the virtual mux script
when it produces the 8-channel VCD, or by the harness when it captures the
VCD directly.

**Pros of VCD-embedded metadata:**
- Single file, no separate config file to manage
- Self-documenting — the VCD contains all information needed to decode it
- No risk of config-file/VCD mismatch

**Cons of VCD-embedded metadata:**
- Not all VCD viewers display `$comment` blocks
- Harder to edit manually
- May be lost if the VCD is converted between formats

**Recommendation:** Support both. The decoder checks for VCD-embedded
metadata first; if absent, falls back to a JSON config file. The harness
writes the metadata when producing the 8-channel VCD.

### 6.3 Channel Map for Evaluators

The evaluators receive a channel map that assigns each stepper to a mux-slot
channel:

```json
{
  "stepper_count": 4,
  "pin_mode": "nodir",
  "steppers": {
    "A": {"step_channel": "S0"},
    "B": {"step_channel": "S1"},
    "C": {"step_channel": "S2"},
    "D": {"step_channel": "S3"}
  }
}
```

This is the same format used by the harness's `MAP` command (white paper
§5.0). The decoder writes this map to the decoded VCD's metadata, and the
evaluators read it from there.

---

## 7. Generalized VCD Analysis for 37 Channels

### 7.1 Current Limitations

The existing `signal_parser.py` handles any number of channels in the VCD
file itself (the `load_vcd()` function reads `$var` declarations dynamically).
However, the harness code has hard-coded assumptions:

1. **Channel pin mapping** — a fixed table mapping D0–D7 to GPIO numbers.
2. **Evaluator channel names** — hard-coded references to `D0`, `D2`,
   `D4`, `D6` for step pins.
3. **Stepper count** — the harness assumes 4 steppers (dir mode) or 8
   steppers (nodir mode).

### 7.2 Generalization Plan

1. **Replace the channel pin mapping table** with a channel map that is
   read from the VCD metadata or from a JSON file.

2. **Replace hard-coded evaluator channel names** with a `Pins` object
   that maps steppers to channels by name.

3. **Generalize the stepper count** to support 1..32 steppers (limited by
   the I2S mux driver's 32 slots).

4. **Add mux-slot channel support** — the signal parser already handles
   any channel name, so no changes are needed in `load_vcd()`. The
   evaluators must recognize `S`-prefixed channels as mux-slot signals.

5. **Handle the 37-channel decoded VCD** — 5 passthrough channels + 32
   mux-slot channels. The evaluators process each stepper's channel
   independently, so the channel count is irrelevant.

### 7.3 Signal Parser Extensions

The existing `signal_parser.py` requires minimal changes:

1. **No changes to `load_vcd()`** — it already handles any number of
   channels by reading `$var` declarations dynamically.

2. **No changes to `stepper_metrics()`** — it operates on a single
   channel's samples, not on a fixed set of channels.

3. **No changes to `channel_metrics()`** — it operates on a single
   channel's samples.

4. **No changes to `dir_to_first_step_us()`** — it operates on two
   channels (dir and step), not on a fixed set of channels.

5. **No changes to `cross_channel_skew_us()`** — it operates on a
   dictionary of channels, not on a fixed set.

The only change needed is in the harness code that passes channel names
to the evaluators. This is the `Pins` object (see §7.4).

### 7.4 Channel-Agnostic Evaluators

The existing evaluators hard-code channel names (`D0`, `D2`, `D4`, `D6`
for step pins). This must be replaced by a `Pins` object that maps steppers
to channels:

```python
class Pins:
    """Maps steppers to channels, not hardcoded channel names."""

    def __init__(self, channel_map):
        self.channel_map = channel_map

    def get_step_channel(self, stepper_id):
        return self.channel_map[stepper_id]["step_channel"]

    def get_dir_channel(self, stepper_id):
        return self.channel_map[stepper_id].get("dir_channel")
```

The evaluators receive a `Pins` object and look up channels by stepper ID,
not by hard-coded names. This makes the evaluators work with any channel
count and any channel naming convention (D0–D7 for physical channels,
S0–S31 for mux-slot channels).

---

## 8. Integration with Existing Test Harness

### 8.1 Harness Updates

The harness must be updated to:

1. **Read the 8-channel VCD** — the Saleae captures 8 physical channels:
   3 I2S bus (data, bclk, ws) + 5 stepper pins.

2. **Run the VCD-to-VCD decoder** — decode the I2S bus signals into 32
   mux-slot channels, producing a 37-channel VCD (8 − 3 + 32).

3. **Evaluate the 37-channel VCD** — the existing signal parser handles
   any number of channels. The evaluators use the `Pins` object to look
   up channels by stepper ID.

4. **Accept a channel map** that includes S0–S31 mux-slot channels. The
   channel map is read from the VCD metadata (if embedded) or from a
   JSON config file.

### 8.2 Pipeline Invocation

```bash
# Step 1: Capture 8 physical channels (3 I2S bus + 5 stepper pins)
sigrok-cli --driver saleae --config channels:0..7 \
    --config sample-rate:8000000 \
    --output capture.sr

# Step 2: Convert to 8-channel VCD
sigrok-cli -I srzip -O vcd -i capture.sr -o capture_8ch.vcd

# Step 3: Run VCD-to-VCD decoder (8 → 37 channels)
python3 i2s_mux_decoder.py --config decoder_config.json

# Step 4: Evaluate decoded 37-channel VCD (existing signal_parser.py)
python3 run_tests.py --tag-key esp32_i2s_mux_4ch
```

### 8.3 Tag Key Updates

The tag key format `{arch}_{driver}_{channel_config}` must be extended to
support the I2S mux driver:

| Driver | Tag Key Suffix | Description |
|--------|----------------|-------------|
| `i2s_direct` | `i2s_direct` | Direct mode (3 steppers max) |
| `i2s_mux` | `i2s_mux` | Mux mode (up to 32 steppers) |
| `i2s_mux_N` | `i2s_mux_N` | Mux mode with N steppers |

The channel config suffix is updated to include the mux slot count:
`{count}/mux` for mux mode, `{count}/direct` for direct mode.

---

## 9. Maximum-Speed Test Scenario

### 9.1 Test: 32 Steppers at Maximum Speed

This test characterizes the I2S mux driver with the maximum number of
steppers (32) at the maximum speed (minimum ticks = 400 for mux mode).

**Program:**

```
QCLR
QSEG 255 400 1     # 255 steps at minimum mux speed (400 ticks)
QSEG 255 400 1     # repeat to fill the queue
...
QRUN 0xFFFFFFFF    # run all 32 steppers simultaneously
```

**What it measures:**

| Metric | Expected | Why |
|--------|----------|-----|
| Step count per stepper | 255 × N segments | Verifies no steps are lost or duplicated |
| Inter-step period | 400 ticks average (25 µs). Wire gaps repeat 6, 6, 6, 7 frames (24 µs, then 28 µs) | Frame grid in `i2s_fill_buffer_mux()` |
| Cross-stepper skew | < 100 µs | Verifies synchronized start across 32 steppers |
| Duty cycle | 16% average (4 µs high / 25 µs). A single pulse is 4/24 or 4/28 | One-frame high pulse |
| Mux frame rate | 250 kHz | Verifies the I2S bus is running at the correct rate |

### 9.2 Sample Rate Requirement

At 400 ticks (25 µs average inter-step period), the step frequency is 40 kHz.
The I2S bus runs at 250 kHz sample rate with 8 MHz BCLK. To resolve both
the stepper signals and the I2S bus, the sample rate must be **≥ 8 MS/s**.

| Sample Rate | Resolves 8 MHz BCLK? | Resolves 40 kHz Steps? | Recommended? |
|-------------|---------------------|------------------------|--------------|
| 4 MS/s | marginal (2 samples/bclk) | yes (100 samples/step) | no |
| 8 MS/s | adequate (4 samples/bclk) | yes (200 samples/step) | yes |
| 16 MS/s | good (8 samples/bclk) | yes (400 samples/step) | yes |
| 24 MS/s | excellent (16 samples/bclk) | yes (600 samples/step) | best |

**Recommendation:** Use **16 MS/s** for the maximum-speed test to provide
comfortable headroom for both the I2S bus and the stepper signals.

### 9.3 Expected Results

On an ESP32 with the I2S mux driver, 32 steppers at 400 ticks:

- Each stepper emits 255 steps. The commanded period is 400 ticks
  (25 µs average, 40 kHz).
- The I2S bus runs at 250 kHz. Every frame is one 32-bit sample of all
  slots. There is no 32-frame mux cycle.
- Each frame carries 1 bit per stepper (32 bits = one `_mux_state` word).
- Slot S is high for one frame (4 µs) per step. Because 400 is not a
  multiple of 64, the step period repeats as 6, 6, 6, then 7 frames
  (24 µs and 28 µs). The low time is the rest: 5, 5, 5, then 6 frames.
- Average duty cycle is 4 µs / 25 µs = 16%.

---

## 10. Quality and Validation

### 10.1 Validation Strategy

The VCD-to-VCD decoder must be validated against known inputs:

1. **Golden fixture** — a known I2S mux capture with known step/dir
   waveforms. The decoder must produce the exact same waveforms.

2. **Cycle-accurate simulation** — the decoder's frame extraction and
   bit sampling must match the I2S protocol specification exactly.

3. **Cross-reference with physical capture** — when the same stepper
   program is run on both the physical Saleae capture and the virtual
   mux capture, the decoded waveforms must match.

### 10.2 Error Handling

The decoder must handle:

1. **Missing I2S bus channels** — if the VCD does not contain the
   expected I2S bus channels, the decoder must report an error and
   exit gracefully.

2. **Malformed I2S frames** — if the bclk/ws/data signals do not
   conform to the I2S protocol (e.g., wrong frame length, missing
   ws transitions), the decoder must report the error and skip the
   affected frames.

3. **Unmapped slots** — if the channel map references a slot that
   does not exist in the VCD, the decoder must report an error.

4. **Out-of-range slots** — if the channel map references a slot
   outside 0–31, the decoder must report an error.

### 10.3 Performance

The decoder must handle 8-channel VCDs with millions of samples in
a reasonable time. The critical path is the frame extraction, which
is O(n) where n is the number of samples. The frame decoding is O(n/32)
where n is the number of frames. The mux-slot synthesis is O(n × 32)
where n is the number of frames and 32 is the number of slots.

For a 10-second capture at 8 MS/s (80 million samples), the decoder
should complete in under 120 seconds on a modern laptop.

---

## 11. Hardware Requirements

### 11.1 Analyzer Requirements

| Item | Minimum | Recommended |
|------|---------|-------------|
| Target board | ESP32 with I2S mux driver | ESP32-S3, ESP32-C6 |
| Logic analyzer | 8 channels, 8 MS/s | 8 channels, 16–24 MS/s |
| USB cable | For the serial console | — |

### 11.2 Channel Budget

| Mode | Channels per stepper | Max steppers on 8 channels |
|------|---------------------|---------------------------|
| `dir` (other drivers) | 2 (step + dir) | 2 steppers (5 channels / 2) |
| `nodir` (other drivers) | 1 (step only) | 5 steppers (5 channels) |
| `nodir` (I2S mux) | 1 (S0, S1, …) | 32 steppers (32 mux slots) |
| `dir` (I2S mux) | 2 (step + dir) | 16 steppers (32 mux slots / 2) |

The I2S mux word is 32 bits (`_mux_state`). Nodir uses one slot per
stepper (32 steppers). Dir-on-mux uses two of those same slots per
stepper, one for step and one for direction (16 steppers). A GPIO
direction pin does not consume a slot; capturing it takes one of the five
passthrough channels. The decoded VCD still has S0–S31 only.

---

## 12. Directory Structure

```
extras/tests/saleae_based/
├── white_paper_saleae_test_harness.md   # Existing white paper
├── white_paper_180_r7_virtual_i2s_mux.md # This white paper
├── scripts/
│   ├── capture.py                       # sigrok-cli capture
│   ├── signal_parser.py                 # edges, metrics, distributions
│   ├── run_tests.py                     # scenario table + evaluators
│   ├── i2s_mux_decoder.py              # NEW: 8 → 37 channel VCD decoding
│   └── tests/                           # hardware-free tests
├── capture/                             # Generated captures (git-ignored)
└── results/                             # Generated results (git-ignored)
```

The new file is:

- **`scripts/i2s_mux_decoder.py`** — The VCD-to-VCD decoder. Reads the
  8-channel VCD, extracts I2S bus signals (data, bclk, ws), decodes the
  mux protocol, and produces a 37-channel VCD with 5 passthrough channels
  + 32 mux-slot channels (S0–S31).

---

## 13. Relationship to Existing Test Infrastructure

The I2S mux extension complements (not replaces) the existing PC-based
and SimAVR-based tests:

| Layer | Tool | Purpose |
|-------|------|---------|
| **Unit tests** | PC-based `test_XX` | Algorithm validation (ramp calculator, queue management) |
| **Simulation** | SimAVR `test_sd_*` | AVR-specific timing validation |
| **Pin-level characterization (physical)** | Saleae-based `SR_XX` | Measured waveform on real hardware (8 channels) |
| **Pin-level characterization (mux)** | Saleae-based `SR_XX` + I2S decoder | Measured waveform on real hardware (37 channels) |
| **CI** | SimAVR stub | Every commit (no hardware needed) |
| **Release** | Full Saleae suite + I2S decoder | Release candidates only |

The I2S mux extension adds a new capability: **mux-mode characterization**
that measures what the I2S mux driver actually emits on the wire, across
up to 32 steppers. This was not available in the original harness.

---

## 14. Known Limitations

### 14.1 I2S Mux Driver Limitations

The ESP32 I2S mux driver has the following limitations:

1. **Maximum 32 steppers** — limited by the 32-bit mux state word.
2. **Minimum speed of 400 ticks** — 5× the direct-mode minimum (80 ticks).
   The cause is the 64-tick frame grid (§3.4), which caps the practical
   rate at 40 kHz. Slots are not time-shared inside the frame.
3. **Direction** — a mux direction pin (`pin | PIN_I2S_FLAG`) is another
   slot 0–31 in the same `_mux_state`. `i2sMuxSetBit()` ignores a slot
   ≥ 32, and the allocation mask rejects a slot already used for step.
   Dir-on-mux therefore fits 16 steppers, not 32, and the decoder does
   not grow to 64 channels. A GPIO direction pin is outside the mux word.

### 14.2 Analyzer Limitations

The Saleae analyzer has a maximum of 8 physical channels. The I2S mux
decoder bridges this gap by decoding the 3 I2S bus channels into 32
mux-slot channels, producing a 37-channel VCD.

The sample rate must be sufficient to resolve the 8 MHz I2S bit clock.
At 4 MS/s, the bclk is sampled at exactly 2 samples per period (Nyquist
limit), which is marginal. At 8 MS/s or higher, the bclk is adequately
resolved.

### 14.3 Decoder Limitations

The VCD-to-VCD decoder has the following limitations:

1. **Requires I2S bus signals** — the decoder cannot operate on a VCD
   that does not contain the I2S bus signals (data, bclk, ws).

2. **Requires correct mux slot mapping** — the decoder uses the channel
   map to determine which mux slot corresponds to which stepper. If the
   map is incorrect, the decoded waveforms will be wrong.

3. **Does not validate the mux state** — the decoder assumes the mux
   state word is correct. If the ESP32 firmware produces incorrect mux
   state, the decoder will propagate the error.

---

## 15. Summary

This document specifies the design for extending the Saleae-based test
harness to characterize the ESP32 I2S mux driver with up to 32 steppers
using an 8-channel logic analyzer.

**The pipeline:**

1. **Capture 8 physical channels** — 3 I2S bus (data, bclk, ws) + 5
   stepper pins from other drivers — at ≥ 8 MS/s.

2. **Decode the I2S bus signals** — extract 32-bit mux state per frame
   from the I2S bus signals (data, bclk, ws), synthesize 32 mux-slot
   channels (S0–S31).

3. **Produce a 37-channel VCD** — 5 passthrough channels + 32 mux-slot
   channels. The 3 I2S bus channels are consumed and do not appear in
   the output.

4. **Evaluate the 37-channel VCD** — the existing signal parser handles
   any number of channels. The evaluators use a `Pins` object to look
   up channels by stepper ID.

**Configuration management:**

- JSON config file specifies which channels are I2S bus, which are
  passthrough, and the mux slot-to-stepper mapping.
- Alternatively, metadata can be embedded in the VCD's `$comment` block,
  eliminating the need for a separate JSON file.

**Maximum-speed test:**

- 32 steppers at 400 ticks (minimum mux speed = 25 µs average inter-step
  period, 40 kHz step frequency) at 16 MS/s sample rate.
