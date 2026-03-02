# ESP32 I2S Output Driver

## Overview

This document describes the architecture, design, implementation, and testing plan for a
new ESP32 stepper output driver based on the I2S peripheral with DMA. The driver targets
two distinct use-cases:

- **Single-stepper mode**: One stepper using one I2S DATA_OUT GPIO. The bitstream provides
  32× higher time resolution within each frame, enabling step rates up to ~200kHz.
- **Multi-stepper mode**: One I2S DATA_OUT line feeds an external serial-to-parallel demux IC
  (e.g., a chain of 74HC595 shift registers). Up to 16 steppers (or 8 with step+dir, or 5
  with step+dir+enable) share the same bitstream at frame-level resolution.

An existing RMT implementation already serves the ESP32 well. The I2S driver is an
**additive** option, not a replacement. Its main value is enabling serial-bus expansion
via a demux IC.

**Target**: ESP32 (original) initially. Will be rolled out to all ESP32 variants with
I2S and the same IDF API once validated.

---

## Hardware Background

### I2S as a Raw Bit Stream

The ESP32 I2S peripheral is a serial audio bus but can be driven as a raw DMA-fed bit
stream:

- **BCLK** – bit clock, drives shift register on demux IC; max ~10 MHz on external
  hardware (the IC limit, not the ESP32 limit)
- **WS/LRCLK** – word-select / frame sync, triggers output latch on demux IC
- **DATA_OUT (SD)** – serial data

FastAccelStepper uses `TICKS_PER_S = 16 000 000`. The I2S configuration:

```
Sample rate: 250 kHz
Bits per frame: 16-bit L + 16-bit R = 32 bits
BCLK = 250 kHz × 32 = 8 MHz
Frame duration = 32 bits / 8 MHz = 4 µs = 64 ticks (@16 MHz)
```

| Mode | Resolution | Max Practical Freq | Notes |
|------|------------|---------------------|-------|
| Single-stepper | 2 ticks (bit-level) | ~200 kHz | 32× resolution within frame |
| Multi-stepper | 64 ticks (frame-level) | ~40 kHz | Limited by frequency granularity |

### Frequency Granularity in Multi-Stepper Mode

In multi-stepper mode, steps are frame-aligned. To emit a step, you need HIGH followed by LOW:

| Step Period | Frequency | Notes |
|-------------|-----------|-------|
| 2 frames (8µs) | 125 kHz | Max theoretical |
| 3 frames (12µs) | 83.3 kHz | 33% drop from max |
| 4 frames (16µs) | 62.5 kHz | |
| 5 frames (20µs) | 50 kHz | |
| 6 frames (24µs) | 41.7 kHz | Practical max |

The jump from 125 kHz to 83 kHz is 33%. For acceptable frequency granularity,
practical max frequency in multi-stepper mode is ~40 kHz.

### Demux IC (Multi-Stepper Mode)

A shift-register chain (e.g., two 74HC595 in series = 16 outputs) receives:
- DATA_OUT → SER input
- BCLK → SRCLK (shift clock)
- WS/LRCLK → RCLK (storage register clock / latch)

Each WS edge latches the last 32 bits from DATA_OUT to the parallel output. The
parallel outputs drive individual step, direction, and enable signals of stepper drivers.

### Single-Stepper Mode

In single-stepper mode, the entire 32-bit frame is used for one stepper's step signal.
The effective bit rate is 2 × 16 × frame_rate = 8 Mbit/s. Each bit represents 2 ticks
(125 ns).

A single bit HIGH (125 ns) is too short for a step pulse. A step pulse consists of
N consecutive 1-bits, where N = pulse_width_ticks / 2. For example, a 4 µs pulse =
32 consecutive 1-bits = one full frame of all-ones. Sub-frame edge placement provides
positioning resolution of 2 ticks.

WS and BCLK should not be output on GPIOs in single-stepper mode (only the DATA_OUT
bitstream is needed). Set WS to `I2S_GPIO_UNUSED` to avoid consuming a GPIO.

---

## Driver Contract

The I2S driver follows the same contract as all other pulse drivers:

1. Execute commands from the command queue exactly as specified
2. Correctly advance `read_idx` after each command is fully consumed
3. Maintain synchronicity with the µC timebase by counting I2S frame ticks

The driver does NOT feed back position or step counts to the ramp generator.
No pulse driver does. Position tracking is handled by `queue_end.pos` updates
in the common queue code.

### Clock Synchronization

`TICKS_PER_S = 16 000 000` and I2S frame rate = 250 000 → 64 ticks/frame. This
is an exact integer ratio, so no drift accumulates if the I2S clock is derived from
the same PLL as the CPU clock. Verify this on hardware.

If a small discrepancy exists (e.g., due to PLL configuration or crystal tolerance),
a Bresenham-style correction can accumulate fractional-tick error per frame and
insert/skip a tick when the error exceeds ±1 tick. The adjustment range is 1–64 ticks.

### Step Counting

Step count is decremented at fill time when the pulse HIGH phase is committed to the
DMA buffer. This means steps are counted before they are physically output. ForceStop
may result in position error of up to `(I2S_BLOCK_COUNT - 1) × max_steps_per_block`
steps. This is consistent with the general contract that `forceStop()` does not
guarantee maintaining exact position.

### Thread Safety

`read_idx` and `next_write_idx` are single bytes (uint8_t), which are atomically
safe to read and write on ESP32. The contract is:

- `read_idx` is updated **only** by the driver (fill routine in I2S task context)
- `next_write_idx` is updated **only** by the ramp generator (StepperTask context)

No additional synchronization (mutex/spinlock) is needed.

---

## Architecture Design

### Mode Selection via Engine

The mode (single-stepper vs multi-stepper) is configured at Engine initialization,
**before** any steppers are connected:

```cpp
// Single-stepper mode: one I2S peripheral = one stepper
engine.initI2sSingleStepper(data_pin, bclk_pin);

// Multi-stepper mode: one I2S peripheral = many steppers via demux
engine.initI2sMultiStepper(data_pin, bclk_pin, ws_pin, signals_per_stepper);
```

### Constants

All constants are derived from `I2S_BLOCK_DURATION_US`:

```c
#define I2S_SAMPLE_RATE_HZ      250000UL
#define I2S_TICKS_PER_FRAME     64
#define I2S_BITS_PER_FRAME      32
#define I2S_BYTES_PER_FRAME     4

#define I2S_BLOCK_DURATION_US   500
#define I2S_BLOCK_COUNT         3
#define I2S_FRAMES_PER_BLOCK    (I2S_BLOCK_DURATION_US * I2S_SAMPLE_RATE_HZ / 1000000UL)
                                // = 125
#define I2S_BLOCK_TICKS         (I2S_FRAMES_PER_BLOCK * I2S_TICKS_PER_FRAME)
                                // = 8000
#define I2S_BYTES_PER_BLOCK     (I2S_FRAMES_PER_BLOCK * I2S_BYTES_PER_FRAME)
                                // = 500

#define MAX_STEP_FREQ_HZ        200000UL
#define I2S_MAX_PULSES_PER_BLOCK ((I2S_BLOCK_DURATION_US * MAX_STEP_FREQ_HZ / 1000000UL) + 1)
                                // = 101
```

Maximum frequency (200 kHz) is enforced by `addQueueEntry()`, not by the I2S driver.
The pulse tracking array is sized to this guaranteed maximum.

### Triple Buffering

The I2S driver uses triple buffering with 3 blocks of 500 µs each:

```
Block 0    Block 1    Block 2
[500µs]    [500µs]    [500µs]
   ↓          ↓          ↓
DMA continuously streams blocks in rotation
```

### DMA Architecture: Two-Stage ISR + Task

The DMA callback runs in ISR context. Since `i2s_channel_write()` acquires internal
mutexes, it cannot be called from ISR. The architecture uses a two-stage design:

**Stage 1 — ISR (IRAM_ATTR)**:
1. Update `busy_block = (busy_block + 1) % 3`
2. Call `xTaskNotifyFromISR()` to wake the I2S fill task

**Stage 2 — I2S Fill Task (high-priority FreeRTOS task)**:
1. Wait for notification via `xTaskNotifyWait()`
2. For each I2S stepper: call `fill_i2s_buffer()` for the available blocks
3. Call `i2s_channel_write()` (blocking, timeout = portMAX_DELAY) to submit
   the filled block to DMA

The CPU core for the I2S fill task is configurable at engine initialization.

### DMA Configuration

The IDF I2S driver is used (preference over register-level access for
forward-compatibility with new IDF releases). Configuration:

```c
#define I2S_DMA_DESC_NUM    2    // minimum for continuous streaming
#define I2S_DMA_FRAME_NUM   I2S_FRAMES_PER_BLOCK  // one block per descriptor
```

The application queues the same three buffer addresses (blocks 0, 1, 2) to
`i2s_channel_write()` in rotation. The blocking write naturally paces the fill
task to the DMA consumption rate.

**Zero-copy assumption**: `i2s_channel_write()` with blocking timeout passes
the buffer pointer to the DMA engine without copying. This must be verified on
hardware (see §Verification Sketch).

### DMA ↔ Fill ↔ StepperTask Sequence

```
Time →
         Block 0 plays    Block 1 plays    Block 2 plays    Block 0 plays
DMA:     ████████████████  ████████████████  ████████████████  ████████████████
                        ↑                 ↑                 ↑
ISR:                    │ on_sent         │ on_sent         │ on_sent
                        │ busy=1          │ busy=2          │ busy=0
                        │ notify task     │ notify task     │ notify task
                        ↓                 ↓                 ↓
I2S Task:          ┌──────────┐      ┌──────────┐      ┌──────────┐
                   │B1 = busy │      │B2 = busy │      │B0 = busy │
                   │fill B2,B0│      │fill B0,B1│      │fill B1,B2│
                   │write to  │      │write to  │      │write to  │
                   │DMA       │      │DMA       │      │DMA       │
                   └──────────┘      └──────────┘      └──────────┘

StepperTask:  ─── manageSteppers() ──── manageSteppers() ────
                  (adds cmds to queue)   (adds cmds to queue)
```

When ISR fires with busy = N:
- Block N is currently being transmitted by DMA (do not touch)
- Block (N+1)%3 and (N+2)%3 are free and available for filling

**Timing budget**: The fill computation must complete within one block period
(500 µs). At 240 MHz ESP32 that is 120k CPU cycles — generous for the fill
algorithm even with 16 steppers.

### Slot Assignment (Multi-Stepper Mode)

When connecting a stepper in multi-stepper mode, bit-slots are allocated:

| Claim Mode | Slots | Signals |
|------------|-------|---------|
| `STEP_ONLY` | 1 | STEP |
| `STEP_DIR` | 2 | STEP, DIR |
| `STEP_DIR_ENABLE` | 3 | STEP, DIR, ENABLE |

```
base_slot = I2sManager::allocateSlots(claim_count)
stepper._i2s_step_slot   = base_slot
stepper._i2s_dir_slot    = (claim >= STEP_DIR) ? base_slot + 1 : -1
stepper._i2s_enable_slot = (claim == STEP_DIR_ENABLE) ? base_slot + 2 : -1
```

Slot = -1 means the signal uses conventional GPIO.

---

## Bit Serialization Order

The ESP32 I2S outputs bits MSB-first within each 16-bit half (L then R).

### Frame Layout in Memory

```
4 bytes per frame: [L_hi, L_lo, R_hi, R_lo]

I2S outputs in time order:
  L_hi[7], L_hi[6], ..., L_hi[0],  L_lo[7], ..., L_lo[0],
  R_hi[7], R_hi[6], ..., R_hi[0],  R_lo[7], ..., R_lo[0]
```

### Multi-Stepper Mode: Slot-to-Bit Mapping

Each slot (0..31) maps to a fixed bit position in the frame:

```c
byte_in_frame = slot / 8;          // 0..3
bit_in_byte   = 7 - (slot % 8);    // MSB first
byte_in_buf   = frame_index * 4 + byte_in_frame;
bit_mask      = 1 << bit_in_byte;
```

**Worked example**: Slot 5 (6th stepper bit)
- `byte_in_frame = 5 / 8 = 0` → L_hi
- `bit_in_byte = 7 - (5 % 8) = 2`
- `bit_mask = 0x04`
- To set: `buf[frame_index * 4 + 0] |= 0x04;`
- To clear: `buf[frame_index * 4 + 0] &= ~0x04;`

### Single-Stepper Mode: Tick-to-Bit Mapping

In single-stepper mode, the bit index represents a temporal position within the frame
(0..31 → first bit out to last bit out). The same formula applies, but `slot` is
replaced by `bit_time_index = tick_pos_within_frame / 2`:

```c
bit_time_index = (tick_pos % I2S_TICKS_PER_FRAME) / 2;  // 0..31
byte_in_frame  = bit_time_index / 8;
bit_in_byte    = 7 - (bit_time_index % 8);
byte_in_buf    = frame_index * 4 + byte_in_frame;
bit_mask       = 1 << bit_in_byte;
```

---

## Buffer Fill Algorithm

### Fill State

Each stepper maintains its own fill state:

```c
struct i2s_fill_state {
  uint32_t tick_pos;              // Current position in 16MHz ticks
  uint16_t remaining_high_ticks;  // Remaining HIGH time for current pulse
  uint16_t remaining_low_ticks;   // Remaining LOW time before next step
  uint16_t pulse_positions[I2S_BLOCK_COUNT][I2S_MAX_PULSES_PER_BLOCK];
  uint8_t  pulse_count[I2S_BLOCK_COUNT];
};
```

The `pulse_positions` array is per-block. It records where this stepper placed
HIGH bits in each block, enabling targeted clearing without memset.

`tick_pos` is `uint32_t` to avoid overflow constraints on block duration. With
`uint16_t`, the maximum would be limited to `65535 / (3 × 64) = 341 frames =
1364 µs` per block.

### No Memset — Per-Stepper Bit Tracking

The buffer is **never** bulk-zeroed. Each stepper is responsible for clearing
only its own previously-set bits before writing new ones. This is essential for
multi-stepper mode where multiple steppers share the same DMA buffer.

At low step rates (e.g., 1 kHz), only 1 byte needs to be touched per block.
This is far more efficient than clearing 500 bytes.

### fill_i2s_buffer() Algorithm

Called once per stepper per block fill cycle. The `block` parameter identifies
which block to fill. The stepper must not write to the busy block.

```
fill_i2s_buffer(stepper, block):

  1. CLEAR previous pulses for this block:
     for each pos in pulse_positions[block][0..pulse_count[block]-1]:
       clear stepper's bit(s) at buf[block][pos]
     pulse_count[block] = 0

  2. CARRY-OVER remaining HIGH phase:
     if remaining_high_ticks > 0:
       Write 1-bits from tick_pos for remaining_high_ticks
       (or to block end, whichever comes first)
       Record positions in pulse_positions[block]
       Advance tick_pos
       Reduce remaining_high_ticks accordingly
       If block full: save state, return

  3. CARRY-OVER remaining LOW phase:
     if remaining_low_ticks > 0:
       Advance tick_pos by min(remaining_low_ticks, ticks_to_block_end)
       Reduce remaining_low_ticks accordingly
       If block full: save state, return

  4. MAIN LOOP while tick_pos < block_end AND queue not empty:

     4a. Read next command from queue:
         if cmd.steps == 0:
           Pause: advance tick_pos by cmd.ticks (may span block end)
           Advance read_idx
         if cmd.steps >= 1:
           if toggle_dir: toggle direction pin
           remaining_high_ticks = PULSE_WIDTH_TICKS
           remaining_low_ticks = cmd.ticks - PULSE_WIDTH_TICKS
           Decrement cmd.steps
           if cmd.steps == 0: advance read_idx

     4b. Emit HIGH phase:
         Write 1-bits from tick_pos for remaining_high_ticks
         Record positions in pulse_positions[block]
         Advance tick_pos
         If block full: save state, return

     4c. Emit LOW phase:
         Advance tick_pos by min(remaining_low_ticks, ticks_to_block_end)
         Reduce remaining_low_ticks accordingly
         If block full: save state, return

     4d. Loop to 4a

  5. Save tick_pos and remaining ticks to state
```

### Single-Stepper vs. Multi-Stepper Fill Differences

| Aspect | Single-Stepper | Multi-Stepper |
|--------|----------------|---------------|
| Bit granularity | Per-bit (2 ticks) | Per-frame (64 ticks) |
| HIGH phase | N consecutive 1-bits across frame boundaries | Set bit-slot in frame byte |
| Pulse width | N × 2 ticks (configurable, min ~1 µs) | Integer frames × 64 ticks |
| Clear operation | Clear all 4 bytes of recorded frames | Clear specific bit in recorded frames |

### Handling Queue Empty

If the queue is empty, `fill_i2s_buffer()` clears previously set bits
(step 1) and returns. No new pulses are emitted. The stepper's fill state
retains `tick_pos` for correct timing when new commands arrive.

---

## ForceStop Algorithm

### Single-Stepper Mode

```
forceStop_single():
  Stop I2S DMA
  memset(all 3 blocks, 0)
  Clear fill_state (tick_pos, remaining ticks, all pulse_positions)
  Restart I2S DMA
```

Memset is acceptable here because only one stepper owns the entire buffer.

### Multi-Stepper Mode

```
forceStop_multi(stepper):
  for each block b in 0..2:
    for each pos in stepper.pulse_positions[b][0..pulse_count[b]-1]:
      clear stepper's bit-slot at buf[b][pos]
    stepper.pulse_positions[b].count = 0
  Clear stepper.fill_state (tick_pos, remaining ticks)
```

Other steppers are not affected. Clearing bits in the currently-transmitting
block is a benign race — worst case, one extra pulse is physically output.
This is consistent with the contract that forceStop does not guarantee
exact position.

---

## Multi-Stepper Signal Service

In multi-stepper mode, the DMA buffer carries no inherent semantics — only
the stepper knows which bit-slot is STEP vs DIR vs ENABLE. The I2S driver
provides low-level service functions for DIR/ENABLE manipulation:

```cpp
void setSlotBit(uint8_t block, uint16_t frame, uint8_t slot, bool value);
bool getSlotBit(uint8_t block, uint16_t frame, uint8_t slot);
```

These use the bit serialization mapping from §Bit Serialization Order.

**Direction and enable timing is not in scope of the pulse driver.** The
caller (stepper logic / ramp generator) is responsible for:

- Asserting DIR with sufficient setup time before STEP frames
- Not changing DIR or ENABLE during frames where STEP is asserted

The service functions do not enforce these constraints. Care must be taken
at the caller level to avoid direction changes during active step pulses.
This interaction is non-trivial and may require a dedicated sequencing layer
above the raw service functions.

---

## Pulse Width Constraints

### Single-Stepper Mode

Pulse width = N consecutive 1-bits. Configurable via `setI2sPulseWidth(ticks)`.
Default: 64 ticks (4 µs = 32 bits = one full frame).

Minimum practical pulse width depends on the stepper driver (typically ≥1 µs
= 8 bits = 16 ticks).

### Multi-Stepper Mode

Pulse width is quantized to frame multiples. `setI2sPulseWidth(ticks)` rounds
up to the next frame boundary. Minimum = 1 frame (4 µs).

| Pulse Width | Min Period | Max Freq |
|-------------|-----------|----------|
| 1 frame (4 µs) | 2 frames (8 µs) | 125 kHz theoretical |
| 2 frames (8 µs) | 4 frames (16 µs) | 62.5 kHz |

Practical max with acceptable granularity remains ~40 kHz.

### Minimum cmd.ticks

| Mode | Min ticks | Reason |
|------|-----------|--------|
| Single-stepper | 80 (5µs) | 200 kHz max, PULSE_WIDTH=64 ticks |
| Multi-stepper | 128 (8µs) | 2 frames minimum for HIGH+LOW |

Enforced by `addQueueEntry()`.

---

## API Design

### Engine-Level Configuration

```cpp
// src/FastAccelStepper.h

struct I2sSingleStepperConfig {
  uint8_t  data_pin;
  uint8_t  bclk_pin;
  uint8_t  cpu_core;             // CPU core for I2S fill task (0 or 1)
  uint16_t pulse_width_ticks;    // Default: 64
};

struct I2sMultiStepperConfig {
  uint8_t  data_pin;
  uint8_t  bclk_pin;
  uint8_t  ws_pin;
  uint8_t  signals_per_stepper;  // 1, 2, or 3
  uint8_t  cpu_core;             // CPU core for I2S fill task (0 or 1)
  uint16_t pulse_width_ticks;    // Default: 64
};

bool initI2sSingleStepper(const I2sSingleStepperConfig& cfg);
bool initI2sMultiStepper(const I2sMultiStepperConfig& cfg);
```

### Stepper Connection

```cpp
// Single-stepper mode: step_pin = (bit_position | ISR_PIN_MASK)
// ISR_PIN_MASK = 64 indicates single-stepper mode
FastAccelStepper* s = engine.stepperConnectToPin(step_pin, DRIVER_I2S);

// Multi-stepper mode: step_pin = slot index (0, 1, 2, ...)
FastAccelStepper* s = engine.stepperConnectToPin(slot_index, DRIVER_I2S);
```

The driver detects mode from `I2sManager::isInitialized()` and interprets
step_pin accordingly.

---

## Memory Budget

| Component | Single-Stepper | Multi-Stepper (16×) |
|-----------|---------------|---------------------|
| Triple buffer (3 × 500 B) | 1500 bytes | 1500 bytes (shared) |
| DMA descriptors (2×) | ~32 bytes | ~32 bytes |
| Per-stepper fill state | ~650 bytes × 1 | ~650 bytes × 16 |
| I2sManager singleton | ~50 bytes | ~50 bytes |
| **Total** | **~2200 bytes** | **~12000 bytes** |

Per-stepper fill state breakdown:
- `tick_pos` + remaining ticks: 8 bytes
- `pulse_positions[3][101]`: 606 bytes
- `pulse_count[3]`: 3 bytes
- Padding: ~33 bytes

---

## Error Handling

- **`i2s_channel_write()` failure**: Retry once, then log warning. DMA continues
  with stale buffer content.
- **DMA underrun** (fill can't keep up): Accepted risk. Log warning. Stale pulses
  may repeat or silence may occur.
- **Stepper disconnect while DMA runs**: No-op for I2S driver. Stepper's bit-slots
  remain zero (no pulses emitted).
- **`init()` called after DMA started**: Undefined behavior (application bug).

---

## File Structure

### New Files

```
src/pd_esp32/
  i2s_constants.h           — I2S timing constants (derived from BLOCK_DURATION_US)
  i2s_manager.h             — I2sManager class declaration
  i2s_manager.cpp           — I2sManager implementation (DMA, ISR, task)
  i2s_fill.h                — i2s_fill_state struct, fill_i2s_buffer() declaration
  i2s_fill.cpp              — fill_i2s_buffer() implementation
  StepperISR_esp32_i2s.cpp  — StepperQueue I2S methods

extras/tests/pc_based/
  test_21.cpp               — PC-based test bed for fill function
```

### Modified Files

```
src/fas_arch/common.h               — add I2S to FasDriver enum
src/fas_arch/common_esp32_idf5.h    — add SUPPORT_ESP32_I2S, QUEUES_I2S
src/pd_esp32/esp32_queue.h          — add I2S-specific members
src/pd_esp32/esp32_queue.cpp        — add I2S dispatch branches
src/FastAccelStepper.h              — add I2S config structs
src/FastAccelStepper.cpp            — implement initI2sXxx()
```

---

## Testing Strategy

### PC-Based Tests (test_21.cpp)

The fill function is testable on PC without I2S hardware. `test_21.cpp` provides:

- **Part 1**: DMA infrastructure simulation (callbacks, block rotation, bit stream
  analysis via `BitStreamAnalyzer`)
- **Part 2**: Fill function tests with queue commands (single step, multi step,
  pause, block boundary, carry-over, max values)

Test cases must be updated to:
- Remove `memset()` before fill calls (fill handles its own bit clearing)
- Verify per-stepper bit clearing (old pulses are removed, other bytes untouched)
- Add multi-stepper shared buffer tests (two steppers, different slots, no interference)
- Add forceStop tests (one stepper stops, other's bits preserved)

### Stage 1 — Single-Stepper Mode, PCNT Validation

Hardware: I2S DATA_OUT → PCNT input + oscilloscope

Tests:
- Verify step count matches commanded steps
- Test max frequency (~200 kHz)
- Verify pulse width timing
- Verify `i2s_channel_write()` zero-copy behavior (see §Verification Sketch)

### Stage 2 — Multi-Stepper Mode, Logic Analyzer

Hardware: I2S DATA_OUT, BCLK, WS → logic analyzer

Tests:
- Decode I2S stream, verify bit-slot timing
- Verify multiple steppers interleaved correctly
- Verify DIR/ENABLE bit stability during STEP pulses

### Stage 3 — Multi-Stepper Mode, 74HC595 + PCNT

Hardware: Full demux chain → PCNT

Tests:
- End-to-end step count validation
- Multiple simultaneous steppers

### Verification Sketch: i2s_channel_write() Zero-Copy

Create a minimal Arduino sketch:
1. Init I2S with 2 DMA descriptors
2. Write 3 rotating buffers via `i2s_channel_write(blocking)`
3. Modify a buffer after write returns and before the next write
4. Observe on logic analyzer whether the modification appears in output

If modification appears → zero-copy confirmed. If not → copy semantics,
and direct DMA descriptor approach should be reconsidered.

---

## Known Limitations

| Aspect | Single-Stepper | Multi-Stepper |
|--------|----------------|---------------|
| Max frequency | ~200 kHz | ~40 kHz (granularity) |
| Resolution | 2 ticks (125 ns) | 64 ticks (4 µs) |
| Steppers per I2S | 1 | Up to 16 |
| External hardware | None | Demux IC required |
| ForceStop latency | DMA stop + restart | Up to 1 extra pulse (race) |
| ForceStop position | Not guaranteed | Not guaranteed |

---

## Open Items

1. **8-bit mode**: Future option to halve frame duration (32 ticks/frame) for higher
   multi-stepper time resolution. Doubles resolution but halves max slots (16 → 8).
   The frame format (8-bit vs 16-bit stereo) should be an **init-time parameter**.
   The bit serialization mapping and fill algorithm must be parameterized
   accordingly. Not a design priority now.

2. **DIR/ENABLE sequencing**: The raw `setSlotBit()` / `getSlotBit()` service
   functions do not enforce timing constraints between DIR changes and STEP pulses.
   Instead of a sequencing layer, the engine maintains these variables for multi-stepper:

   ```c
   uint32_t bit_mask;       // Which bits are static (enable/direction)
   uint32_t bit_target;     // Target value for bits where bit_mask is 1
   uint32_t bit_actual[3];  // Actual value of those bits per block
   ```

   The engine checks the DMA-finished block and performs bulk updates when needed.
   This avoids direction changes during active step frames.

3. **`i2s_channel_write()` copy semantics**: Must be verified on hardware. If the
   IDF implementation copies the buffer, the direct DMA descriptor approach (bypassing
   the IDF driver) should be used instead.
