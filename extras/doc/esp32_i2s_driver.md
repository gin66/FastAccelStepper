# ESP32 I2S Output Driver

## Overview

This document describes the architecture, design, implementation, and testing plan for the
ESP32 stepper output driver based on the I2S peripheral with DMA. The driver targets
two distinct use-cases:

- **I2S_DIRECT mode**: One stepper using one I2S DATA_OUT GPIO. The bitstream provides
  32× higher time resolution within each frame, enabling step rates up to ~200kHz.
- **I2S_MUX mode**: One I2S DATA_OUT line feeds an external serial-to-parallel demux IC
  (e.g., a chain of 74HC595 shift registers). Up to 32 steppers share the same bitstream
  at frame-level resolution. DIR and ENABLE signals are controlled via bitmask
  configuration (supporting inverted/non-inverted polarity).

An existing RMT implementation already serves the ESP32 well. The I2S driver is an
**additive** option, not a replacement. Its main value is enabling serial-bus expansion
via a demux IC.

**Target**: ESP32 (original) on IDF ≥5.3. Will be rolled out to other ESP32 variants with
I2S once validated.

## Implementation Status

### ✅ I2S_DIRECT Mode — Complete & Working

- **Single-stepper mode** with bit-level resolution (2 ticks/bit)
- **DMA double buffering** with 2 blocks (ping-pong)
- **I2S configuration**: 250kHz sample rate, 16-bit stereo, 8MHz BCLK
- **Block duration**: 500µs (125 frames × 4µs per frame)
- **Pulse width**: 32 ticks (2µs = 16 bits)
- **ISR-driven fill**: `on_sent` callback fills buffers directly
- **`memset` clearing**: Buffer cleared before fill in callback
- **PC-based tests**: `test_21.cpp` with comprehensive test coverage
- **Hardware validated**: Working on ESP32

### ⬜ I2S_MUX Mode — Not Yet Implemented

- Multi-stepper slot management
- DIR/ENABLE bitmask support
- Frame-level fill algorithm
- Engine-level initialization API
- WS pin output for demux latch
- Hardware validation with 74HC595

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
| I2S_DIRECT | 2 ticks (bit-level) | ~200 kHz | 32× resolution within frame |
| I2S_MUX | 64 ticks (frame-level) | ~40 kHz | Limited by frequency granularity |

### Frequency Granularity in I2S_MUX Mode

In I2S_MUX mode, steps are frame-aligned. To emit a step, you need HIGH followed by LOW:

| Step Period | Frequency | Notes |
|-------------|-----------|-------|
| 2 frames (8µs) | 125 kHz | Max theoretical |
| 3 frames (12µs) | 83.3 kHz | 33% drop from max |
| 4 frames (16µs) | 62.5 kHz | |
| 5 frames (20µs) | 50 kHz | |
| 6 frames (24µs) | 41.7 kHz | Practical max |

The jump from 125 kHz to 83 kHz is 33%. For acceptable frequency granularity,
practical max frequency in I2S_MUX mode is ~40 kHz.

### Demux IC (I2S_MUX Mode)

A shift-register chain (e.g., two 74HC595 in series = 16 outputs) receives:
- DATA_OUT → SER input
- BCLK → SRCLK (shift clock)
- WS/LRCLK → RCLK (storage register clock / latch)

Each WS edge latches the last 32 bits from DATA_OUT to the parallel output. The
parallel outputs drive individual step, direction, and enable signals of stepper drivers.

### I2S_DIRECT Mode

In I2S_DIRECT mode, the entire 32-bit frame is used for one stepper's step signal.
The effective bit rate is 2 × 16 × frame_rate = 8 Mbit/s. Each bit represents 2 ticks
(125 ns).

A single bit HIGH (125 ns) is too short for a step pulse. A step pulse consists of
N consecutive 1-bits, where N = pulse_width_ticks / 2. For example, a 2 µs pulse =
16 consecutive 1-bits. Sub-frame edge placement provides positioning resolution of
2 ticks.

WS and BCLK should not be output on GPIOs in I2S_DIRECT mode (only the DATA_OUT
bitstream is needed). Set WS and BCLK to `I2S_GPIO_UNUSED` to avoid consuming GPIOs.

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
the same PLL as the CPU clock.

### Step Counting

Step count is decremented at fill time when the pulse HIGH phase is committed to the
DMA buffer. This means steps are counted before they are physically output. ForceStop
may result in position error of up to `(I2S_BLOCK_COUNT - 1) × max_steps_per_block`
steps. This is consistent with the general contract that `forceStop()` does not
guarantee maintaining exact position.

### Thread Safety

`read_idx` and `next_write_idx` are single bytes (uint8_t), which are atomically
safe to read and write on ESP32. The contract is:

- `read_idx` is updated **only** by the driver (fill routine in I2S callback context)
- `next_write_idx` is updated **only** by the ramp generator (StepperTask context)

No additional synchronization (mutex/spinlock) is needed.

---

## Architecture

### Driver Types

The `FasDriver` enum provides two I2S modes:

```cpp
enum class FasDriver : uint8_t {
  MCPWM_PCNT = 0,
  RMT = 1,
#if defined(SUPPORT_ESP32_I2S)
  I2S_DIRECT = 2,   // One stepper per I2S peripheral
  I2S_MUX = 3,      // Multiple steppers via demux IC
#endif
  DONT_CARE = 255
};
```

Macros for user code:
```cpp
#define DRIVER_I2S_DIRECT FasDriver::I2S_DIRECT
#define DRIVER_I2S_MUX    FasDriver::I2S_MUX
```

### I2S_DIRECT Mode — Stepper Connection

No engine initialization needed. Each `stepperConnectToPin()` call creates its
own `I2sManager` with a dedicated I2S peripheral:

```cpp
FastAccelStepper* s = engine.stepperConnectToPin(step_pin, DRIVER_I2S_DIRECT);
```

Internally, `tryAllocateQueue()` calls `I2sManager::create(data_pin, I2S_GPIO_UNUSED, I2S_GPIO_UNUSED)`,
allocates a `StepperQueue`, and stores the manager pointer in `q->i2s_mgr`.

### I2S_MUX Mode — Engine Initialization

I2S_MUX mode requires explicit engine initialization **before** connecting steppers.
This is an engine member function that creates the shared `I2sManager` and stores it
as a static member of `StepperQueue`:

```cpp
// Engine member function
bool FastAccelStepperEngine::initI2sMux(
    uint8_t data_pin, uint8_t bclk_pin, uint8_t ws_pin,
    uint32_t dir_bitmask, uint32_t dir_inverted_bitmask,
    uint32_t enable_bitmask, uint32_t enable_inverted_bitmask);
```

**Bitmask parameters**: Each bit position (0–31) in the 32-bit frame corresponds to
a slot on the shift register output. The bitmasks define which slots carry DIR or
ENABLE signals and their polarity:

- `dir_bitmask`: Bits that are direction outputs (active-high)
- `dir_inverted_bitmask`: Bits that are direction outputs (active-low / inverted)
- `enable_bitmask`: Bits that are enable outputs (active-high)
- `enable_inverted_bitmask`: Bits that are enable outputs (active-low / inverted)

Bits not set in any bitmask are available as STEP outputs.

**Example**: Two steppers with step+dir+enable on 74HC595:
```cpp
// Slot layout: [STEP0, DIR0, EN0, STEP1, DIR1, EN1, ...]
// Slots 0,3 = STEP; Slots 1,4 = DIR (active-high); Slots 2,5 = ENABLE (active-low)
engine.initI2sMux(
    DATA_PIN, BCLK_PIN, WS_PIN,
    0b00010010,   // dir_bitmask: slots 1,4
    0b00000000,   // dir_inverted_bitmask: none
    0b00000000,   // enable_bitmask: none
    0b00100100    // enable_inverted_bitmask: slots 2,5 (active-low)
);
```

The function:
1. Creates a shared `I2sManager` with WS pin enabled
2. Stores it in `StepperQueue::_i2s_mux_manager` (static)
3. Stores the bitmasks for DIR/ENABLE signal handling
4. Sets `StepperQueue::_i2s_mux_initialized = true`

### I2S_MUX Mode — Stepper Connection

After engine initialization, connect steppers using slot indices with `PIN_I2S_FLAG`:

```cpp
uint8_t i2s_pin = slot_index | PIN_I2S_FLAG;  // PIN_I2S_FLAG = 0x40
FastAccelStepper* s = engine.stepperConnectToPin(i2s_pin, DRIVER_I2S_MUX);
```

`tryAllocateQueue()` validates that the slot is not a DIR/ENABLE slot, not already
allocated, and assigns the shared `I2sManager` pointer to `q->i2s_mgr`.

### Constants (i2s_constants.h)

All constants are derived from `I2S_BLOCK_DURATION_US`:

```c
#define I2S_SAMPLE_RATE_HZ 250000UL
#define I2S_TICKS_PER_FRAME 64
#define I2S_BITS_PER_FRAME 32
#define I2S_BYTES_PER_FRAME 4

#define I2S_BLOCK_DURATION_US 500
#define I2S_BLOCK_COUNT 2
#define I2S_FRAMES_PER_BLOCK \
  (I2S_BLOCK_DURATION_US * I2S_SAMPLE_RATE_HZ / 1000000UL)
#define I2S_BLOCK_TICKS (I2S_FRAMES_PER_BLOCK * I2S_TICKS_PER_FRAME)
#define I2S_BYTES_PER_BLOCK (I2S_FRAMES_PER_BLOCK * I2S_BYTES_PER_FRAME)

#define I2S_DMA_DESC_NUM 2
#define I2S_DMA_FRAME_NUM I2S_FRAMES_PER_BLOCK

#define I2S_DIRECT_MIN_SPEED_TICKS 80
#define I2S_MUX_MIN_SPEED_TICKS 400
#define I2S_DEFAULT_PULSE_WIDTH_TICKS 32
```

### Double Buffering (Ping-Pong)

The I2S driver uses double buffering with 2 DMA descriptors of 500 µs each:

```
Block 0    Block 1    Block 0    Block 1
[500µs]    [500µs]    [500µs]    [500µs]
   ↓          ↓          ↓          ↓
DMA continuously streams blocks in rotation
```

- **Block N**: Currently being transmitted by DMA (busy)
- **Block (N+1)%2**: Available for filling

### DMA Architecture: ISR-Driven

The `on_sent` callback fires when DMA finishes transmitting a block. The callback:

1. Receives a pointer to the just-transmitted buffer (now available for reuse)
2. Clears the buffer with `memset`
3. Iterates all queues, calling the appropriate fill function for each I2S stepper
   whose `i2s_mgr` matches this manager

```
DMA:  ████ Block 0 ████  ████ Block 1 ████  ████ Block 0 ████
                       ↑                  ↑
ISR:                   │ on_sent          │ on_sent
                       │ memset buf       │ memset buf
                       │ fill(buf)        │ fill(buf)
                       ↓                  ↓

StepperTask:  ─── manageSteppers() ──── manageSteppers() ────
                  (adds cmds to queue)   (adds cmds to queue)
```

**Timing budget**: Fill must complete within 500 µs. At 240 MHz ESP32 that is
120k CPU cycles — sufficient for the fill algorithm.

**Callback fill dispatch**: The callback selects the appropriate fill function
based on mode. I2S_DIRECT queues call `i2s_fill_buffer()` (bit-level).
I2S_MUX queues call a separate optimized `i2s_fill_buffer_mux()` (frame-level).
The selection is made in `StepperQueue::fill_i2s_buffer()`.

### I2sManager

```cpp
class I2sManager {
 public:
  static I2sManager* create(gpio_num_t data_pin, gpio_num_t bclk_pin,
                            gpio_num_t ws_pin);
  void handleTxDone(uint8_t* buf);

 private:
  i2s_chan_handle_t _chan;
};
```

- `create()`: Configures I2S channel, registers `on_sent` callback, enables channel
- `handleTxDone()`: ISR callback — clears buffer, iterates queues, calls fill
- For I2S_DIRECT: `ws_pin = I2S_GPIO_UNUSED`, `bclk_pin = I2S_GPIO_UNUSED`
- For I2S_MUX: all three pins connected to 74HC595

**Storage**: For I2S_MUX, the shared manager is stored in
`StepperQueue::_i2s_mux_manager` (static member). For I2S_DIRECT, each queue has
its own manager in `q->i2s_mgr`.

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

### I2S_MUX Mode: Slot-to-Bit Mapping

Each slot (0..31) maps to a fixed bit position in the frame:

```c
byte_in_frame = slot / 8;          // 0..3
bit_in_byte   = 7 - (slot % 8);    // MSB first
byte_in_buf   = frame_index * 4 + byte_in_frame;
bit_mask      = 1 << bit_in_byte;
```

**Worked example**: Slot 5 (6th output bit)
- `byte_in_frame = 5 / 8 = 0` → L_hi
- `bit_in_byte = 7 - (5 % 8) = 2`
- `bit_mask = 0x04`
- To set: `buf[frame_index * 4 + 0] |= 0x04;`
- To clear: `buf[frame_index * 4 + 0] &= ~0x04;`

### I2S_DIRECT Mode: Tick-to-Bit Mapping

In I2S_DIRECT mode, the bit index represents a temporal position within the frame
(0..31 → first bit out to last bit out):

```c
bit_time_index = (tick_pos % I2S_TICKS_PER_FRAME) / 2;  // 0..31
byte_in_frame  = bit_time_index / 8;
bit_in_byte    = 7 - (bit_time_index % 8);
byte_in_buf    = frame_index * 4 + byte_in_frame;
bit_mask       = 1 << bit_in_byte;
```

Note: I2S byte order is little-endian but bits are MSB-first, so byte index is
XORed with 1 in the implementation (`byte_index = (bit_pos >> 3) ^ 1`).

---

## Buffer Fill Algorithm

### Fill State

Each stepper maintains its own fill state:

```c
struct i2s_fill_state {
  uint16_t remaining_low_ticks;
  uint8_t  remaining_high_ticks;
  uint8_t  off_ticks;             // sub-bit fractional tick remainder
};
```

The fill state carries over between blocks. When a pulse or pause spans a block
boundary, the remaining ticks are preserved and consumed in the next block.

### I2S_DIRECT Fill: i2s_fill_buffer()

Called once per stepper per block. Operates at bit-level granularity (2 ticks/bit).

**Algorithm**:
```
i2s_fill_buffer(queue, buf, state):
  bit_pos = 0

  loop:
    1. Emit remaining HIGH bits:
       While remaining_high_ticks > 0:
         Set bits in buf at bit_pos (byte-aligned writes: buf[byte] = 0xff >> offset)
         Advance bit_pos, decrement remaining_high_ticks
         If block full: save state, return true

    2. Skip remaining LOW ticks:
       If remaining_low_ticks > 0:
         Advance bit_pos by remaining_low_ticks / ticks_per_bit
         Track sub-bit remainder in off_ticks
         If block full: save state, return true

    3. Read next queue entry:
       If queue empty: save state, return false
       Handle toggle_dir
       Set remaining_low_ticks = entry.ticks (+ off_ticks carry)
       If entry has steps:
         remaining_high_ticks = PULSE_WIDTH_TICKS
         remaining_low_ticks -= PULSE_WIDTH_TICKS
         Decrement entry.steps
       If entry.steps == 0: advance read_idx
```

Returns `true` if block is full, `false` if queue is empty.

### I2S_MUX Fill: i2s_fill_buffer_mux() — To Be Implemented

Called once per stepper per block. Operates at frame-level granularity (64 ticks/frame).
Each stepper owns one bit slot in the 32-bit frame. This is a separate, optimized
fill function — not a mode branch inside `i2s_fill_buffer()`.

**Key differences from I2S_DIRECT**:
- Writes single bits per frame instead of contiguous byte regions
- Resolution is 64 ticks (1 frame) instead of 2 ticks (1 bit)
- Uses OR operations (`buf[offset] |= mask`) to set bits without disturbing other steppers
- DIR/ENABLE bits are pre-filled by `init_mux_buffer()` before step fill runs
- `max_speed_in_ticks` is set to `I2S_MUX_MIN_SPEED_TICKS` (400)

**Algorithm**:
```
i2s_fill_buffer_mux(queue, buf, state, slot, byte_offset, bit_mask):
  frame_pos = 0

  loop:
    1. Emit remaining HIGH frames:
       While remaining_high_frames > 0:
         buf[frame_pos * 4 + byte_offset] |= bit_mask
         frame_pos++, remaining_high_frames--
         If block full: save state, return true

    2. Skip remaining LOW frames:
       If remaining_low_frames > 0:
         Advance frame_pos
         If block full: save state, return true

    3. Read next queue entry (same as I2S_DIRECT)
```

Note: DIR/ENABLE bits are already present in the buffer from `init_mux_buffer()`.
The MUX fill function only writes STEP bits.

### Buffer Initialization & Clearing

The buffer initialization differs between modes:

- **I2S_DIRECT**: `memset(buf, 0, I2S_BYTES_PER_BLOCK)` — simple zero fill,
  then the single stepper writes its step pulses.
- **I2S_MUX**: A per-I2S-module initialization function replaces `memset`.
  It writes the current DIR/ENABLE state into every frame of the buffer,
  propagating the static signal levels across all 125 frames. Step bits
  are left clear. The MUX fill functions then only OR in step pulses.

This approach avoids a separate DIR/ENABLE pass after filling — the buffer
is already correct for DIR/ENABLE before any step fill runs.

### Callback Fill Dispatch

```cpp
void I2sManager::handleTxDone(uint8_t* buf) {
  if (is_mux) {
    // Pre-fill buffer with DIR/ENABLE bits for all frames,
    // step bits cleared — replaces memset(0)
    init_mux_buffer(buf);
  } else {
    memset(buf, 0, I2S_BYTES_PER_BLOCK);
  }

  for (uint8_t i = 0; i < NUM_QUEUES; i++) {
    StepperQueue* q = fas_queue[i];
    if (q && q->use_i2s && q->i2s_mgr == this) {
      q->fill_i2s_buffer(buf);  // dispatches to direct or mux fill
    }
  }
}
```

`StepperQueue::fill_i2s_buffer()` selects the appropriate optimized fill function
based on whether this is an I2S_DIRECT or I2S_MUX queue.

`init_mux_buffer()` builds a 4-byte frame template from the current DIR/ENABLE
state of all connected MUX steppers and replicates it across all frames in the
block. This is a per-I2S-module operation — it reads the bitmasks and each
stepper's current direction/enable state to compute the template once, then
fills the buffer with it (e.g., via `memset`-style 4-byte pattern fill).

---

## DIR/ENABLE Signal Handling (I2S_MUX Mode)

In I2S_MUX mode, DIR and ENABLE signals are carried as bits in the I2S frame,
controlled by the bitmasks provided at `initI2sMux()` time.

### Bitmask Configuration

Four bitmasks define signal routing and polarity:

| Bitmask | Meaning |
|---------|---------|
| `dir_bitmask` | Slots that carry direction signal (active-high) |
| `dir_inverted_bitmask` | Slots that carry direction signal (active-low) |
| `enable_bitmask` | Slots that carry enable signal (active-high) |
| `enable_inverted_bitmask` | Slots that carry enable signal (active-low) |

### DIR Handling

DIR bits are propagated by `init_mux_buffer()` before any step fill runs.
The function reads each stepper's current direction state and sets/clears
the corresponding DIR slot bits across all frames in the block. Direction
changes take effect at the next block boundary (worst case 500 µs latency).

### ENABLE Handling

ENABLE bits are propagated by `init_mux_buffer()` alongside DIR bits.
The function reads each stepper's auto-enable state and applies the
correct polarity (inverted bitmask bits output the logical complement).

### Association

Each MUX stepper's STEP slot implicitly identifies which DIR/ENABLE slots
belong to it. The mapping is defined by the user's physical wiring and
bitmask configuration. The library does not enforce a specific slot ordering.

---

## ForceStop

### Current Implementation (I2S_DIRECT)

```cpp
void StepperQueue::forceStop_i2s() {
  _isRunning = false;
  _fill_state = {};
}
```

When `_isRunning` is false, `fill_i2s_buffer()` returns immediately. The buffer
is cleared by `memset` at the next callback. Pulses already in the DMA buffer
will still be output (up to 500 µs of stale data).

---

## Pulse Width Constraints

### I2S_DIRECT Mode

Pulse width = N consecutive 1-bits. Currently fixed at 32 ticks (2 µs = 16 bits).

Minimum practical pulse width depends on the stepper driver (typically ≥1 µs
= 8 bits = 16 ticks).

### I2S_MUX Mode

Pulse width is quantized to frame multiples (64 ticks). Minimum = 1 frame (4 µs).

| Pulse Width | Min Period | Max Freq |
|-------------|-----------|----------|
| 1 frame (4 µs) | 2 frames (8 µs) | 125 kHz theoretical |
| 2 frames (8 µs) | 4 frames (16 µs) | 62.5 kHz |

Practical max with acceptable granularity remains ~40 kHz.

### Minimum cmd.ticks

| Mode | Min ticks | Constant | Reason |
|------|-----------|----------|--------|
| I2S_DIRECT | 80 (5µs) | `I2S_DIRECT_MIN_SPEED_TICKS` | 200 kHz max, PULSE_WIDTH=32 ticks |
| I2S_MUX | 400 (25µs) | `I2S_MUX_MIN_SPEED_TICKS` | 40 kHz max, frame-level granularity |

Enforced by `max_speed_in_ticks` per queue.

---

## StepperQueue Union Layout

The I2S fields share a union with RMT fields in `StepperQueue`:

```cpp
union {
#ifdef SUPPORT_ESP32_RMT
  struct {
    RMT_CHANNEL_T channel;
    bool _rmtStopped;
    bool lastChunkContainsSteps;
    rmt_encoder_handle_t _tx_encoder;
    bool _channel_enabled;
  };
#endif
#ifdef SUPPORT_ESP32_I2S
  struct {
    struct i2s_fill_state _fill_state;
    I2sManager* i2s_mgr;
  };
#endif
};
```

**Important**: Since both `SUPPORT_ESP32_RMT` and `SUPPORT_ESP32_I2S` are defined,
`_initVars()` must NOT write to RMT-specific union members unconditionally.
`_rmtStopped = true` is set only in the RMT-specific `init_rmt()` functions,
not in the shared `_initVars()`.

---

## Memory Budget

| Component | I2S_DIRECT | I2S_MUX (16 steppers) |
|-----------|-----------|---------------------|
| DMA buffers (2 × 500 B) | 1000 bytes | 1000 bytes (shared) |
| DMA descriptors (2×) | ~32 bytes | ~32 bytes |
| Per-stepper fill state | 4 bytes × 1 | 4 bytes × 16 |
| I2sManager | ~16 bytes | ~16 bytes (shared) |
| **Total** | **~1050 bytes** | **~1110 bytes** |

---

## File Structure

### I2S Files

```
src/pd_esp32/
  i2s_constants.h           — I2S timing constants (derived from BLOCK_DURATION_US)
  i2s_manager.h             — I2sManager class declaration
  i2s_manager.cpp           — I2sManager implementation (DMA, callback, fill dispatch)
  i2s_fill.h                — i2s_fill_state struct, fill function declarations
  i2s_fill.cpp              — i2s_fill_buffer() (I2S_DIRECT) implementation
  StepperISR_esp32_i2s.cpp  — StepperQueue I2S methods (init, start, stop, fill dispatch)
```

### Modified Files

```
src/fas_arch/common.h               — I2S_DIRECT/I2S_MUX in FasDriver enum
src/fas_arch/common_esp32.h         — QUEUES_I2S_MUX, QUEUES_I2S_DIRECT, PIN_I2S_FLAG
src/fas_arch/common_esp32_idf5.h    — SUPPORT_ESP32_I2S conditional on SOC_I2S_NUM
src/pd_esp32/esp32_queue.h          — I2S union members, static MUX manager
src/pd_esp32/esp32_queue.cpp        — I2S dispatch, tryAllocateQueue(), initI2sMux()
src/FastAccelStepperEngine.h        — initI2sMux() declaration
```

### Test Files

```
extras/tests/pc_based/
  test_21.cpp               — PC-based tests for I2S_DIRECT fill function
```

---

## Testing Strategy

### PC-Based Tests (test_21.cpp) ✅ I2S_DIRECT Complete

The fill function is testable on PC without I2S hardware. `test_21.cpp` provides:

- DMA infrastructure simulation (callbacks, block rotation, bit stream analysis)
- Fill function tests with queue commands (single step, multi step,
  pause, block boundary, carry-over, max values)
- Comprehensive edge case coverage

### Future Test Needs — I2S_MUX

- MUX fill: frame-level bit placement for single stepper
- Multi-stepper shared buffer: different slots, no interference
- DIR/ENABLE bitmask: correct polarity, static across block
- Slot allocation: validation of PIN_I2S_FLAG, reject DIR/ENABLE slots
- Hardware validation: logic analyzer on DATA_OUT/BCLK/WS
- End-to-end: 74HC595 chain + PCNT step count verification

---

## I2S_MUX Implementation Plan

### Phase 1: Engine Initialization

1. Add `initI2sMux()` to `FastAccelStepperEngine`
2. Add static members to `StepperQueue`: `_i2s_mux_manager`, DIR/ENABLE bitmasks
3. `initI2sMux()` creates shared `I2sManager` with WS pin, stores bitmasks,
   sets `_i2s_mux_initialized = true`

### Phase 2: Queue Allocation

1. Extend `tryAllocateQueue()` for `DRIVER_I2S_MUX`:
   - Validate slot not in DIR/ENABLE bitmasks
   - Validate slot not already allocated (`_i2s_mux_allocated_bitmask`)
   - Set `q->i2s_mgr = StepperQueue::_i2s_mux_manager`
   - Set `q->_i2s_mux_slot` to slot index
   - Set `max_speed_in_ticks = I2S_MUX_MIN_SPEED_TICKS`
2. Store slot's precomputed `byte_offset` and `bit_mask` for fast fill

### Phase 3: MUX Fill Algorithm

1. Implement `i2s_fill_buffer_mux()` as separate optimized function
2. Frame-level iteration: one bit per frame per stepper
3. OR bits into shared buffer (`buf[offset] |= mask`)
4. Set DIR/ENABLE bits statically for entire block
5. `StepperQueue::fill_i2s_buffer()` dispatches to correct fill function

### Phase 4: Testing & Validation

1. PC-based tests for MUX fill in `test_21.cpp`
2. Hardware validation with logic analyzer
3. End-to-end test with 74HC595 + PCNT

---

## Known Limitations

| Aspect | I2S_DIRECT | I2S_MUX |
|--------|-----------|---------|
| Max frequency | ~200 kHz | ~40 kHz (granularity) |
| Resolution | 2 ticks (125 ns) | 64 ticks (4 µs) |
| Steppers per I2S | 1 | Up to 32 |
| External hardware | None | Demux IC required |
| Pulse width | Fixed 32 ticks | Fixed 64 ticks (1 frame) |
| WS pin | Unused | Required for demux |
| Min speed ticks | 80 | 400 |
