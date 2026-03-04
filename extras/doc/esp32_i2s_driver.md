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

## Executive Summary

The ESP32 I2S driver provides a high-resolution stepper output using the I2S peripheral with DMA. The current implementation supports **single-stepper mode** with comprehensive PC-based testing. **Multi-stepper mode** (for serial demux expansion) is designed but not yet implemented.

### Key Achievements
- ✅ **Single-stepper mode** operational with bit-level resolution (2 ticks/bit)
- ✅ **28 comprehensive tests** in `test_21.cpp` covering all edge cases
- ✅ **DMA integration** with double buffering (ping-pong)
- ✅ **Correct queue management** with block boundary handling
- ✅ **Constants aligned** with design document specifications

### Current Limitations
- ⚠️ **Multi-stepper mode not implemented** (single-stepper only)
- ⚠️ **Basic forceStop** (doesn't stop DMA or clear buffer)
- ⚠️ **Fixed pulse width** (64 ticks, not configurable)

### Next Steps
1. **Driver architecture implementation**: Extend `FasDriver` enum with I2S driver types
2. **Multi-stepper foundation**: Per-stepper bit tracking, slot allocation
3. **Enhanced forceStop**: DMA control, buffer clearing
4. **Hardware validation**: PCNT tests, logic analyzer verification
5. **Feature completion**: Configurable pulse width, WS pin support

## Current Implementation Status (March 2025)

### ✅ Implemented Features
- **Single-stepper mode** with bit-level resolution (2 ticks per bit)
- **DMA double buffering** with 2 blocks (ping-pong)
- **I2S configuration**: 250kHz sample rate, 16-bit stereo, 8MHz BCLK
- **Block duration**: 500µs (125 frames × 4µs per frame)
- **Pulse width**: 64 ticks (4µs = 32 bits = one full frame)
- **PC-based tests**: `test_21.cpp` with 28 comprehensive tests
- **Fill algorithm**: Handles block boundaries, carry-over ticks, and queue management
- **DMA callback integration**: ISR-driven buffer rotation

### ⚠️ Partially Implemented
- **Multi-stepper mode**: Not yet implemented (single-stepper only)
- **Engine API**: Basic initialization exists, needs expansion for multi-stepper
- **ForceStop**: Basic implementation, needs DMA stop/restart for single-stepper

### ❌ Not Implemented
- **Multi-stepper slot management**: Allocation, bit-slot mapping
- **DIR/ENABLE bit service**: Functions for multi-stepper signal control
- **Configurable pulse width**: Currently fixed at 64 ticks
- **WS pin support**: Currently hardcoded to `I2S_GPIO_UNUSED`
- **Hardware validation**: PCNT, logic analyzer, 74HC595 tests

### Recent Fixes
- **Test failures**: Fixed incorrect use of `fillAndDetectPulses()` return value
- **Constants**: Updated to match design doc (`I2S_BLOCK_COUNT=2`, derived from `I2S_BLOCK_DURATION_US`)
- **Fill algorithm**: Correct block boundary handling and queue pointer advancement

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
**before** any steppers are connected. The planned architecture extends the existing
`FasDriver` enum to support I2S modes:

```cpp
// Planned FasDriver enum extension for I2S support
// COMPILE-TIME validation with SOC_I2S_NUM
enum class FasDriver : uint8_t { 
    MCPWM_PCNT = 0, 
    RMT = 1, 
#if SUPPORT_I2S
    RMT_I2S_DIRECT = 2,      // Single-stepper I2S mode
#if SOC_I2S_NUM >= 1
    RMT_I2S0_MUX = 3,        // Multi-stepper I2S mode using I2S0
#endif
#if SOC_I2S_NUM >= 2
    RMT_I2S1_MUX = 4,        // Multi-stepper I2S mode using I2S1
#endif
#if SOC_I2S_NUM >= 3
    RMT_I2S2_MUX = 5,        // Multi-stepper I2S mode using I2S2 (ESP32-P4)
#endif
#endif // SUPPORT_I2S
    DONT_CARE = 255 
};

// I2S support conditional on SOC_I2S_NUM
#if SOC_I2S_NUM >= 1
#define SUPPORT_I2S 1
#endif

// Single-stepper mode: one I2S peripheral = one stepper
// Uses RMT_I2S_DIRECT driver type
engine.initI2sSingleStepper(data_pin, bclk_pin);

// Multi-stepper mode: one I2S peripheral = many steppers via demux
// Uses RMT_I2S0_MUX, RMT_I2S1_MUX, or RMT_I2S2_MUX driver type
engine.initI2sMultiStepper(data_pin, bclk_pin, ws_pin, signals_per_stepper);
```

#### Pin Flag for I2S Control

Similar to `PIN_EXTERNAL_FLAG` (0x80) used for externally controlled pins,
I2S-controlled pins will use `PIN_I2S_FLAG` (0x40):

```cpp
#define PIN_I2S_FLAG 0x40  // Bitmask for I2S-controlled pins

// Usage in multi-stepper mode:
uint8_t i2s_pin = physical_pin_number | PIN_I2S_FLAG;
```

#### Driver Selection Flow

1. **Single-stepper mode**:
   - User calls `connectToStepperPin(pin, RMT_I2S_DIRECT)`
   - System automatically allocates available I2S module (based on `SOC_I2S_NUM`) and initializes DMA
   - No engine initialization required

2. **Multi-stepper mode**:
   - User must first call `engine.initI2sMultiStepper()` for the desired I2S module
   - Then call `connectToStepperPin(pin | PIN_I2S_FLAG, RMT_I2S0_MUX)`, `RMT_I2S1_MUX`, or `RMT_I2S2_MUX`
   - Only I2S modules defined by `SOC_I2S_NUM` are available
   - Engine manages shared I2S peripheral and demux timing

**Note**: The driver uses **compile-time validation** with `SOC_I2S_NUM`:
- `FasDriver` enum only includes I2S modules that exist on the target
- Code trying to use non-existent I2S modules won't compile
- Clean API: users only see available I2S driver options

The number of I2S modules varies across ESP32 variants:

| ESP32 Variant | I2S Modules | Notes |
|---------------|-------------|-------|
| ESP32         | 2 (I2S0, I2S1) | Original ESP32 |
| ESP32-S2      | 1 (I2S0) | Single I2S module |
| ESP32-S3      | 2 (I2S0, I2S1) | Dual I2S modules |
| ESP32-C2      | 1 (I2S0) | Single I2S module |
| ESP32-C3      | 1 (I2S0) | Single I2S module |
| ESP32-C5      | 1 (I2S0) | Single I2S module |
| ESP32-C6      | 1 (I2S0) | Single I2S module |
| ESP32-H2      | 1 (I2S0) | Single I2S module |
| ESP32-P4      | 3 (I2S0, I2S1, I2S2) | Triple I2S modules |

The driver architecture uses `SOC_I2S_NUM` (defined in ESP-IDF) for **compile-time
validation** of I2S module availability:

1. **Conditional I2S Support**: If `SOC_I2S_NUM >= 1`, `SUPPORT_I2S` is defined
2. **Conditional Enum Values**: I2S driver values only in `FasDriver` if `SUPPORT_I2S`
3. **Module-specific Values**: `RMT_I2S1_MUX` only if `SOC_I2S_NUM >= 2`, etc.

**API Design Principle**: Compile-time validation > runtime validation
- Code that tries to use `RMT_I2S1_MUX` on ESP32-S2 (1 I2S module) won't compile
- ESP32 variants without I2S (`SOC_I2S_NUM < 1`) compile without I2S support
- Cleaner API: users only see available options
- No runtime error handling needed for invalid I2S module requests

### Runtime I2S Module Handling

The driver needs to work with any of the available I2S modules (I2S0, I2S1, I2S2)
dynamically. The implementation approach:

1. **I2S Port Mapping**: Convert `FasDriver` to `i2s_port_t` (ESP-IDF I2S port identifier)
   ```cpp
    static i2s_port_t fasDriverToI2sPort(FasDriver driver) {
        switch (driver) {
    #if SUPPORT_I2S
            case FasDriver::RMT_I2S_DIRECT:
            case FasDriver::RMT_I2S0_MUX: return I2S_NUM_0;
    #if SOC_I2S_NUM >= 2
            case FasDriver::RMT_I2S1_MUX: return I2S_NUM_1;
    #endif
    #if SOC_I2S_NUM >= 3
            case FasDriver::RMT_I2S2_MUX: return I2S_NUM_2;
    #endif
    #endif // SUPPORT_I2S
            default: return I2S_NUM_MAX; // Invalid (non-I2S driver)
        }
    }
   ```

2. **Per-I2S State Management**: Each I2S module needs its own state:
   ```cpp
   struct I2sModuleState {
       i2s_port_t port;
       bool initialized;
       bool in_use;
       uint8_t stepper_count;
       // DMA buffers, ISR state, etc.
   };
   
   // Array of all possible I2S modules
   I2sModuleState i2s_modules[SOC_I2S_NUM];
   ```

3. **Dynamic Initialization**: Initialize I2S peripheral when first stepper connects:
   ```cpp
   bool initializeI2sModule(i2s_port_t port) {
       if (port >= SOC_I2S_NUM) return false;
       if (i2s_modules[port].initialized) return true;
       
       // ESP-IDF I2S configuration
       i2s_config_t i2s_config = { ... };
       i2s_pin_config_t pin_config = { ... };
       
       i2s_driver_install(port, &i2s_config, 0, NULL);
       i2s_set_pin(port, &pin_config);
       
       i2s_modules[port].initialized = true;
       return true;
   }
   ```

4. **Resource Management**: Track which I2S modules are in use:
   - Single-stepper mode (`RMT_I2S_DIRECT`): Auto-select available I2S module
   - Multi-stepper mode (`RMT_I2Sx_MUX`): Use specified I2S module
   - Prevent double-initialization of same I2S module
   - Handle module exhaustion (all I2S modules in use)

### Constants (Current Implementation)

All constants are derived from `I2S_BLOCK_DURATION_US` as per design:

```c
// I2S timing: 16-bit stereo mode
// Sample rate: 250kHz
// Bits per frame: 16-bit L + 16-bit R = 32 bits
// BCLK = 250kHz × 32 bits = 8MHz
// Frame duration = 32 bits / 8MHz = 4µs
// At 16MHz stepper reference: 4µs × 16 = 64 ticks per frame
#define I2S_SAMPLE_RATE_HZ 250000UL
#define I2S_TICKS_PER_FRAME 64
#define I2S_BITS_PER_FRAME 32
#define I2S_BYTES_PER_FRAME 4

// All constants derived from I2S_BLOCK_DURATION_US (per design doc)
#define I2S_BLOCK_DURATION_US 500
#define I2S_BLOCK_COUNT 2
#define I2S_FRAMES_PER_BLOCK \
  (I2S_BLOCK_DURATION_US * I2S_SAMPLE_RATE_HZ / 1000000UL)
#define I2S_BLOCK_TICKS (I2S_FRAMES_PER_BLOCK * I2S_TICKS_PER_FRAME)
#define I2S_BYTES_PER_BLOCK (I2S_FRAMES_PER_BLOCK * I2S_BYTES_PER_FRAME)

// Total buffer (all 2 blocks combined)
#define I2S_TOTAL_FRAMES (I2S_BLOCK_COUNT * I2S_FRAMES_PER_BLOCK)
#define I2S_TOTAL_BYTES (I2S_BLOCK_COUNT * I2S_BYTES_PER_BLOCK)

// Max frequency enforced by addQueueEntry(), not by I2S driver
#define MAX_STEP_FREQ_HZ 200000UL

// Max pulses per block (for tracking array size)
// 500µs × 200kHz = 100, +1 for safety
#define I2S_MAX_PULSES_PER_BLOCK \
  ((I2S_BLOCK_DURATION_US * MAX_STEP_FREQ_HZ / 1000000UL) + 1)

// Min step period: 2 frames = 8µs = 128 ticks (for multi-stepper mode)
#define I2S_MIN_SPEED_TICKS 128

// Default pulse width: 32 bits = 1 frame = 4µs = 64 ticks
#define I2S_DEFAULT_PULSE_WIDTH_TICKS 64

// DMA: minimum for continuous streaming (per design doc)
#define I2S_DMA_DESC_NUM 2
#define I2S_DMA_FRAME_NUM I2S_FRAMES_PER_BLOCK
```

**Note**: Current implementation uses `I2S_BLOCK_COUNT = 2` (ping-pong) instead of 3 (triple buffer) as originally designed. This simplifies the implementation while maintaining functionality.

Maximum frequency (200 kHz) is enforced by `addQueueEntry()`, not by the I2S driver.
The pulse tracking array is sized to this guaranteed maximum.

### Double Buffering (Current Implementation)

The I2S driver uses double buffering (ping-pong) with 2 blocks of 500 µs each:

```
Block 0    Block 1    Block 0    Block 1
[500µs]    [500µs]    [500µs]    [500µs]
   ↓          ↓          ↓          ↓
DMA continuously streams blocks in rotation
```

**Design vs Implementation**: The original design specified triple buffering (3 blocks), but the current implementation uses double buffering (2 blocks). This simplifies the implementation while maintaining functionality:
- **Block 0**: Currently being transmitted by DMA (busy)
- **Block 1**: Available for filling
- When DMA finishes Block 0, it becomes available for filling while Block 1 is transmitted

### DMA Architecture: ISR-Driven (Current Implementation)

The current implementation uses a simplified ISR-driven architecture:

**DMA Callback (IRAM_ATTR)**:
1. Update `busy_block = (busy_block + 1) % 2`
2. Call `fill_i2s_buffer()` for all steppers directly from ISR
3. The fill function writes to the available buffer (not the busy one)

**Simplification**: Since `fill_i2s_buffer()` only writes to memory (no mutexes or blocking calls), it can run safely in ISR context. This avoids the complexity of a separate FreeRTOS task.

**Limitation**: The fill computation must complete within the DMA block period (500µs). At 240 MHz ESP32, this is 120k CPU cycles — sufficient for the fill algorithm.

**Future Enhancement**: For multi-stepper mode with many steppers, a two-stage architecture (ISR + task) may be needed to avoid ISR overrun.

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

### DMA ↔ Fill ↔ StepperTask Sequence (Current Implementation)

```
Time →
         Block 0 plays    Block 1 plays    Block 0 plays    Block 1 plays
DMA:     ████████████████  ████████████████  ████████████████  ████████████████
                        ↑                 ↑                 ↑
ISR:                    │ on_sent         │ on_sent         │ on_sent
                        │ busy=1          │ busy=0          │ busy=1
                        │ fill Block 0    │ fill Block 1    │ fill Block 0
                        ↓                 ↓                 ↓
Fill:              ┌──────────┐      ┌──────────┐      ┌──────────┐
                   │B1 = busy │      │B0 = busy │      │B1 = busy │
                   │fill B0   │      │fill B1   │      │fill B0   │
                   └──────────┘      └──────────┘      └──────────┘

StepperTask:  ─── manageSteppers() ──── manageSteppers() ────
                  (adds cmds to queue)   (adds cmds to queue)
```

When ISR fires with busy = N:
- Block N is currently being transmitted by DMA (do not touch)
- Block (N+1)%2 is free and available for filling

**Timing budget**: The fill computation must complete within one block period
(500 µs). At 240 MHz ESP32 that is 120k CPU cycles — sufficient for single-stepper
fill algorithm.

**Queue**: The StepperTask adds commands to the queue, and the fill function
advances `read_idx` as it consumes commands. The fill function correctly handles
block boundaries and carry-over ticks.

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

### Fill State (Current Implementation)

Each stepper maintains its own fill state:

```c
struct i2s_fill_state {
  uint16_t remaining_high_ticks;  // Remaining HIGH time for current pulse
  uint16_t remaining_low_ticks;   // Remaining LOW time before next step
};
```

**Simplification**: The current implementation does not track `tick_pos` or `pulse_positions`. Instead:
- `tick_pos` is implicit: the fill function starts at the beginning of each block
- Pulse positions are not tracked for clearing (simplified single-stepper mode)
- The fill function handles block boundaries by returning `true` when block is full

**Limitation**: Without `pulse_positions` tracking, the implementation cannot support:
- Multi-stepper mode (needs per-stepper bit clearing)
- Efficient forceStop (needs to clear only stepper's bits)
- No-memset optimization (currently uses memset for simplicity)

**Future Enhancement**: For multi-stepper mode, the struct should be expanded to:
```c
struct i2s_fill_state {
  uint32_t tick_pos;              // Current position in 16MHz ticks
  uint16_t remaining_high_ticks;  // Remaining HIGH time for current pulse
  uint16_t remaining_low_ticks;   // Remaining LOW time before next step
  uint16_t pulse_positions[I2S_BLOCK_COUNT][I2S_MAX_PULSES_PER_BLOCK];
  uint8_t  pulse_count[I2S_BLOCK_COUNT];
};
```

### Buffer Clearing (Current Implementation)

**Single-stepper mode**: The buffer is cleared with `memset()` before filling.
This is acceptable for single-stepper mode since only one stepper uses the buffer.

**Multi-stepper mode (future)**: The buffer should **never** be bulk-zeroed.
Each stepper must track its own generated pulses and individually clear them
before writing new ones. This is essential for multi-stepper mode where multiple
steppers share the same DMA buffer.

At low step rates (e.g., 1 kHz), only 1 byte needs to be touched per block.
This is far more efficient than clearing 500 bytes.

**Current Limitation**: The implementation uses memset for simplicity in
single-stepper mode. This must be changed for multi-stepper support.

### fill_i2s_buffer() Algorithm (Current Implementation)

Called once per stepper per block fill cycle. The function receives the buffer
to fill and updates the fill state.

**Key characteristics**:
- Returns `true` if block is full, `false` if queue is empty or partially filled
- Advances `read_idx` as commands are consumed
- Handles block boundaries by returning early when block is full
- Carries over `remaining_high_ticks` and `remaining_low_ticks` across blocks
- Uses bit-level resolution (2 ticks per bit) for single-stepper mode

**Algorithm**:
```
fill_i2s_buffer(stepper, buf, state):

  1. Initialize tick_pos = 0 (implicit start of block)

  2. CARRY-OVER remaining HIGH phase:
     if state->remaining_high_ticks > 0:
       Write 1-bits from tick_pos for state->remaining_high_ticks
       Advance tick_pos by state->remaining_high_ticks
       state->remaining_high_ticks = 0
       If block full: update read_idx, return true

  3. CARRY-OVER remaining LOW phase:
     if state->remaining_low_ticks > 0:
       Advance tick_pos by min(state->remaining_low_ticks, ticks_to_block_end)
       state->remaining_low_ticks -= ticks_advanced
       If block full: update read_idx, return true

  4. MAIN LOOP while tick_pos < block_end AND queue not empty:

     4a. Read next command from queue:
         if cmd.steps == 0:
           Pause: state->remaining_low_ticks = cmd.ticks
           Advance read_idx
         if cmd.steps >= 1:
           if toggle_dir: toggle direction pin
           state->remaining_high_ticks = PULSE_WIDTH_TICKS (64)
           state->remaining_low_ticks = cmd.ticks - PULSE_WIDTH_TICKS
           Decrement cmd.steps
           if cmd.steps == 0: advance read_idx

     4b. Emit HIGH phase (if any):
         if state->remaining_high_ticks > 0:
           Write 1-bits from tick_pos for state->remaining_high_ticks
           Advance tick_pos by state->remaining_high_ticks
           state->remaining_high_ticks = 0
           If block full: update read_idx, return true

     4c. Emit LOW phase (if any):
         if state->remaining_low_ticks > 0:
           Advance tick_pos by min(state->remaining_low_ticks, ticks_to_block_end)
           state->remaining_low_ticks -= ticks_advanced
           If block full: update read_idx, return true

     4d. Loop to 4a

  5. Save state (remaining_high_ticks, remaining_low_ticks)
  6. Update read_idx if queue consumed
  7. Return false (block not full)
```

### Single-Stepper vs. Multi-Stepper Fill Differences

| Aspect | Single-Stepper (Current) | Multi-Stepper (Future) |
|--------|----------------|---------------|
| Bit granularity | Per-bit (2 ticks) | Per-frame (64 ticks) |
| HIGH phase | N consecutive 1-bits across frame boundaries | Set bit-slot in frame byte |
| Pulse width | Fixed 64 ticks (4µs) | Integer frames × 64 ticks (configurable) |
| Clear operation | memset entire buffer | Clear specific bit in recorded frames |
| Buffer sharing | Exclusive (one stepper) | Shared (multiple steppers) |
| State tracking | Simple (remaining ticks) | Complex (tick_pos, pulse_positions) |

### Handling Queue Empty

If the queue is empty, `fill_i2s_buffer()` clears previously set bits
(step 1) and returns. No new pulses are emitted. The stepper's fill state
retains `tick_pos` for correct timing when new commands arrive.

---

## ForceStop Algorithm

### Current Implementation

The current implementation has basic forceStop support:

```cpp
void StepperQueue::forceStop_i2s() {
  // Clear fill state
  _fill_state.remaining_high_ticks = 0;
  _fill_state.remaining_low_ticks = 0;
  
  // Reset queue pointers
  read_idx = next_write_idx;
}
```

**Limitations**:
- Does not stop DMA (pulses in buffer continue to output)
- Does not clear buffer (existing pulses still output)
- Simple but incomplete

### Future Enhancement

#### Single-Stepper Mode
```
forceStop_single():
  Stop I2S DMA
  memset(all blocks, 0)
  Clear fill_state (tick_pos, remaining ticks, all pulse_positions)
  Restart I2S DMA
```

Memset is acceptable here because only one stepper owns the entire buffer.

#### Multi-Stepper Mode
```
forceStop_multi(stepper):
  for each block b in 0..I2S_BLOCK_COUNT-1:
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
// Planned driver type constants for I2S
// COMPILE-TIME validation with SOC_I2S_NUM
#if SUPPORT_I2S
#define DRIVER_RMT_I2S_DIRECT FasDriver::RMT_I2S_DIRECT
#if SOC_I2S_NUM >= 1
#define DRIVER_RMT_I2S0_MUX   FasDriver::RMT_I2S0_MUX
#endif
#if SOC_I2S_NUM >= 2
#define DRIVER_RMT_I2S1_MUX   FasDriver::RMT_I2S1_MUX
#endif
#if SOC_I2S_NUM >= 3
#define DRIVER_RMT_I2S2_MUX   FasDriver::RMT_I2S2_MUX
#endif
#endif // SUPPORT_I2S

// Single-stepper mode: uses RMT_I2S_DIRECT driver type
// No PIN_I2S_FLAG needed for single-stepper mode
FastAccelStepper* s = engine.stepperConnectToPin(step_pin, DRIVER_RMT_I2S_DIRECT);

// Multi-stepper mode: uses RMT_I2S0_MUX, RMT_I2S1_MUX, or RMT_I2S2_MUX driver type
// PIN_I2S_FLAG (0x40) indicates I2S-controlled pin
uint8_t i2s_pin = slot_index | PIN_I2S_FLAG;
FastAccelStepper* s = engine.stepperConnectToPin(i2s_pin, DRIVER_RMT_I2S0_MUX);
```

The driver selection is based on the `FasDriver` enum value passed to `stepperConnectToPin()`.
For multi-stepper mode, the `PIN_I2S_FLAG` bitmask indicates that the pin number is actually
a slot index for I2S-controlled output.

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

### PC-Based Tests (test_21.cpp) ✅ COMPLETE

The fill function is testable on PC without I2S hardware. `test_21.cpp` provides:

- **Part 1**: DMA infrastructure simulation (callbacks, block rotation, bit stream
  analysis via `BitStreamAnalyzer`)
- **Part 2**: Fill function tests with queue commands (single step, multi step,
  pause, block boundary, carry-over, max values)
- **28 comprehensive tests** covering edge cases and boundary conditions

**Recent Fix**: Tests incorrectly used `fillAndDetectPulses()` return value to check
if queue was consumed. Fixed to use `q.read_idx == q.next_write_idx` instead.

### Test Coverage

| Test Category | Count | Status |
|---------------|-------|--------|
| DMA callback infrastructure | 6 | ✅ |
| Fill function basic | 12 | ✅ |
| Fill function edge cases | 10 | ✅ |
| **Total** | **28** | **✅** |

### Test Categories:
1. **DMA Infrastructure**: Callback invocation, block rotation, buffer clearing
2. **Basic Fill**: Single/multi steps, pauses, step+pause+step sequences
3. **Block Boundaries**: Partial fills, carry-over ticks, exact boundaries
4. **Edge Cases**: Empty queue, min speed, max steps (255), long pauses
5. **Maximum Values**: 65535 ticks, max pause, consecutive partial steps

### Future Test Needs
- Multi-stepper shared buffer tests (different slots, no interference)
- ForceStop tests with DMA control
- Per-stepper bit clearing verification (no memset)
- Hardware validation tests (PCNT, logic analyzer)

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

### Current Implementation (Single-Stepper Only)

| Aspect | Current | Future (Multi-Stepper) |
|--------|---------|------------------------|
| Max frequency | ~200 kHz | ~40 kHz (granularity) |
| Resolution | 2 ticks (125 ns) | 64 ticks (4 µs) |
| Steppers per I2S | 1 | Up to 16 |
| External hardware | None | Demux IC required |
| ForceStop | Basic (no DMA control) | DMA stop + restart |
| Buffer clearing | memset | Per-stepper bit tracking |
| Pulse width | Fixed 64 ticks | Configurable |
| WS pin | Unused | Required for demux |

### Implementation Gaps

1. **Multi-stepper mode not implemented**: Single-stepper only
2. **No per-stepper bit tracking**: Uses memset for simplicity
3. **Fixed pulse width**: 64 ticks (4µs), not configurable
4. **Simple forceStop**: Doesn't stop DMA or clear buffer
5. **No WS pin support**: Hardcoded to `I2S_GPIO_UNUSED`
6. **No hardware validation**: PC-based tests only

### Design vs Implementation Differences

| Design Specification | Current Implementation |
|----------------------|------------------------|
| Triple buffering (3 blocks) | Double buffering (2 blocks) |
| ISR + FreeRTOS task | ISR-only |
| Per-stepper bit tracking | memset clearing |
| Configurable pulse width | Fixed 64 ticks |
| Multi-stepper support | Single-stepper only |
| ForceStop with DMA control | Basic forceStop |

---

## Open Items & Future Work

### High Priority
1. **Multi-stepper mode implementation**: Slot allocation, bit-slot mapping, shared buffer
2. **Per-stepper bit tracking**: Replace memset with targeted bit clearing
3. **Enhanced forceStop**: Stop DMA, clear buffer, restart DMA
4. **Hardware validation**: PCNT tests, logic analyzer verification

### Medium Priority
5. **Configurable pulse width**: `setI2sPulseWidth(ticks)` API
6. **WS pin support**: Required for multi-stepper demux IC
7. **Engine API expansion**: `initI2sMultiStepper()` with configuration struct
8. **DIR/ENABLE service functions**: For multi-stepper signal control

### Low Priority
9. **8-bit mode**: Halve frame duration (32 ticks/frame) for higher multi-stepper resolution
10. **Triple buffering**: Upgrade from double to triple buffering (3 blocks)
11. **ISR + Task architecture**: For multi-stepper with many steppers
12. **Zero-copy verification**: Verify `i2s_channel_write()` doesn't copy buffers

### Implementation Plan

**Phase 1 (Core)**: Multi-stepper foundation
- Expand `i2s_fill_state` with `tick_pos` and `pulse_positions`
- Implement per-stepper bit tracking (no memset)
- Add slot allocation and bit-slot mapping

**Phase 2 (Features)**: Multi-stepper functionality
- Implement `initI2sMultiStepper()` API
- Add WS pin support for demux IC
- Implement DIR/ENABLE service functions

**Phase 3 (Polish)**: Enhancements
- Configurable pulse width
- Enhanced forceStop with DMA control
- Hardware validation tests

**Phase 4 (Optimization)**: Performance
- Triple buffering (3 blocks)
- ISR + Task architecture if needed
- Zero-copy verification and optimization
