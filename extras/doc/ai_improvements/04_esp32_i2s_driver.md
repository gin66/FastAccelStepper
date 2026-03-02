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
Each bit within the frame represents 2 ticks (125 ns), giving 32× higher resolution than
frame-level timing. This enables step rates up to ~200 kHz with fine timing control.

The step_pin parameter encodes both the bit position and mode flag:
```
step_pin = (bit_position | ISR_PIN_MASK)  // ISR_PIN_MASK = 64 indicates single-stepper mode
```

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

This is a **design change** from other drivers: I2S mode cannot use statically
allocated arrays because memory requirements differ dramatically:

| Mode | Buffer Size | Pulse Tracking Array | Per-Stepper |
|------|-------------|---------------------|-------------|
| Single-stepper (200 kHz) | 500µs = 100 steps | 3×100 = 300 entries | 300 × 1 = 300 total |
| Multi-stepper 16× (40 kHz) | 500µs = 20 steps | 3×20 = 60 entries | 60 × 16 = 960 total |

### Triple Buffering

The I2S driver uses triple buffering with 3 blocks of ~500µs each:

```
Block 0    Block 1    Block 2
[500µs]    [500µs]    [500µs]
   ↓          ↓          ↓
DMA ←─────── completed blocks re-queued
```

Constants:
```c
#define I2S_SAMPLE_RATE_HZ     250000UL
#define I2S_TICKS_PER_FRAME    64        // 4µs per frame
#define I2S_BITS_PER_FRAME     32        // 16-bit L + 16-bit R
#define I2S_BYTES_PER_FRAME    4

#define I2S_BLOCK_COUNT        3
#define I2S_FRAMES_PER_BLOCK   125       // 500µs per block
#define I2S_BYTES_PER_BLOCK    (I2S_FRAMES_PER_BLOCK * I2S_BYTES_PER_FRAME)
```

### DMA Callback Flow

1. **DMA start**: Blocks 0, 1, 2 are enqueued. `busy_block = 0`.
2. **DMA completion callback**:
   - Completed block is re-queued to DMA
   - `busy_block = (busy_block + 1) % 3`
   - `fill_i2s_buffer()` called for all steppers on the new `busy_block`
3. **DMA never stops**: It continuously streams the triple buffer.

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

Bit-to-byte mapping accounts for I2S L/R serialization order (MSB first).

---

## Buffer Fill Algorithm

### Tick Pointer (tp)

Each stepper maintains its own tick pointer (`tp`) in 16 MHz ticks:

```c
struct i2s_fill_state {
  uint16_t tick_pos;              // Current position in 16MHz ticks (0..BLOCK_TICKS-1)
  uint16_t remaining_high_ticks;  // Remaining HIGH time for current pulse
  uint16_t remaining_low_ticks;   // Remaining LOW time before next step
  uint16_t* pulse_positions;      // Dynamically allocated array
  uint8_t  pulse_count;           // Pulses in current block
};
```

### fill_i2s_buffer() Algorithm

```
For each stepper:
  1. tp = tp % (3 * buffer_size * 64 ticks)
  
  2. If tp is inside busy_block:
       Advance tp to beginning of "next next" buffer
       (e.g., if busy=0, next=1, next_next=2 → advance to start of buffer 2)
  
  3. If high_ticks == 0 AND low_ticks == 0:
       Read next command from queue
       If cmd.steps == 0:
         high_ticks = 0
         low_ticks = cmd.ticks
         Advance queue read pointer
       If cmd.steps >= 1:
         high_ticks = PULSE_WIDTH_TICKS (configurable, typically 64)
         low_ticks = cmd.ticks - PULSE_WIDTH_TICKS
         Decrement cmd.steps
         If cmd.steps == 0: advance queue read pointer
  
  4. If high_ticks > 0:
       Add HIGH bit to buffer at frame/bit position for tp
       Record position in pulse_positions array
       tp += bit_time (2 ticks in single-stepper, 64 in multi-stepper)
       high_ticks -= bit_time
       If tp now in busy_block: exit loop
  
  5. If low_ticks > 0:
       Calculate free_ticks from tp to busy_block start
       If free_ticks > low_ticks:
         tp += low_ticks
         low_ticks = 0
         Continue loop (go to step 3)
       If free_ticks <= low_ticks:
         tp += free_ticks
         low_ticks -= free_ticks
         Exit loop
  
  6. Save tp and remaining ticks to state
```

### Handling Queue Empty

If the queue is empty, `fill_i2s_buffer()` simply advances `tp` to the "next next"
buffer start. No pulses are emitted.

### DMA Underrun Handling

If `fill_i2s_buffer()` cannot keep up with DMA consumption:
- Emit warning
- Consider reducing I2S buffer size for tighter timing feedback

---

## Pulse Tracking and Dynamic Allocation

### Memory Requirements

| Mode | Steps per Block (500µs) | 3 Blocks Total | Array Size |
|------|-------------------------|----------------|------------|
| Single-stepper @ 200kHz | 100 | 300 | 300 × uint16_t = 600 bytes |
| Multi-stepper @ 40kHz | 20 | 60 | 60 × uint16_t = 120 bytes |

### Dynamic Allocation

```cpp
bool StepperQueue::init_i2s(uint8_t step_pin, uint8_t claim) {
  I2sManager& mgr = I2sManager::instance();
  
  if (mgr.isSingleStepperMode()) {
    _fill_state.pulse_positions = (uint16_t*)malloc(300 * sizeof(uint16_t));
  } else {
    _fill_state.pulse_positions = (uint16_t*)malloc(60 * sizeof(uint16_t));
  }
  // ...
}
```

This is a **breaking change** from other drivers that use static allocation.

---

## Timing Constraints

### Minimum cmd.ticks

| Mode | Min ticks | Reason |
|------|-----------|--------|
| Single-stepper | 80 (5µs) | 200 kHz max, PULSE_WIDTH=64 ticks |
| Multi-stepper | 128 (8µs) | 2 frames minimum for HIGH+LOW |

The engine must enforce these minimums when converting speeds to ticks.

### Configurable Pulse Width

```cpp
void setI2sPulseWidth(uint16_t ticks);  // Default: 64 ticks (4µs)
```

Smaller pulse width reduces ISR processing time but must meet stepper driver
minimum pulse width requirements (typically 1-2µs).

---

## API Design

### Engine-Level Configuration

```cpp
// src/FastAccelStepper.h

struct I2sSingleStepperConfig {
  uint8_t  data_pin;
  uint8_t  bclk_pin;
  uint16_t pulse_width_ticks;  // Default: 64
};

struct I2sMultiStepperConfig {
  uint8_t  data_pin;
  uint8_t  bclk_pin;
  uint8_t  ws_pin;
  uint8_t  signals_per_stepper;  // 1, 2, or 3
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

## File Structure

### New Files

```
src/pd_esp32/
  i2s_constants.h           — I2S timing constants
  i2s_manager.h             — I2sManager class declaration
  i2s_manager.cpp           — I2sManager implementation
  i2s_fill.h                — fill_i2s_buffer() declaration
  i2s_fill.cpp              — fill_i2s_buffer() implementation
  StepperISR_esp32_i2s.cpp  — StepperQueue I2S methods
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

## Implementation Notes

### Bit Serialization Order

The ESP32 I2S outputs bits MSB-first within each 16-bit half (L then R).
The fill algorithm must map slot indices to correct bit positions:

```cpp
// For slot 0-15: in L channel (bytes 0-1)
// For slot 16-31: in R channel (bytes 2-3)
uint8_t byte_idx = (slot < 16) ? (14 - slot/8) : (30 - slot/8);
uint8_t bit_idx = slot % 8;
```

### Graceful Stop

DMA never stops. To halt a stepper:
1. Stop adding new commands to queue
2. Let queue drain naturally
3. `fill_i2s_buffer()` will emit zeros after queue empties

### Direction Pin Timing

In multi-stepper mode with DIR in frame, direction must be set before the STEP pulse.
The fill algorithm writes DIR bits at the start of each command's span.

---

## Testing Strategy

### Stage 1 — Single-Stepper Mode, PCNT Validation

Hardware: I2S DATA_OUT → PCNT input + oscilloscope

Tests:
- Verify step count matches commanded steps
- Test max frequency (~200 kHz)
- Verify pulse width timing

### Stage 2 — Multi-Stepper Mode, Logic Analyzer

Hardware: I2S DATA_OUT, BCLK, WS → logic analyzer

Tests:
- Decode I2S stream, verify bit-slot timing
- Verify multiple steppers interleaved correctly
- Verify DIR bit stability during STEP pulses

### Stage 3 — Multi-Stepper Mode, 74HC595 + PCNT

Hardware: Full demux chain → PCNT

Tests:
- End-to-end step count validation
- Multiple simultaneous steppers

---

## Known Limitations

| Aspect | Single-Stepper | Multi-Stepper |
|--------|----------------|---------------|
| Max frequency | ~200 kHz | ~40 kHz (granularity) |
| Resolution | 2 ticks (125 ns) | 64 ticks (4 µs) |
| Steppers per I2S | 1 | Up to 16 |
| External hardware | None | Demux IC required |
| Memory per stepper | 600 bytes | 120 bytes |
| Dynamic allocation | Required | Required |

---

## Open Items

1. **8-bit mode**: Future option to halve frame duration (32 ticks) for higher
   multi-stepper resolution. Requires I2S reconfiguration.

2. **Pulse position array overflow**: If frequency exceeds tracking capacity,
   behavior is undefined. Engine should enforce frequency limits.

3. **Direction delay**: Some stepper drivers require direction setup time.
   Multi-stepper mode may need configurable delay before STEP after DIR change.
