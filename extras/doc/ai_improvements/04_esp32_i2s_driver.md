# ESP32 I2S Output Driver

## Overview

This document describes the architecture, design, implementation, and testing plan for a
new ESP32 stepper output driver based on the I2S peripheral with DMA. The driver targets
two distinct use-cases:

- **Single-pin mode**: one I2S DATA_OUT GPIO carries the step signal. The frame word
  (minimum 8 bits) can also encode DIR in a second bit-slot, with two physical output
  options: a minimal 74HC595 (QA=STEP, QB=DIR, same IC as serial mode) or the ESP32's
  I2S parallel (LCD) output mode, which drives one GPIO per bit simultaneously with no
  external IC. Primarily useful for functional testing and simple boards.
- **Serial mode**: one I2S DATA_OUT line feeds an external serial-to-parallel demux IC
  (e.g., a chain of 74HC595 shift registers). A single I2S peripheral can then control
  many steppers simultaneously.

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

At BCLK = 10 MHz and a frame width of W bits (minimum 8), the time resolution per frame
is:

```
frame_period = W / BCLK = W / 10 MHz
```

| Frame width (bits) | Frame period | Max step rate (theory) |
|--------------------|-------------|------------------------|
|  8                 |  800 ns     |  625 kHz               |
| 16                 |  1.6 µs     |  312 kHz               |
| 32                 |  3.2 µs     |  156 kHz               |

Step pulse width equals at least one frame period, which is ≥ 800 ns — acceptable for
most stepper drivers (typical minimum is 1–2 µs; a 2-frame pulse is safe).

FastAccelStepper uses `TICKS_PER_S = 16 000 000`. One 16 MHz tick = 62.5 ns. Converting:

```
ticks_per_frame = W * TICKS_PER_S / BCLK_HZ
```

For W=8, BCLK=10 MHz: 12.8 ticks/frame → rounded to 13 (≈ 812 ns).

### Demux IC (Serial Mode)

A shift-register chain (e.g., two 74HC595 in series = 16 outputs) receives:
- DATA_OUT → SER input
- BCLK → SRCLK (shift clock)
- WS/LRCLK → RCLK (storage register clock / latch)

Each WS edge latches the last W bits from DATA_OUT to the 8/16-bit parallel output. The
parallel outputs drive individual step, direction, and enable signals of stepper drivers.

### Single-Pin Mode (Step Only or Step+Dir)

Single-pin refers to using a single I2S DATA_OUT wire — not necessarily a single stepper
signal. The 8-bit frame word has room for both STEP (bit 0) and DIR (bit 1) of one
stepper. Two approaches to expose both signals as physical GPIOs:

**Option A — Minimal shift register (74HC595)**
Use the same IC as serial mode; only QA (STEP) and QB (DIR) are wired. BCLK and WS are
still needed. Functional but adds 3 wires even for one stepper.

**Option B — I2S parallel (LCD) mode**
The ESP32 I2S peripheral has a parallel output mode (used for TFT displays) where all W
bits of each frame word are simultaneously clocked out on W separate GPIOs. With W=8:
bit 0 → GPIO_STEP, bit 1 → GPIO_DIR. No external IC. BCLK still needed as sample clock.
The remaining 6 GPIOs are unused.

For pure STEP-only testing (no DIR output), DATA_OUT can drive one GPIO directly; BCLK
and WS can be left unconnected (but still configured in the I2S peripheral).

In all single-pin cases one I2S instance = one stepper.

---

## Architecture Design

### Two Modes

```
Single-pin, STEP only               Single-pin, STEP+DIR                Serial mode
─────────────────────               ────────────────────                ───────────
ESP32 I2S                           ESP32 I2S (parallel/LCD mode)       ESP32 I2S
  DATA_OUT ──→ GPIO_STEP              bit 0 ──→ GPIO_STEP                 DATA_OUT ──→ 74HC595 chain
                                      bit 1 ──→ GPIO_DIR                  BCLK    ──→ SRCLK
                                      (no external IC)                    WS      ──→ RCLK
                                                                            QA ──→ stepper 0 STEP
              OR                                OR                          QB ──→ stepper 0 DIR
                                                                            QC ──→ stepper 1 STEP
  DATA_OUT ──→ 74HC595 QA (STEP)    DATA_OUT ──→ 74HC595 QA (STEP)       QD ──→ stepper 1 DIR
              BCLK, WS needed                   QB (DIR)                       ...
```

### Global I2S Buffer Manager

Both modes share a common **circular DMA buffer** managed by a singleton
`I2sManager`. The DMA hardware streams this buffer to the I2S peripheral at a fixed
rate determined by BCLK and bits_per_frame. The stepper task fills the buffer far enough
ahead of the DMA read pointer.

```
Ring buffer (DMA SRAM, MALLOC_CAP_DMA):

  frame 0   frame 1   ...   frame N-1   frame 0  (wraps)
  [byte...]  [byte...]       [byte...]

  bits within each frame word:
    bit 0 → stepper 0 STEP (or signal 0)
    bit 1 → stepper 0 DIR
    bit 2 → stepper 1 STEP
    bit 3 → stepper 1 DIR
    ...
```

In single-pin mode the frame width is effectively the I2S DMA word width (8 bits minimum)
and only one bit-slot is used. In serial mode all slots are assigned.

### Slot Assignment

When connecting a stepper, the caller declares which signals it needs. The driver allocates
consecutive bit-slots in the frame word starting from the next free slot:

| Claim mode         | Slots allocated | Signals               |
|--------------------|-----------------|-----------------------|
| `STEP_ONLY`        | 1               | STEP                  |
| `STEP_DIR`         | 2               | STEP, DIR             |
| `STEP_DIR_ENABLE`  | 3               | STEP, DIR, ENABLE     |

```
base_slot = I2sManager::allocateSlots(claim_count)

stepper._i2s_step_slot   = base_slot
stepper._i2s_dir_slot    = (claim >= STEP_DIR)    ? base_slot + 1 : -1
stepper._i2s_enable_slot = (claim == STEP_DIR_ENABLE) ? base_slot + 2 : -1
```

A slot index of -1 means that signal is managed via a conventional GPIO (existing
enable/direction pin mechanism). This lets steppers mix: e.g., STEP in the I2S frame,
DIR and ENABLE on ordinary GPIOs.

With 8 slots in one frame byte, packing examples:

| Config                  | Steppers per byte |
|-------------------------|------------------|
| 4 × STEP_DIR            | 4                |
| 2 × STEP_DIR_ENABLE     | 2 (+ 2 spare)    |
| 8 × STEP_ONLY           | 8                |
| mixed (3+2+3)           | 2 STEP_DIR_ENABLE + 1 STEP_DIR + remainder |

### Position Tracking

The DMA descriptor chain (N descriptors, each covering `buf_frames / N` frames) raises an
event when each descriptor finishes. The `I2sManager` interrupt handler increments a
`dma_frame_counter` (64-bit, monotonic). This gives the approximate DMA read position
in frames.

The stepper task reads `dma_frame_counter` and ensures the buffer is written at least
`forward_frames` ahead:

```
write_horizon = dma_frame_counter + forward_frames
```

`forward_frames` is chosen to equal 2× the stepper task period:

```
forward_frames = 2 * task_period_s * BCLK_HZ / bits_per_frame
```

At 4 ms task period, W=16, 10 MHz BCLK: forward_frames = 2 × 0.004 × 625 000 = 5 000 frames
= 5 000 × 2 bytes = 10 KB total buffer size.

Recommended buffer size: 2× forward_frames to allow for jitter.

---

## API Design

### FasDriver Enum Extension

```cpp
// src/fas_arch/common.h
enum class FasDriver : uint8_t {
  MCPWM_PCNT = 0,
  RMT        = 1,
  I2S        = 2,   // new
  DONT_CARE  = 255
};
#define DRIVER_I2S FasDriver::I2S
```

### Engine-Level Serial Mode Configuration

```cpp
// src/FastAccelStepper.h  (inside FastAccelStepperEngine)

struct I2sSerialConfig {
  uint8_t  data_pin;         // I2S DATA_OUT GPIO
  uint8_t  bclk_pin;         // I2S BCLK GPIO
  uint8_t  ws_pin;           // I2S WS/LRCLK GPIO
  uint32_t bclk_hz;          // BCLK frequency, e.g. 10000000
  uint8_t  bits_per_frame;   // 8, 16, or 32
  uint8_t  signals_per_stepper; // typically 2 (STEP+DIR) or 3 (STEP+DIR+EN)
  uint16_t buf_frames;       // ring buffer size in frames (e.g. 20000)
};

// Call once before stepperConnectToPin() with DRIVER_I2S in serial mode.
// Returns false if I2S hardware is unavailable or config is invalid.
bool configureI2sSerial(const I2sSerialConfig& cfg);
```

### Single-Pin Mode (Per-Stepper)

No engine-level config needed. The step_pin **is** the GPIO wired to I2S DATA_OUT:

```cpp
FastAccelStepper* s = engine.stepperConnectToPin(step_gpio, DRIVER_I2S);
```

The driver internally assigns this stepper to bit-slot 0 in a dedicated 1-bit frame
(frame width = 8 bits minimum, only bit 0 used).

### Serial Mode (Per-Stepper)

After `configureI2sSerial()` is called, the step_pin argument is interpreted as a
**logical channel index** (0, 1, 2, …), not a GPIO number:

```cpp
FastAccelStepper* s0 = engine.stepperConnectToPin(0, DRIVER_I2S); // channel 0
FastAccelStepper* s1 = engine.stepperConnectToPin(1, DRIVER_I2S); // channel 1
```

`stepperConnectToPin()` must detect whether the I2S serial mode has been configured and
interpret the argument accordingly.

### Direction Pin in Serial Mode

In serial mode, the DIR signal occupies a bit-slot in the I2S frame. The stepper must
call `setDirectionPin()` with a special sentinel (e.g., `PIN_I2S_VIRTUAL` or let the
driver manage direction within the buffer). The engine translates direction changes into
bit writes in the frame word, synchronized with the step pattern.

---

## File Structure

### New Files

```
src/pd_esp32/
  i2s_manager.h               — I2sManager class declaration
  i2s_manager.cpp             — I2sManager implementation (DMA setup, slot alloc,
                                 buffer fill helpers)
  StepperISR_esp32_i2s.cpp    — StepperQueue I2S methods (init_i2s, startQueue_i2s,
                                 forceStop_i2s, fill_i2s_buffer)
```

### Modified Files

```
src/fas_arch/common.h                  — add I2S to FasDriver enum
src/fas_arch/common_esp32_idf5.h       — add SUPPORT_ESP32_I2S, QUEUES_I2S
src/pd_esp32/esp32_queue.h             — add I2S-specific members to StepperQueue
src/pd_esp32/esp32_queue.cpp           — add I2S dispatch branches in init/start/stop
src/FastAccelStepper.h                 — add I2sSerialConfig, configureI2sSerial()
src/FastAccelStepper.cpp               — implement configureI2sSerial(), detect mode
```

---

## Detailed Implementation

### Phase 1 — I2sManager Skeleton

`i2s_manager.h`:

```cpp
#pragma once
#include <stdint.h>
#include <stddef.h>
#include <driver/i2s_std.h>

class I2sManager {
 public:
  struct Config {
    uint8_t  data_pin;
    uint8_t  bclk_pin;
    uint8_t  ws_pin;
    uint32_t bclk_hz;
    uint8_t  bits_per_frame;   // 8, 16, or 32
    uint16_t buf_frames;       // ring buffer length in frames
    bool     single_pin_mode;  // true → 1 stepper, DATA_OUT = step pin
  };

  static I2sManager& instance();

  bool init(const Config& cfg);

  // Allocate `count` consecutive bit-slots (1=STEP_ONLY, 2=STEP_DIR, 3=STEP_DIR_ENABLE).
  // Returns base slot index (0..bits_per_frame-1), or -1 on failure.
  int8_t allocateSlots(uint8_t count);

  // Current DMA read position (in frames, monotonic).
  uint32_t IRAM_ATTR dmaFramePosition() const;

  // Set/clear a bit-slot value for a range of frames.
  // Handles ring-buffer wrap-around.
  void IRAM_ATTR setBits(uint8_t slot, uint32_t frame_start,
                         uint32_t frame_end, bool value);

  uint8_t  bitsPerFrame()  const { return _cfg.bits_per_frame; }
  uint32_t bclkHz()        const { return _cfg.bclk_hz; }
  uint16_t bufFrames()     const { return _cfg.buf_frames; }
  bool     isInitialized() const { return _initialized; }

 private:
  I2sManager() = default;
  Config               _cfg{};
  bool                 _initialized = false;
  i2s_chan_handle_t    _tx_chan = nullptr;
  uint8_t*             _buf    = nullptr;  // DMA buffer (MALLOC_CAP_DMA)
  uint8_t              _bytes_per_frame = 0;
  volatile uint32_t    _dma_frame_pos = 0; // updated in DMA callback (IRAM)
  uint8_t              _next_slot = 0;

  static bool IRAM_ATTR on_sent(i2s_chan_handle_t handle,
                                i2s_event_data_t* event, void* ctx);
};
```

Key implementation notes for `i2s_manager.cpp`:

1. **DMA buffer allocation**: `heap_caps_malloc(buf_bytes, MALLOC_CAP_DMA | MALLOC_CAP_INTERNAL)`
2. **I2S channel creation** (IDF5):
   ```cpp
   i2s_chan_config_t chan_cfg = I2S_CHANNEL_DEFAULT_CONFIG(I2S_NUM_AUTO, I2S_ROLE_MASTER);
   i2s_new_channel(&chan_cfg, &_tx_chan, NULL);
   ```
3. **STD mode config** (8-bit mono for W=8, or 16-bit mono for W=16):
   ```cpp
   i2s_std_config_t std_cfg = {
     .clk_cfg  = I2S_STD_CLK_DEFAULT_CONFIG(bclk_hz / bits_per_frame),
     .slot_cfg = I2S_STD_MSB_SLOT_DEFAULT_CONFIG(
                   I2S_DATA_BIT_WIDTH_16BIT, I2S_SLOT_MODE_MONO),
     .gpio_cfg = { .bclk = bclk_pin, .ws = ws_pin, .dout = data_pin,
                   .din = I2S_GPIO_UNUSED, .mclk = I2S_GPIO_UNUSED },
   };
   i2s_channel_init_std_mode(_tx_chan, &std_cfg);
   ```
4. **DMA callback**: Register `i2s_event_callbacks_t.on_sent` → increments `_dma_frame_pos`
   by the number of frames in the completed DMA descriptor.
5. **Buffer writing**: `setBits()` computes `byte_index = (frame % buf_frames) * bytes_per_frame + slot/8`
   and applies `|= (1<<(slot%8))` or `&= ~(1<<(slot%8))`.

### Phase 2 — StepperQueue I2S Methods

`esp32_queue.h` additions:

```cpp
#ifdef SUPPORT_ESP32_I2S
  bool   use_i2s;
  int8_t _i2s_step_slot;    // bit-slot for STEP in the I2S frame
  int8_t _i2s_dir_slot;     // bit-slot for DIR    (-1 → use GPIO)
  int8_t _i2s_enable_slot;  // bit-slot for ENABLE (-1 → use GPIO)
  uint32_t _i2s_write_frame; // next frame to write into buffer
  bool   _i2sRunning;

  // claim: 1=STEP_ONLY, 2=STEP_DIR, 3=STEP_DIR_ENABLE
  bool     init_i2s(uint8_t channel_num, uint8_t step_pin_or_slot,
                    uint8_t claim);
  void     startQueue_i2s();
  void     forceStop_i2s();
  bool     isReadyForCommands_i2s();
  uint16_t _getPerformedPulses_i2s();
  void     fill_i2s_buffer();   // called from stepper task, not ISR
#endif
```

`esp32_queue.cpp` dispatch additions (mirrors the existing RMT pattern):

```cpp
bool StepperQueue::init(FastAccelStepperEngine* engine, uint8_t queue_num,
                        uint8_t step_pin) {
  ...
#ifdef SUPPORT_ESP32_I2S
  if (/* I2S driver requested */) {
    use_i2s = true;
    return init_i2s(queue_num, step_pin);
  }
#endif
  ...
}
```

### Phase 3 — Buffer Fill Algorithm

`StepperISR_esp32_i2s.cpp` — `fill_i2s_buffer()`:

```
Called from manageSteppers() (stepper task, every 4 ms).
For each stepper using the I2S driver:
  1. Compute write_horizon = dmaFramePosition() + forward_frames
  2. While _i2s_write_frame < write_horizon AND queue not empty:
     a. Read current queue entry (steps S, ticks T, countUp D)
     b. frames_per_step = (T * bclk_hz) / (bits_per_frame * TICKS_PER_S)
                          (integer arithmetic, round up)
     c. pulse_frames    = max(2, step_pulse_us * bclk_hz /
                              (1e6 * bits_per_frame))
     d. Write DIR bit-slot for all frames in this entry's span
     e. For each step i in 0..S-1:
          step_frame = _i2s_write_frame + i * frames_per_step
          setBits(STEP_slot, step_frame, step_frame+pulse_frames, HIGH)
          setBits(STEP_slot, step_frame+pulse_frames,
                  step_frame+frames_per_step, LOW)
     f. Advance _i2s_write_frame by S * frames_per_step
     g. If queue entry consumed, increment read_idx
  3. Zero-fill STEP slot for remaining frames up to write_horizon
     (ensures no stale pulses if stepper stops).
```

Critical: `fill_i2s_buffer()` must **not** write frames that DMA has already passed. Guard:

```cpp
if (frame < dmaFramePosition()) continue; // already in the past
```

### Phase 4 — Engine API

```cpp
// FastAccelStepper.cpp
bool FastAccelStepperEngine::configureI2sSerial(const I2sSerialConfig& cfg) {
  I2sManager::Config mgr_cfg = { cfg.data_pin, cfg.bclk_pin, cfg.ws_pin,
                                  cfg.bclk_hz, cfg.bits_per_frame,
                                  cfg.buf_frames, false };
  return I2sManager::instance().init(mgr_cfg);
}

FastAccelStepper* FastAccelStepperEngine::stepperConnectToPin(
    uint8_t step_pin_or_slot, FasDriver driver_type) {
  ...
  if (driver_type == FasDriver::I2S) {
    // determine single-pin vs serial mode from I2sManager::isInitialized()
    if (!I2sManager::instance().isInitialized()) {
      // single-pin mode: init I2sManager internally
      I2sManager::Config cfg = { step_pin_or_slot, /* default BCLK/WS pins */,
                                  10000000, 8, 20000, true };
      I2sManager::instance().init(cfg);
    }
    // proceed with normal queue init
  }
  ...
}
```

### Phase 5 — Arch Header Updates

`common_esp32_idf5.h` (CONFIG_IDF_TARGET_ESP32 block):

```cpp
#define SUPPORT_ESP32_I2S
#define QUEUES_I2S 1         // one I2S instance (up to bits_per_frame/signals slots)
#define NEED_I2S_HEADERS
```

```cpp
#ifdef NEED_I2S_HEADERS
#include <driver/i2s_std.h>
#include <driver/i2s_types.h>
#endif
```

`common_esp32.h`:

```cpp
#define NUM_QUEUES (QUEUES_MCPWM_PCNT + QUEUES_RMT + QUEUES_I2S)
```

---

## Timing and Resolution Analysis

### Single-Pin Mode

```
BCLK = 10 MHz, bits_per_frame = 8 (minimum I2S word width)
bit period      = 100 ns
frame period    = 800 ns  (≈ 12.8 FastAccelStepper ticks at 16 MHz)
step resolution = 800 ns  (one frame = one time quantum)
max step rate   = 625 kHz (limited by Nyquist: period ≥ 2 frames)
pulse width     = ≥ 800 ns (set to 2 frames = 1.6 µs for driver compatibility)
```

This is roughly equivalent to the RMT driver's resolution (tick = 62.5 ns) at lower
accuracy but achieves adequate step rates for most stepper applications.

### Serial Mode — 8 channels, 8-bit frame

```
BCLK = 10 MHz, bits_per_frame = 8
frame period    = 800 ns
channels        = 8 (e.g., 4 steppers × step+dir)
max step rate   ≈ 312 kHz per stepper
DMA bandwidth   = 10 MHz / 8 = 1.25 MB/s
buffer size (8 ms forward) = 10 000 frames × 1 byte = 10 KB
```

### Serial Mode — 16 channels, 16-bit frame

```
BCLK = 10 MHz, bits_per_frame = 16
frame period    = 1.6 µs
channels        = 16 (e.g., 8 steppers × step+dir, or 5 × step+dir+en)
max step rate   ≈ 156 kHz per stepper
DMA bandwidth   = 10 MHz / 8 = 1.25 MB/s (bytes transferred)
buffer size (8 ms forward) = 5 000 frames × 2 bytes = 10 KB
```

---

## Testing Strategy

### Stage 1 — Single-Pin Mode, PCNT Validation

Hardware setup:
```
ESP32 GPIO_X (I2S DATA_OUT) ──┬──→ PCNT input
                               └──→ scope / logic analyzer
```

Test procedure (mirrors existing `seq_01a.sh` pattern):

1. Configure stepper: `engine.stepperConnectToPin(GPIO_X, DRIVER_I2S)`
2. Move N steps at speed S
3. Read PCNT count
4. Assert count == N within ±1 step

New test scripts:
```
extras/tests/esp32_hw_based/seq_i2s_01a.sh   — single step, verify count
extras/tests/esp32_hw_based/seq_i2s_01b.sh   — ramp up/down, verify count
extras/tests/esp32_hw_based/seq_i2s_02.sh    — max speed, measure PCNT rate
```

PCNT wiring: pin to be determined by the StepperDemo firmware. A new "motor mode" may need
to be added to StepperDemo to select the I2S driver.

### Stage 2 — Serial Mode, Logic Analyzer Validation

Hardware setup:
```
ESP32 DATA_OUT ──→ logic analyzer CH0
ESP32 BCLK     ──→ logic analyzer CH1
ESP32 WS       ──→ logic analyzer CH2
```

Verification:
- Decode I2S stream using analyzer's protocol decoder
- Verify bit-slot 0 (stepper 0 STEP) toggles at the expected frequency
- Verify bit-slot 1 (DIR) matches commanded direction
- Measure pulse width on STEP slot ≥ 1.6 µs

### Stage 3 — Serial Mode, 74HC595 + PCNT

Hardware setup:
```
ESP32 DATA_OUT ──→ 74HC595 SER
ESP32 BCLK     ──→ 74HC595 SRCLK
ESP32 WS       ──→ 74HC595 RCLK
74HC595 QA     ──→ PCNT input (stepper 0 STEP)
74HC595 QB     ──→ oscilloscope (stepper 0 DIR)
```

Test procedure:
- Run existing seq_0x.sh tests adapted for serial mode
- Compare PCNT counts to commanded step counts
- Repeat for stepper 1 (74HC595 QC for STEP, QD for DIR)

### Stage 4 — Regression Tests

After I2S driver is functional, run the full existing test suite with `DRIVER_RMT` to
confirm no regressions in the RMT driver:
```
make -C extras/tests/esp32_hw_based test
```

---

## StepperDemo Firmware Changes

The StepperDemo app must be extended to support I2S driver selection. Two new motor
configuration modes are proposed:

- `Mi<slot>` — I2S serial mode, slot = logical channel index
- The `engine.configureI2sSerial()` call should be issued once in `setup()` before
  any `Mi<slot>` motor connection

For single-pin testing, the existing `Mx` motor number can reuse the pin number with a new
driver type flag. Exact UI design is left to implementation.

---

## Known Limitations and Trade-offs

| Aspect | I2S Driver | RMT Driver |
|--------|------------|------------|
| Step resolution | 800 ns – 3.2 µs | 62.5 ns |
| Max step rate | 156–625 kHz | > 1 MHz |
| CPU overhead | Continuous (fill task) | Low (callback only) |
| DMA RAM usage | 10–40 KB shared | ~128 B per channel |
| Number of steppers | Up to bits_per_frame (serial) | Up to 8 (ESP32) |
| External hardware | Demux IC needed (serial mode) | None |
| Timing jitter | ≤ 1 frame period | < 1 tick (62.5 ns) |

The continuous buffer-fill approach consumes more CPU than RMT's encoder callback. The
stepper task must be fast enough to stay ahead of DMA. At 4 ms task rate and 10 KB buffer
(8 ms of data), there is comfortable margin — the fill calculation is O(queued steps) and
is typically cheap.

For applications requiring sub-microsecond timing or maximum step rates, the RMT driver
remains the better choice. The I2S driver's advantage is expansion capacity via the
demux IC: a single I2S peripheral can serve up to 16 (or 32) steppers with adequate
resolution for most CNC and automation applications.

---

## Open Questions

1. **BCLK frequency**: Is 10 MHz the actual ESP32 I2S output limit, or the external IC
   limit? Needs verification. Higher BCLK (e.g., 20–40 MHz) improves resolution
   significantly.
2. **I2S word bit order**: ESP32 I2S outputs MSB first by default. The shift register
   chain must match (74HC595 shifts MSB in first → output order reversal must be accounted
   for in slot assignment).
3. **WS pin in single-pin mode**: WS toggles every W bits; whether this causes interference
   on nearby signals should be checked. WS pin can be left unconnected or routed to a
   test point.
4. **Direction pin timing**: In serial mode, the DIR bit must be set before the first STEP
   pulse in the frame. The fill algorithm must write DIR bits ahead of STEP bits by at
   least the driver's direction-setup time (typically 0 µs for most drivers, but some need
   up to 200 µs).
5. **`isReadyForCommands_i2s()`**: Since the I2S driver continuously streams, a queue
   entry is "consumed" when its frames have been written to the buffer, not when DMA has
   sent them. The ready check should be buffer-space based, not running-state based.
6. **SUPPORT_SELECT_DRIVER_TYPE**: Currently only defined when both MCPWM_PCNT and RMT
   queues are nonzero. A new condition must include I2S.

---

## Implementation Order

1. `I2sManager` skeleton: init, allocateSlots, setBits (no DMA yet; use i2s_channel_write)
2. Single-pin mode in StepperQueue: `init_i2s`, `startQueue_i2s`, `forceStop_i2s`
3. Buffer fill algorithm: `fill_i2s_buffer()` called from stepper task
4. Stage 1 tests pass (PCNT validates step counts in single-pin mode)
5. Engine API: `configureI2sSerial()`, driver type detection in `stepperConnectToPin()`
6. Serial mode: multi-slot assignment, DIR bit management in frame
7. Stage 2 tests pass (logic analyzer validates serial stream)
8. Stage 3 tests pass (PCNT via 74HC595 validates serial mode end-to-end)
9. DMA callback upgrade: replace `i2s_channel_write` with low-level DMA descriptor
   management for lower latency and zero-copy fill
10. Update README and arch documentation
