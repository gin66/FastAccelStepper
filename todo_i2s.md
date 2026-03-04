# I2S Driver Implementation TODO

This document tracks the gaps between the design document (`04_esp32_i2s_driver.md`) and the current implementation, with a detailed execution plan.

---

## Gap Analysis: Design Document vs Current Implementation

### Critical Gaps (Core Functionality)

| # | Component | Design Doc | Current | Status |
|---|-----------|------------|---------|--------|
| 1 | **Block Duration** | 500µs (125 frames) | 500µs (125 frames) | ✅ Fixed |
| 1b | **DMA Descriptors** | 2 × 125 frames | 2 × 125 frames | ✅ Fixed |
| 2 | **No Memset Fill** | Per-stepper bit tracking | Per-stepper bit tracking | ✅ Fixed |
| 3 | **tick_pos type** | `uint32_t` | `uint32_t` | ✅ Fixed |
| 4 | **pulse_positions** | Per-block `[3][101]` | Per-block `[3][101]` | ✅ Fixed |
| 5 | **Single-stepper resolution** | Bit-level (2 ticks) | Bit-level (2 ticks) | ✅ Fixed |
| 6 | **Multi-stepper mode** | Full support | Not implemented | ❌ Missing |
| 7 | **Engine init API** | `initI2sSingleStepper()` / `initI2sMultiStepper()` | `initI2sSingleStepper()` | ✅ Partial |
| 8 | **Config structs** | `I2sSingleStepperConfig`, `I2sMultiStepperConfig` | `I2sSingleStepperConfig` | ✅ Partial |

### Medium Gaps (Important Features)

| # | Component | Design Doc | Current | Status |
|---|-----------|------------|---------|--------|
| 9 | **DMA Architecture** | ISR + FreeRTOS Task | ISR-only | ⚠️ Different |
| 10 | **ForceStop DMA** | Stop/restart DMA | No DMA control | ⚠️ Incomplete |
| 11 | **DIR/ENABLE bits** | `bit_mask`, `bit_target`, `bit_actual[3]` | None | ❌ Missing |
| 12 | **Slot assignment** | `allocateSlots()`, slot mapping | None | ❌ Missing |
| 13 | **setSlotBit/getSlotBit** | Service functions | None | ❌ Missing |
| 14 | **Pulse width config** | `setI2sPulseWidth(ticks)` | Configurable via `pulse_width_bits` | ✅ Partial |
| 15 | **WS pin support** | Required for multi-stepper | Always UNUSED | ⚠️ Single only |

### Test Gaps

| # | Test Area | Status |
|---|-----------|--------|
| 16 | Per-stepper bit clearing (no memset) | ✅ Implemented |
| 17 | Multi-stepper shared buffer tests | ❌ Missing |
| 18 | Multi-stepper forceStop tests | ❌ Missing |
| 19 | Single-stepper bit-level tests | ✅ Implemented |
| 20 | Hardware tests (PCNT, logic analyzer, 74HC595) | ❌ Missing |

---

## Execution Phases

### Phase 1: Fix Core Constants and Data Structures [HIGH]

**Files to modify:**
- `src/pd_esp32/i2s_constants.h`
- `src/pd_esp32/i2s_fill.h`

**Changes:**

1. `i2s_constants.h`:
```c
// Change from:
#define I2S_BLOCK_COUNT 3
#define I2S_FRAMES_PER_BLOCK 250

// To:
#define I2S_BLOCK_DURATION_US   500
#define I2S_BLOCK_COUNT         3
#define I2S_FRAMES_PER_BLOCK    (I2S_BLOCK_DURATION_US * I2S_SAMPLE_RATE_HZ / 1000000UL)  // = 125
#define I2S_BLOCK_TICKS         (I2S_FRAMES_PER_BLOCK * I2S_TICKS_PER_FRAME)  // = 8000
#define I2S_BYTES_PER_BLOCK     (I2S_FRAMES_PER_BLOCK * I2S_BYTES_PER_FRAME)  // = 500
#define MAX_STEP_FREQ_HZ        200000UL
#define I2S_MAX_PULSES_PER_BLOCK ((I2S_BLOCK_DURATION_US * MAX_STEP_FREQ_HZ / 1000000UL) + 1)  // = 101
```

2. `i2s_fill.h` - Fix struct:
```c
struct i2s_fill_state {
  uint32_t tick_pos;              // Was uint16_t - need 32-bit for overflow handling
  uint16_t remaining_high_ticks;
  uint16_t remaining_low_ticks;
  uint16_t pulse_positions[I2S_BLOCK_COUNT][I2S_MAX_PULSES_PER_BLOCK];  // Per-block tracking
  uint8_t pulse_count[I2S_BLOCK_COUNT];  // Per-block count
};
```

---

### Phase 2: Implement No-Memset Per-Bit Clearing [HIGH]

**Files to modify:**
- `src/pd_esp32/i2s_fill.cpp`

**Changes:**
- Remove `memset(buf, 0, I2S_BYTES_PER_BLOCK)` at start of fill
- Clear only bits at positions recorded in `pulse_positions[block]`
- Record new pulse positions per block

**Algorithm:**
```c
bool i2s_fill_buffer(StepperQueueBase* q, uint8_t* buf, uint8_t block, 
                     struct i2s_fill_state* state) {
  // Step 1: Clear previous pulses for this block only
  for (uint8_t i = 0; i < state->pulse_count[block]; i++) {
    uint16_t frame = state->pulse_positions[block][i];
    // Clear only this stepper's bits (not entire frame)
    buf[frame * I2S_BYTES_PER_FRAME + 0] &= ~step_bit_mask;
    buf[frame * I2S_BYTES_PER_FRAME + 1] &= ~step_bit_mask;
    buf[frame * I2S_BYTES_PER_FRAME + 2] &= ~step_bit_mask;
    buf[frame * I2S_BYTES_PER_FRAME + 3] &= ~step_bit_mask;
  }
  state->pulse_count[block] = 0;
  
  // Step 2-N: Fill as before, but record positions
  // ... when writing a pulse:
  state->pulse_positions[block][state->pulse_count[block]++] = frame_idx;
}
```

**Why this matters:** In multi-stepper mode, multiple steppers share the same DMA buffer. Bulk-clearing would erase other steppers' pulses.

---

### Phase 3: Implement Single-Stepper Bit-Level Resolution [HIGH]

**Files to modify:**
- `src/pd_esp32/i2s_fill.cpp`
- `src/pd_esp32/i2s_fill.h`

**Changes:**
- Track `tick_pos` at 2-tick granularity (each bit = 2 ticks)
- Write N consecutive 1-bits for pulse width
- Support pulse spanning frame boundaries

**Algorithm for single-stepper:**
```c
static void write_high_bits(uint8_t* buf, uint32_t start_tick, uint16_t num_ticks) {
  uint16_t start_bit = start_tick / 2;  // Each bit = 2 ticks
  uint16_t end_bit = (start_tick + num_ticks) / 2;
  
  for (uint16_t bit = start_bit; bit < end_bit; bit++) {
    uint16_t frame = bit / 32;
    uint16_t bit_in_frame = bit % 32;
    uint8_t byte_in_frame = bit_in_frame / 8;
    uint8_t bit_in_byte = 7 - (bit_in_frame % 8);  // MSB first
    buf[frame * 4 + byte_in_frame] |= (1 << bit_in_byte);
  }
}
```

**This enables:** Step rates up to ~200kHz (vs ~40kHz for frame-level)

---

### Phase 4: Update PC Tests for Per-Bit Clearing [HIGH]

**Files to modify:**
- `extras/tests/pc_based/test_21.cpp`

**Tests to update:**
- Remove memset in test setup - let fill handle clearing
- Verify only recorded positions are cleared

**New tests to add:**

1. `test_fill_clears_only_recorded_positions`
   - Pre-fill buffer with 0xFF
   - Run fill with 3 pulses
   - Verify only those 3 frames are modified
   - Run fill with empty queue
   - Verify those 3 frames are cleared, rest unchanged

2. `test_fill_preserves_other_bytes`
   - Simulate multi-stepper by setting bits in "other slots"
   - Run fill for "our slot"
   - Verify "other slots" unchanged

3. `test_fill_single_stepper_bit_resolution`
   - Test pulse placement at tick 0, 2, 4, 6 (bit-level)
   - Verify correct bit positions in buffer

4. `test_fill_variable_pulse_width`
   - Test 16 ticks (8 bits)
   - Test 64 ticks (32 bits = 1 frame)
   - Test 128 ticks (64 bits = 2 frames)

---

### Phase 5: Add Engine-Level I2S Configuration API [MEDIUM]

**Files to modify:**
- `src/FastAccelStepper.h`
- `src/FastAccelStepper.cpp`
- `src/pd_esp32/i2s_manager.h`
- `src/pd_esp32/i2s_manager.cpp`

**New structs in FastAccelStepper.h:**
```cpp
struct I2sSingleStepperConfig {
  uint8_t data_pin;
  uint8_t bclk_pin;
  uint8_t cpu_core;           // 0 or 1 for I2S fill task affinity
  uint16_t pulse_width_ticks; // Default: 64 (4µs)
};

struct I2sMultiStepperConfig {
  uint8_t data_pin;
  uint8_t bclk_pin;
  uint8_t ws_pin;             // Required for demux IC latch
  uint8_t signals_per_stepper;  // 1=STEP_ONLY, 2=STEP_DIR, 3=STEP_DIR_ENABLE
  uint8_t cpu_core;
  uint16_t pulse_width_ticks;
};
```

**New API in FastAccelStepperEngine:**
```cpp
bool initI2sSingleStepper(const I2sSingleStepperConfig& cfg);
bool initI2sMultiStepper(const I2sMultiStepperConfig& cfg);
```

**Behavior:**
- Must be called before `stepperConnectToPin()`
- `initI2sSingleStepper()` sets mode to single-stepper (1 I2S = 1 stepper)
- `initI2sMultiStepper()` sets mode to multi-stepper (1 I2S = up to 16 steppers)

---

### Phase 6: Implement Multi-Stepper Slot Management [MEDIUM]

**Files to modify:**
- `src/pd_esp32/i2s_manager.h`
- `src/pd_esp32/i2s_manager.cpp`
- `src/pd_esp32/esp32_queue.h`
- `src/pd_esp32/StepperISR_esp32_i2s.cpp`

**New members in I2sManager:**
```cpp
class I2sManager {
private:
  uint32_t _allocated_slots;      // Bitmap of used slots (32 bits = 32 slots)
  bool _is_multi_stepper;
  uint8_t _signals_per_stepper;
  
public:
  int8_t allocateSlots(uint8_t count);  // Returns base slot or -1 if full
  void freeSlots(uint8_t base_slot, uint8_t count);
  bool isMultiStepper() const { return _is_multi_stepper; }
};
```

**Slot-to-bit mapping (from design doc):**
```c
// ESP32 I2S outputs MSB-first within each 16-bit half (L then R)
byte_in_frame = slot / 8;          // 0..3
bit_in_byte = 7 - (slot % 8);      // MSB first
byte_in_buf = frame_index * 4 + byte_in_frame;
bit_mask = 1 << bit_in_byte;
```

**Example - Slot 5 (6th stepper bit):**
- `byte_in_frame = 5 / 8 = 0` → L_hi
- `bit_in_byte = 7 - (5 % 8) = 2`
- `bit_mask = 0x04`
- Set: `buf[frame * 4 + 0] |= 0x04;`
- Clear: `buf[frame * 4 + 0] &= ~0x04;`

**StepperQueue new members:**
```cpp
#ifdef SUPPORT_ESP32_I2S
  int8_t _i2s_step_slot;
  int8_t _i2s_dir_slot;    // -1 if using conventional GPIO
  int8_t _i2s_enable_slot; // -1 if using conventional GPIO
  // ... existing members
#endif
```

---

### Phase 7: Implement DIR/ENABLE Bit Management [MEDIUM]

**Files to modify:**
- `src/pd_esp32/i2s_manager.h`
- `src/pd_esp32/StepperISR_esp32_i2s.cpp`

**Engine-side variables (from HUMAN feedback in design doc):**
```c
// In I2sManager or per-stepper state:
uint32_t bit_mask;       // Which bits are static (enable/direction)
uint32_t bit_target;     // Target value for bits where bit_mask is 1
uint32_t bit_actual[3];  // Actual value of those bits per block
```

**How it works:**
- `bit_mask`: OR of all DIR/ENABLE slot bitmasks
- `bit_target`: Current desired state of those bits
- `bit_actual[b]`: What's currently in block b

**On DMA block completion:**
```cpp
void applyStaticBits(uint8_t completed_block) {
  if (bit_actual[completed_block] != bit_target) {
    // Bulk update all frames in the block
    for (uint16_t f = 0; f < I2S_FRAMES_PER_BLOCK; f++) {
      uint32_t* frame = (uint32_t*)&_bufs[completed_block][f * 4];
      *frame = (*frame & ~bit_mask) | (bit_target & bit_mask);
    }
    bit_actual[completed_block] = bit_target;
  }
}
```

**Service functions (low-level):**
```cpp
void setSlotBit(uint8_t block, uint16_t frame, uint8_t slot, bool value);
bool getSlotBit(uint8_t block, uint16_t frame, uint8_t slot);
```

**Note:** Timing constraints (DIR before STEP) are caller's responsibility.

---

### Phase 8: Add Configurable Pulse Width [MEDIUM]

**Files to modify:**
- `src/pd_esp32/i2s_fill.h`
- `src/pd_esp32/i2s_fill.cpp`
- `src/FastAccelStepper.h`
- `src/FastAccelStepper.cpp`

**New API:**
```cpp
void FastAccelStepper::setI2sPulseWidth(uint16_t ticks);
```

**Implementation:**

| Mode | Calculation | Min | Max |
|------|-------------|-----|-----|
| Single-stepper | N consecutive 1-bits (N = ticks / 2) | 16 ticks (1µs) | 64 ticks (4µs) recommended |
| Multi-stepper | Frames = (ticks + 63) / 64 | 64 ticks (1 frame) | Unlimited |

**Default:** 64 ticks (4µs = 32 bits = one full frame)

---

### Phase 9: Add Multi-Stepper PC Tests [MEDIUM]

**Files to modify:**
- `extras/tests/pc_based/test_21.cpp`

**New tests:**

1. `test_multi_stepper_slot_mapping`
   - Write to slot 0, verify bit 7 of byte 0
   - Write to slot 5, verify bit 2 of byte 0
   - Write to slot 15, verify bit 0 of byte 1 (R channel start)
   - Write to slot 31, verify bit 0 of byte 3

2. `test_multi_stepper_two_steppers_no_interference`
   - Create two simulated steppers with slots 0 and 5
   - Fill buffer for stepper A (slot 0)
   - Fill buffer for stepper B (slot 5)
   - Verify both have correct pulses, no overlap

3. `test_multi_stepper_forceStop_preserves_others`
   - Two steppers running
   - Call forceStop on stepper A
   - Verify stepper A's bits cleared
   - Verify stepper B's bits unchanged

4. `test_multi_stepper_shared_buffer`
   - All three blocks filled by multiple steppers
   - Verify correct interleaving
   - Verify no race conditions (simulate ISR timing)

---

### Phase 10: Implement Improved ForceStop [MEDIUM]

**Files to modify:**
- `src/pd_esp32/StepperISR_esp32_i2s.cpp`
- `src/pd_esp32/i2s_manager.cpp`

**Single-stepper mode:**
```cpp
void StepperQueue::forceStop_i2s_single() {
  // Stop DMA
  i2s_channel_disable(I2sManager::instance().channel());
  
  // Clear all buffers (ok since only one stepper)
  for (int i = 0; i < I2S_BLOCK_COUNT; i++) {
    memset(I2sManager::instance().blockBuf(i), 0, I2S_BYTES_PER_BLOCK);
  }
  
  // Clear fill state
  _fill_state = {};
  
  // Restart DMA
  i2s_channel_enable(I2sManager::instance().channel());
}
```

**Multi-stepper mode:**
```cpp
void StepperQueue::forceStop_i2s_multi() {
  // Clear only this stepper's bits in all blocks
  for (int b = 0; b < I2S_BLOCK_COUNT; b++) {
    for (int i = 0; i < _fill_state.pulse_count[b]; i++) {
      uint16_t frame = _fill_state.pulse_positions[b][i];
      uint8_t* buf = I2sManager::instance().blockBuf(b);
      // Clear only this stepper's slot bits
      clearSlotBits(buf, frame, _i2s_step_slot);
    }
    _fill_state.pulse_count[b] = 0;
  }
  
  // Clear state (but don't touch other steppers!)
  _fill_state.tick_pos = 0;
  _fill_state.remaining_high_ticks = 0;
  _fill_state.remaining_low_ticks = 0;
}
```

**Note:** Clearing bits in the currently-transmitting block is a benign race - worst case one extra pulse. This is consistent with forceStop contract.

---

### Phase 11: Create Hardware Test Sketches [LOW]

**New directories:**
- `extras/tests/esp32_hw_based/i2s_zero_copy_test/`
- `extras/tests/esp32_hw_based/i2s_pcnt_test/`
- `extras/tests/esp32_hw_based/i2s_logic_analyzer_test/`
- `extras/tests/esp32_hw_based/i2s_74hc595_test/`

#### Stage 0: Zero-Copy Verification

**Purpose:** Verify `i2s_channel_write()` passes buffer pointer to DMA without copying.

**Test:**
```cpp
// i2s_zero_copy_test.ino
void loop() {
  uint8_t* buf = I2sManager::instance().blockBuf(0);
  
  // Write buffer to DMA
  i2s_channel_write(chan, buf, I2S_BYTES_PER_BLOCK, &written, portMAX_DELAY);
  
  // Modify buffer AFTER write returns
  buf[0] = 0xFF;  // This should appear in output if zero-copy
  
  delay(10);  // Wait for DMA to transmit
  
  // Logic analyzer: did 0xFF appear at start of block?
}
```

**If not zero-copy:** Use direct DMA descriptor approach instead of IDF driver.

#### Stage 1: Single-Stepper PCNT Validation

**Hardware:** I2S DATA_OUT → PCNT input + oscilloscope

**Tests:**
- Step count matches commanded steps
- Max frequency (~200 kHz)
- Pulse width timing
- Direction pin timing

#### Stage 2: Multi-Stepper Logic Analyzer

**Hardware:** I2S DATA_OUT, BCLK, WS → logic analyzer

**Tests:**
- Decode I2S stream, verify bit-slot timing
- Multiple steppers interleaved correctly
- DIR/ENABLE bit stability during STEP pulses

#### Stage 3: 74HC595 + PCNT End-to-End

**Hardware:** Full demux chain (74HC595) → PCNT

**Tests:**
- End-to-end step count validation
- Multiple simultaneous steppers
- ForceStop behavior

---

## Execution Order

```
Phase 1 (Constants) ──┐
                      ├──► Phase 4 (PC Tests) ──► Phase 9 (Multi Tests)
Phase 2 (No Memset) ──┘
                      │
Phase 3 (Bit-Level) ──┘

Phase 5 (Engine API) ──┬──► Phase 6 (Slots) ──► Phase 7 (DIR/ENABLE)
                       │
Phase 8 (Pulse Width) ─┘

Phase 10 (ForceStop) ──► Phase 11 (Hardware Tests)
```

**Recommended sequence:**
1. Phase 1 → Phase 2 → Phase 4 (verify with tests)
2. Phase 3 (build on Phase 2)
3. Phase 5 → Phase 6 → Phase 7 (multi-stepper foundation)
4. Phase 8 (polish)
5. Phase 9 (test multi-stepper)
6. Phase 10 (ForceStop)
7. Phase 11 (hardware validation)

---

## File Structure (Design Doc Reference)

### New Files (some may exist partially)
```
src/pd_esp32/
  i2s_constants.h           — I2S timing constants
  i2s_manager.h             — I2sManager class declaration
  i2s_manager.cpp           — I2sManager implementation (DMA, ISR, task)
  i2s_fill.h                — i2s_fill_state struct, fill_i2s_buffer() declaration
  i2s_fill.cpp              — fill_i2s_buffer() implementation
  StepperISR_esp32_i2s.cpp  — StepperQueue I2S methods

extras/tests/pc_based/
  test_21.cpp               — PC-based test (exists, needs updates)
```

### Modified Files
```
src/fas_arch/common.h               — add I2S to FasDriver enum (if needed)
src/fas_arch/common_esp32_idf5.h    — SUPPORT_ESP32_I2S, QUEUES_I2S (exists)
src/pd_esp32/esp32_queue.h          — I2S-specific members (exists, needs expansion)
src/pd_esp32/esp32_queue.cpp        — I2S dispatch branches (exists)
src/FastAccelStepper.h              — I2S config structs, API
src/FastAccelStepper.cpp            — initI2sXxx() implementation
```

---

## Notes

- **WS pin:** Currently hardcoded to `I2S_GPIO_UNUSED`. Multi-stepper requires WS for 74HC595 latch.
- **DMA descriptors:** Currently 16 × 500 frames. Design doc says 2 × 125 frames minimum. May need adjustment.
- **ISR safety:** `i2s_fill_buffer` is called from ISR context. Must be `IRAM_ATTR` and avoid mutexes.
- **Core affinity:** I2S fill task should be configurable (Phase 5).
