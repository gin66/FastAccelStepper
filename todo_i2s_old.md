# I2S Driver – Status Summary

## Current Status

**Pulse width:** Changed from 1µs to **2µs** for EMC compliance (both L and R bytes = 0xFF)
**Constants updated:** BCLK=4MHz, stereo 8-bit, frame=4µs, I2S_TICKS_PER_FRAME=64
**Ready for HW testing:** All PC-based tests pass (28/28)

## Architecture (Confirmed Working)

- **BCLK = 4 MHz, stereo 8-bit**
- **Frame = L-byte + R-byte = 4µs = 64 ticks**
- **I2S_BLOCK_COUNT = 3** (triple buffer)
- **I2S_FRAMES_PER_BLOCK = 250** (~1ms per block)
- **I2S_BLOCK_TICKS = 16000** (250 frames × 64 ticks)
- **DATA_OUT (GPIO32):** Serial pin, transmits bits one at a time
- **Step pulse:** 4µs HIGH (both L and R bytes = 0xFF = full frame HIGH)

## Implementation Files

| File | Status |
|---|---|
| `src/fas_arch/common_esp32_idf5.h` | ✅ Added SUPPORT_ESP32_I2S |
| `src/fas_arch/common_esp32.h` | ✅ QUEUES_I2S in NUM_QUEUES |
| `src/pd_esp32/esp32_queue.h` | ✅ I2S fields + _i2s_pulse_positions |
| `src/pd_esp32/esp32_queue.cpp` | ✅ I2S dispatch + StepperTask I2S block |
| `src/pd_esp32/i2s_manager.h` | ✅ DMA callback registration |
| `src/pd_esp32/i2s_manager.cpp` | ✅ DMA callback integration |
| `src/pd_esp32/i2s_fill.h` | ✅ Uses StepperQueueBase directly |
| `src/pd_esp32/i2s_fill.cpp` | ✅ Block boundary handling |
| `src/pd_esp32/i2s_constants.h` | ✅ Shared timing constants |
| `src/pd_esp32/StepperISR_esp32_i2s.cpp` | ✅ Uses i2s_fill_state struct |
| `src/pd_test/test_queue.h` | ✅ Added _i2s_tick_pos field |
| `extras/tests/pc_based/test_21.cpp` | ✅ 28 tests (all pass) |
| `CMakeLists.txt` | ✅ esp_driver_i2s dependency |
| `pio_espidf/StepperDemo/src/StepperDemo.cpp` | ✅ M9 config (GPIO32) |

## Test Infrastructure

### test_21: DMA + Fill Function Tests ✅ COMPLETE

**Part 1: Low-Level Infrastructure Tests ✅**
- DMA callback invocation
- Triple buffer rotation (0→1→2→0)
- Write block availability
- Buffer clearing on consume
- Bit stream pulse detection
- 10 rounds × 7 pulses test

**Part 2: Fill Function Tests ✅**
- Single/multi step commands
- Pause commands
- Step + pause + step sequences
- Block boundary handling
- Partial steps across blocks
- Carry ticks for long pauses

**Part 3: Edge Cases ✅**
- Empty queue
- Minimum speed (128 ticks)
- Max steps (255)
- Long pause (50000 ticks)
- Pulse at exact block boundary
- Multiple mixed commands
- Consecutive partial steps
- Max ticks (65535) - single and multiple steps
- Max pause (65535 ticks)

## Completed Work

### 1. DMA Callback Integration ✅
- Added `i2s_tx_done_callback()` - IRAM_ATTR callback for ESP-IDF
- Added `registerDmaCallback(cb, user_data)` - stores callback pointer
- Added `handleTxDone()` - advances DMA block and invokes user callback
- Added `startDma()` - preloads buffers and enables I2S channel
- Callback registered with `i2s_channel_register_event_callback()`

### 2. Queue Type Unification ✅
- Removed redundant `i2s_queue_entry` and `i2s_stepper_queue` structs
- `i2s_fill_buffer()` now takes `StepperQueueBase*` directly
- Uses `queue_entry` from `fas_queue/base.h` - no copying needed
- Simpler architecture, less memory overhead

### 3. Block Boundary Handling ✅
- `remaining_high_ticks` - tracks pending pulse position across blocks
- `remaining_low_ticks` - tracks pause remainder across blocks
- Handles multi-block pulses (e.g., 65535 ticks = 5 blocks)
- Correctly decrements step count when placing pending pulses

## Known Issues

1. **Watchdog timeout on CPU1:** Need to optimize `fill_i2s_buffer()` or use IDF5 watchdog API
2. **~2 second delay after `f`:** Needs investigation
3. **PCNT = 0:** `gpio_iomux_in()` override not fixed

## Multi-Motor Triple Buffer Architecture

For multiple motors sharing one I2S peripheral:

### Fixed Triple Buffer Layout
- **Buffer (n):** Currently being read by DMA
- **Buffer (n+1):** Prepared (ready for DMA)
- **Buffer (n+2):** Currently being filled by motor drivers

### Buffer Filling Sequence
1. Engine tells I2S driver which buffer (n) is currently read by DMA
2. Buffer (n+1) is prepared (already filled)
3. Buffer (n+2) is being filled by all motors
4. Some motors may **hang** if remaining space insufficient for step pulse
5. DMA finishes buffer (n), advances to buffer (n+1)
6. Old buffer (n) becomes free = new (n+3) = (n) modulo 3
7. Motors that were hanging can now **spill over** into free buffer
8. After spill-over, buffer (n+2) is marked as prepared

### Key Implementation Points
- Three fixed buffers (not ring buffer per motor)
- Motor drivers track own fill position across boundaries
- Spill-over handling: if pulse doesn't fit, wait for next buffer
- Engine coordinates buffer state transitions
- All motors fill same shared buffer (different GPIO bits)

## Hardware Testing Checklist

Completed integration work:
- [x] Block boundary handling verified in tests
- [x] DMA callback integration working
- [x] Multi-block pulse sequences correct
- [x] Edge cases (65535 ticks, slow pulses) working
- [x] StepperISR_esp32_i2s.cpp uses i2s_fill_state struct (no more old variables)

Hardware test plan:
- [ ] Single motor at various speeds (1kHz - 200kHz)
- [ ] Acceleration/deceleration ramps
- [ ] Long-running stability (no WDT)
- [ ] Verify no 2-second delay issue
