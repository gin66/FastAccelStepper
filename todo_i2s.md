# I2S Driver – Status Summary

## Current Status

**Pulse width:** Changed from 1µs to **2µs** for EMC compliance (both L and R bytes = 0xFF)
**Constants updated:** BCLK=4MHz, stereo 8-bit, frame=4µs, I2S_TICKS_PER_FRAME=64

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
| `src/pd_esp32/i2s_manager.h` | ✅ Singleton I2sManager |
| `src/pd_esp32/i2s_manager.cpp` | 🔄 Needs DMA callback integration |
| `src/pd_esp32/i2s_constants.h` | ✅ Shared timing constants |
| `src/pd_esp32/StepperISR_esp32_i2s.cpp` | 🔄 Needs block boundary handling |
| `src/pd_test/test_queue.h` | ✅ Added _i2s_tick_pos field |
| `CMakeLists.txt` | ✅ esp_driver_i2s dependency |
| `pio_espidf/StepperDemo/src/StepperDemo.cpp` | ✅ M9 config (GPIO32) |

## Test Infrastructure

### test_21: DMA Tests (IN PROGRESS)

**Part 1: Low-Level Infrastructure Tests ✅ COMPLETE**
- Pure DMA infrastructure tests (no driver code)
- DMA callback invocation
- Triple buffer rotation (0→1→2→0)
- Write block availability
- Buffer clearing on consume
- Bit stream pulse detection
- 10 rounds × 7 pulses test
- **DMA callback only analyzes buffer content (no driver invocation)**

**Part 2: Driver Integration Tests (TODO)**
- DMA callback must invoke driver's `fill_i2s_buffer()`
- Process commands through queue
- Verify pulse timing from DMA buffers
- Test multi-block pulse sequences
- Test edge cases:
  - 255 pulses at 65535 ticks
  - 100 pulses at 5000 ticks
  - Slow pulses across block boundaries
  - Acceleration/deceleration ramps
- Verify:
  - Correct pulse positions in buffer
  - All pulses eventually processed
  - `_i2s_tick_pos` correctly carried across blocks

## Next Steps

### 1. **Driver: Block Boundary Handling (High Priority)**
**Status:** Partially implemented
**Issue:** When a pulse's tick position exceeds `I2S_BLOCK_TICKS`, the driver needs to:
- Track remaining position in `_i2s_tick_pos` for next fill
- Handle the 3 cases correctly:
  1. HIGH phase doesn't fit → wait for next block
  2. HIGH phase fits, steps > 1 → write pulse, decrement steps, save tick_pos, wait
  3. HIGH phase fits, steps = 1 → write pulse, consume entry

**Files to update:**
- `src/pd_esp32/StepperISR_esp32_i2s.cpp`: Implement tick-based tracking
- `src/pd_test/test_queue.h`: Already has `_i2s_tick_pos`

### 2. **Driver: DMA Callback Integration (High Priority)**
**Status:** Not implemented
**Requirement:**
- Register I2S DMA TX_DONE callback using `i2s_channel_register_event_callback()`
- On callback: advance `_dma_block`, signal buffer free
- Coordinate with `fill_i2s_buffer()` for block availability

**Files to update:**
- `src/pd_esp32/i2s_manager.h`: Add callback registration
- `src/pd_esp32/i2s_manager.cpp`: Implement callback handler

### 3. **Driver: Integration Tests (Medium Priority)**
Create test_22 for driver-level tests:
- Process commands through queue
- Verify pulse timing from DMA buffers
- Test multi-block pulse sequences
- Test edge cases (65535 ticks, 255 steps)

### 4. **Driver: Multi-Motor Support (Future)**
**Status:** Architecture defined, not implemented
- Each motor uses different bit position in buffer
- `write_pulse_to_buffer()` sets specific bit based on motor slot
- All motors share same triple buffer

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

Before hardware testing:
- [ ] Block boundary handling verified in tests
- [ ] DMA callback integration working
- [ ] Multi-block pulse sequences correct
- [ ] Edge cases (65535 ticks, slow pulses) working

Hardware test plan:
- [ ] Single motor at various speeds (1kHz - 200kHz)
- [ ] Acceleration/deceleration ramps
- [ ] Long-running stability (no WDT)
- [ ] Verify no 2-second delay issue
