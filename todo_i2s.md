# I2S Driver – Status Summary

## Current Status

**Pulse width:** Changed from 1µs to **2µs** for EMC compliance (both L and R bytes = 0xFF)
**Constants updated:** BCLK=4MHz, stereo 8-bit, frame=4µs, I2S_TICKS_PER_FRAME=64

## Architecture (Confirmed Working)

- **BCLK = 4 MHz, stereo 8-bit**
- **Frame = L-byte + R-byte = 4µs = 64 ticks**
- **I2S_FRAMES_PER_TASK = 1000** (4ms buffer per task)
- **I2S_BYTES_PER_TASK = 2000** (1000 frames × 2 bytes)
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
| `src/pd_esp32/i2s_manager.cpp` | ✅ init, clearWorkBuf, flush |
| `src/pd_esp32/i2s_constants.h` | ✅ Shared timing constants |
| `src/pd_esp32/StepperISR_esp32_i2s.cpp` | ✅ 4µs pulse implementation |
| `CMakeLists.txt` | ✅ esp_driver_i2s dependency |
| `pio_espidf/StepperDemo/src/StepperDemo.cpp` | ✅ M9 config (GPIO32) |

## Critical Issues to Fix

### 1. **DMA Buffer Tracking (Critical)**
**Status:** Not implemented
**Guidance:**
- The I2S driver needs a `write_pointer` to track how far the DMA has read
- When DMA read pointer meets write_pointer, there are no more commands
- Write pointer can be half-buffer pointer
- If DMA reads from unprepared buffer, all step bits must be 0 (to prevent unwanted movement)
- **All bits for the step pin in the I2S buffer must be 0 when idle**

**Implementation needed:**
- Add `_i2s_write_pointer` field to track DMA read position
- On each `fill_i2s_buffer()` call, clear old pulses up to write_pointer
- Ensure buffer beyond write_pointer is all zeros

### 2. **Command Processing Loop (Critical)**
**Status:** test_21 hangs, process_all loop missing
**Guidance:**
- Follow test_18 pattern: loop until all commands are done
- `process_all()` should keep calling `fill_i2s_buffer()` until queue empty AND drain complete

**Implementation needed:**
- In test_21: implement `process_all()` similar to test_18
- Loop condition: `read_idx != next_write_idx || _i2s_drain > 0`

### 3. **Pause Entry Insertion for Long Pulse Periods**
**Status:** Implemented but test hangs
**Guidance:**
- When a pulse doesn't fit in buffer, add a PAUSE entry for remaining ticks
- Then continue with remaining steps on next fill
- For 255 pulses at 65535 ticks (~1s total):
  - Pulse at frame 0
  - Next pulse at frame 1023 (beyond buffer)
  - Insert pause for remaining ticks to buffer end
  - Continue with remaining steps on next fill

**Current implementation issues:**
- Pause insertion code exists but needs verification
- Queue shifting logic needs testing
- Need to ensure wp (next_write_idx) is properly updated

### 4. **Test Hang Issue**
**Status:** test_21 hangs on execution
**Possible causes:**
- Infinite loop in DMA simulation
- Queue entry corruption
- Missing termination condition

## Known Issues

1. **Watchdog timeout on CPU1:** Not yet addressed - need to optimize `fill_i2s_buffer()` further or use IDF5 watchdog API
2. **~2 second delay after `f`:** Needs investigation - queue filling or drain timing issue
3. **PCNT = 0:** `gpio_iomux_in()` override not yet fixed

## Test Requirements

### test_21: I2S Driver Tests
- **Process all commands loop:** Implement `process_all()` following test_18 pattern
- **DMA simulation:** Simulate DMA read pointer advancing
- **Edge cases:**
  - 255 pulses at 65535 ticks (pulse at start + pause pattern)
  - 100 pulses at 5000 ticks (~12 pulses/buffer)
  - Ramps from slow to fast
- **Verification:**
  - Correct pulse positions in buffer
  - All pulses eventually processed
  - Buffer is cleared after each fill

## Future Work

1. **Hardware testing:** Test on real ESP32 to verify:
   - No watchdog timeouts
   - Correct pulse timing
   - No 2-second delay issue
2. **Multi-stepper (demux mode):** Design bit-slot system for multiple steppers per I2S peripheral
3. **Performance optimization:** Reduce buffer processing time to prevent WDT
4. **GPIO fix:** Implement correct gpio_iomux_in() for PCNT
