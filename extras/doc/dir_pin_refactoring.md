# Direction Pin Handling Refactoring

## Executive Summary

This document describes a refactoring of direction and enable pin handling in FastAccelStepper. The key changes are:

1. **DIR-external pins are handled ONLY in `addQueueEntry()`** - never in queue/ISR context
2. **`addQueueEntry()` inserts sufficient pause before a direction change** - ensures existing pulse drivers can be reused without modification
3. **`toggle_dir` mechanism remains** - AVR/SAM/MCPWM code unchanged

**Key Insight**: By having `addQueueEntry()` ensure sufficient pause before a direction change appears in the queue, **existing pulse drivers are reused without change**. The drivers continue to use `toggle_dir` as before - timing is now handled upstream.

**Main Impact**:
- **RMT**: Removal of the injected pause workaround
- **I2S**: Clean implementation without synchronization complexity
- **AVR/SAM/MCPWM**: No changes required

**Processing Model**:

| Pin Type | Processing Context | Mechanism |
|----------|-------------------|-----------|
| **Enable pins** | `addQueueEntry()` | 4ms delay acceptable, `setEnablePinState()` or callback |
| **Direction (External)** | `addQueueEntry()` ONLY | `repeat_entry` mechanism, callback to external hardware |
| **Direction (I2S/GPIO)** | `addQueueEntry()` inserts pause, Queue/ISR applies | `toggle_dir` + pause ensures safe timing |

**Forbidden Combination**: RMT step pin + I2S direction pin (independent hardware blocks, no synchronization).

---

## 1. Current State

### 1.1 Direction Pin Toggle Mechanism

The current implementation uses a `toggle_dir` flag in the `queue_entry` structure:

```cpp
struct queue_entry {
  uint8_t steps;
  uint8_t toggle_dir : 1;      // Flag: toggle direction pin before this entry
  uint8_t countUp : 1;
  uint8_t moreThanOneStep : 1;
  uint8_t hasSteps : 1;
  // ...
};
```

When `addQueueEntry()` detects a direction change between the previous and new command, it sets `toggle_dir = 1`. The ISR or driver then toggles the direction pin when processing that entry.

### 1.2 Platform Implementations

| Platform | Toggle Location | Mechanism |
|----------|-----------------|-----------|
| AVR | ISR | Direct port register XOR |
| SAM | ISR | Direct port register XOR |
| Pico | PIO | PIO state machine tracks state |
| ESP32 RMT | ISR | `LL_TOGGLE_PIN` macro using `gpio_ll_*` |
| ESP32 I2S | Fill callback | `LL_TOGGLE_PIN` macro |

### 1.3 Current Code Flow

```
addQueueEntry():
  1. Compare new dir with queue_end.dir
  2. If queue is empty and not running:
       - Toggle direction pin directly
       - Update queue_end.dir
  3. Else if direction change needed:
       - Set toggle_dir = 1 in entry
       - Update queue_end.dir

ISR/Driver:
  1. Check entry.toggle_dir
  2. If set: toggle direction pin
  3. Process steps
```

**Key Point**: `addQueueEntry()` already checks if the queue is running. If not running, it toggles the direction pin directly rather than setting `toggle_dir`.

---

## 2. Identified Issues

### 2.1 Buffering Timing Problem

For ESP32 RMT and I2S drivers, pulses are buffered in hardware/DMA before being emitted to GPIO pins. The `toggle_dir` approach has a fundamental timing ambiguity:

```
Timeline of events:
┌─────────────────────────────────────────────────────────────────┐
│ DMA Buffer:  [Step 1][Step 2][Step 3][Step 4][Step 5]...       │
│                              ↑                                  │
│                         toggle_dir processed here               │
│                                                                 │
│ Actual pulses emitted:                                          │
│   Step 1, Step 2, Step 3  →  DIR TOGGLE  →  Step 4, Step 5     │
│                              ↑                                  │
│                    Which direction for Step 3?                  │
└─────────────────────────────────────────────────────────────────┘
```

**Problem**: The toggle happens at an arbitrary point within the buffer. Steps already in the buffer may be emitted with the wrong direction.

### 2.2 I2S MUX Special Case

For I2S multiplexed mode, both step and direction pins are controlled via the I2S bitstream. The direction is encoded in a bit mask that is only applied when the I2S buffer is empty:

- I2S buffer fill happens in callback
- Direction bit mask changes when processing a toggle_dir entry
- **Critical**: Bit mask change only takes effect when buffer transitions from empty to non-empty

**Problem**: Without proper timing, the direction mask may change mid-buffer, causing steps to be output with inconsistent direction bits.

### 2.3 RMT Chunk Boundaries

The RMT driver fills memory in chunks. When `toggle_dir` is processed:

```cpp
// In StepperISR_idf4_esp32_rmt.cpp
if (entry[rp & QUEUE_LEN_MASK].toggle_dir) {
    LL_TOGGLE_PIN(dirPin);
    entry[rp & QUEUE_LEN_MASK].toggle_dir = false;
}
```

This toggle happens when pulling a new command, but RMT may still be outputting pulses from the previous chunk.

**Current Workaround**: The RMT driver checks if pulses are in the buffer. If yes, it will insert a buffer with no steps. **This breaches the driver contract** by modifying timing without the upper layer's knowledge.

### 2.4 External Direction Pin Complexity

External direction pins (controlled via callbacks to external hardware like I/O expanders) require special handling:

```cpp
#if defined(SUPPORT_EXTERNAL_DIRECTION_PIN)
if (toggle_dir && (dirPin & PIN_EXTERNAL_FLAG)) {
    repeat_entry = toggle_dir;  // Defer to task context
    toggle_dir = false;
}
#endif
```

This adds conditional logic in multiple places and the `repeat_entry` mechanism is tightly coupled to `toggle_dir`.

### 2.5 Code Duplication

Each platform implements its own toggle logic:

| Platform | Code Location | Implementation |
|----------|---------------|----------------|
| AVR | `avr_queue.cpp:31-32` | Macro `Stepper_ToggleDirection` |
| SAM | `sam_queue.cpp:159,189` | Direct `*port ^= mask` |
| ESP32 RMT | Multiple files | `LL_TOGGLE_PIN(dirPin)` |
| ESP32 I2S | `i2s_fill.cpp:52,138` | `LL_TOGGLE_PIN(q->dirPin)` |

No unified abstraction exists, making maintenance and testing difficult.

---

## 3. Proposed Solution

### 3.1 Core Concept: Pause Insertion for Buffered Platforms

The `toggle_dir` mechanism remains unchanged. The key improvement is that `addQueueEntry()` now ensures sufficient pause before a direction change, allowing existing pulse drivers to work correctly without modification.

```
┌─────────────────────────────────────────────────────────────────┐
│                    addQueueEntry() Context                       │
│                     (~4ms processing time)                       │
├─────────────────────────────────────────────────────────────────┤
│  Enable pins (ALWAYS)                                            │
│  - Acceptable delay for enable/disable                          │
│  - Uses setEnablePinState() or callback for external            │
├─────────────────────────────────────────────────────────────────┤
│  Direction - External pins (ONLY HERE - NEVER in Queue/ISR)      │
│  - Callback to external hardware (I/O expanders)                │
│  - Uses repeat_entry mechanism                                  │
│  - External hardware timing managed externally                   │
│  - toggle_dir cleared, repeat_entry set instead                 │
├─────────────────────────────────────────────────────────────────┤
│  Direction - I2S/GPIO (PAUSE INSERTION)                          │
│  - Query platform for required pause duration via               │
│    getDrainPauseTicksForDirChange()                             │
│  - If pause > 0: insert pause command before dir change         │
│  - Set toggle_dir = 1 in entry as before                        │
│  - THIS IS THE KEY: Pause ensures driver can apply dir toggle   │
│    without any buffering/timing concerns                        │
└─────────────────────────────────────────────────────────────────┘

┌─────────────────────────────────────────────────────────────────┐
│                      Queue/ISR Context                           │
├─────────────────────────────────────────────────────────────────┤
│  Direction - I2S/GPIO (UNCHANGED)                                │
│  - Check toggle_dir flag as before                              │
│  - Toggle direction pin if set                                  │
│  - NO changes to existing ISR/driver code                       │
│  - AVR/SAM/MCPWM: same code, getDrainPauseTicks() returns 0     │
│  - RMT: toggle_dir works, no injected pause needed anymore      │
│  - I2S: toggle_dir works at buffer boundary                     │
└─────────────────────────────────────────────────────────────────┘

KEY ARCHITECTURAL BENEFIT:
═════════════════════════
The pause insertion in addQueueEntry() ensures that by the time
the driver sees toggle_dir=1, all previous pulses have already
been emitted. This means:

  • EXISTING PULSE DRIVERS ARE REUSED WITHOUT CHANGE
  • AVR/SAM/MCPWM code unchanged (getDrainPauseTicks returns 0)
  • No injected pauses needed (RMT)
  • I2S-MUX: BEFORE (drain) + AFTER (dir change alone in buffer) = deterministic

Special case for I2S-MUX:
  • BEFORE: I2S_BLOCK_TICKS pause ensures buffer fully drained
  • AFTER: dir change command >= I2S_BLOCK_TICKS ensures it's alone in new buffer
  • Bit mask change happens at clean buffer boundary
```

### 3.2 Forbidden Combination: RMT Step + I2S Direction

```
❌ RMT step pin + I2S direction pin = UNSUPPORTED

Reason:
- RMT and I2S are independent hardware blocks
- RMT generates step pulses via its own DMA
- I2S direction would be in separate I2S DMA bitstream
- No synchronization mechanism between RMT and I2S DMA
- Direction changes cannot be coordinated with RMT step output

Allowed combinations:
✓ RMT step + GPIO direction
✓ RMT step + External direction
✓ I2S step + I2S direction (MUX mode)
✓ I2S step + GPIO direction
✓ I2S step + External direction
```

### 3.3 StepperQueue Protocol Extension

```cpp
// Protocol - each pd_*/ driver must implement
class StepperQueue : public StepperQueueBase {
public:
    // ... existing protocol methods ...
    
    // Set enable pin to specific state (called from addQueueEntry)
    // PRECONDITION: Must NOT be called for EXTERNAL enable pins
    void setEnablePinState(bool high);
    
    // Return required pause ticks BEFORE direction change
    // Used by addQueueEntry to insert sufficient pause for buffered platforms
    // Returns 0 for non-buffered platforms (AVR, SAM, MCPWM)
    // Returns I2S_BLOCK_TICKS for I2S-MUX (buffer must be empty before mask change)
    uint16_t getDrainPauseTicksForDirChange();
    
    // Return minimum ticks for direction change command itself (I2S-MUX only)
    // This is the AFTER pause - ensures dir change is alone in buffer
    // Returns 0 for all platforms except I2S-MUX (returns I2S_BLOCK_TICKS)
    uint16_t getDirChangeMinTicks();
    
    // Optional: Check if pulses are still being emitted
    // Used for enable pin safety checks in auto-enable logic
    bool hasPulsesInFlight();
};
```

### 3.4 Direction State in Queue Entry

The `toggle_dir` field remains unchanged:

```cpp
// In base.h - queue_entry structure (UNCHANGED)
struct queue_entry {
  uint8_t steps;
  uint8_t toggle_dir : 1;      // Flag: toggle direction pin before this entry
  uint8_t countUp : 1;
  uint8_t moreThanOneStep : 1;
  uint8_t hasSteps : 1;
#if defined(SUPPORT_EXTERNAL_DIRECTION_PIN)
  uint8_t repeat_entry : 1;
#endif
  uint16_t ticks;
  // ...
};
```

**For external direction pins**: `toggle_dir` is not used and stays off, `repeat_entry` is set instead. The callback handles the direction change in task context.

**For GPIO/I2S direction pins**: `toggle_dir` is set as before, but now `addQueueEntry()` has already inserted a pause to ensure safe timing.

### 3.5 Updated Code Flow

```
FastAccelStepper::addQueueEntry(cmd, start):
  1. Existing checks (ticks, dir pin defined)
  
  2. If direction change detected:
     
     a. If EXTERNAL direction pin:
        - toggle_dir stays off (not used for external pins)
        - Set repeat_entry = 1
        - (Callback happens in task context)
        
     b. Else if I2S-MUX direction pin (I2S controls both step and dir):
        - Query: pause_ticks = getDrainPauseTicksForDirChange()
        - Insert pause command with pause_ticks (BEFORE: I2S_BLOCK_TICKS to drain buffer)
        - If queue too full: return AQE_DIR_PIN_IS_BUSY
        - Set toggle_dir = 1 in entry
        - Override entry.ticks = max(entry.ticks, getDirChangeMinTicks()) (AFTER: ensures dir change alone in buffer)
        
     c. Else (GPIO direction pin, RMT/MCPWM/AVR/SAM):
        - Query: pause_ticks = getDrainPauseTicksForDirChange()
        - If pause_ticks > 0:
          - Insert pause command with pause_ticks
          - If queue too full: return AQE_DIR_PIN_IS_BUSY
        - Set toggle_dir = 1 in entry (as before)
  
  3. If enable pin change needed (auto-enable):
     - If hasPulsesInFlight(): return AQE_WAIT_FOR_ENABLE
     - If EXTERNAL: use callback
     - Else: setEnablePinState(...)
  
  4. StepperQueue::addQueueEntry(cmd, start)
     - Stores toggle_dir in entry
     - No change to toggle logic

Queue/ISR (UNCHANGED for I2S/GPIO direction):
  1. Check entry.toggle_dir
  2. If set: toggle direction pin
  3. Process steps
  
  Note: For buffered platforms (RMT/I2S), the pause inserted by
        addQueueEntry() ensures toggle happens at a safe point.
```

### 3.6 Platform Pause Query

The `getDrainPauseTicksForDirChange()` method returns the required pause before a direction change appears in the queue output:

```cpp
// In platform-specific queue implementation

// AVR - no buffer, no pause needed
uint16_t StepperQueue::getDrainPauseTicksForDirChange() {
    return 0;
}

// ESP32 RMT - need to wait for DMA buffer drain
uint16_t StepperQueue::getDrainPauseTicksForDirChange() {
    // one command fills one buffer half
    return MIN_CMD_TICKS;
}

// ESP32 I2S-MUX (I2S controls both step and dir) - need BEFORE and AFTER pauses
// BEFORE: I2S_BLOCK_TICKS pause to drain buffer completely
// AFTER: dir change command itself needs >= I2S_BLOCK_TICKS to be alone in new buffer
// The bit mask for direction is only applied when buffer is empty
uint16_t StepperQueue::getDrainPauseTicksForDirChange() {
    return I2S_BLOCK_TICKS;
}

// Minimum ticks for a direction change command in I2S-MUX mode
uint16_t StepperQueue::getDirChangeMinTicks() {
    return I2S_BLOCK_TICKS;
}
```

---

## 4. Platform-Specific Implementation

### 4.1 ESP32

```cpp
// In esp32_queue.cpp

void StepperQueue::setEnablePinState(bool high) {
    if (_enablePin & PIN_EXTERNAL_FLAG) {
        // Should not be called for external pins
        return;
    }
    gpio_ll_set_level(&GPIO, (gpio_num_t)_enablePin, high ? 1 : 0);
}

uint16_t StepperQueue::getDrainPauseTicksForDirChange() {
#if defined(USE_RMT)
    // RMT: one command fills one buffer half
    return MIN_CMD_TICKS;
#elif defined(USE_I2S)
    // I2S-MUX: I2S_BLOCK_TICKS BEFORE pause to drain buffer
    // Direction bit mask is only applied when buffer is empty
    return I2S_BLOCK_TICKS;
#else
    // MCPWM: no buffer, no pause needed
    return 0;
#endif
}

uint16_t StepperQueue::getDirChangeMinTicks() {
#if defined(USE_I2S)
    // I2S-MUX: dir change command needs I2S_BLOCK_TICKS to be alone in buffer
    // This is the AFTER pause, combined with the direction change
    return I2S_BLOCK_TICKS;
#else
    return 0;
#endif
}
```

### 4.2 I2S-MUX Direction Change Detail

For I2S multiplexed mode where I2S controls both step and direction pins, the direction is encoded in the bit mask. The mask change is only applied when the I2S buffer is empty.

```
Timeline for I2S-MUX direction change:
┌─────────────────────────────────────────────────────────────────┐
│ Buffer N:   [Step A][Step A][Step A][Step A] (old direction)    │
│                                                                 │
│             ─── BEFORE pause (I2S_BLOCK_TICKS) ───              │
│                                                                 │
│ Buffer N+1: [Dir Change Only] ← new bit mask applied            │
│             (>= I2S_BLOCK_TICKS, no steps, just direction)      │
│                                                                 │
│ Buffer N+2: [Step B][Step B][Step B] (new direction)            │
└─────────────────────────────────────────────────────────────────┘

Requirements:
1. BEFORE: I2S_BLOCK_TICKS pause ensures Buffer N is fully transmitted
2. AFTER: Dir change command >= I2S_BLOCK_TICKS, ensuring it fills Buffer N+1 alone
3. New bit mask (direction) is applied at Buffer N+1 boundary
4. Next steps (Buffer N+2) use the new direction
```

**ISR/Driver code UNCHANGED**:
- RMT ISR continues to check `toggle_dir` and call `LL_TOGGLE_PIN(dirPin)`
- I2S fill continues to check `toggle_dir` and toggle in bitstream
- The pause inserted by `addQueueEntry()` ensures the toggle happens at a safe point
- **RMT benefit**: The workaround that injected pause buffers is no longer needed

### 4.2 AVR

```cpp
// In avr_queue.cpp

void StepperQueue::setEnablePinState(bool high) {
    if (high) {
        *_enablePinPort |= _enablePinMask;
    } else {
        *_enablePinPort &= ~_enablePinMask;
    }
}

uint16_t StepperQueue::getDrainPauseTicksForDirChange() {
    return 0;  // No buffer, direction change is immediate
}
```

**ISR code UNCHANGED** - continues to use `Stepper_ToggleDirection` macro when `toggle_dir` is set.

### 4.3 SAM

```cpp
// In sam_queue.cpp

void StepperQueue::setEnablePinState(bool high) {
    if (high) {
        *_enablePinPort |= _enablePinMask;
    } else {
        *_enablePinPort &= ~_enablePinMask;
    }
}

uint16_t StepperQueue::getDrainPauseTicksForDirChange() {
    return 0;  // No buffer
}
```

**ISR code UNCHANGED** - continues to toggle via `*port ^= mask` when `toggle_dir` is set.

### 4.4 Pico

```cpp
// In pico_queue.cpp

void StepperQueue::setEnablePinState(bool high) {
    gpio_put(_enablePin, high);
}

uint16_t StepperQueue::getDrainPauseTicksForDirChange() {
    return 0;  // PIO has small FIFO, minimal delay
}
```

**PIO program UNCHANGED** - continues to handle direction toggle internally.

### 4.5 Test Driver (Stub)

```cpp
// In test_queue.h
void StepperQueue::setEnablePinState(bool high) {
    (void)high;
}

uint16_t StepperQueue::getDrainPauseTicksForDirChange() {
    return 0;
}

bool StepperQueue::hasPulsesInFlight() {
    return false;
}
```

---

## 5. Handling External Direction Pins

External direction pins are handled in `addQueueEntry()` context using the existing `repeat_entry` mechanism:

```cpp
// In addQueueEntry()
#if defined(SUPPORT_EXTERNAL_DIRECTION_PIN)
bool dir = (cmd->count_up == dirHighCountsUp);
if ((dir != queue_end.dir) && (dirPin & PIN_EXTERNAL_FLAG)) {
    e->repeat_entry = 1;
    // toggle_dir stays 0 (not used for external pins)
}
e->countUp = cmd->count_up;
#endif
```

The callback happens in task context via `externalDirPinChangeCompletedIfNeeded()`.

---

## 6. Enable Pins

### 6.1 Processing in addQueueEntry Context

Enable pins are always processed in `addQueueEntry()` context. The ~4ms processing delay is acceptable for enable/disable operations:

```cpp
// In FastAccelStepper.cpp
bool FastAccelStepper::enableOutputs() {
    // For safety, check if pulses in flight
    if (q->hasPulsesInFlight()) {
        return false;
    }
    
    if (_enablePinLowActive != PIN_UNDEFINED) {
        if (_enablePinLowActive & PIN_EXTERNAL_FLAG) {
            return _engine->_externalCallForPin(_enablePinLowActive, HIGH) == HIGH;
        } else {
            q->setEnablePinState(HIGH);
            return true;
        }
    }
    // ... similar for high-active
}
```

### 6.2 Auto-Enable Integration

```cpp
// In addQueueEntry() - auto-enable logic
if (_autoEnable) {
    if (_auto_disable_delay_counter == 0) {
        // Outputs are disabled, need to enable
        if (!enableOutputs()) {
            return AQE_WAIT_FOR_ENABLE_PIN_ACTIVE;
        }
        // Insert on-delay pauses if configured...
    }
}
```

---

## 7. Impact on Ramp Generator

### 7.1 No Changes Required

The ramp generator (`RampControl.cpp`, `RampGenerator.cpp`) requires **no modifications**. The `RAMP_STATE_REVERSE` logic continues to work as before.

### 7.2 How Direction Changes Flow Through

```
Ramp Generator                    fill_queue()                     addQueueEntry()
     │                                │                                 │
     │  RAMP_STATE_REVERSE            │                                 │
     │  (deceleration commands)       │                                 │
     ├──command(steps, count_up)─────►├──addQueueEntry(cmd, true)──────►│
     │                                │                                 │
     │  First cmd in new direction:   │                                 │
     ├──command(steps, !count_up)────►├──addQueueEntry(cmd, true)──────►│
     │                                │                                 ├─ detects dir change
     │                                │                                 ├─ query getDrainPauseTicks()
     │                                │                                 ├─ insert pause if needed
     │                                │                                 ├─ set toggle_dir in entry
     │                                │                                 └─ enqueue cmd
     │                                │◄─────────────AQE_OK─────────────┘
     │◄──afterCommandEnqueued()───────┤
```

The pause insertion uses the same pattern as existing `_dir_change_delay_ticks` logic (lines 113-122 of `FastAccelStepper.cpp`).

---

## 8. Advantages of the New Approach

### 8.1 Core Benefit: Existing Pulse Drivers Reused Without Change

This is the key architectural improvement. By having `addQueueEntry()` insert the necessary pause before a direction change, existing pulse drivers work correctly without modification:

| Component | Responsibility | Complexity |
|-----------|---------------|------------|
| `addQueueEntry()` | Timing management, pause insertion | Medium |
| Pulse drivers | Check `toggle_dir`, toggle pin (unchanged) | **No change** |
| Ramp generator | Unchanged | N/A |

**Before**: Each buffered driver had to understand buffering and inject workarounds
**After**: Drivers unchanged - timing handled upstream by pause insertion

### 8.2 Resolves Buffering Issues

| Issue | Old | New |
|-------|-----|-----|
| I2S timing | Toggle mid-buffer, ambiguous | Pause ensures buffer drained before toggle |
| RMT timing | Toggle at chunk boundary with injected pause workaround | Pause inserted by `addQueueEntry()`, no workaround needed |
| Pulse/dir sync | Ambiguous | Deterministic via pause insertion |
| Driver contract | Breached with RMT workaround | Maintained - drivers unchanged |

### 8.3 RMT-Specific Benefit: Remove Injected Pause

The current RMT driver injects pause buffers when it detects a direction change. This is a **breach of the driver contract**:

```cpp
// Current workaround in RMT driver
if (entry.toggle_dir && !pulses_in_buffer) {
    inject_pause_buffer();  // Hidden from upper layers!
}
```

With the new approach:
- `addQueueEntry()` inserts the pause command explicitly
- RMT driver sees the pause as a regular entry
- No hidden timing modifications
- **Driver contract preserved**
- **RMT ISR code unchanged**

### 8.4 I2S-MUX Specific Benefit: Deterministic Direction Change

For I2S multiplexed mode, direction is encoded in the bit mask, which is only applied when the buffer transitions from empty to non-empty. The current approach has timing ambiguity:

```cpp
// Current: bit mask change may happen mid-buffer
if (toggle_dir) {
    dir_mask = new_mask;  // When does this take effect?
}
```

With the new approach:
- `addQueueEntry()` inserts **BEFORE pause** (I2S_BLOCK_TICKS) to drain buffer completely
- Direction change command has **AFTER duration** (>= I2S_BLOCK_TICKS) ensuring it's alone in new buffer
- Bit mask change happens at buffer boundary - deterministic
- **No synchronization logic needed**
- **I2S driver code unchanged**

```
Before (ambiguous):
  Buffer: [Step][Step][Step][DIR_CHANGE][Step][Step]
                           ↑ When does mask change?

After (deterministic):
  [Step][Step][Step] | PAUSE (I2S_BLOCK_TICKS) | [DIR_CHANGE] | [Step][Step]
                      ← buffer empty →            ← alone →
                   mask applied at buffer boundary
```

### 8.5 Clear Processing Model

| Pin Type | Context | Timing |
|----------|---------|--------|
| Enable | `addQueueEntry()` | ~4ms acceptable |
| Direction (External) | `addQueueEntry()` ONLY | Deferred via callback, never in ISR |
| Direction (GPIO - RMT/AVR/SAM) | `addQueueEntry()` inserts pause, ISR toggles | Pause ensures safe toggle |
| Direction (I2S-MUX) | `addQueueEntry()` BEFORE pause + AFTER ticks | Buffer empty before, dir change alone after |

### 8.6 ISR Code Unchanged

| Platform | Before | After |
|----------|--------|-------|
| AVR | XOR toggle | **Unchanged** - XOR toggle |
| SAM | XOR toggle | **Unchanged** - XOR toggle |
| ESP32 RMT | `LL_TOGGLE_PIN` + injected pause workaround | **Unchanged** - `LL_TOGGLE_PIN` (workaround removed) |
| ESP32 I2S | `LL_TOGGLE_PIN` + sync complexity | **Unchanged** - `LL_TOGGLE_PIN` (pause ensures sync) |
| Pico PIO | PIO handles toggle | **Unchanged** - PIO handles toggle |

---

## 9. Error Handling

### 9.1 AQE_DIR_PIN_IS_BUSY

Returned when queue is too full to insert the required drain pause:

```cpp
// In addQueueEntry()
if (pause_ticks > 0) {
    if (queueEntries() > QUEUE_LEN - 1) {
        return AQE_DIR_PIN_IS_BUSY;  // Retry later
    }
    insertPauseCommand(pause_ticks);
}
```

The existing retry logic handles this:

```cpp
// In fill_queue()
if (aqeRetry(res)) {
    break;  // Retry on next fill_queue() call
}
```

### 9.2 AQE_WAIT_FOR_ENABLE_PIN_ACTIVE

Returned when auto-enable cannot proceed due to pulses in flight.

---

## 10. Migration Path

**Important**: Use platform-by-platform approach. Complete one platform end-to-end before moving to next.

### 10.1 Recommended Order

1. **AVR** - Simplest (no buffer, `getDrainPauseTicksForDirChange()` returns 0, no pause inserted)
2. **SAM** - Similar to AVR
3. **ESP32 MCPWM** - Similar to AVR (no buffer)
4. **ESP32 RMT** - Pause insertion, remove injected pause workaround
5. **ESP32 I2S** - Pause insertion
6. **Pico** - Minimal pause needed
7. **Test/Stub** - Update last

### 10.2 Per-Platform Phases

#### Phase 1: Add Protocol Extensions

1. Add `setEnablePinState()` to platform driver
2. Add `getDrainPauseTicksForDirChange()` to platform driver
3. Run tests — should pass (no behavior change)

#### Phase 2: Update addQueueEntry Logic

1. Implement pause insertion using `getDrainPauseTicksForDirChange()`
2. Handle DIR-external pins (set `repeat_entry`, don't set `toggle_dir`)
3. Run tests — behavior unchanged for non-buffered platforms

#### Phase 3: Remove Workarounds (RMT only)

1. Remove injected pause workaround from RMT driver
2. Run tests — pause now inserted by `addQueueEntry()`

#### Phase 4: Add Enable Pin Protocol

1. Replace direct `digitalWrite()` with `setEnablePinState()`
2. Update auto-enable logic
3. Run tests

**Note**: ISR/driver toggle logic (`toggle_dir` handling) remains unchanged in all phases.

**Checkpoint**: Each platform passes all tests before moving to next.

---

## 11. Files to Modify

| File | Changes |
|------|---------|
| `src/fas_queue/protocol.h` | Document new protocol methods |
| `src/FastAccelStepper.cpp` | Add pause insertion using `getDrainPauseTicksForDirChange()`, `getDirChangeMinTicks()` for I2S-MUX, handle DIR-external |
| `src/pd_esp32/esp32_queue.{h,cpp}` | Implement `setEnablePinState()`, `getDrainPauseTicksForDirChange()`, `getDirChangeMinTicks()` |
| `src/pd_avr/avr_queue.{h,cpp}` | Implement `setEnablePinState()`, `getDrainPauseTicksForDirChange()` (returns 0), `getDirChangeMinTicks()` (returns 0) |
| `src/pd_sam/sam_queue.{h,cpp}` | Implement `setEnablePinState()`, `getDrainPauseTicksForDirChange()` (returns 0), `getDirChangeMinTicks()` (returns 0) |
| `src/pd_pico/pico_queue.{h,cpp}` | Implement `setEnablePinState()`, `getDrainPauseTicksForDirChange()` (returns 0), `getDirChangeMinTicks()` (returns 0) |
| `src/pd_test/test_queue.h` | Add stub methods |

**Files NOT modified**: 
- `src/fas_queue/base.h` (queue_entry unchanged, `toggle_dir` stays)
- `src/pd_esp32/StepperISR_*.cpp` (ISR toggle logic unchanged)
- `src/pd_esp32/i2s_fill.cpp` (I2S toggle logic unchanged)
- `src/pd_avr/avr_queue.cpp` ISR code (toggle logic unchanged)
- `src/pd_sam/sam_queue.cpp` ISR code (toggle logic unchanged)
- `src/fas_ramp/RampControl.cpp`, `src/fas_ramp/RampGenerator.cpp`

---

## 12. Backward Compatibility

### 12.1 API Compatibility

- Public API unchanged
- Queue entry binary layout unchanged (`toggle_dir` stays)
- Binary compatible with previous versions

### 12.2 Behavior Changes

- Direction changes occur at precise command boundaries (pause inserted for buffered platforms)
- Pause automatically inserted for buffered platforms (RMT, I2S)
- RMT no longer inserts hidden pause buffers (workaround removed)
- Explicit error for RMT step + I2S direction combination
- DIR-external pins handled only in `addQueueEntry()` context

---

## 13. Conclusion

This refactoring improves direction pin handling with minimal changes to existing code:

1. **KEY BENEFIT**: Existing pulse drivers are reused without change - timing logic centralized in `addQueueEntry()`
2. **Resolves** timing ambiguity for buffered drivers (I2S clean implementation, RMT removes injected pause workaround)
3. **Clarifies** processing context - DIR-external handled ONLY in `addQueueEntry()`, never in queue/ISR
4. **Documents** forbidden RMT+I2S direction combination
5. **Platform-driven** timing via `getDrainPauseTicksForDirChange()` - pause inserted before dir change entry
6. **Preserves** `toggle_dir` mechanism - AVR/SAM/MCPWM code unchanged
7. **Preserves** the ramp generator
8. **Binary compatible** - queue entry layout unchanged

**Architecture Summary**:
```
addQueueEntry()     →  [pause if needed] → [toggle_dir=1 in entry]
                         ↓
Pulse drivers       →  Check toggle_dir → Toggle pin as before
                         ↑
                    Existing driver code unchanged - pause ensures safe timing
```

The result is a cleaner codebase with deterministic pin behavior, clear platform responsibilities, and minimal changes to proven ISR code.
