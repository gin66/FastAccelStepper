# I2S Driver IRAM_ATTR Analysis — Missing Annotations

## Executive Summary

The I2S driver's ISR calling chain has **3 functions missing `IRAM_ATTR`** that are
called from interrupt context. On ESP32, code fetched from external SPI flash causes
cache misses that can take **hundreds of microseconds** — far too long for an ISR.
Functions in the ISR path must reside in IRAM (or RTC Fast Memory) to guarantee
deterministic, low-latency execution.

---

## ISR Calling Graph (Complete)

```
I2S DMA TX-done event (ESP-IDF hardware interrupt)
  │
  ├─► i2s_tx_done_callback()          [i2s_manager.cpp:8]    ✅ IRAM_ATTR
  │     │
  │     └─► I2sManager::handleTxDone(buf)  [i2s_manager.cpp:123]  ✅ IRAM_ATTR
  │            │
  │            ├─► init_mux_buffer(buf)          [i2s_manager.cpp:96]   ✅ IRAM_ATTR  (mux mode only)
  │            ├─► memset(buf, 0, ...)           [stdlib]               OK (intrinsic)
  │            │
  │            └─► StepperQueue::fill_i2s_buffer(buf)  [StepperISR_esp32_i2s.cpp:39]  ❌ MISSING
  │                   │
  │                   ├─► i2s_fill_buffer_mux(...)     [i2s_fill.cpp:8]   ✅ IRAM_ATTR
  │                   │   │
  │                   │   └─► SET_DIRECTION_PIN_STATE() → esp32_set_direction_pin_state()
  │                   │       └─► I2sManager::i2sMuxSetBit(slot, value)  [i2s_manager.cpp:104]  ❌ MISSING
  │                   │
  │                   └─► i2s_fill_buffer_direct(...)  [i2s_fill.cpp:72]  ✅ IRAM_ATTR
  │                       │
  │                       └─► LL_TOGGLE_PIN(dirPin) → gpio_ll_set_level()  [esp32/pd_config.h:96]  OK (LL intrinsic)
```

---

## Missing IRAM_ATTR — Detailed Findings

### 1. `StepperQueue::fill_i2s_buffer(uint8_t* buf)`
- **File:** `src/pd_esp32/StepperISR_esp32_i2s.cpp`
- **Line:** 39
- **Declaration:** `src/pd_esp32/esp32_queue.h:129` (also missing IRAM_ATTR)
- **Called from:** `I2sManager::handleTxDone()` (line 141 in `i2s_manager.cpp`)
- **Why it matters:** This is the **bottleneck function** — it iterates over queue
  entries, reads `q->entry[]`, accesses `q->read_idx` / `q->next_write_idx`, and
  calls `SET_DIRECTION_PIN_STATE`. All of this happens in ISR context. If this
  function is fetched from flash, the cache miss during a step burst will cause
  missed timing deadlines.

### 2. `I2sManager::i2sMuxSetBit(uint8_t slot, bool value)`
- **File:** `src/pd_esp32/i2s_manager.cpp`
- **Line:** 104
- **Declaration:** `src/pd_esp32/i2s_manager.h:20` (also missing IRAM_ATTR)
- **Called from:** `esp32_set_direction_pin_state()` (inline, `esp32_queue.h:196`)
  which is used by the `SET_DIRECTION_PIN_STATE` macro, called from
  `i2s_fill_buffer_mux()` at line 52 of `i2s_fill.cpp`.
- **Why it matters:** In mux mode, every direction change in the fill loop goes
  through this function. It accesses `StepperQueue::_i2s_mux_manager` and writes
  to the mux state. If the mux driver is on a separate I2C expander, this call
  itself may take significant time — but the *function fetch* must still come
  from IRAM.

### 3. `I2sManager::i2sMuxGetBit(uint8_t slot)`
- **File:** `src/pd_esp32/i2s_manager.cpp`
- **Line:** 116
- **Declaration:** `src/pd_esp32/i2s_manager.h:21` (also missing IRAM_ATTR)
- **Called from:** Not directly in the ISR path currently, but declared alongside
  `i2sMuxSetBit` and used by `FastAccelStepperEngine::i2sMuxGetBit()` (lines 95-102
  in `FastAccelStepperEngine.cpp`). If this function is ever called from ISR
  context (e.g., for debugging or status queries), it needs IRAM_ATTR.

---

## Header/Implementation Mismatches

| Function | .cpp definition | .h declaration |
|---|---|---|
| `i2s_fill_buffer_direct` | ✅ `IRAM_ATTR` (line 72) | ❌ no IRAM_ATTR (i2s_fill.h:18) |
| `i2s_fill_buffer_mux` | ✅ `IRAM_ATTR` (line 8) | ❌ no IRAM_ATTR (i2s_fill.h:20) |
| `handleTxDone` | ✅ `IRAM_ATTR` (line 123) | ❌ no IRAM_ATTR (i2s_manager.h:14) |
| `fill_i2s_buffer` | ❌ no IRAM_ATTR (StepperISR_esp32_i2s.cpp:39) | ❌ no IRAM_ATTR (esp32_queue.h:129) |

**Note:** On ESP-IDF, the `IRAM_ATTR` macro expands to `IRAM` which is typically
`__attribute__((section(".iram1")))`. The linker places the function in IRAM based
on the *definition*. However, best practice is to declare it in the header too,
so callers (including inline functions) know the function resides in IRAM.

---

## Recommended Fixes

### Fix 1: Add IRAM_ATTR to `StepperQueue::fill_i2s_buffer`

```cpp
// In StepperISR_esp32_i2s.cpp line 39:
void IRAM_ATTR StepperQueue::fill_i2s_buffer(uint8_t* buf) {
```

```cpp
// In esp32_queue.h line 129:
void IRAM_ATTR fill_i2s_buffer(uint8_t* buf);
```

### Fix 2: Add IRAM_ATTR to `I2sManager::i2sMuxSetBit`

```cpp
// In i2s_manager.cpp line 104:
void IRAM_ATTR I2sManager::i2sMuxSetBit(uint8_t slot, bool value) {
```

```cpp
// In i2s_manager.h line 20:
void IRAM_ATTR i2sMuxSetBit(uint8_t slot, bool value);
```

### Fix 3: Add IRAM_ATTR to `I2sManager::i2sMuxGetBit`

```cpp
// In i2s_manager.cpp line 116:
bool IRAM_ATTR I2sManager::i2sMuxGetBit(uint8_t slot) {
```

```cpp
// In i2s_manager.h line 21:
bool IRAM_ATTR i2sMuxGetBit(uint8_t slot);
```

### Fix 4: Add IRAM_ATTR to header declarations (best practice)

Update `i2s_fill.h` lines 18-21 and `i2s_manager.h` line 14 to match the
`.cpp` definitions.

---

## Files to Modify

| File | Lines to Change |
|---|---|
| `src/pd_esp32/StepperISR_esp32_i2s.cpp` | 39 (add IRAM_ATTR) |
| `src/pd_esp32/esp32_queue.h` | 129 (add IRAM_ATTR) |
| `src/pd_esp32/i2s_manager.cpp` | 104, 116 (add IRAM_ATTR) |
| `src/pd_esp32/i2s_manager.h` | 14, 20, 21 (add IRAM_ATTR) |
| `src/pd_esp32/i2s_fill.h` | 18, 20 (add IRAM_ATTR) |

---

## Risk Assessment

| Severity | Function | Impact |
|---|---|---|
| **HIGH** | `fill_i2s_buffer` | Called on every DMA TX-done event; misses here cause dropped steps |
| **MEDIUM** | `i2sMuxSetBit` | Called on every direction change in mux mode; less frequent but still in ISR path |
| **LOW** | `i2sMuxGetBit` | Not in ISR path currently, but good to fix for consistency |
