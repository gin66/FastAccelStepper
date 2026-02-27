# StepperISR Driver Interface Analysis

## Overview

The `addQueueEntry()` function in `StepperISR.cpp` calls into architecture-specific drivers to control pulse generation. This document analyzes the interfaces provided by each driver implementation.

---

## Interface Summary Table

### Hardware → Queue (Driver Outputs)

| Interface | AVR | ESP32 MCPWM/PCNT | ESP32 RMT (IDF4) | ESP32 RMT (IDF5) | ESP32-C3/S3 RMT | SAM Due | RP2040/RP2350 |
|-----------|-----|------------------|------------------|------------------|-----------------|---------|---------------|
| `isRunning()` | `_isRunning` flag | `_isRunning` flag | `_isRunning` flag | `_isRunning` flag | `_isRunning` flag | `_hasISRactive` | PIO FIFO/PC state |
| `isReadyForCommands()` | always `true` | check timer value | check `!_rmtStopped` | check `!_rmtStopped` | check `!_rmtStopped` | always `true` | always `true` |
| `_getPerformedPulses()` | N/A | PCNT counter read | returns 0 | returns 0 | returns 0 | N/A | N/A (uses PIO RX) |
| `getCurrentPosition()` | from `queue_end` | from `queue_end` (PCNT not used) | from `queue_end` | from `queue_end` | from `queue_end` | from `queue_end` | PIO step count + offset |

### Queue → Hardware (Driver Inputs)

| Interface | AVR | ESP32 MCPWM/PCNT | ESP32 RMT (IDF4) | ESP32 RMT (IDF5) | ESP32-C3/S3 RMT | SAM Due | RP2040/RP2350 |
|-----------|-----|------------------|------------------|------------------|-----------------|---------|---------------|
| `init(queue_num, step_pin)` | Timer + compare config | MCPWM + PCNT config | RMT channel config | RMT + encoder config | RMT channel config | PWM + GPIO config | Claim PIO SM |
| `startQueue()` | Enable compare interrupt | Apply command + run | Fill RMT buffer + start | `rmt_transmit()` | Fill RMT buffer + start | Attach PWM peripheral | Push commands to FIFO |
| `forceStop()` | Disable interrupt, disconnect | Stop timer | `stop_rmt(true)` | `rmt_disable()` | `stop_rmt(true)` | Disable PWM channel | Disable SM, clear FIFO |
| `connect()` | no-op | GPIO matrix connect | `idle_out_en = 1` | `rmt_enable()` (channel created in init) | `idle_out_en = 1` | GPIO config | `pio_gpio_init()` |
| `disconnect()` | no-op | GPIO matrix disconnect | `idle_out_en = 0` | `rmt_del_channel()` | `idle_out_en = 0` | PWM channel disable | `gpio_init()` |
| `setDirPin(pin, dirUp)` | Port register mask | `LL_TOGGLE_PIN()` | `LL_TOGGLE_PIN()` | `LL_TOGGLE_PIN()` | `LL_TOGGLE_PIN()` | Port register mask | Attach to state machine |
| `setDirPinState(high)` | N/A | N/A | N/A | N/A | N/A | N/A | `pio_sm_exec()` |

### manageSteppers() Invocation

| Architecture | Mechanism | Location |
|--------------|-----------|----------|
| **AVR** | Timer OVF ISR | `ISR(TIMERx_OVF_vect)` in `StepperISR_avr.cpp` |
| **ESP32** | FreeRTOS task | `StepperTask()` in `StepperISR_esp32.cpp` |
| **SAM Due** | Timer ISR | `TC5_Handler()` in `StepperISR_due.cpp` |
| **RP2040/RP2350** | FreeRTOS task | `StepperTask()` in `StepperISR_rp_pico.cpp` |

### Static/Class Methods

| Method | AVR | ESP32 MCPWM/PCNT | ESP32 RMT (IDF4) | ESP32 RMT (IDF5) | ESP32-C3/S3 RMT | SAM Due | RP2040/RP2350 |
|--------|-----|------------------|------------------|------------------|-----------------|---------|---------------|
| `isValidStepPin(pin)` | Fixed timer pins | `gpio_get_drive_capability()` | `gpio_get_drive_capability()` | `gpio_get_drive_capability()` | `gpio_get_drive_capability()` | PWM channel pins | `pin < 32` |
| `queueNumForStepPin(pin)` | Fixed mapping | N/A | N/A | N/A | N/A | N/A | N/A |

---

## Architecture-Specific Members

| Member | Type | AVR | ESP32 MCPWM | ESP32 RMT IDF4 | ESP32 RMT IDF5 | ESP32xx RMT | SAM Due | Pico |
|--------|------|-----|-------------|----------------|----------------|-------------|---------|------|
| `channel` | `enum channels` | ✓ | N/A | N/A | N/A | N/A | N/A | N/A |
| `driver_data` | `void*` | N/A | `mapping_s*` | N/A | N/A | N/A | `PWMCHANNELMAP*` | N/A |
| `channel` | `RMT_CHANNEL_T` | N/A | N/A | ✓ | ✓ | ✓ | N/A | N/A |
| `_channel_enabled` | `bool` | N/A | N/A | N/A | ✓ | N/A | N/A | N/A |
| `_tx_encoder` | `rmt_encoder_handle_t` | N/A | N/A | N/A | ✓ | N/A | N/A | N/A |
| `_nextCommandIsPrepared` | `bool` | N/A | ✓ | N/A | N/A | N/A | N/A | N/A |
| `_noMoreCommands` | `bool` | ✓ | N/A | N/A | N/A | N/A | N/A | N/A |
| `_pauseCommanded` | `bool` | N/A | N/A | N/A | N/A | N/A | ✓ | N/A |
| `_hasISRactive` | `bool` | N/A | N/A | N/A | N/A | N/A | ✓ | N/A |
| `_connected` | `bool` | N/A | N/A | N/A | N/A | N/A | ✓ | N/A |
| `_rmtStopped` | `bool` | N/A | N/A | ✓ | ✓ | ✓ | N/A | N/A |
| `lastChunkContainsSteps` | `bool` | N/A | N/A | ✓ | ✓ | ✓ | N/A | N/A |
| `pio` | `RP PIO` | N/A | N/A | N/A | N/A | N/A | ✓ |
| `sm` | `uint` | N/A | N/A | N/A | N/A | N/A | N/A | ✓ |
| `_isActive` | `bool` | N/A | N/A | N/A | N/A | N/A | N/A | ✓ |
| `pos_offset` | `int32_t` | N/A | N/A | N/A | N/A | N/A | N/A | ✓ |
| `adjust_80MHz` | `uint16_t` | N/A | N/A | N/A | N/A | N/A | N/A | ✓ |
| `_step_pin` | `uint8_t` | N/A | ✓ | ✓ | ✓ | ✓ | ✓ | ✓ |
| `_queue_num` | `uint8_t` | N/A | N/A | N/A | N/A | N/A | ✓ | N/A |
| `timePWMInterruptEnabled` | `uint32_t` | N/A | N/A | N/A | N/A | N/A | ✓ | N/A |
| `_dirPinPort` | port pointer | N/A | N/A | N/A | N/A | N/A | ✓ | N/A |
| `_dirPinMask` | port mask | N/A | N/A | N/A | N/A | N/A | ✓ | N/A |
| `_dirTogglePinPort` | port pointer | ✓ | N/A | N/A | N/A | N/A | N/A | N/A |
| `_dirTogglePinMask` | port mask | ✓ | N/A | N/A | N/A | N/A | N/A | N/A |

---

## Pulse Generation Mechanisms

| Architecture | Mechanism | ISR/Callback | Position Tracking |
|--------------|-----------|--------------|-------------------|
| **AVR** | Timer compare output (OC1A/OC1B) | Timer COMP vect | Queue state only |
| **ESP32 MCPWM/PCNT** | MCPWM generates pulses, PCNT counts | PCNT h_lim + MCPWM TEA | Hardware PCNT counter |
| **ESP32 RMT (IDF4)** | RMT peripheral with threshold/end interrupts | TX_END + TX_THR interrupts | Queue state only |
| **ESP32 RMT (IDF5)** | RMT with encoder callback | `on_trans_done` callback | Queue state only |
| **ESP32-C3/S3 RMT** | RMT peripheral | TX_END + TX_THR interrupts | Queue state only |
| **SAM Due** | PWM generator + GPIO pin interrupt | GPIO RISING + PWM period | Queue state only |
| **RP2040/RP2350** | PIO state machine | TX FIFO not full | PIO RX FIFO returns count |

---

## Direction Pin Handling

| Architecture | Toggle Mechanism | Special Handling |
|--------------|------------------|------------------|
| **AVR** | Direct port toggle via `_dirTogglePinPort` | `SUPPORT_DIR_TOGGLE_PIN_MASK` |
| **ESP32 MCPWM** | `LL_TOGGLE_PIN()` in `apply_command()` | gpio_ll functions |
| **ESP32 RMT** | `LL_TOGGLE_PIN()` in `rmt_fill_buffer()` / `startQueue_rmt()` | Handled in buffer fill |
| **SAM Due** | `*_dirPinPort ^= _dirPinMask` | Direct port XOR |
| **Pico** | `setDirPinState()` via `pio_sm_exec()` | State machine control |

---

## Queue Entry Fields Usage by Architecture

| Field | AVR | ESP32 MCPWM | ESP32 RMT | SAM Due | Pico |
|-------|-----|-------------|-----------|---------|------|
| `steps` | ✓ decremented in ISR | ✓ | ✓ | ✓ decremented in ISR | ✓ |
| `toggle_dir` | ✓ | ✓ | ✓ | ✓ | N/A (handled via `dirPinState`) |
| `countUp` | ✓ | ✓ | ✓ | ✓ | ✓ |
| `moreThanOneStep` | ✓ | N/A | N/A | N/A | N/A |
| `hasSteps` | ✓ | ✓ | ✓ | ✓ | N/A |
| `ticks` | ✓ | ✓ | ✓ | ✓ | ✓ |
| `repeat_entry` | ✓ | ✓ | ✓ | N/A | N/A |
| `dirPinState` | N/A | N/A | N/A | N/A | ✓ |
| `start_pos_last16` | N/A | ✓ | ✓ | N/A | N/A |
| `end_pos_last16` | ✓ | N/A | N/A | ✓ | N/A |

---

## Key Implementation Differences

### 1. `isRunning()` Implementation

```cpp
// AVR - simple flag
inline bool isRunning() { return _isRunning; }

// ESP32 RMT - async stop detection
inline bool isRunning() { return _isRunning; }  // _rmtStopped checked separately

// SAM Due - ISR state tracking
bool isRunning() { return _hasISRactive; }

// Pico - PIO state inspection
bool isRunning() {
    if (!pio_sm_is_tx_fifo_empty(pio, sm)) return true;
    return pio_sm_get_pc(pio, sm) != 0;
}
```

### 2. Position Tracking

| Architecture | Method |
|--------------|--------|
| **AVR** | `queue_end.pos` (no hardware counter) |
| **ESP32 MCPWM** | `queue_end.pos` only (PCNT not used) |
| **ESP32 RMT** | `queue_end.pos` only (no hardware counter) |
| **SAM Due** | `queue_end.pos` (no hardware counter) |
| **Pico** | `getCurrentStepCount()` from PIO RX FIFO + `pos_offset` |

### 3. Command Preparation

| Architecture | Look-ahead |
|--------------|------------|
| **AVR** | None (ISR handles each command) |
| **ESP32 MCPWM** | `_nextCommandIsPrepared` flag for PCNT h_lim preload |
| **ESP32 RMT** | Double-buffered (PART_SIZE symbols each) |
| **SAM Due** | None |
| **Pico** | FIFO-based (push as many as possible) |

---

## Preprocessor Defines by Architecture

| Define | AVR | ESP32 MCPWM | ESP32 RMT IDF4 | ESP32 RMT IDF5 | SAM Due | Pico |
|--------|-----|-------------|----------------|----------------|---------|------|
| `SUPPORT_AVR` | ✓ | | | | | |
| `SUPPORT_ESP32` | | ✓ | ✓ | ✓ | | |
| `SUPPORT_ESP32_MCPWM_PCNT` | | ✓ | | | | |
| `SUPPORT_ESP32_RMT` | | | ✓ | ✓ | | |
| `SUPPORT_ESP32_RMT_V2` | | | | ✓ | | |
| `SUPPORT_SAM` | | | | | ✓ | |
| `SUPPORT_RP_PICO` | | | | | | ✓ |
| `SUPPORT_DIR_PIN_MASK` | | | | | ✓ | |
| `SUPPORT_DIR_TOGGLE_PIN_MASK` | ✓ | | | | | |
| `SUPPORT_EXTERNAL_DIRECTION_PIN` | ✓ | ✓ | ✓ | ✓ | | |
| `SUPPORT_QUEUE_ENTRY_START_POS_U16` | | ✓ | ✓ | ✓ | | |
| `SUPPORT_QUEUE_ENTRY_END_POS_U16` | ✓ | | | | ✓ | |
| `NEED_FIXED_QUEUE_TO_PIN_MAPPING` | ✓ | | | | | |
| `NEED_ADJUSTABLE_MAX_SPEED_DEPENDING_ON_STEPPER_COUNT` | ✓ | | | | | |

---

## Source Files

| File | Architecture |
|------|--------------|
| `StepperISR_avr.cpp` | AVR (ATmega328, ATmega2560) |
| `StepperISR_esp32.cpp` | ESP32 dispatcher (routes to MCPWM or RMT) |
| `StepperISR_idf4_esp32_mcpwm_pcnt.cpp` | ESP32 MCPWM/PCNT (IDF 4.x) |
| `StepperISR_idf4_esp32_rmt.cpp` | ESP32 RMT (IDF 4.x) |
| `StepperISR_idf5_esp32_rmt.cpp` | ESP32 RMT (IDF 5.x, V2 driver) |
| `StepperISR_idf4_esp32c3_rmt.cpp` | ESP32-C3 RMT |
| `StepperISR_idf4_esp32s3_rmt.cpp` | ESP32-S3 RMT |
| `StepperISR_esp32xx_rmt.cpp` | Shared RMT buffer fill code |
| `StepperISR_due.cpp` | SAM Due (Arduino DUE) |
| `StepperISR_rp_pico.cpp` | RP2040/RP2350 (Raspberry Pi Pico) |
