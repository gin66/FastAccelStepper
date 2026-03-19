# StepperISR Driver Architecture

This document describes the StepperISR layer architecture, which sits between the FastAccelStepper high-level API and the hardware-specific pulse generation drivers.

## Abstract Model

Each `StepperQueue` instance is a **generic pulse generator** that:

1. Accepts commands specifying `steps` and `ticks` (period between steps)
2. Outputs step pulses on a GPIO pin
3. Tracks position based on executed pulses

Depending on the microcontroller, these pulse generators can be assigned to GPIO pins either freely (ESP32, Pico) or only to specific pins (AVR Timer1 on OC1A/OC1B).

---

## Core Data Structures

### Command Interface (from RampGenerator)

```cpp
struct stepper_command_s {
    uint16_t ticks;    // Period between steps (in TICKS_PER_S units)
    uint8_t steps;     // Number of steps (0 = pause/no steps)
    bool count_up;     // Direction: true = count up, false = count down
};
```

- `ticks = 0`: Reserved for stopped motor
- `steps = 0`: Creates a pause for `ticks` duration (no pulses)
- `steps > 0`: Generates `steps` pulses, each `ticks` apart

### Internal Queue Entry

```cpp
struct queue_entry {
    uint8_t steps;           // Number of steps
    uint8_t toggle_dir : 1;  // Direction changed mid-command
    uint8_t countUp : 1;     // Direction
    uint8_t moreThanOneStep : 1;
    uint8_t hasSteps : 1;    // Has steps vs pause
    uint16_t ticks;          // Period between steps
    // Optional platform-specific fields follow
};
```

---

## StepperQueueBase Common Interface

All architecture drivers inherit from `StepperQueueBase` (defined in `fas_queue/base.h`) and implement these methods:

| Method | Description |
|--------|-------------|
| `init(queue_num, step_pin)` | One-time hardware setup (called by `tryAllocateQueue()`) |
| `startQueue()` | Begin pulse generation from queue head |
| `forceStop()` | Immediately stop and clear queue |
| `connect()` | Attach pulse generator to GPIO |
| `disconnect()` | Detach pulse generator from GPIO |
| `isRunning()` | True if actively generating pulses |
| `isReadyForCommands()` | True if can accept new commands |
| `setDirPin(dir_pin)` | Configure direction pin |
| `isValidStepPin(pin)` | Check if pin can generate pulses (static) |
| `getCurrentPosition()` | Return current step count |
| `addQueueEntry(cmd, start)` | Add command to queue (common implementation) |
| `_initVars()` | Zero all fields and set platform defaults (no hardware access) |

---

## Architecture Implementations

### Directory Structure

The `pd_` prefix in directory names stands for "pulse driver" (though "platform driver" is also suitable).

```
src/
  fas_queue/
    base.h              ← StepperQueueBase definition
    stepper_queue.h     ← Architecture dispatcher
    stepper_queue.cpp   ← Common queue implementation
  pd_avr/
    avr_queue.h/cpp     ← AVR Timer implementation
  pd_esp32/
    esp32_queue.h/cpp   ← ESP32 queue wrapper
    StepperISR_idf4_esp32_mcpwm_pcnt.cpp  ← IDF4 MCPWM/PCNT
    StepperISR_idf4_esp32_rmt.cpp         ← IDF4 RMT
    StepperISR_idf4_esp32c3_rmt.cpp       ← IDF4 ESP32-C3
    StepperISR_idf4_esp32s3_rmt.cpp       ← IDF4 ESP32-S3
    StepperISR_idf5_esp32_mcpwm_pcnt.cpp ← IDF5 MCPWM/PCNT (ESP32, S3, C6, H2)
    StepperISR_idf5_esp32_rmt.cpp         ← IDF5 RMT
    StepperISR_esp32xx_rmt.cpp            ← Shared RMT code
  pd_sam/
    sam_queue.h/cpp     ← SAM Due implementation
  pd_pico/
    pico_queue.h/cpp    ← RP2040/RP2350 PIO implementation
  pd_test/
    test_queue.h        ← PC-based test stub
```

### Platform Capabilities

| Architecture | Pulse Mechanism | Channels | Pin Flexibility |
|--------------|-----------------|----------|-----------------|
| **AVR** | Timer compare output (OC1A/OC1B/OC4A-C) | 2-3 | Fixed to specific pins |
| **ESP32 MCPWM/PCNT** | MCPWM generates, PCNT counts | 2-6 | Any GPIO |
| **ESP32 RMT (IDF4)** | RMT peripheral DMA | 2-8 | Any GPIO |
| **ESP32 RMT (IDF5)** | RMT with encoder callback | 2-8 | Any GPIO |
| **ESP32-C3 RMT** | RMT peripheral | 2 | Any GPIO |
| **ESP32-C6/H2 MCPWM/PCNT** | MCPWM generates, PCNT counts | 2 | Any GPIO |
| **ESP32-S3 RMT** | RMT peripheral | 2-4 | Any GPIO |
| **SAM Due** | PWM + pin change interrupt | 6 | Limited (specific PWM pins) |
| **RP2040/RP2350** | PIO state machine | 8 | GPIO 0-31 |

---

## Interface Summary by Architecture

### Hardware → Queue (Driver Outputs)

| Interface | AVR | ESP32 MCPWM | ESP32 RMT | SAM Due | Pico |
|-----------|-----|-------------|-----------|---------|------|
| `isRunning()` | `_isRunning` flag | `_isRunning` flag | `_isRunning` flag | `_hasISRactive` | PIO FIFO/PC |
| `isReadyForCommands()` | always `true` | check timer | check `!_rmtStopped` | always `true` | always `true` |
| `getCurrentPosition()` | from `queue_end` | from `queue_end` | from `queue_end` | from `queue_end` | PIO count + offset |

### Queue → Hardware (Driver Inputs)

| Interface | AVR | ESP32 | SAM Due | Pico |
|-----------|-----|-------|---------|------|
| `init()` | Timer + compare config | No-op (driver inits called by `tryAllocateQueue()`) | PWM + GPIO config | Set `_step_pin` |
| `startQueue()` | Enable compare interrupt | Apply command + run | Attach PWM peripheral | Push to FIFO |
| `forceStop()` | Disable interrupt | Stop peripheral | Disable PWM channel | Disable SM |
| `connect()` | no-op | GPIO matrix | GPIO config | `pio_gpio_init()` |
| `disconnect()` | no-op | GPIO matrix | PWM channel disable | `gpio_init()` |

### manageSteppers() Invocation

| Architecture | Mechanism | Location |
|--------------|-----------|----------|
| **AVR** | Timer OVF ISR | `pd_avr/avr_queue.cpp` |
| **ESP32** | FreeRTOS task | `pd_esp32/esp32_queue.cpp` |
| **SAM Due** | Timer ISR | `pd_sam/sam_queue.cpp` |
| **RP2040/RP2350** | FreeRTOS task | `pd_pico/pico_queue.cpp` |

---

## Driver Contract

All pulse drivers must adhere to these contracts regardless of architecture.

### Command Queue Ownership

The command queue uses single-producer / single-consumer semantics with two
index variables:

| Index | Type | Owner (writer) | Reader |
|-------|------|----------------|--------|
| `read_idx` | `volatile uint8_t` | Driver (ISR / fill routine) | Both |
| `next_write_idx` | `volatile uint8_t` | Ramp generator (`manageSteppers()`) | Both |

Both indices are single bytes, which are atomically safe to read and write on
all supported architectures (AVR, ESP32, SAM, Pico). No mutex or spinlock is
required.

**Invariant**: `read_idx` is only advanced by the driver after a command has
been fully consumed (all steps emitted or pause duration elapsed). The ramp
generator only advances `next_write_idx` after writing a new queue entry.
The queue is empty when `read_idx == next_write_idx`.

### Driver Responsibilities

Each pulse driver must:

1. **Execute commands exactly as specified** — emit the correct number of
   step pulses with the correct tick spacing
2. **Advance `read_idx`** — after each command is fully consumed
3. **Maintain time synchronicity** — the driver's output timing must match
   the `TICKS_PER_S` timebase. If the driver's hardware clock differs from
   the command queue's tick rate, the driver must compensate (e.g., via
   Bresenham-style fractional-tick correction)

The driver does **not**:
- Feed back position or step counts to the ramp generator
- Interpret command semantics beyond steps/ticks/direction
- Manage acceleration or deceleration

Position tracking is handled by `queue_end.pos` updates in the common queue
code (`addQueueEntry()`), not by the driver.

### Direction Setup Time

The time delta between a direction pin change and the next step pulse may be
influenced by driver properties (e.g., DMA buffer granularity, timer resolution),
but is **not guaranteed by the driver**. The driver outputs commands as given —
it does not insert additional delays between direction changes and step pulses.

It is the responsibility of command generation (ramp generator / `addQueueEntry()`)
to insert sufficient pause commands between a direction change and the next step
to meet the stepper driver's minimum direction setup time (`MIN_DIR_DELAY_US`).

### forceStop() Contract

`forceStop()` immediately halts pulse generation for the stepper. The
contract is:

- **Best effort stop**: The driver stops as quickly as hardware allows.
  For DMA-based drivers (I2S, potentially RMT), pulses already committed to
  hardware buffers may still be physically output.
- **Position not guaranteed**: `forceStop()` does not guarantee that the
  reported position exactly matches the number of physically output pulses.
  The position error is bounded by the driver's buffering depth (e.g., for
  I2S: up to `(BLOCK_COUNT - 1) × max_steps_per_block` steps).
- **Queue is cleared**: After `forceStop()`, the queue is logically empty.
  Any remaining commands are discarded.
- **State reset**: The driver resets its internal state (`_isRunning = false`,
  fill state cleared, etc.) so the stepper can accept new commands.

---

## Key Implementation Differences

### Position Tracking

| Architecture | Method |
|--------------|--------|
| **AVR/SAM** | `queue_end.pos` (no hardware counter) |
| **ESP32 MCPWM** | `queue_end.pos` (PCNT available but not used) |
| **ESP32 RMT** | `queue_end.pos` only |
| **Pico** | `getCurrentStepCount()` from PIO RX FIFO + `pos_offset` |

### Running State Detection

```cpp
// AVR/SAM: Simple flag
volatile bool _isRunning;

// ESP32 RMT: Async stop detection
volatile bool _isRunning;
bool _rmtStopped;  // Set when end interrupt fires

// Pico: Check PIO state
bool isRunning() {
    return !pio_sm_is_tx_fifo_empty(pio, sm) || 
           pio_sm_get_pc(pio, sm) != 0;
}
```

### Direction Pin Handling

| Architecture | Toggle Mechanism |
|--------------|------------------|
| **AVR** | Direct port toggle via `_dirTogglePinPort` |
| **ESP32** | `LL_TOGGLE_PIN()` in apply/buffer fill |
| **SAM Due** | Direct port XOR |
| **Pico** | `pio_sm_exec()` state machine control |

---

## Timing Constants

| Architecture | TICKS_PER_S | MIN_CMD_TICKS | MIN_DIR_DELAY_US |
|--------------|-------------|---------------|------------------|
| AVR | 16,000,000 | 640 | 40 µs |
| ESP32 | 16,000,000 | 3200 | 200 µs |
| SAM Due | 21,000,000 | 4200 | 200 µs |
| Pico | 16,000,000 | 3200 | 200 µs |

---

## Preprocessor Defines by Architecture

| Define | AVR | ESP32 MCPWM | ESP32 RMT | ESP32 I2S | SAM | Pico |
|--------|-----|-------------|-----------|-----------|-----|------|
| `SUPPORT_AVR` | ✓ | | | | | |
| `SUPPORT_ESP32` | | ✓ | ✓ | ✓ | | |
| `SUPPORT_ESP32_MCPWM_PCNT` | | ✓ | | | | |
| `SUPPORT_ESP32_RMT` | | | ✓ | | | |
| `SUPPORT_ESP32_I2S` | | | | ✓ | | |
| `SUPPORT_SAM` | | | | | ✓ | |
| `SUPPORT_RP_PICO` | | | | | | ✓ |

---

## Adding a New Architecture

Porting to a new microcontroller requires several coordinated changes across the codebase.

### Step 1: Architecture Detection in fas_arch/common.h

Add detection for the new architecture to the preprocessor chain in `fas_arch/common.h`:

```cpp
#elif defined(YOUR_NEW_ARCH_DETECTION_MACRO)
#include "fas_arch/your_arch.h"
```

Common detection macros:
- `ARDUINO_ARCH_*` for Arduino cores
- Compiler-defined macros like `__ARM_ARCH`, `__AVR__`
- SDK-defined macros like `PICO_RP2040`, `ESP_PLATFORM`

### Step 2: Create Architecture Header

Create `fas_arch/your_arch.h` defining all required constants and macros:

```cpp
#ifndef FAS_ARCH_YOUR_ARCH_H
#define FAS_ARCH_YOUR_ARCH_H

// 1. Platform identification
#define SUPPORT_YOUR_ARCH

// 2. Include platform headers
#include <Arduino.h>  // or your SDK headers

// 3. Interrupt control (required)
#define fasDisableInterrupts()  // disable interrupts
#define fasEnableInterrupts()   // restore interrupts

// 4. Core timing constants (required)
#define TICKS_PER_S 16000000L   // your timer frequency
#define MIN_CMD_TICKS (TICKS_PER_S / 5000)  // minimum command duration
#define MIN_DIR_DELAY_US 200    // direction pin setup time
#define MAX_DIR_DELAY_US 4095   // maximum direction delay

// 5. Queue configuration (required)
#define QUEUE_LEN 32            // queue depth (power of 2)
#define NUM_QUEUES 4            // number of steppers supported
#define MAX_STEPPER NUM_QUEUES  // usually same as NUM_QUEUES

// 6. Optional features (define if supported)
//   - Direct port access for dir pin (if needed, declare
//     _dirPinPort/_dirPinMask in your StepperQueue subclass)

// 7. Task management (if using RTOS)
#define noop_or_wait vTaskDelay(1)  // or your idle function

// 8. Debug LED timing
#define DEBUG_LED_HALF_PERIOD 50

// 9. Fixed queue-to-pin mapping (if hardware requires)
// #define NEED_FIXED_QUEUE_TO_PIN_MAPPING

#endif
```

### Step 3: Create Queue Header

Create `pd_yourarch/yourarch_queue.h`:

```cpp
#ifndef PD_YOURARCH_QUEUE_H
#define PD_YOURARCH_QUEUE_H

#include "FastAccelStepper.h"
#include "fas_queue/base.h"

class StepperQueue : public StepperQueueBase {
 public:
  // Architecture-specific state fields
  volatile bool _isRunning;
  // Add your hardware-specific fields:
  // - Timer/channel identifiers
  // - Peripheral handles
  // - State flags

  // Required methods
  bool isRunning();
  bool isReadyForCommands();

  // Common interface (implement in .cpp)
  AqeResultCode addQueueEntry(const struct stepper_command_s* cmd, bool start);
  int32_t getCurrentPosition();
  uint32_t ticksInQueue();
  bool hasTicksInQueue(uint32_t min_ticks);
  bool getActualTicksWithDirection(struct actual_ticks_s* speed);

  void init(uint8_t queue_num, uint8_t step_pin);
  void startQueue();
  void forceStop();
  void _initVars();
  void connect();
  void disconnect();

  void setDirPin(uint8_t dir_pin, bool _dirHighCountsUp);
  static bool isValidStepPin(uint8_t step_pin);

#if defined(NEED_FIXED_QUEUE_TO_PIN_MAPPING)
  static int8_t queueNumForStepPin(uint8_t step_pin);
#endif
};

#endif
```

### Step 4: Implement Queue Methods

Create `pd_yourarch/yourarch_queue.cpp` implementing all methods:

**Critical implementations:**

| Method | Purpose | Notes |
|--------|---------|-------|
| `_initVars()` | Zero all fields, set platform defaults | No hardware access, called by `tryAllocateQueue()` and `engine.init()` |
| `init()` | One-time hardware setup | Called by `tryAllocateQueue()` only; must not call `_initVars()` |
| `isRunning()` | Check if pulses are being generated | Can use flag or hardware state |
| `isReadyForCommands()` | Check if queue can accept commands | Usually `true`, may check hardware state |
| `startQueue()` | Begin pulse generation from queue head | Configure and enable timer/peripheral |
| `forceStop()` | Emergency stop, clear queue | Disable hardware, reset state |
| `connect()` | Attach step pin to peripheral | May configure GPIO matrix |
| `disconnect()` | Detach step pin | Return pin to GPIO mode |
| `isValidStepPin()` | Check if pin can generate pulses | Hardware constraint check |
| `setDirPin()` | Configure direction pin | May need port register setup |

**Position tracking options:**

1. **Software-only** (AVR, SAM, ESP32 RMT):
   - Track in `queue_end.pos`
   - Update in ISR/callback after each command

2. **Hardware counter** (ESP32 MCPWM with PCNT, Pico PIO):
   - Read from hardware register
   - May need offset tracking

**Running state detection:**

```cpp
// Simple flag approach
bool isRunning() { return _isRunning; }

// Hardware state approach (Pico PIO example)
bool isRunning() {
    return !pio_sm_is_tx_fifo_empty(pio, sm) || 
           pio_sm_get_pc(pio, sm) != 0;
}
```

### Step 5: Update Dispatcher

Add to `fas_queue/stepper_queue.h`:

```cpp
#elif defined(SUPPORT_YOUR_ARCH)
#include "pd_yourarch/yourarch_queue.h"
```

### Step 6: Engine Initialization (if needed)

If your architecture needs special initialization (RTOS task, interrupt setup), implement:

```cpp
// In your .cpp file
#if defined(SUPPORT_CPU_AFFINITY)
void fas_init_engine(FastAccelStepperEngine* engine, uint8_t cpu_core) {
    // Your initialization
}
#else
void fas_init_engine(FastAccelStepperEngine* engine) {
    // Your initialization
}
#endif
```

### Step 7: Testing

1. Create PC-based test stub in `pd_test/test_queue.h` with your architecture's `#ifdef`
2. Add test cases in `extras/tests/pc_based/`
3. Run: `make -C extras/tests/pc_based`

### Common Pitfalls

- **Missing interrupt macros**: `fasDisableInterrupts()`/`fasEnableInterrupts()` must be defined
- **Wrong TICKS_PER_S**: Must match your timer/counter frequency, not CPU frequency
- **Queue overflow**: Ensure `QUEUE_LEN` is power of 2 and matches queue index masking
- **Position drift**: Verify `getCurrentPosition()` accounts for commands in progress
- **Pin validation**: `isValidStepPin()` must reject pins your hardware cannot drive
