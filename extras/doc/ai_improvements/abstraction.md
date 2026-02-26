# StepperISR Driver Abstraction

The StepperISR layer sits between the FastAccelStepper high-level API and the
hardware-specific pulse generation drivers. It abstracts different microcontroller
timer/peripheral implementations into a uniform command queue interface.

## Abstract Model

Think of each `StepperQueue` instance as a **generic pulse generator** that:

1. Accepts commands specifying `steps` and `ticks` (period between steps)
2. Outputs step pulses on a GPIO pin
3. Tracks position based on executed pulses

The key insight: depending on the microcontroller, these pulse generators can be
assigned to GPIO pins either freely (ESP32, Pico) or only to specific pins
(AVR Timer1 on OC1A/OC1B).

## Core Data Structures

### Command Interface (from RampGenerator)

```cpp
struct stepper_command_s {
    uint16_t ticks;    // Period between steps (in TICKS_PER_S units)
    uint8_t steps;    // Number of steps (0 = pause/no steps)
    bool count_up;    // Direction: true = count up, false = count down
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
    uint8_t hasSteps : 1;   // Has steps vs pause
    uint16_t ticks;          // Period between steps
};
```

## Public Interface (StepperQueue)

All architecture drivers implement this interface:

| Method | Description |
|--------|-------------|
| `init(queue_num, step_pin)` | Initialize hardware, return false if pin invalid |
| `startQueue()` | Begin pulse generation from queue head |
| `forceStop()` | Immediately stop and clear queue |
| `connect()` | Attach pulse generator to GPIO |
| `disconnect()` | Detach pulse generator from GPIO |
| `isRunning()` | True if actively generating pulses |
| `isReadyForCommands()` | True if can accept new commands |
| `setDirPin(dir_pin)` | Configure direction pin |
| `isValidStepPin(pin)` | Check if pin can generate pulses |
| `getCurrentPosition()` | Return current step count |
| `addQueueEntry(cmd, start)` | Add command to queue |

## Architecture-Specific Implementations

| Architecture | Pulse Mechanism | Channels | Pin Flexibility |
|--------------|----------------|----------|-----------------|
| **AVR** | Timer compare output (OC1A/OC1B/OC4A-C) | 2-3 | Fixed to specific pins |
| **ESP32 MCPWM/PCNT** | MCPWM generates, PCNT counts | 0-6 | Any GPIO |
| **ESP32 RMT (IDF4)** | RMT peripheral DMA | 2-8 | Any GPIO |
| **ESP32 RMT (IDF5)** | RMT with encoder callback | 2-8 | Any GPIO |
| **ESP32-C3/S3 RMT** | RMT peripheral | 2-4 | Any GPIO |
| **SAM Due** | PWM + pin change interrupt | 6 | Limited (specific PWM pins) |
| **RP2040/RP2350** | PIO state machine | 8 | GPIO 0-31 |

## Key Differences Between Implementations

### 1. Pin Assignment

**Fixed mapping (AVR, SAM)**:
```cpp
// AVR: Fixed queue-to-pin mapping
uint8_t queueNumForStepPin(uint8_t pin) {
    if (pin == 9 || pin == 10) return 0;  // OC1A/OC1B
    ...
}
```

**Flexible mapping (ESP32, Pico)**:
```cpp
// ESP32 RMT/Pico PIO: Any queue can drive any valid GPIO
// isValidStepPin() returns true for all valid pins
```

### 2. Position Tracking

- **AVR/Due**: Position tracked via queue state
  ```cpp
  int32_t getCurrentPosition() {
      return queue_end.pos + performedSteps;
  }
  ```

- **ESP32 MCPWM**: Hardware pulse counter
  ```cpp
  uint16_t _getPerformedPulses_mcpwm_pcnt() {
      return PCNT.cnt_unit[pcnt_unit].cnt_val;
  }
  ```

- **ESP32 RMT**: No hardware counter, position from queue only

- **Pico**: PIO RX FIFO returns step count
  ```cpp
  int32_t getCurrentStepCount() {
      return (int32_t)pio->rxf[sm] - startPos;
  }
  ```

### 3. Running State Detection

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

### 4. Minimum Command Constraints

| Architecture | MIN_CMD_TICKS | Notes |
|--------------|---------------|-------|
| AVR | 640 | F_CPU / 25000 = 16MHz / 25000 |
| ESP32 | 3200 | Higher overhead from ESP-IDF |
| SAM Due | 4200 | 84MHz / 20 |
| Pico | 125 | 125MHz / 1000 (PIO runs at system clock) |

### 5. Timer/Peripheral Frequency

| Architecture | TICKS_PER_S | Source |
|--------------|-------------|--------|
| AVR | 16,000,000 | F_CPU |
| ESP32 | 16,000,000 | APB 80MHz / 5 |
| SAM Due | 21,000,000 | 84MHz / 4 |
| Pico | System clock | 125MHz default |

## Required Preprocessor Defines

| Define | Purpose |
|--------|---------|
| `NUM_QUEUES` | Number of pulse generators available |
| `QUEUE_LEN` | Command queue depth (16 for AVR, 32 for others) |
| `TICKS_PER_S` | Timer tick frequency |
| `MIN_CMD_TICKS` | Minimum command duration in ticks |
| `SUPPORT_DIR_PIN_MASK` | Type for direct port access (AVR) |
| `NEED_FIXED_QUEUE_TO_PIN_MAPPING` | Set if pins are hardcoded |

## Implementation Pattern

Each architecture driver (`StepperISR_<arch>.cpp`) must provide:

1. **Global queue array**: `StepperQueue fas_queue[NUM_QUEUES];`
2. **Engine initialization**: `fas_init_engine(engine)`
3. **ISRs**: Handle pulse generation and advance queue read pointer
4. **Hardware setup**: Configure timer/RMT/PIO/PCNT in `init()`

The common `StepperISR.cpp` handles:
- `addQueueEntry()` - Converts `stepper_command_s` to `queue_entry`
- Queue state tracking (`queue_end`)
- Direction pin management

## Directory Structure

```
src/
  StepperISR.h           # Common interface definition
  StepperISR.cpp         # Shared queue management
  StepperISR_avr.cpp     # AVR Timer implementation
  StepperISR_esp32.cpp   # ESP32 MCPWM/PCNT
  StepperISR_idf4_esp32_rmt.cpp       # ESP32 RMT (IDF4)
  StepperISR_idf5_esp32_rmt.cpp       # ESP32 RMT (IDF5)
  StepperISR_idf4_esp32c3_rmt.cpp     # ESP32-C3 RMT
  StepperISR_idf4_esp32s3_rmt.cpp     # ESP32-S3 RMT
  StepperISR_due.cpp     # SAM Due implementation
  StepperISR_rp_pico.cpp # Raspberry Pi Pico PIO
```

## Adding a New Architecture

To port to a new microcontroller:

1. Define architecture-specific constants in `fas_arch/common_<arch>.h`
2. Create `StepperISR_<arch>.cpp` implementing the public interface
3. Implement hardware-specific:
   - `init(queue_num, step_pin)` - Setup timer/peripheral
   - `connect()/disconnect()` - Attach/detach GPIO
   - `startQueue()` - Begin pulse generation
   - `forceStop()` - Emergency stop
   - ISR or callback for command completion
4. Add conditional compilation in `StepperISR.h`
