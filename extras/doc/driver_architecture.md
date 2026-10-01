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

`ticks` is deliberately 16 bit: the esp32 pulse generator registers are 16 bit
and this keeps AVR and esp32 on the same code path. The queue is only filled for
~10 ms ahead, so the application can react to position/speed/acceleration changes
almost instantly; wider tick values would only make the driver slower. Step rates
slower than 65535 ticks per step are produced by inserting `steps = 0` pause
commands between the step commands.

### Internal Queue Entry

```cpp
struct queue_entry {
    uint8_t steps;           // Number of steps (0 = pause only)
    uint8_t toggle_dir : 1;  // Direction changed mid-command
    uint8_t countUp : 1;     // Direction
    uint8_t hasSteps : 1;    // Has steps vs pause
    uint8_t dirPinState : 1; // Direction pin state (HIGH=1, LOW=0) at entry time
    uint16_t ticks;          // Period between steps
#if defined(SUPPORT_QUEUE_ENTRY_END_POS_U16)
    uint16_t end_pos_last16; // Position after this entry (AVR, SAM, SAMD)
#endif
#if defined(SUPPORT_QUEUE_ENTRY_START_POS_U16)
    uint16_t start_pos_last16; // Position at this entry (ESP32)
#endif
};
```

The `dirPinState` field is always present (not conditional). The `end_pos_last16` and `start_pos_last16` fields are conditional on platform feature flags.

---

## StepperQueueBase Common Interface

All architecture drivers inherit from `StepperQueueBase` (defined in `fas_queue/base.h`) and implement these methods.

### Public-facing methods (declared in `fas_queue/protocol.h`, implemented in shared .cpp files)

| Method | File | Description |
|--------|------|-------------|
| `init(queue_num, step_pin)` | per-platform .cpp | One-time hardware setup (called by `tryAllocateQueue()`) |
| `startQueue()` | per-platform .cpp | Begin pulse generation from queue head |
| `forceStop()` | per-platform .cpp | Immediately stop and clear queue |
| `connect()` | per-platform .cpp | Attach pulse generator to GPIO |
| `disconnect()` | per-platform .cpp | Detach pulse generator from GPIO |
| `isRunning()` | per-platform header | True if actively generating pulses |
| `isReadyForCommands()` | per-platform .cpp | True if can accept new commands |
| `setDirPin(dir_pin, _dirHighCountsUp)` | per-platform header | Configure direction pin |
| `isValidStepPin(pin)` | per-platform .cpp (static) | Check if pin can generate pulses |
| `getCurrentPosition()` | `queue_get_position.cpp` | Return current step count |
| `addQueueEntry(cmd, start)` | `queue_add_entry.cpp` | Add command to queue (common implementation) |
| `addDirChangePauseToQueue(cmd, start, dir_change_delay_ticks)` | per-platform header (inline) | Insert drain + direction-change pause commands |
| `ticksInQueue()` | `queue_utils.cpp` | Total ticks remaining in queue |
| `hasTicksInQueue(min_ticks)` | `queue_utils.cpp` | Whether queue has at least `min_ticks` of runway |
| `getActualTicksWithDirection(speed)` | `queue_utils.cpp` | Current step rate and direction from head command |
| `_initVars()` | `queue_init.cpp` | Zero all fields, call `_base_initVars()` then `_pd_initVars()` |

### Base class fields (in `StepperQueueBase`, defined in `fas_queue/base.h`)

| Field / Method | Description |
|----------------|-------------|
| `_base_initVars()` | Zero common fields (called by `_initVars()`) |
| `_pd_initVars()` | Platform-specific zeroing (declared in each platform's header) |
| `entry[QUEUE_LEN]` | Queue entries array |
| `read_idx` / `next_write_idx` | Single-producer/single-consumer indices |
| `queue_end` | Position, direction, count_up state |
| `ignore_commands` | Commands suspended during `forceStopAndNewPosition()` |
| `dirPin` / `dirHighCountsUp` | Direction pin configuration |
| `max_speed_in_ticks` | Maximum speed ceiling |
| `queueEntries()` | Number of entries in queue (inline) |
| `isQueueFull()` / `isQueueEmpty()` | Queue state checks (inline) |
| `hasStepsInQueue()` | Whether any step commands remain (inline) |
| `getMaxSpeedInTicks()` | Return max speed ceiling (inline) |
| `setAbsoluteSpeedLimit(ticks)` | Set max speed ceiling (conditional: `SUPPORT_UNSAFE_ABS_SPEED_LIMIT_SETTING`) |
| `_last_pause_ticks` / `_nr_of_pauses` / `clear_pause_stats()` | Pause statistics (conditional: `SUPPORT_PAUSE_CMD_COUNTING`, ESP32) |
| `_injected_pause_ticks` | Pause ticks injected for current `moveTimed()` call (always present) |

---

## Architecture Implementations

### Directory Structure

The `pd_` prefix in directory names stands for "pulse driver" (though "platform driver" is also suitable).

```
src/
  fas_queue/
    base.h              ← StepperQueueBase definition
    stepper_queue.h     ← Architecture dispatcher
    protocol.h          ← Protocol interface (direction pin macros, driver dispatch)
    queue_add_entry.cpp ← Common addQueueEntry() + addDirChangePauseToQueue()
    queue_utils.cpp     ← ticksInQueue(), hasTicksInQueue(), getActualTicksWithDirection()
    queue_get_position.cpp ← getCurrentPosition()
  pd_avr/
    avr_queue.h/cpp     ← AVR Timer implementation
  pd_esp32/
    esp32_queue.h/cpp   ← ESP32 queue wrapper (RMT + MCPWM + I2S dispatch)
    i2s_fill.cpp/h      ← I2S buffer fill helpers
    i2s_manager.cpp/h   ← I2S multiplex manager
    StepperISR_idf4_esp32_mcpwm_pcnt.cpp  ← IDF4 MCPWM/PCNT
    StepperISR_idf4_esp32_rmt.cpp         ← IDF4 RMT (legacy fill)
    StepperISR_idf4_esp32c3_rmt.cpp       ← IDF4 ESP32-C3
    StepperISR_idf4_esp32s3_rmt.cpp       ← IDF4 ESP32-S3
    StepperISR_idf5_esp32_mcpwm_pcnt.cpp  ← IDF5 MCPWM/PCNT (ESP32, S3, C6, H2)
    StepperISR_idf5_esp32_rmt.cpp         ← IDF5/6 RMT V2 encoder (only path, no legacy)
    StepperISR_idf5_esp32_rmt_encode.cpp  ← IDF5/6 RMT V2 fill encoder helper
    StepperISR_idf6_esp32_mcpwm_pcnt.cpp  ← IDF6 MCPWM/PCNT
    StepperISR_esp32xx_rmt.cpp            ← Shared RMT code
    StepperISR_esp32xx_rmt_encode.cpp     ← Shared RMT V2 code
  pd_sam/
    sam_queue.h/cpp     ← SAM Due implementation
  pd_samd/
    samd_queue.h/cpp    ← SAMD51 TCC implementation
  pd_pico/
    pico_queue.h/cpp    ← RP2040/RP2350 PIO implementation
    pico_pio.cpp/h      ← PIO program and state machine
  pd_teensy/
    teensy_queue.h/cpp  ← Teensy 4.x QuadTimer implementation
  pd_test/
    test_queue.h        ← PC-based test stub
```

### Platform Capabilities

| Architecture | Pulse Mechanism | Channels | Pin Flexibility |
|--------------|-----------------|----------|-----------------|
| **AVR** | Timer compare output (OC1A/OC1B/OC4A-C) | 2-3 | Fixed to specific pins |
| **ESP32 MCPWM/PCNT (IDF4/5/6)** | MCPWM generates, PCNT counts | 2-8 | Any GPIO |
| **ESP32 RMT (IDF4)** | RMT peripheral DMA (legacy fill) | 2-8 | Any GPIO |
| **ESP32 RMT (IDF5/6)** | RMT V2 encoder callback | 2-8 | Any GPIO |
| **ESP32 I2S (IDF5/6)** | I2S DMA bitstream | 3-64 | Any GPIO (direct/mux) |
| **ESP32P4 RMT** | RMT V2 only (no MCPWM/PCNT) | Config | Any GPIO |
| **ESP32-C3 RMT** | RMT peripheral | 2 | Any GPIO |
| **ESP32-C6/H2 MCPWM/PCNT** | MCPWM generates, PCNT counts | 0-2 | Any GPIO |
| **SAM Due** | PWM + pin change interrupt | 6 | Limited (specific PWM pins) |
| **SAMD51** | TCC PWM + overflow ISR | 3-5 | Specific TCC channels |
| **Teensy 4.x** | QuadTimer (TMR) + digitalWriteFast | 16 | Any digital pin |
| **RP2040/RP2350** | PIO state machine | 4-8 | GPIO 0-31 |

---

## Interface Summary by Architecture

### Hardware → Queue (Driver Outputs)

| Interface | AVR | ESP32 MCPWM | ESP32 RMT | ESP32 I2S | SAM Due | SAMD51 | Teensy | Pico |
|-----------|-----|-------------|-----------|-----------|---------|--------|--------|------|
| `isRunning()` | `_isRunning` flag | `_isRunning` flag | `_isRunning` + `_rmtStopped` | `_isRunning` | `_hasISRactive` | `_isRunning` | `_isRunning` | `_isActive` + PIO state |
| `isReadyForCommands()` | always `true` | check timer | check `!_rmtStopped` | check `!_rmtStopped` | always `true` | always `true` | always `true` | PIO FIFO/PC |
| `getCurrentPosition()` | from `queue_end` | from `queue_end` | from `queue_end` | from `queue_end` | from `queue_end` | from `queue_end` | from `queue_end` | PIO count + offset |

### Queue → Hardware (Driver Inputs)

| Interface | AVR | ESP32 | SAM Due | SAMD51 | Teensy | Pico |
|-----------|-----|-------|---------|--------|--------|------|
| `init()` | Timer + compare config | No-op (driver inits called by `tryAllocateQueue()`) | PWM + GPIO config | TCC config | TMR config | Set `_step_pin` |
| `startQueue()` | Enable compare interrupt | Apply command + run | Attach PWM peripheral | TCC start | TMR prime | Push to FIFO |
| `forceStop()` | Disable interrupt | Stop peripheral | Disable PWM channel | Stop TCC | Stop TMR | Disable SM |
| `connect()` | no-op | GPIO matrix (driver-dispatched) | GPIO config | TCC GPIO | digitalWriteFast | `pio_gpio_init()` |
| `disconnect()` | no-op | GPIO matrix (driver-dispatched) | PWM channel disable | TCC disable | digitalWrite | `gpio_init()` |

### manageSteppers() Invocation

| Architecture | Mechanism | Location |
|--------------|-----------|----------|
| **AVR** | Timer OVF ISR | `pd_avr/avr_queue.cpp` |
| **ESP32** | FreeRTOS task | `pd_esp32/esp32_queue.cpp` |
| **SAM Due** | Timer ISR | `pd_sam/sam_queue.cpp` |
| **SAMD51** | TCC overflow ISR | `pd_samd/samd_queue.cpp` |
| **Teensy 4.x** | QuadTimer compare match ISR | `pd_teensy/teensy_queue.cpp` |
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

### Buffer Depth / In-Flight (Read-Ahead) Contract

A pulse driver may consume commands out of the queue into its own hardware
pipeline ahead of the pin ("in flight"). The ramp generator fills the queue to
`_forward_planning_in_ticks` (default 20 ms, set with
`setForwardPlanningTimeInMs()`), so **while a move is running the queue must
not run low**. The contract is:

```
forward_planning_ticks  >  driver maximum in-flight time
```

Each driver must state how much it can drain at most; a driver that buffers
more than the forward planning time can drain the queue mid-move, find it
empty, and stop or under-run (that is the 040 bug).

| Driver | Maximum in-flight (read-ahead) |
|--------|--------------------------------|
| AVR / SAM / SAMD / Teensy / Pico | one command (no buffered pipeline) |
| ESP32 MCPWM/PCNT | ~one command (generator latches at TEZ/TEP) |
| ESP32 I2S | `I2S_BLOCK_COUNT*I2S_BLOCK_TICKS` = 2 x 500 us = 1 ms |
| ESP32 RMT (IDF4) | `2*PART_SIZE` symbols (RMT memory half ping-pong) |
| ESP32 RMT (IDF5/6) | `(2*PART_SIZE + min_chunk_size) * RMT_MAX_SYMBOL_TICKS` = `3*RMT_BLOCK_TICKS` = 24000 t = 1.5 ms; every sub-entry `<= RMT_MAX_SYMBOL_TICKS = RMT_BLOCK_TICKS/PART_SIZE` (so one RMT half is `<= 2*RMT_BLOCK_TICKS` = 1 ms) |

The IDF5/6 RMT row is the 040 F2 fix: `rmt_encode_fill()` splits each step's
low phase so every RMT sub-entry is capped, bounding the buffer's playback time
well below `forward_planning_ticks`. The direction-change drain uses the same
`3*RMT_BLOCK_TICKS`. Before F2 the encoder could pack long symbols and drain the
queue; see `implemented/idf6_rmt_slow.md`.

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
Buffered drivers (RMT, I2S) skip those pauses when `SUPPORT_PAUSE_CMD_COUNTING`
shows the caller already queued them. See `extras/doc/esp32_i2s_driver.md`.

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

Direction pin control is abstracted through a **preprocessor-driven macro system**
declared in `fas_queue/protocol.h`. Each platform's `StepperQueue` header defines
these macros according to its capabilities. The common `addQueueEntry()` code
uses only these macros — it never references platform-specific GPIO functions.

There are four control modes:

| Mode | Description | Platforms |
|------|-------------|-----------|
| **Synchronized with commands** | Pin state changes happen in the ISR alongside step pulses | AVR, SAM, SAMD51, Teensy, Pico |
| **Synchronized with buffer** | Driver controls pin at buffer boundaries; timing depends on buffer size | ESP32 I2S (both direct and mux) |
| **Queue controlled** | `addQueueEntry()` controls pin as GPIO; requires queue empty | ESP32 RMT (IDF5/6) |
| **Externally controlled** | Pin controlled via application callback (e.g. I/O expander) | Any, via `PIN_EXTERNAL_FLAG` |

Required macros (defined in each platform's header):

| Macro | Purpose | Example values |
|-------|---------|----------------|
| `SET_DIRECTION_PIN_STATE(q, high)` | Set absolute HIGH/LOW state | Port register write, `gpio_ll_set_level()`, `pio_sm_exec()` |
| `AFTER_SET_DIR_PIN_DELAY_US` | Microseconds to delay after setting pin (when queue empty/not running) | 5 (Teensy), 30 (SAMD51, SAM) |
| `SET_ENABLE_PIN_STATE(q, pin, high)` | Set enable pin state (called in `addQueueEntry()` context, ~4 ms rate) | `digitalWrite()`, GPIO LL |

Direction-change pause insertion (in `addDirChangePauseToQueue()`):

| Driver | Before-pause pauses | Before-pause ticks | After-pause ticks |
|--------|-------------------|--------------------|--------------------|
| RMT (IDF4) | 1 × MIN_CMD_TICKS | 0 | 0 |
| RMT (IDF5/6) | 0 (tick-based drain) | 3 × RMT_BLOCK_TICKS (1.5 ms) | 0 |
| I2S GPIO DIR | 2 × I2S_BLOCK_TICKS (old dir) | 2 × I2S_BLOCK_TICKS | 0 |
| I2S mux DIR | none (mask applies to next block) | 0 | I2S_BLOCK_TICKS |
| MCPWM/PCNT | 1 × MIN_CMD_TICKS | MIN_CMD_TICKS | 0 |

The drain pauses are generated incrementally: each `addQueueEntry()` call may
insert one pause and return `AQE_DIR_CHANGE_PAUSE_INJECTED`; the caller retries
until the recorded pause state (tracked by `_nr_of_pauses` / `_last_pause_ticks`)
satisfies the driver's requirement.

See `extras/doc/esp32_i2s_driver.md` for I2S multiplex details.

---

## Timing Constants

| Architecture | TICKS_PER_S | MIN_CMD_TICKS | MIN_DIR_DELAY_US | Notes |
|--------------|-------------|---------------|------------------|-------|
| AVR | `F_CPU` (typically 16 MHz) | `TICKS_PER_S / 25000` | 40 µs | Uses actual CPU clock, not hardcoded |
| ESP32 | 16,000,000 | `TICKS_PER_S / 5000` | 200 µs | Fixed 16 MHz |
| SAM Due | 21,000,000 | `TICKS_PER_S / 5000` | 200 µs | Fixed 21 MHz |
| SAMD51 | 16,000,000 | `TICKS_PER_S / 5000` | 200 µs | Dedicated GCLK (DFLL48M/3) |
| Teensy 4.x | `(150000000L >> FAS_TEENSY_TMR_PRESCALE)` | `TICKS_PER_S / 5000` | 200 µs | Default 9.375 MHz (prescale 4), configurable 0–7 |
| Pico | 16,000,000 | `TICKS_PER_S / 5000` | 200 µs | 80 MHz / 5 prescaler |

The precomputed log2 constants in `fas_ramp/RampCalculator.h` only exist for
`16000000L` and `21000000L`. Any other `TICKS_PER_S` falls back to the generic
runtime-variable path (`log2_timer_freq*` in `RampControl.cpp`). That path works,
but it costs a little RAM and compute per conversion.

**Recommendation for a new driver: add another precomputed constant** instead of
relying on the generic path. Add the three constants for your `TICKS_PER_S` to
`log2/Log2RepresentationConst.h` (regenerate with `extras/gen_log2_const`) and a
matching `#elif (TICKS_PER_S == ...)` branch in `RampCalculator.h` supplying:

- `LOG2_TICKS_PER_S` — log2 of `TICKS_PER_S`
- `LOG2_TICKS_PER_S_DIV_SQRT_OF_2` — log2 of `TICKS_PER_S / sqrt(2)`
- `LOG2_ACCEL_FACTOR` — log2 of `TICKS_PER_S^2 / 2`

and the `US_TO_TICKS` / `TICKS_TO_US` conversions. Follow the existing 16 MHz and
21 MHz branches as templates.

**Note for Teensy:** The configurable prescale means `TICKS_PER_S` varies between
1.17 MHz and 150 MHz depending on `FAS_TEENSY_TMR_PRESCALE`. Only prescale 4
(default, 9.375 MHz) and below produce values that fit in the 16-bit pipeline
constants (≤ 32.7 MHz). Prescales 5–7 use the generic runtime log2 path.

### TICKS_PER_S Upper Bound (subtle indirect requirement)

`queue_entry::ticks` and `stepper_command_s::ticks` are `uint16_t`, and several
internal pipeline constants express fixed time spans (1 ms and 2 ms) that are
stored in 16-bit variables:

```cpp
uint16_t max_speed_in_ticks = TICKS_PER_S / 1000;  // base.h, 1 ms
uint16_t ps = TICKS_PER_S / 500;                   // RampControl.cpp, 2 ms
```

For these to be representable, `TICKS_PER_S / 500` must fit in 16 bits:
`TICKS_PER_S <= 65535 * 500 ≈ 32.7 MHz`. At 16 MHz / 21 MHz the values fit
comfortably; at e.g. 72 MHz, `TICKS_PER_S / 500 == 144000` silently overflows.

This requirement is **indirect**: no compile-time assertion enforces it, so a
port that sets `TICKS_PER_S` to the raw CPU/timer clock (e.g. 72 MHz) can compile
but produce wrong motion.

**Therefore keep `TICKS_PER_S` near 16 MHz.** If the timer clock is much higher,
divide it down with a hardware prescaler and set `TICKS_PER_S` to the *prescaled*
frequency (as the Pico effectively does with its 80 MHz clock, and STM32 must do
via the TIM prescaler) instead of passing the raw clock.

---

## SUPPORT_ macros

Chip and framework detection stays in `fas_arch/` and `pd_*/pd_config.h`.
Those headers are the only place that tests `ARDUINO_ARCH_*`,
`ESP_IDF_VERSION`, `__AVR__`, and the other toolchain macros. From that
they define one `SUPPORT_...` macro per capability the build actually
has, such as `SUPPORT_ESP32_RMT` or `SUPPORT_QUEUE_ENTRY_END_POS_U16`.

The rest of `src/` — the ramp, the queue, `FastAccelStepper` — tests
those `SUPPORT_` macros and does not test the architecture again. A
new behavior difference is a new flag in the platform header, not
another chip test in the shared file. Production code then compiles
the same way for every target that offers the capability.

### Key feature flags

| Macro | Meaning | Platforms |
|-------|---------|-----------|
| `SUPPORT_QUEUE_ENTRY_END_POS_U16` | Queue entry stores position after entry | AVR, SAM, SAMD51 |
| `SUPPORT_QUEUE_ENTRY_START_POS_U16` | Queue entry stores position at entry | ESP32 |
| `SUPPORT_PAUSE_CMD_COUNTING` | Driver tracks pause count for direction-change drain | ESP32 (RMT, MCPWM) |
| `SUPPORT_SELECT_DRIVER_TYPE` | User can choose between multiple driver types at allocation | ESP32 (when ≥2 drivers available) |
| `SUPPORT_DYNAMIC_ALLOCATION` | Queues allocated via `new` instead of static array | ESP32 (IDF5+) |
| `SUPPORT_UNSAFE_ABS_SPEED_LIMIT_SETTING` | User can override max speed ceiling | AVR, ESP32, SAMD51, Teensy |
| `SUPPORT_CPU_AFFINITY` | Engine can be pinned to a specific CPU core | ESP32 |
| `SUPPORT_TASK_RATE_CHANGE` | Task rate can be changed at runtime | ESP32, Pico |
| `SUPPORT_ESP32_RMT_V2` | RMT V2 encoder API available (no legacy fill) | All IDF5/6 chips |
| `SUPPORT_ESP32_RMT_SYNC` | RMT TX synchronisation (ESP32 classic, not S3/C3) | ESP32 (IDF5/6) |
| `SUPPORT_ESP32_PULSE_COUNTER` | PCNT unit count (0 = not available) | Varies by chip |
| `NEED_FIXED_QUEUE_TO_PIN_MAPPING` | Stepper count limited by hardware pin mapping | AVR |
| `NEED_GENERIC_GET_CURRENT_POSITION` | Use generic (non-hardware-counter) position | AVR, SAM, SAMD51, Teensy |

## Preprocessor Defines by Architecture

| Define | AVR | ESP32 MCPWM | ESP32 RMT | ESP32 I2S | SAM | SAMD51 | Teensy | Pico |
|--------|-----|-------------|-----------|-----------|-----|--------|--------|------|
| `SUPPORT_AVR` | ✓ | | | | | | | |
| `SUPPORT_ESP32` | | ✓ | ✓ | ✓ | | | | |
| `SUPPORT_ESP32_MCPWM_PCNT` | | ✓ | | | | | | |
| `SUPPORT_ESP32_RMT` | | | ✓ | | | | | |
| `SUPPORT_ESP32_I2S` | | | | ✓ | | | | |
| `SUPPORT_SAM` | | | | | ✓ | | | |
| `SUPPORT_SAMD51` | | | | | | ✓ | | |
| `SUPPORT_TEENSY4` | | | | | | | ✓ | |
| `SUPPORT_RP_PICO` | | | | | | | | ✓ |

---

## Adding a New Architecture

Porting to a new microcontroller requires several coordinated changes across the codebase.
The common queue code (shared across all platforms) lives in `fas_queue/` —
you do **not** implement `addQueueEntry()`, `ticksInQueue()`, `hasTicksInQueue()`,
or `getActualTicksWithDirection()` yourself. Those are already in
`queue_add_entry.cpp`, `queue_utils.cpp`, and `queue_get_position.cpp`.

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

### Step 2: Create Platform Config Header

Create `pd_yourarch/pd_config.h` defining all required constants and feature flags:

```cpp
#ifndef PD_YOURARCH_CONFIG_H
#define PD_YOURARCH_CONFIG_H

// 1. Queue topology (required)
#define QUEUE_LEN 32            // queue depth (power of 2)
#define NUM_QUEUES 4            // number of steppers supported
#define MAX_STEPPER (NUM_QUEUES)

// 2. Core timing (required)
#define TICKS_PER_S 16000000L   // your timer frequency (see Timing Constants)
#define MIN_CMD_TICKS (TICKS_PER_S / 5000)
#define MIN_DIR_DELAY_US 200    // direction pin setup time (µs)
#define MAX_DIR_DELAY_US (65535 / (TICKS_PER_S / 1000000))
#define DELAY_MS_BASE 4         // base for debug LED timing

// 3. Debug (optional)
#define DEBUG_LED_HALF_PERIOD 50
#define noop_or_wait            // empty for non-RTOS, vTaskDelay(1) for FreeRTOS

// 4. Feature flags (define if supported)
#define SUPPORT_QUEUE_ENTRY_END_POS_U16    // queue entry stores position after entry
#define SUPPORT_QUEUE_ENTRY_START_POS_U16  // queue entry stores position at entry
#define SUPPORT_UNSAFE_ABS_SPEED_LIMIT_SETTING  // user can override max speed
#define SUPPORT_CPU_AFFINITY               // engine can be pinned to CPU core
#define SUPPORT_TASK_RATE_CHANGE           // task rate changeable at runtime
#define NEED_FIXED_QUEUE_TO_PIN_MAPPING    // steppers limited by hardware pins
#define NEED_GENERIC_GET_CURRENT_POSITION  // use software position tracking

// 5. Interrupt control (required — in fas_arch/your_arch.h)
#define fasDisableInterrupts()  // disable interrupts
#define fasEnableInterrupts()   // restore interrupts

// 6. Task management (if using RTOS)
#define noop_or_wait vTaskDelay(1)

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
#include "../fas_queue/protocol.h"   // injects protocol methods

  // Architecture-specific state fields
  volatile bool _isRunning;
  // Add your hardware-specific fields:
  // - Timer/channel handles
  // - Peripheral state
  // - Direction pin port/mask pointers

  // Platform-specific init (called by _initVars wrapper)
  inline void _pd_initVars() {
    _isRunning = false;
    max_speed_in_ticks = 80;   // default ceiling (adjust for your hardware)
  }

  // Required methods (declared in protocol.h, implemented in your .cpp)
  bool isRunning() const;
  bool isReadyForCommands() const;
  void init(uint8_t queue_num, uint8_t step_pin);
  void startQueue();
  void forceStop();
  void connect();
  void disconnect();
  void setDirPin(uint8_t dir_pin, bool _dirHighCountsUp);
  static bool isValidStepPin(uint8_t step_pin);
  int32_t getCurrentPosition();

#if defined(NEED_FIXED_QUEUE_TO_PIN_MAPPING)
  static int8_t queueNumForStepPin(uint8_t step_pin);
#endif
};

// Direction pin control macros (required — see protocol.h)
#define SET_DIRECTION_PIN_STATE(q, high) /* your implementation */
#define SET_ENABLE_PIN_STATE(q, pin, high) /* your implementation */
// Optional: #define AFTER_SET_DIR_PIN_DELAY_US 30  // µs delay when queue empty

#endif
```

### Step 4: Implement Queue Methods

Create `pd_yourarch/yourarch_queue.cpp` implementing the methods declared
in `protocol.h` (injected via `#include` in the header):

**Critical implementations:**

| Method | Purpose | Notes |
|--------|---------|-------|
| `_pd_initVars()` | Platform-specific zeroing | Called by `_initVars()` wrapper in `queue_init.cpp` after `_base_initVars()` |
| `init()` | One-time hardware setup | Called by `tryAllocateQueue()` only; must not call `_initVars()` |
| `isRunning()` | Check if pulses being generated | Flag or hardware state |
| `isReadyForCommands()` | Check if queue can accept commands | Usually `true` |
| `startQueue()` | Begin pulse generation | Configure and enable timer/peripheral |
| `forceStop()` | Emergency stop, clear queue | Disable hardware, reset state |
| `connect()` | Attach step pin to peripheral | May configure GPIO matrix |
| `disconnect()` | Detach step pin | Return pin to GPIO mode |
| `isValidStepPin()` | Reject invalid pins | Hardware constraint check |
| `getCurrentPosition()` | Return current step count | Software (`queue_end.pos`) or hardware counter |

**Direction pin control (macros):**

Each platform must define `SET_DIRECTION_PIN_STATE(q, high)` and
`SET_ENABLE_PIN_STATE(q, pin, high)` macros. These are called from the
common `addQueueEntry()` code in `queue_add_entry.cpp` — your platform
header defines them, the common code uses them.

Optionally define `AFTER_SET_DIR_PIN_DELAY_US` (µs) for a delay when the
queue is empty and not running (needed when direction pin is shared).

**Position tracking options:**

1. **Software-only** (AVR, SAM, SAMD51, Teensy):
   - Track in `queue_end.pos`
   - Update in ISR/callback after each command
   - Define `NEED_GENERIC_GET_CURRENT_POSITION`

2. **Hardware counter** (ESP32 MCPWM with PCNT, Pico PIO):
   - Read from hardware register
   - May need offset tracking (`pos_offset`)

**Running state detection:**

```cpp
// Simple flag approach (most platforms)
bool isRunning() const { return _isRunning; }

// Hardware state approach (Pico combines flag + PIO state)
bool isRunning() const {
    return _isActive &&
           (!pio_sm_is_tx_fifo_empty(pio, sm) ||
            pio_sm_get_pc(pio, sm) != 0);
}
```

### Step 5: Update Dispatcher

Add to `fas_queue/stepper_queue.h`:

```cpp
#elif defined(SUPPORT_YOUR_ARCH)
#include "pd_yourarch/yourarch_queue.h"
```

### Step 6: Engine Initialization (if needed)

If your architecture needs special initialization (RTOS task, interrupt setup),
implement in your .cpp file. The dispatcher in `stepper_queue.h` already
declares both variants:

```cpp
#if defined(SUPPORT_CPU_AFFINITY)
void fas_init_engine(FastAccelStepperEngine* engine, uint8_t cpu_core);
#else
void fas_init_engine(FastAccelStepperEngine* engine);
#endif
```

### Step 7: Testing

1. Create PC-based test stub in `pd_test/test_queue.h` with your architecture's `#ifdef`
2. Add test cases in `extras/tests/pc_based/`
3. Run: `make -C extras/tests/pc_based`

### Common Pitfalls

- **Missing interrupt macros**: `fasDisableInterrupts()`/`fasEnableInterrupts()` must be defined
- **Wrong TICKS_PER_S**: Must match your timer/counter frequency, not CPU frequency, and must stay below ~32.7 MHz so the pipeline's 1 ms/2 ms constants fit in `uint16_t` (see Timing Constants above)
- **Queue overflow**: Ensure `QUEUE_LEN` is power of 2 and matches queue index masking
- **Position drift**: Verify `getCurrentPosition()` accounts for commands in progress
- **Pin validation**: `isValidStepPin()` must reject pins your hardware cannot drive
- **Direction pin macros**: `SET_DIRECTION_PIN_STATE` and `SET_ENABLE_PIN_STATE` must be defined in your platform header — the common code will not compile without them
- **`_pd_initVars()` not called**: The common `_initVars()` in `queue_init.cpp` calls both `_base_initVars()` and `_pd_initVars()` — your platform must define `_pd_initVars()`
