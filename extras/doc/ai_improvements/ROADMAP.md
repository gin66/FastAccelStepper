# FastAccelStepper Driver Refactoring Roadmap

## Problem Statement

`StepperISR.h` currently defines `StepperQueue` as a single class containing
fields and methods for **all** supported architectures behind a wall of
`#ifdef` guards:

```cpp
class StepperQueue {
  ...
#if defined(SUPPORT_AVR)
  volatile bool _isRunning;
  enum channels channel;
  inline bool isRunning() { return _isRunning; }
#endif
#if defined(SUPPORT_ESP32)
  volatile bool _isRunning;
  bool _nextCommandIsPrepared;
  ...
#endif
#if defined(SUPPORT_RP_PICO)
  PIO pio;
  uint sm;
  bool _isActive;
  ...
#endif
  ...
};
```

This has several drawbacks:
- Hard to read: understanding one architecture requires mentally filtering out all others
- Hard to extend: adding a new architecture means touching the shared header
- Coupling problem: arch-specific methods like `startQueue()` need both
  hardware state (arch-specific fields) and queue state (common fields),
  making it hard to separate responsibilities cleanly
- The `.cpp` files (`StepperISR_avr.cpp`, `StepperISR_rp_pico.cpp`, etc.)
  implement methods of this monolithic class

---

## Design Decision: Per-Architecture StepperQueue

### Core Idea

Define `StepperQueue` **per architecture** in the respective `pd_*/` directory.
A common base struct holds all shared fields. Each architecture extends it with
its own hardware-specific fields and method implementations.

```
src/
  fas_queue/
    base.h          ← StepperQueueBase: all common fields
  pd_avr/
    avr_queue.h     ← class StepperQueue : public StepperQueueBase (AVR)
    avr_queue.cpp   ← AVR-specific method implementations
  pd_esp32/
    esp32_queue.h   ← class StepperQueue : public StepperQueueBase (ESP32)
    esp32_queue.cpp ← ESP32-specific method implementations
  pd_pico/
    pico_queue.h    ← class StepperQueue : public StepperQueueBase (Pico)
    pico_queue.cpp  ← Pico-specific method implementations
  pd_sam/
    sam_queue.h     ← class StepperQueue : public StepperQueueBase (SAM)
    sam_queue.cpp   ← SAM-specific method implementations
  StepperISR.h      ← thin header: includes base + correct pd_*/ header
  StepperISR.cpp    ← pure common code, zero #ifdefs
```

### Shape of the Common Base

```cpp
// fas_queue/base.h
struct StepperQueueBase {
  struct queue_entry entry[QUEUE_LEN];
  volatile bool ignore_commands;
  volatile uint8_t read_idx;
  volatile uint8_t next_write_idx;
  bool dirHighCountsUp;
  uint8_t dirPin;
  struct queue_end_s queue_end;
  uint16_t max_speed_in_ticks;
  // dir pin mask fields (conditionally)
  // common method declarations (addQueueEntry, getCurrentPosition, ...)
};
```

### Shape of a Per-Architecture Queue

```cpp
// pd_avr/avr_queue.h
class StepperQueue : public StepperQueueBase {
 public:
  // AVR-specific hardware fields
  volatile bool _noMoreCommands;
  volatile bool _isRunning;
  enum channels channel;

  // Methods have access to both hardware fields (above)
  // and queue fields (inherited from StepperQueueBase)
  inline bool isRunning() const { return _isRunning; }
  inline bool isReadyForCommands() const { return true; }
  void startQueue();    // can freely access read_idx, entry[], channel
  void forceStop();
  bool init(FastAccelStepperEngine* engine, uint8_t queue_num,
            uint8_t step_pin);
  void connect();
  void disconnect();
  static bool isValidStepPin(uint8_t step_pin);
};
```

### Why This Solves the Coupling Problem

Previously `startQueue()` needed both queue fields (`read_idx`, `entry[]`)
and hardware fields (`channel`, `pio`, `_isRunning`). As a method of a mixed
class this was messy but worked. Any attempt to move logic into a separate
`DriverData` struct hit the problem that the struct didn't own the queue fields.

With the per-arch `StepperQueue`, **all fields are in scope naturally** —
there is no coupling problem. `startQueue()` is just a regular method of the
class that happens to own both the hardware and queue state.

### Why Not a Pure Driver Struct Approach

An earlier attempt moved hardware fields into a nested `driver` struct:
```cpp
class StepperQueue {
  AvrDriverData driver;  // hardware fields
  // + common fields
};
```
This forced every access like `driver._isRunning`, `driver.channel`, etc., and
methods that needed both sides (most of them) still had to reach across the
boundary. It also left `StepperISR.h` full of `#ifdefs`. Abandoned.

### StepperISR.h Becomes a Thin Dispatcher

```cpp
// StepperISR.h
#include "fas_arch/common.h"
#include "fas_queue/base.h"

#if defined(SUPPORT_AVR)
#include "pd_avr/avr_queue.h"
#elif defined(SUPPORT_ESP32)
#include "pd_esp32/esp32_queue.h"
#elif defined(SUPPORT_RP_PICO)
#include "pd_pico/pico_queue.h"
#elif defined(SUPPORT_SAM)
#include "pd_sam/sam_queue.h"
#elif defined(TEST)
#include "pd_test/test_queue.h"
#endif

extern StepperQueue fas_queue[NUM_QUEUES];
```

### StepperISR.cpp Becomes Truly Common

```cpp
// StepperISR.cpp — no architecture #ifdefs
#include "StepperISR.h"

AqeResultCode StepperQueue::addQueueEntry(...) { ... }
int32_t StepperQueue::getCurrentPosition() { ... }
uint32_t StepperQueue::ticksInQueue() { ... }
// etc.
```

---

## Implementation Steps

### Step 1 — Define StepperQueueBase
- Extract all architecture-independent fields from `StepperQueue` into
  `fas_queue/base.h` as `StepperQueueBase`
- Keep common method declarations here
- No `#ifdef` inside the base

### Step 2 — AVR (simplest architecture, proof of concept)
- Create `pd_avr/avr_queue.h` with `class StepperQueue : public StepperQueueBase`
- Move AVR-specific fields and method declarations into it
- Rename `StepperISR_avr.cpp` → `pd_avr/avr_queue.cpp`
- Verify AVR builds (simavr tests)

### Step 3 — TEST stub
- Create `pd_test/test_queue.h` for PC-based tests
- Migrate `StepperISR_test.cpp`
- Run PC tests to validate the base + derived structure works

### Step 4 — SAM
- Create `pd_sam/sam_queue.h` + `pd_sam/sam_queue.cpp`
- Migrate from `StepperISR_due.cpp`

### Step 5 — Pico
- Create `pd_pico/pico_queue.h` + `pd_pico/pico_queue.cpp`
- Migrate from `StepperISR_rp_pico.cpp`

### Step 6 — ESP32
- Most complex: multiple sub-drivers (MCPWM/PCNT, RMT V1, RMT V2)
- Create `pd_esp32/esp32_queue.h` + per-variant `.cpp` files
- Migrate from `StepperISR_esp32.cpp`, `StepperISR_idf4_*.cpp`,
  `StepperISR_idf5_*.cpp`, `StepperISR_esp32xx_rmt.cpp`

### Step 7 — Clean up StepperISR.h and StepperISR.cpp
- `StepperISR.h` becomes the thin dispatcher (see above)
- `StepperISR.cpp` has zero `#ifdef` guards for architectures
- Remove `pd_avr/avr_driver.h`, `pd_esp32/esp32_driver.h` etc. if superseded

---

## Non-Goals

- No runtime polymorphism / virtual functions (embedded, overhead unacceptable)
- No change to the external `FastAccelStepper` API
- No change to `queue_entry` layout or timing semantics
