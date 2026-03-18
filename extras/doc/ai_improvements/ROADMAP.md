# FastAccelStepper Driver Refactoring Roadmap

## Problem Statement

`StepperQueue` previously defined all fields and methods for all supported architectures behind a wall of `#ifdef` guards in a single header:

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

**Drawbacks:**
- Hard to read: understanding one architecture requires mentally filtering out all others
- Hard to extend: adding a new architecture means touching the shared header
- Coupling problem: arch-specific methods need both hardware state and queue state

---

## Design Decision: Per-Architecture StepperQueue

### Core Idea

Define `StepperQueue` **per architecture** in the respective `pd_*/` directory. A common base struct holds all shared fields. Each architecture extends it with its own hardware-specific fields and method implementations.

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
  pd_test/
    test_queue.h    ← class StepperQueue : public StepperQueueBase (PC tests)
  fas_queue/
    stepper_queue.h ← thin header: includes base + correct pd_*/ header
    stepper_queue.cpp ← pure common code, zero #ifdefs
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
  void init(uint8_t queue_num, uint8_t step_pin);  // hardware setup only, called by tryAllocateQueue()
  void _initVars();     // zero fields + set defaults, no hardware access
  void connect();
  void disconnect();
  static bool isValidStepPin(uint8_t step_pin);
};
```

### Why This Solves the Coupling Problem

Previously `startQueue()` needed both queue fields (`read_idx`, `entry[]`) and hardware fields (`channel`, `pio`, `_isRunning`). As a method of a mixed class this was messy but worked. Any attempt to move logic into a separate `DriverData` struct hit the problem that the struct didn't own the queue fields.

With the per-arch `StepperQueue`, **all fields are in scope naturally** — there is no coupling problem. `startQueue()` is just a regular method of the class that happens to own both the hardware and queue state.

### stepper_queue.h Becomes a Thin Dispatcher

```cpp
// fas_queue/stepper_queue.h
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

---

## Implementation Status

### Step 1 — Define StepperQueueBase ✅ COMPLETE
- `fas_queue/base.h` defines `StepperQueueBase` with all common fields
- Common method declarations present
- No `#ifdef` inside the base

### Step 2 — AVR ✅ COMPLETE
- `pd_avr/avr_queue.h` with `class StepperQueue : public StepperQueueBase`
- `pd_avr/avr_queue.cpp` contains AVR-specific implementations
- AVR builds and tests pass

### Step 3 — TEST stub ✅ COMPLETE
- `pd_test/test_queue.h` for PC-based tests
- PC tests validate the base + derived structure

### Step 4 — SAM ✅ COMPLETE
- `pd_sam/sam_queue.h` + `pd_sam/sam_queue.cpp`
- Migrated from legacy `StepperISR_due.cpp`

### Step 5 — Pico ✅ COMPLETE
- `pd_pico/pico_queue.h` + `pd_pico/pico_queue.cpp`
- Migrated from legacy `StepperISR_rp_pico.cpp`

### Step 6 — ESP32 ⚠️ PARTIAL
Queue structure migrated:
- `pd_esp32/esp32_queue.h` + `pd_esp32/esp32_queue.cpp`

Driver files NOT unified (7 files remain in `pd_esp32/`):
- `StepperISR_esp32xx_rmt.cpp` — Common RMT buffer fill
- `StepperISR_idf4_esp32_mcpwm_pcnt.cpp` — IDF4 MCPWM/PCNT
- `StepperISR_idf4_esp32_rmt.cpp` — IDF4 ESP32 RMT
- `StepperISR_idf4_esp32c3_rmt.cpp` — IDF4 ESP32-C3 RMT
- `StepperISR_idf4_esp32s3_rmt.cpp` — IDF4 ESP32-S3 RMT
- `StepperISR_idf5_esp32_rmt.cpp` — IDF5 RMT

**Rationale for not unifying:** Each driver file handles specific IDF version and chip variant combinations. Template-based unification was deemed too complex for the marginal benefit. The current structure works and is tested.

### Step 7 — Clean up ✅ MOSTLY COMPLETE
- `fas_queue/stepper_queue.h` is the thin dispatcher
- Common queue code in `fas_queue/stepper_queue.cpp`
- Legacy `StepperISR.h` removed
- Platform-specific conditionals removed from common code

---

## Non-Goals

- No runtime polymorphism / virtual functions (embedded, overhead unacceptable)
- No change to the external `FastAccelStepper` API
- No change to `queue_entry` layout or timing semantics
- No dynamic memory allocation in critical paths
