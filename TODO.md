# Refactoring: Move PIO resources to StepperQueue, Consolidate init into tryAllocateQueue()

## Motivation

The Pico version crashes after commit 4f2e1e4 because `tryAllocateQueue()` passes
`nullptr` to `init()`, which then dereferences it in `claim_pio_sm()`:

```cpp
bool StepperQueue::claim_pio_sm(FastAccelStepperEngine* engine) {
  for (uint8_t i = 0; i < engine->claimed_pios; i++) {  // CRASH: engine is nullptr
```

Root causes:
1. PIO state (`claimed_pios`, `pio[]`) lives on `FastAccelStepperEngine`, forcing
   `claim_pio_sm()` to receive an engine pointer — but `tryAllocateQueue()` doesn't
   have one.
2. `StepperQueue::init()` is called from two places: from `tryAllocateQueue()` (Pico)
   and from `FastAccelStepper::init()` (all platforms). This double-init pattern is
   fragile and inconsistent.

## Design

### Move PIO resources to StepperQueue static members

`claimed_pios` and `pio[]` are shared PIO resources — they belong on `StepperQueue`
as static members, not on the engine. This:
- Removes `#ifdef SUPPORT_RP_PICO` from `FastAccelStepperEngine.h`
- Makes `claim_pio_resources()` self-contained (no engine pointer needed)
- Keeps platform-specific state in the platform-specific code

### Separate resource acquisition from initialization

For Pico, PIO/SM claiming can fail. By making it a static function called *before*
touching any queue, we get "fail before you mutate":

1. `tryAllocateQueue(engine, step_pin)` — static, entry point
2. → `claim_pio_resources(step_pin)` — static, returns `PioResources` struct or fails
3. → If failed: return `nullptr` (no queue touched, nothing to clean up)
4. → Find free queue slot
5. → Set `pio`/`sm` from claimed resources
6. → `queue->init(queue_num, step_pin)` — member variable initialization
7. → `setupSM()`, `connect()`

### Remove engine from init()

No driver actually uses the `engine` parameter in `init()`. Pico was the only one
that passed it through to `claim_pio_sm()`, which is being replaced by the static
`claim_pio_resources()`. The signature becomes:

```cpp
bool init(uint8_t queue_num, uint8_t step_pin);
```

`_initVars()` / `_pd_initVars()` remain as internal helpers called by `init()`.

### Consolidate init call site

`StepperQueue::init()` is called from `tryAllocateQueue()` for all platforms.
`FastAccelStepper::init()` still calls `_queue()->init()` for compatibility with
PC-based tests that bypass `stepperConnectToPin()` and call `s.init(NULL, 0, 0)`
directly. This is idempotent — in production the queue is already initialized by
`tryAllocateQueue()`.

---

## Files to Change

### 1. src/FastAccelStepperEngine.h

**Status: DONE**

- [x] Remove the `#ifdef SUPPORT_RP_PICO` block (`claimed_pios`, `pio[]`)
- [x] Remove `friend class StepperQueue` (no longer accesses engine internals)

### 2. src/fas_queue/protocol.h

**Status: DONE**

- [x] Add `FastAccelStepperEngine* engine` parameter to `tryAllocateQueue()` signatures
- [x] Change `bool init(FastAccelStepperEngine* engine, uint8_t queue_num, uint8_t step_pin)` to `bool init(uint8_t queue_num, uint8_t step_pin)`

### 3. src/FastAccelStepperEngine.cpp

**Status: DONE**

- [x] Pass `this` to all `tryAllocateQueue()` calls (4 locations):
  - Line 107: dynamic allocation with driver type
  - Line 111: dynamic allocation without driver type
  - Line 134: static allocation with driver type
  - Line 138: static allocation without driver type

### 4. src/FastAccelStepper.cpp

**Status: DONE**

- [x] Change `_queue()->init(engine, _queue_num, step_pin)` to `_queue()->init(_queue_num, step_pin)`
  (kept for test compatibility, idempotent in production)
- [x] Return `true` unconditionally (init can no longer fail)

### 5. src/pd_pico/pico_queue.h

**Status: DONE**

- [x] Add static members: `static uint8_t s_claimed_pios;` and `static PIO s_pio[NUM_PIOS];`
- [x] Add `PioResources` struct and static `claim_pio_resources()` declaration
- [x] Remove `claim_pio_sm(FastAccelStepperEngine* engine)` declaration

### 6. src/pd_pico/pico_queue.cpp

**Status: DONE**

- [x] Define static members: `uint8_t StepperQueue::s_claimed_pios = 0;` etc.
- [x] Implement static `claim_pio_resources(engine, step_pin, &out)` using static members
- [x] Remove old `claim_pio_sm(FastAccelStepperEngine* engine)` method
- [x] Update `tryAllocateQueue(engine, step_pin)`:
  1. Validate step_pin
  2. Call `claim_pio_resources(engine, step_pin, &res)` — fails fast, no queue touched
  3. Find free queue slot
  4. Set `queue->pio` and `queue->sm` from `res`
  5. Call `queue->init(queue_num, step_pin)`
  6. Call `queue->setupSM()`, `queue->connect()`
- [x] Update `fas_init_engine()`: initialize `StepperQueue::s_claimed_pios = 0`

### 7. src/pd_avr/avr_queue.cpp

**Status: DONE**

- [x] Update `tryAllocateQueue()` to accept `FastAccelStepperEngine* engine` parameter
- [x] Remove `engine` from `init()` signature

### 8. src/pd_sam/sam_queue.cpp

**Status: DONE**

- [x] Update `tryAllocateQueue()` to accept `FastAccelStepperEngine* engine` parameter
- [x] Remove `engine` from `init()` signature

### 9. src/pd_esp32/esp32_queue.cpp

**Status: DONE**

- [x] Remove `engine` from `init()` signature
- [x] Update all `tryAllocateQueue()` variants to accept `FastAccelStepperEngine* engine`
  parameter (including recursive calls)

Variants updated:
- [x] `tryAllocateQueue(engine, FasDriver driver, uint8_t step_pin)` — dynamic + select_driver
- [x] `tryAllocateQueue(engine, uint8_t step_pin)` — dynamic only
- [x] `tryAllocateQueue(engine, uint8_t step_pin)` — static only
- [x] `tryAllocateQueue(engine, FasDriver driver, uint8_t step_pin)` — static + select_driver

### 10. extras/tests/pc_based/StepperISR_test.cpp

**Status: DONE**

- [x] Update `tryAllocateQueue()` signature to include engine parameter
- [x] Update `init()` signature (remove engine parameter)
- [x] All PC-based tests pass

---

## New Protocol Summary

### tryAllocateQueue()

```cpp
#if defined(SUPPORT_SELECT_DRIVER_TYPE)
static StepperQueue* tryAllocateQueue(FastAccelStepperEngine* engine,
                                      FasDriver driver, uint8_t step_pin);
#else
static StepperQueue* tryAllocateQueue(FastAccelStepperEngine* engine,
                                      uint8_t step_pin);
#endif
```

Returns a fully initialized StepperQueue, or nullptr on failure.

### init()

```cpp
bool init(uint8_t queue_num, uint8_t step_pin);
```

Called internally by `tryAllocateQueue()`. Initializes member variables and
performs driver-specific hardware setup. No engine pointer needed — no driver
uses it. Also called from `FastAccelStepper::init()` for test compatibility
(idempotent in production).

---

## Testing

After all changes:

1. [x] Run PC-based tests: `make -C extras/tests/pc_based` — All tests passed
2. [ ] Build Pico target and verify no crash => human
3. [ ] Build other platform targets (AVR, ESP32, SAM) => human
4. [ ] Run SimAVR tests if available => human

---

## Documentation

After all code changes are complete and tested:

1. [ ] Update any API documentation that references the old init flow
2. [ ] Update inline documentation in header files if needed
