# 015 RMT panics the firmware on ESP-IDF 5.5 — every RMT CONFIG kills the board

## Priority

**CRITICAL** — a driver the library lists as available crashes the MCU at
`stepperConnectToPin()`. Not a wrong step count, not a slow one: the firmware
never returns from the call, so the application cannot use RMT on this SDK at
all. Found by the platform-release matrix (`scripts/run_matrix.py`), which is
the first thing that has ever compared the *same* firmware against three
different ESP-IDF majors on real silicon.

## Finding

`StepperQueue::connect_rmt()` aborts the firmware on **ESP-IDF 5.5.3**
(PlatformIO `espressif32 @ 6.13.0`, env `esp32_idf_V6_13_0`):

```
D (420) rmt: new simple encoder @0x3ffb8a7c
Guru Meditation Error: Core  0 panic'ed (LoadProhibited). Exception was unhandled.
PC      : 0x4008ce5f  PS      : 0x00060833  A0      : 0x8008c85c  A1      : 0x3ffb4f80
A2      : 0x800dff10  A3      : 0x000000a0  EXCCAUSE: 0x0000001c  EXCVADDR: 0x800dff14
```

`EXCVADDR` is not a heap address, and the instruction that faults is not in RMT
code — it is in the allocator, walking a pool header that is not a pool header.
Decoded against that build's ELF:

```
block_locate_free            framework-espidf@3.50503.0/components/heap/tlsf/tlsf_control_functions.h:616
  (inlined by) tlsf_malloc   .../tlsf/tlsf.c:444
multi_heap_malloc_impl       .../heap/multi_heap.c:216
heap_caps_calloc             .../heap/heap_caps.c:255
rmt_new_tx_channel           .../components/esp_driver_rmt/src/rmt_tx.c:262
StepperQueue::connect_rmt()  src/pd_esp32/StepperISR_rmt_v2.cpp:115
StepperQueue::init_rmt()     src/pd_esp32/StepperISR_rmt_v2.cpp:98
StepperQueue::tryAllocateQueue(...)   src/pd_esp32/esp32_queue.cpp:255
FastAccelStepperEngine::stepperConnectToPin(...)  src/FastAccelStepperEngine.cpp:113
connect_stepper              extras/tests/saleae_based/common/saleae_app.cpp:806
handle_config                extras/tests/saleae_based/common/saleae_app.cpp:1334
```

So the RMT setup is where it surfaces, and **heap metadata is already corrupt
by then**. The encoder allocation on the line above succeeds; the channel
allocation on line 115 is the one that trips. What corrupted it is not in this
backtrace — the deepest frames are the allocator, by definition — so the
defect is upstream of RMT or is an out-of-bounds write in the RMT path itself.

## Scope: exactly RMT, on exactly one SDK

Every measurement of the release matrix, 350 runs, `results/` and
`reports/esp32_platform_matrix.md`:

| row | ESP-IDF | passed | RMT panic | stack overflow |
|---|---|---|---|---|
| `arduino-4.4.0` | 4.4.7 | 43 | 0 | 0 |
| `arduino-5.3.0` | 4.4.7 | 43 | 0 | 0 |
| `arduino-6.13.0` | 4.4.7 | 43 | 0 | 0 |
| `idf-5.3.0` | 4.4.3 | 43 | 0 | 0 |
| **`idf-6.13.0`** | **5.5.3** | **23** | **34** | 6 |
| `idf-7.1.2` | 6.1.0 | 60 | 0 | 0 |

Within the broken row the split is clean and driver-shaped, not random:

| contains an `rmt` stepper | result |
|---|---|
| SR_01…SR_17, SR_21, SR_25, SR_26, SR_27, SR_30 (22 scenarios, configs `1ch`/`2ch`/`mixed_rmt_mcpwm`) | **panic** |
| `scale --driver rmt`, n = 1…8 (8 points) | **panic** |
| `sync`, all 4 combinations containing rmt | **panic** |
| SR_18/19/20 (`mcpwm_pcnt`), SR_23 (`i2s_direct`), `scale:mcpwm_pcnt` 1…6, `scale:i2s_direct`, `scale:i2s_mux` 1…8 | **pass** |

Reproduced on two separate flashes of the same env, hours apart, so it is not
a warm-boot artifact. The 6 stack overflows in that row are a second defect:
[016](016_i2s_direct_stack_overflow_idf55.md).

## What is left to find

- Whether the corruption is in `StepperISR_rmt_v2.cpp` itself (the F2 encoder
  translator is the part that is new on IDF5) or in shared ESP32 code that RMT
  happens to reach first. `init_rmt()` writes `_step_pin`, calls
  `rmt_new_simple_encoder()` and then `connect_rmt()`; a debug build with a
  heap poisoning/guard allocator between those two lines localises it in one
  run.
- IDF 5.5 changed the RMT driver's channel allocation and its `heap_caps`
  capability requests. Compare `rmt_new_tx_channel()`'s allocation flags
  between `framework-espidf@3.50503.0` and the 6.1 tree the same source works
  against.
- **Whether 5.4 is affected.** The matrix measures one version per major, so
  5.4 and 5.3.1 are unknown. This matters for the fix's scope: if it is a
  5.5-only regression, the answer is "IDF >= 5.4.4"; if it is "any 5.x with a
  new-enough RMT driver", it is a library bug.

## Regression test once fixed

`python3 scripts/run_matrix.py --targets idf-6.13.0 --force` — the RMT column
of the matrix is the whole test. A fix that only stops the panic is not
enough: the 34 panicking measurements must come back `pass`, because a board
that no longer crashes but emits the wrong period is not a fix. Add
`idf-6.13.0` to the Arduino/ESP-IDF CI matrix so a future RMT change is
measured on three IDF majors rather than one.