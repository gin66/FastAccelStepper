# 015 RMT panics the firmware on ESP-IDF 5.5 — the harness overflows the main task's stack

## Priority

**CRITICAL** — a driver the library lists as available crashes the MCU at
`stepperConnectToPin()`. Not a wrong step count, not a slow one: the firmware
never returns from the call, so the application cannot use RMT on this SDK at
all. Found by the platform-release matrix (`scripts/run_matrix.py`), which is
the first thing that has ever compared the *same* firmware against three
different ESP-IDF majors on real silicon.

**Root cause found: the harness's own stack budget, not the library and not the
RMT driver.** `CONFIG_ESP_MAIN_TASK_STACK_SIZE` is 3584 B and this app runs its
whole protocol parser *and* the driver-construction call chain on that task.
Measured on the failing build, the main task has **216 bytes left** when the
panic hits; `stepperConnectToPin()` never returns on IDF 5.5.3 because the
stack runs out. The overflow writes *upward*, past the stack top, into the DRAM
tlsf pool, which is what corrupts the free list and produces the
`LoadProhibited`. See "Root cause" below. The same defect is
[016](016_i2s_direct_stack_overflow_idf55.md).

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

So the RMT setup is where it surfaces. Nothing in the library, in the RMT driver
or in IDF 5.5.3's RMT driver is at fault; see "Root cause".

## Root cause: the main task's stack, not the RMT driver

### The measurement

Instrumented on the connected ESP32-DevKitC, IDF 5.5.3, the build the matrix
blamed. `uxTaskGetStackHighWaterMark()` (bytes) at the deepest point reached:

| task | stack | free | used |
|---|---|---|---|
| `main` (`app_main` → `saleae_app_loop`) | 3584 B | **216 B** | 3368 B (94 %) |
| `StepperTask` (`fas_init_engine`) | 6000 B | 5492 B | 508 B |

`StepperTask` is not the problem — it uses 8 % of its stack. The **main task is
the problem**: 216 bytes of margin. On IDF 6.1 (the working row) the same
command leaves 312 bytes, i.e. **IDF 6.1 is equally over budget and merely does
not tip over.** That is why the fault is driver-shaped and SDK-shaped rather
than random: RMT is one of the deeper driver-construction chains, and IDF 5.5.3
happens to need ~100 bytes more than 6.1 on the way to `tlsf_malloc`.

`engine.init()` — which is what creates `StepperTask` — is called *lazily from
inside* `handle_config` (`saleae_app.cpp:1272`), immediately before the steppers
are connected. So every `CONFIG` first builds a 6 KB task and then, on the very
same task, descends into the driver.

### Why it presents as a corrupted heap

An overflow runs off the *top* of the stack, i.e. toward higher addresses, and on
ESP32 that is where the DRAM tlsf pool begins. Boot log of the failing build:

```
heap_init: At 3FFB4858 len 0002B7A8 (173 KiB): DRAM     <- pool base
```

and the pool's own control block sits at `0x3FFB486C` (visible as `A2`/`A8` in
the register dumps), while the main task's frames measure `0x3FFB5310`–
`0x3FFB55E0` with panic-time `A1` down to `0x3FFB4F80`. The stack is inside the
pool's address range, so the overflow lands in live pool memory and overwrites a
free-list pointer. Everything else follows from that:

- The garbage pointer **changes between runs** — `0x800dff10`, then
  `0x800e0058`, then `0x8000bd86`. It is stack residue, not a constant.
- **Every** garbage value starts with `0x800` and none of them is inside any
  region in `memory.ld`, so each is a load from the stack, not a wild pointer
  into mapped memory.
- `A15 : 0x0000cdcd` appears in *every* register dump: `0xcd` is FreeRTOS's
  stack fill pattern (`configSTACK_FILL_PATTERN`), so a callee-saved register
  was loaded from stack memory.
- The heap is **intact at idle** — `heap_caps_check_integrity_all()` returns OK
  before `CONFIG` — because `StepperTask` only starts hammering
  `manageSteppers()` once `engine.init()` has run, i.e. inside the first
  `CONFIG`.
- The crash site **moves** when stack consumption changes. Adding one `printf`
  to `init_rmt` moved it from `rmt_new_tx_channel`'s `heap_caps_calloc` to
  `esp_intr_alloc_intrstatus` to inside `tlsf_check` itself.

### The proof

`CONFIG_ESP_MAIN_TASK_STACK_SIZE` 3584 → 8192, nothing else changed:

```
CONFIG 1 rmt dir  ->  OK CONFIG n=1 mode=dir stride=2 drivers=rmt maxspeed0=80
MAP               ->  MAP count=1 mode=dir stride=2 ch=2,0 bus=- slots=- marker=255
DIAG2 (integrity) ->  integ=OK
```

The panic is gone and RMT connects. Nothing in `src/` was touched to get there.

### Why the harness is the wrong place to be spending 4 KB

The stack is not consumed by driver construction, which is unavoidable and
small (`StepperQueue` is `new`'d, so the queue array never touches the stack).
It is consumed by two avoidable things, and **the second turned out to be twice
the size of the first**, which is why the obvious fix was not enough:

**(a) libc.** On ESP32 `sal_sscanf`/`sal_snprintf` were plain libc
`sscanf`/`snprintf` (`saleae_str.h`), so every command line dragged in newlib's
whole `__svfscanf`/`__svfprintf` machinery — while the protocol only ever
formats `%s`, `%u`, `%d`, `%lu`, `%ld`:

| operation | stack |
|---|---|
| `handle_line` frame + one libc `sscanf` | 1496 B |
| one libc `snprintf` | 384 B |

**(b) Reply buffers on the stack.** A local array reserves its stack slot for
the **whole function**, so these were held while `handle_config` descended into
`rmt_new_tx_channel`:

| buffer | ESP32 (32-stepper build) |
|---|---|
| `handle_config` `buf[SALEAE_CFG_REPLY_MAX]` | **1152 B** |
| `handle_line` `cmd`+`arg1..arg4` (live across `handle_config`) | **497 B** |
| `handle_map` `buf[CHANNELS*4 + MAX_STEPPERS*4 + 96]` | 256 B |

The command loop is a single task, so all of these are now `static`.
`SALEAE_CFG_REPLY_MAX` alone was 1152 B — a third of the entire task.

## The fix

1. **A formatter for the five conversions the protocol uses**
   (`saleae_str.h`), plus `sal_tokenize()` for the one line-split the harness
   did. No libc `sscanf`/`snprintf` is linked on a non-AVR build any more;
   `avr` keeps `snprintf_P`, whose avr-libc vfprintf is already small.
   `sal_tokenize` replaces the old `sscanf` on *every* platform including AVR,
   because the field widths are runtime values (`SALEAE_ARG2_MAX` is 24…384).
2. **The reply and argument buffers moved to `static`** — the larger half of
   the win. Neutral on AVR, where the stack lives in `.data` too.
3. **`driver_supported()` is now guarded by `SUPPORT_SELECT_DRIVER_TYPE`**, its
   only call site's condition. It was a `-Wunused-function` warning on AVR
   before this work; warnings are not allowed.

Measured peak stack, `CONFIG 1 rmt dir`, IDF 5.5.3, `CONFIG_ESP_MAIN_TASK_STACK_SIZE`
left at its **3584 B default** — no Kconfig change:

| | peak used | free | result |
|---|---|---|---|
| before | 4272 B | −688 B | **panic** |
| after | **2336 B** | **1248 B** | `OK CONFIG n=1 mode=dir stride=2 drivers=rmt` |

The `handle_config` parse phase went from 1136 B to **0 B**. So the cliff is
gone rather than moved: 015 and 016 both fit in the platform default again, on
the SDK that could not fit them before.

Verified on the connected board with the default stack, IDF 5.5.3:

```
CONFIG 1 rmt dir                    -> OK ... drivers=rmt          (was: panic)
CONFIG 2 mcpwm_pcnt,rmt dir         -> OK ... maxspeed1=80         (was: panic)
CONFIG 2 i2s_direct,i2s_direct nodir -> OK ... maxspeed1=80         (016's n=2)
MAP                                 -> MAP count=1 mode=dir stride=2 ch=2,0 ...
```

`mcpwm_pcnt` and `i2s_direct` reach a slightly higher peak than `rmt`
(`after_connect` 1296 B free vs 1248 B), so 1248 B is not the tightest case —
it is the RMT case, which is the one that used to crash.

### Guards added, so this cannot come back silently

- `TestStackBudget` in `scripts/tests/test_saleae.py` fails the build if any
  `char buf[...]` of 64 bytes or more is a stack local again, and records the
  2336 B figure so the budget claim stays checkable. This is the guard that was
  missing: the whole matrix passed 43/43 on five rows while the sixth was
  unrecoverable, because nothing measured the stack.
- `TestSaleaeFmt` fails the build if a format string uses a conversion
  `sal_snprintf` does not implement (it returns −1 and prints nothing, which is
  safe but silent), or reintroduces a width/flag/pad.
- `test_config_line_fits_the_firmware_line_buffer` now checks that each
  `sal_field` width is `sizeof(its own buffer) - 1`, which cannot drift — a
  stronger property than matching literals in an sscanf format string.

### Still open

- **MAP is short on builds without configurable driver type.** `handle_map`
  emits only `count`/`mode`/`stride` and `ch=-` when
  `SUPPORT_SELECT_DRIVER_TYPE` is undefined, because the full form's buffer is
  136 B on a 328P for a reply of at most 66. The pin list is what it gives up;
  the host already parsed `ch=-` into an empty pin list, so `run_tests.py` is
  unchanged. A build with driver selection keeps the full form.
- **Blocking the idle loop does not fix the IDF 4.4.3 idle watchdog, and was
  measured and rejected.** `saleae_hal_idle()` was made `vTaskDelay(1)` --
  StepperDemo's own idiom, which is why StepperDemo never trips it -- and the
  watchdog still fired, with no command outstanding: `QINFO` alone reproduces
  it, 43 WDT lines, and it fires identically after a `CONFIG`. `main` now blocks
  for a whole tick, so IDLE gets the CPU, and the subscription that trips is
  IDLE's own. 5.5.3 and 6.1 never show it with identical firmware. So this is
  IDF 4.4's idle-task watchdog, not the harness's loop. It was not free
  either: the feeder is the same loop, and `sync` mcpwm_pcnt+i2s_direct grew
  trailing pulses (67 where 64 were commanded, period still exactly 10.0 us) --
  though that is intermittent under the spin too (1 in 3, against 2 in 2 with
  the block), so the block is not the cause. The spin is kept and the reason is
  written down at the call site. The mcpwm_pcnt trailing-pulse flake itself is
  **not** explained and is worth its own item.
- **The IDF 4.4.3 idle task watchdog still fires ~5 s into an idle board**, and
  it is *not* the application's spin: the idle path is now `vTaskDelay(1)`
  (`saleae_hal_idle()`, matching `DELAY_MS(10)` in
  `examples/StepperDemo/StepperDemo.ino`), verified as `vTaskDelay(1)` in the
  disassembly, and the WDT appears with **no command outstanding** (`QINFO`
  alone reproduces it, 43 WDT lines, same as after a `CONFIG`). Since the
  subscription that trips is IDLE0's and `main` now blocks for a whole tick,
  this looks like IDF 4.4's idle-task WDT handling rather than anything the
  harness does — 5.5.3 and 6.1 do not show it with identical firmware. Left
  open deliberately; the spin it replaced was still wrong to keep.
- **The `i2s_mux` path mangles any command from n ≥ 16** (`CONFIG 16 i2s_mux,…`
  answers `ERR unknown '2s_mux,i2s_mux'`; n=32 gives `'2s_mdir'`). Verified
  pre-existing: a pristine checkout of this branch fails identically, so it is
  neither caused nor fixed by this work. It is *not* the `uint8_t linelen` wrap
  documented above — `linelen` is already `uint16_t` and an n=16 line is only
  144 characters — so the mechanism is still open. It belongs to the mux
  backlog, not here.
- **Whether 016's n = 2 failure is fully explained.** The stack overflow is
  this defect, and `CONFIG 2 i2s_direct,i2s_direct nodir` now connects on 5.5.3
  as it does on 6.1. Re-run the `scale:i2s_direct` sweep three times to confirm
  the intermittency is gone before closing 016.

### Answers to the open questions this replaces

- *"Whether the corruption is in `StepperISR_rmt_v2.cpp` or in shared ESP32
  code"*: neither. The RMT connect path is byte-identical to 1.3.4 apart from
  adding `_rmt_fill_state.remaining_low_ticks = 0`, verified by diffing
  `StepperISR_idf5_esp32_rmt.cpp` at v1.3.4 against today's
  `StepperISR_rmt_v2.cpp`.
- *"Whether 5.4 is affected"*: it is a **harness stack budget**, not an SDK
  defect, so "affected" meant only "the chain exceeded 3584 B on that SDK". It
  is fixed for every SDK at once, and the budget is now guarded by a test rather
  than by an SDK allowlist.
- *"A debug build with a heap poisoning/guard allocator localises it in one
  run"*: poisoning was tried and is **not** the right tool. With
  `CONFIG_HEAP_POISONING_COMPREHENSIVE=y` the crash merely moved to a later
  allocation inside the same call (`esp_intr_alloc_intrstatus_bind`), still with
  a corrupt free list (`A2 = 1`) — poisoning detects overflow of an *allocated*
  block, and the block being overwritten here is a *free* block plus the stack.
  `uxTaskGetStackHighWaterMark()` localised it immediately, once the libc
  `sscanf` in the measurement path was replaced so the numbers meant something.

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
a warm-boot artifact. The 6 stack overflows in that row are **not a second
defect** — see [016](016_i2s_direct_stack_overflow_idf55.md), which has the same
root cause as this one.

## Regression test once fixed

`python3 scripts/run_matrix.py --targets idf-6.13.0 --force` — the RMT column
of the matrix is the whole test. A fix that only stops the panic is not
enough: the 34 panicking measurements must come back `pass`, because a board
that no longer crashes but emits the wrong period is not a fix. Add
`idf-6.13.0` to the Arduino/ESP-IDF CI matrix so a future RMT change is
measured on three IDF majors rather than one.

Add a **stack-budget check** alongside it, because the bug was invisible to
every existing test: the whole matrix passed 43/43 on five rows while the
harness was ~100 bytes from an unrecoverable corruption on the sixth. A build
that asserts the measured peak stack usage of the protocol loop against a
stated ceiling catches this class of defect on every platform, in CI, with no
hardware.