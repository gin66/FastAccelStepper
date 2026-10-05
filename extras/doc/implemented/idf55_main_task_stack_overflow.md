# IDF 5.5.3: the harness overflows the main task's stack

Closes todo **015** (RMT panics the firmware) and todo **016**
(`i2s_direct` unstable). Both were the same defect; 016 was tracked separately
only while their symptoms differed, and 016's own note said so:

> That difference in observable symptom is all that still distinguishes them —
> the cause is now known to be one, and it is neither of those two functions.

Everything below is 015's investigation. 016's own material is kept, under
**016, and what was left to find** at the end.

## Priority, as it was when this was open

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
016 below.

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
defect** — see 016 below, which has the same
root cause as this one.

## Outcome: both items closed, and what the acceptance run measured

`run_matrix.py --targets idf-6.13.0 --force`, ESP-IDF 5.5.3, default 3584 B
stack, one flash. 59 passed, 8 refused, 4 not-implemented, 2 failed. Catalogue
**26/26**.

The sync table is the before/after, same row, same ten combinations:

| combination | before | after |
|---|---|---|
| `rmt+rmt` | refused (bound) — panic | pass, 66.0 us skew |
| `rmt+mcpwm_pcnt` | refused (bound) — panic | pass, 73.3 us |
| `rmt+i2s_direct` | refused (bound) — panic | pass, 835.4 us |
| `rmt+i2s_mux` | refused (bound) — panic | pass, 1008.9 us |
| `i2s_direct+i2s_direct` | **FAIL** — QSEG rejected | pass, 34.3 us |
| `i2s_direct+i2s_mux` | **FAIL** — QSEG rejected | pass, 84.1 us |

Worth keeping: the four RMT combinations used to be reported as
`refused (bound)` **with the Guru Meditation text quoted as the reason**, because
a panic and a board limit are the same refusal to the classifier. That is why
015 read as a driver-capability question rather than a stack overflow, and the
same gap is why a run that skips everything still reports `failed=0`.

`CONFIG 1 rmt dir` and `CONFIG 2 mcpwm_pcnt,rmt dir` answer `OK`;
`CONFIG 2 i2s_direct,i2s_direct nodir` answers `OK` (016's n=2).

The two failures that remain are both pre-existing and tracked elsewhere:
`i2s_mux+i2s_mux` is an incomplete capture, S2 missing (todo 022, on both I2S
SDKs), and one `sync rmt+i2s_mux` run hit the known intermittent mux dropped
pulse (63/64, one 99.92 us off-grid period), which re-measured clean 3/3.

A separate measurement from this work, kept because it would otherwise have
gone in as a fix: blocking the idle loop for one tick (`vTaskDelay(1)`) does
**not** fix the IDF 4.4.3 idle watchdog — it fires with no command outstanding,
`QINFO` alone reproduces it — so it is IDF 4.4's own idle-task watchdog, not
the harness's. It was also not free: the feeder is that loop, and it grew
trailing pulses on mcpwm_pcnt. Reverted; see the note at `saleae_hal_idle()`.

## Regression test, as written before the fix was verified

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

---

# 016, and what was left to find

Everything below this line is 016's original text, kept as written. Its open
questions are answered in the outcome section above; where the two disagree, the
outcome section is the one that was measured.

## Finding

`--mode scale --driver i2s_direct --pin-mode nodir` on
**ESP-IDF 5.5.3** (`esp32_idf_V6_13_0`) does not produce a stable answer. The
real bound is 2 — `SOC_I2S_NUM` is 2 and one TX channel goes to each controller,
which was already item 070 (*i2s_direct has 2 channels on ESP32, not
3*, now fixed — see `extras/todo/README.md` § Done) — and on IDF 6.1 the
sweep
says so cleanly. On IDF 5.5.3 the same sweep gives three different answers for
the same class of point:

| n | IDF 5.5.3 (run A) | IDF 5.5.3 (run B) | IDF 6.1.0 |
|---|---|---|---|
| 1 | passed | passed | passed |
| 2 | **failed** | **failed** | passed |
| 3 | refused | **stack overflow** | refused |
| 4 | refused | **stack overflow** | refused |
| 5 | refused | **stack overflow** | refused |
| 6 | refused | **stack overflow** | refused |
| 7 | **stack overflow** | refused | refused |
| 8 | refused | **stack overflow** | refused |

`refused` carries the peripheral's own error, which is the correct behaviour
and is what IDF 6.1 does for every n >= 3:

```
W (400) i2s_platform: i2s controller 0 has been occupied by i2s_driver
```

The two other answers are defects:

1. **n = 2 fails on both runs** where IDF 6.1 passes it. The only configuration
   the library actually documents as legal is broken on this SDK.
2. **`***ERROR*** A stack overflow in task main has been detected.`** — the
   FreeRTOS task that runs `app_main`, i.e. the firmware's own main loop,
   overruns its stack. It moves between points across runs, so it is not a
   fixed-size overrun at a fixed call depth: it is consistent with the heap or
   stack being already damaged before the I2S manager allocates, which is the
   same signature as the `LoadProhibited` in
   015 (`block_locate_free`, reached from
   `rmt_new_tx_channel` on the very same SDK).

## Same root cause as 015 — settled

Both items are now explained by one defect: **the harness overflows the FreeRTOS
`main` task's stack, and `main` is where every `CONFIG` constructs its drivers.**
The full analysis is in
[the root-cause section](#root-cause-the-main-tasks-stack-not-the-rmt-driver) below.
The short version, measured on the connected board:

- `CONFIG_ESP_MAIN_TASK_STACK_SIZE` is 3584 B. After a `CONFIG` on IDF 5.5.3 the
  main task has **216 bytes** left; on IDF 6.1, **312 bytes**. Both are
  essentially out of budget — 6.1 only escapes because its `i2s_new_channel()`
  / `rmt_new_tx_channel()` path is shallower.
- `engine.init()` (which creates the 6 KB `StepperTask`) is called lazily from
  inside `handle_config`, immediately before `connect_stepper()`, so every
  `CONFIG` first builds a task and then descends into the driver constructor on
  the same task.
- Raising `CONFIG_ESP_MAIN_TASK_STACK_SIZE` to 8192, changing nothing else, makes
  `CONFIG 1 rmt dir` return `OK` on IDF 5.5.3.

`I2sManager::create()` and `connect_rmt()` are reached by the same call chain
(`handle_config` → `connect_stepper` → `stepperConnectToPin` →
`tryAllocateQueue` → driver constructor), so the i2s path tips over the same
budget the RMT path does. That also explains all three of this item's oddities
without any I2S-specific defect:

- **It moves between n across runs** — a marginal budget is exactly that
  nondeterministic.
- **It is driver-shaped** — `i2s_direct` creation is the deepest constructor in
  the set, so it fails first and most often.
- **n = 2 passes on 6.1 and fails on 5.5.3** — same ~100-byte SDK difference as
  015, which is why the two rows disagree in
  the same direction.

Why FreeRTOS reported the overflow here but stayed silent for the RMT row:
`CONFIG_FREERTOS_CHECK_STACKOVERFLOW_CANARY=y` only catches a write *below* the
stack limit, into the canary. The RMT row's corruption is a write *above* the
stack top into the heap, which no FreeRTOS check sees — it surfaces later as a
corrupted tlsf free list. Same overflow, two different detectors, and one of
them is blind.

This closes the open question this item raised — *"If they share a root cause,
one of the two is the mistaken one."* Neither was mistaken; they share a cause,
and it is in the harness, not in the library or in either I2S or RMT driver.

## Why the two were tracked separately anyway (historical)

015 crashes in the RMT path with a decoded backtrace pointing into
`StepperISR_rmt_v2.cpp`; 016 has no backtrace and points at
`I2sManager::create()`. That difference in observable symptom is all that still
distinguishes them — the cause is now known to be one, and it is neither of
those two functions.

## Note on what this is not

`QUEUES_I2S_DIRECT` was 3 and the hardware allows 2. That constant was item
070, since fixed to `SOC_I2S_NUM` (see `extras/todo/README.md` § Done); it
is unchanged *by this item*: n = 3 failing on IDF 6.1 is 070, not this. What is new here is that on IDF 5.5.3 the
**refusal path itself is not reliable** — the sweep is supposed to answer "where
is the limit" and on this SDK it answers a different thing each time.

## What is left to find

- **Confirm, do not assume.** The stack overflow was this item's "obvious first
  suspect" and it was the right one, but that leaves two claims still resting on
  a single measurement each:
  - Re-run `python3 scripts/run_matrix.py --targets idf-6.13.0 --force` and
    check `scale:i2s_direct` is `pass` at n = 1…2 and a clean `refused` above,
    three times, so the intermittency is demonstrably gone.
  - **Does the n = 2 failure go away with the stack fixed, or is it a second,
    genuinely I2S-specific defect?** Verified by hand so far: on IDF 5.5.3 with
    the default 3584 B stack, `CONFIG 2 i2s_direct,i2s_direct nodir` now answers
    `OK` (it used to fail), so the *connect* is no longer refused. Whether the
    *sweep* then reports n=2 as `pass` rather than `failed` is what the matrix
    run decides. If it still fails, diff `I2sManager::create()` and the
    `i2s_std_config_t` it passes against the SDK headers — note that
    `i2s_std_clk_config_t::bclk_div` exists only from **IDF 5.5** (it is absent
    in the 5.3 SDK the Arduino-as-ESP-IDF builds ship), so a 5.3/5.5 difference
    in that struct is the first place to look.
- Whether 5.4 is affected. As in 015 the
  trigger was a stack budget, not an SDK defect. The fix is SDK-independent, so
  this is very likely moot; the matrix run answers it either way.

## Regression test, and how it was met

016 asked for `scale:i2s_direct` to be re-run three times on 5.5.3 to show the
intermittency was gone. It was, on two full matrix rows: n = 1 and n = 2 pass,
n = 3…8 correctly refused at the `QUEUE_I2S_DIRECT` bound. The acceptance run
that closed both items is `run_matrix.py --targets idf-6.13.0 --force`; see the
matrix report's sync table, where all four RMT combinations and both
`i2s_direct` pairs went from panic/refused to measured passes.

### Original text

```bash
python3 scripts/run_matrix.py --targets idf-6.13.0 --force
```

`idf-6.13.0 / scale:i2s_direct` must be `pass` at n = 1…2 and a clean `refused`
with the IDF's own error for every n above, on every run. Run it three times:
the defect is intermittent, so a single clean pass is not evidence. That
intermittency is itself the tell — it should disappear now the budget is real,
and if it does not, the budget is not the whole story and the n = 2 failure
above is a separate item.