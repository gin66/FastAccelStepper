# 016 i2s_direct is unstable on ESP-IDF 5.5 — nondeterministic refusals and a stack overflow

## Priority

**HIGH** — a driver that connects is not reliably usable, and one of the ways
it fails is memory corruption. Same SDK and **the same root cause** as
[015](015_rmt_panics_on_esp_idf_5_5.md) — this is now settled, see "Same root
cause as 015" below. Kept as a separate item only because the observable
symptom and the debugging path differ.

## Finding

`--mode scale --driver i2s_direct --pin-mode nodir` on
**ESP-IDF 5.5.3** (`esp32_idf_V6_13_0`) does not produce a stable answer. The
real bound is 2 — `SOC_I2S_NUM` is 2 and one TX channel goes to each controller,
which is already [070](070_i2s_direct_channels.md) — and on IDF 6.1 the sweep
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
   [015](015_rmt_panics_on_esp_idf_5_5.md) (`block_locate_free`, reached from
   `rmt_new_tx_channel` on the very same SDK).

## Same root cause as 015 — settled

Both items are now explained by one defect: **the harness overflows the FreeRTOS
`main` task's stack, and `main` is where every `CONFIG` constructs its drivers.**
The full analysis is in
[015](015_rmt_panics_on_esp_idf_5_5.md#root-cause-the-main-tasks-stack-not-the-rmt-driver).
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
  [015](015_rmt_panics_on_esp_idf_5_5.md), which is why the two rows disagree in
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

## Why the two are tracked separately anyway

015 crashes in the RMT path with a decoded backtrace pointing into
`StepperISR_rmt_v2.cpp`; 016 has no backtrace and points at
`I2sManager::create()`. That difference in observable symptom is all that still
distinguishes them — the cause is now known to be one, and it is neither of
those two functions.

## Note on what this is not

`QUEUE_I2S_DIRECT` is 3 and the hardware allows 2. The constant is the subject
of [070](070_i2s_direct_channels.md) and is unchanged by this item: n = 3
failing on IDF 6.1 is 070, not this. What is new here is that on IDF 5.5.3 the
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
- Whether 5.4 is affected. As in [015](015_rmt_panics_on_esp_idf_5_5.md) the
  trigger was a stack budget, not an SDK defect. The fix is SDK-independent, so
  this is very likely moot; the matrix run answers it either way.

## Regression test once fixed

```bash
python3 scripts/run_matrix.py --targets idf-6.13.0 --force
```

`idf-6.13.0 / scale:i2s_direct` must be `pass` at n = 1…2 and a clean `refused`
with the IDF's own error for every n above, on every run. Run it three times:
the defect is intermittent, so a single clean pass is not evidence. That
intermittency is itself the tell — it should disappear now the budget is real,
and if it does not, the budget is not the whole story and the n = 2 failure
above is a separate item.