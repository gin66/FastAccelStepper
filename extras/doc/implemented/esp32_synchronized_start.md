# ESP32 synchronized start (per-driver release)

Priority: **050** — platform-specific part of the engine synchronized start
(one item per open platform).

Status: **implemented** (EXPERIMENTAL).

The generic fallback remains active on all ESP32 builds.
A native RMT synchronized release is implemented for IDF5/6 targets with
`SOC_RMT_SUPPORT_TX_SYNCHRO` (`rmt_new_sync_manager()`; ESP32 classic has the
RMT V2 driver API but no sync manager and keeps the fallback;
see [engine_synchronized_start.md](engine_synchronized_start.md),
`src/pd_esp32/esp32_queue.cpp:593-683`).
Drivers I2S direct, MCPWM/PCNT, and RMT (IDF4) still use the generic fallback.

## Background

`addQueueEntry(NULL, true)` does **not** only queue an entry: when the queue is
not running it immediately calls `startQueue()`, which triggers the driver
(`fas_queue/queue_add_entry.cpp:111-116`). The generic fallback therefore works
only because the whole loop holds interrupts off:

- Drivers whose first step is emitted later from an ISR (I2S mux DMA `on_sent`)
  are already effectively synchronized by the fallback — as long as the callback
  interrupt really is masked by `portDISABLE_INTERRUPTS()` (needs verification,
  see Risks).
- Drivers that start synchronously inside the loop (RMT `rmt_transmit()`, MCPWM
  `timer_start = 2`) are started one after another, with a gap equal to the loop
  body between them. This is the actual sync gap to close.

Because injection is also the start, a native release **cannot** be bolted on
after an inject loop. The design must be split into "arm" (prepare, do not
trigger) and "trigger" (one hardware action per group). This is the central
constraint of this document.

## Driver inventory

| Group | Driver | Chip support | Resource sharing | Native sync |
|-------|--------|--------------|------------------|-------------|
| A1 — I2S mux | `FasDriver::I2S_MUX` | all targets with `SOC_I2S_NUM >= 1` (incl. ESP32-C6, ESP32-H2) | all mux steppers share the single global `StepperQueue::_i2s_mux_manager` (`esp32_queue.cpp:15-17`) | fallback already sufficient (deferred DMA callback); no separate trigger needed |
| A2 — I2S direct | `FasDriver::I2S_DIRECT` | same as above | one `I2sManager*` per stepper, own DMA channel | none (no cross-channel trigger) |
| C — RMT | `FasDriver::RMT` | ESP32, S2, S3, C3, C6, H2, P4 (`pd_config_idf*.h`) | all channels share one RMT peripheral/clock; each stepper has its own channel (numeric in IDF4, handle in IDF5/6) | IDF5/6 with `SOC_RMT_SUPPORT_TX_SYNCHRO`: `rmt_new_sync_manager()` over the armed channels; IDF4 and ESP32 classic: no public API |
| D — MCPWM/PCNT | `FasDriver::MCPWM_PCNT` | ESP32, S3, C6, H2 | one MCPWM timer + one PCNT unit per stepper | none implemented; MCPWM global SYNC groups are an open investigation |

Correction of an earlier assumption: **ESP32-C6 and ESP32-H2 support both RMT
and I2S** (`pd_config_idf5.h:81-110` sets `SUPPORT_ESP32_RMT` /
`SUPPORT_ESP32_RMT_V2` and `QUEUES_RMT 2`; `SOC_I2S_NUM == 1` enables
`SUPPORT_ESP32_I2S` at `pd_config_idf5.h:177`). They are not MCPWM-only.

## Hardware resource map

| Stepper type | I2S mux | I2S direct | RMT (IDF4) | RMT (IDF5/6) | MCPWM/PCNT |
|-------------|---------|------------|------------|--------------|------------|
| Channel resource | shared (`_i2s_mux_manager`) | one `I2sManager*` per stepper | one channel number per stepper | one channel handle per stepper | one timer/PCNT per stepper |
| Group key | all mux steppers | none | all RMT steppers | all RMT steppers | none |
| Max steppers | `QUEUES_I2S_MUX` (32 dynamic) | `QUEUES_I2S_DIRECT` (= `SOC_I2S_NUM`, 2 on the ESP32) | 8/4/2 (ESP32/S3/C3) | 8/4/2/2/2/`CONFIG_SOC_RMT_TX_CANDIDATES_PER_GROUP` | 6/4/2/2 (ESP32/S3/C6/H2) |
| Shared clock? | yes (one I2S stream) | no | yes (RMT peripheral) | yes (RMT peripheral) | no |
| Deferred trigger? | yes (DMA `on_sent` ISR) | yes (DMA `on_sent` ISR, per stepper) | no (register write in `startQueue_rmt()`) | no (`rmt_transmit()` in `startQueue_rmt()`) | no (`timer_start = 2` in `startQueue_mcpwm_pcnt()`) |

Note: RMT is grouped **by driver**, not per channel. All RMT channels live on one
peripheral, and the sync manager synchronizes *several* channels; treating each
channel as its own group would defeat the purpose.

## Design: arm then trigger

### Two-phase queue API

Add a per-driver split of the existing start path. `startQueue()` keeps its
current behaviour (arm + trigger) for the normal single-stepper case; the
engine uses the two phases.

| Driver | Arm | Trigger |
|--------|-----|---------|
| I2S mux | set `_isRunning = true` (sets `_fill_state` ready; first steps appear in the next DMA callback) | no-op |
| I2S direct | set `_isRunning = true` | no-op (per-stepper DMA; fallback) |
| RMT IDF5/6 | do the dir-pin toggle + `apply_command`/encoder reset, enable the channel, collect the channel for the group trigger | create one sync manager over the armed channels, call `rmt_transmit()` on all of them (the last one starts the group), then delete the manager |
| RMT IDF4 | run the existing `startQueue_rmt()` | no-op (fallback) |
| MCPWM/PCNT | run the existing `startQueue_mcpwm_pcnt()` | no-op (fallback) |

The arm step for RMT must still perform everything `startQueue_rmt()` does
before the transmit, so `startQueue_rmt()` is refactored into
`rmt_arm(...)` + `rmt_trigger(...)`.

### Group classification (inside the critical section)

Classification uses `StepperQueue::_driver_type`, `_i2s_mux_manager` and, for
RMT, simply "is RMT". It is recomputed on every call because the set of
connected steppers can change between calls.

- Group A1: all steppers with `_driver_type == I2S_MUX` (one group).
- Group C: all steppers with `_driver_type == RMT` (one group under IDF5/6;
  no-op under IDF4).
- Everything else: arm only, no group trigger.

### Engine coordination

`synchronizedStart()` receives the stepper array, so it can reach each queue via
`FastAccelStepper::_queue()` (`FastAccelStepper.h:842`). The platform function
must therefore take the array (the earlier sketch of a parameterless
`esp32SynchronizedRelease()` cannot see it).

```cpp
AqeResultCode FastAccelStepperEngine::synchronizedStart(
    FastAccelStepper** const steppers, uint8_t cnt) {
  AqeResultCode rc = AqeResultCode::OK;
  fasDisableInterrupts();

  // Phase 1: arm every not-yet-running stepper (no hardware trigger).
  // Skip running queues; an empty queue must not abort the others.
  for (uint8_t i = 0; i < cnt; i++) {
    StepperQueue* q = steppers[i]->_queue();
    if (q == nullptr || q->isRunning()) {
      continue;
    }
    if (q->isQueueEmpty()) {
      continue;  // matches generic contract: empty does not stop the others
    }
    q->syncStart_arm();  // per-driver dispatch
  }

  // Phase 2: one native trigger per group; groups without a native
  // trigger (I2S direct, MCPWM, RMT IDF4) did their trigger in arm().
  esp32_sync_trigger_i2s_mux();  // no-op today (documented)
  esp32_sync_trigger_rmt();      // create/start/delete the group sync manager

  fasEnableInterrupts();
  return rc;
}
```

`syncStart_arm()` / `esp32_sync_trigger_*()` are thin dispatchers next to the
existing `startQueue()` dispatch in `esp32_queue.cpp` / `esp32_queue.h`.

## Implementation plan

1. **RMT IDF5/6 (the real win).**
   - Split `startQueue_rmt()` (`StepperISR_rmt_v2.cpp:128-203`) into an
     arm part (dir toggle, encoder reset, channel enable; collect the channel)
     and a trigger part (create a sync manager over the armed channels, call
     `rmt_transmit()` on each, delete the manager).
   - The IDF sync manager is created with the complete set of managed channels
     and puts each of them in a waiting state until `rmt_transmit()` has been
     called on all of them. There is no add/remove-channel API, and all channels
     must be enabled before creation, so the manager cannot be kept alive
     across `connect`/`disconnect`. Instead it is created on demand for exactly
     the channels armed in one `synchronizedStart()` call and deleted right
     after the group has started. Channels must use the same clock
     source/resolution (they do: `RMT_CLK_SRC_DEFAULT`,
     `resolution_hz = TICKS_PER_S`, `StepperISR_rmt_v2.cpp:102-103`).
   - `SOC_RMT_SUPPORT_TX_SYNCHRO` gates the native path; targets without it
     (ESP32 classic) keep the arm==trigger fallback via `syncStart_arm_rmt()`.
2. **Dispatch layer.** Add `syncStart_arm()` and (for RMT) the group trigger in
   `esp32_queue.h` / `esp32_queue.cpp`, mirroring `startQueue()`.
3. **Engine.** Rewrite the ESP32 `synchronizedStart()` (`esp32_queue.cpp:541-553`)
   to the two-phase form above. This changes the platform implementation only;
   the generic contract is unchanged.
4. **I2S mux.** Keep the fallback; document that the deferred DMA `on_sent`
   callback already groups mux steppers. Only add a native hook if the IRQ
   masking check fails.
5. **I2S direct / MCPWM / RMT IDF4.** Keep the fallback. For MCPWM, open a
   separate investigation: MCPWM has global SYNC0/1/2 groups that can reset
   several timers from one trigger, but the current code only uses the
   per-timer `timer_sync` form (`StepperISR_idf4_esp32_mcpwm_pcnt.cpp:439-457`)
   and starts each timer with `timer_start = 2`, which is not part of the sync
   group action.

## Risks and open questions

- **IRQ masking.** `portDISABLE_INTERRUPTS()` masks down to
  `XCHAL_EXCM_LEVEL`; ESP32 high-level interrupts can still fire. If the I2S
  `on_sent` callback runs at a high level, even the I2S mux fallback is not
  guaranteed and a native hook is needed after all. Confirm the callback IRQ
  priority.
- **Arm/trigger vs. running state.** Arm sets `_isRunning = true` before the
  group trigger. With interrupts off no other code observes this half-state, but
  it must be documented and kept consistent with `isReadyForCommands()`.
- **Sync manager lifetime.** The manager is created on demand for the armed
  channels and deleted once the group has started; it is not kept alive across
  `connect_rmt()`/`disconnect_rmt()`, because IDF offers no add/remove-channel
  API and requires all managed channels to be enabled at creation time.
- **IDF4 RMT.** No public sync-manager API in IDF4; register-level sync is not
  supported by this plan.
- **MCPWM.** Global sync feasibility is unproven; keep the fallback until
  investigated.

## References

- `src/pd_esp32/esp32_queue.cpp` — fallback `synchronizedStart()`, module
  dispatch (`startQueue()`, `connect()`, `disconnect()`).
- `src/pd_esp32/esp32_queue.h` — `_driver_type`, driver predicates, module
  function declarations.
- `src/pd_esp32/i2s_manager.h` / `.cpp` — shared mux manager and the
  `on_sent` → `handleTxDone()` per-stepper fill loop (`i2s_manager.cpp:127-148`).
- `src/pd_esp32/StepperISR_esp32_i2s.cpp` — `startQueue_i2s()` (`_isRunning`
  gate; `fill_i2s_buffer()` returns early when not running).
- `src/pd_esp32/StepperISR_rmt_v2.cpp` — IDF5/6 RMT arm/trigger split
  point (`startQueue_rmt()`).
- `src/pd_esp32/StepperISR_rmt_v1_esp32.cpp`,
  `StepperISR_rmt_v1_esp32c3.cpp`,
  `StepperISR_rmt_v1_esp32s3.cpp` — IDF4 per-chip RMT `startQueue`.
- `src/pd_esp32/StepperISR_rmt_v1.cpp` — shared IDF4 `rmt_fill_buffer()` /
  `rmt_apply_command()` helper (not a startQueue).
- `src/pd_esp32/StepperISR_idf4_esp32_mcpwm_pcnt.cpp`,
  `StepperISR_idf5_esp32_mcpwm_pcnt.cpp`,
  `StepperISR_idf6_esp32_mcpwm_pcnt.cpp` — MCPWM/PCNT `startQueue`.
- `src/fas_queue/queue_add_entry.cpp` — `addQueueEntry(NULL, true)` starts the
  queue immediately.
- `src/FastAccelStepper.h` — `_queue()` accessor.
- `src/FasNAxis.h`, `src/FasTimed.h` — prefill + `synchronizedStart()` callers.
- [engine_synchronized_start.md](../doc/implemented/engine_synchronized_start.md) — generic
  layer and the fallback contract (running steppers skipped, empty queue does
  not stop the others).
