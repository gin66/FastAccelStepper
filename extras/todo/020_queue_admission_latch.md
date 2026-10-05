# 020 The queue admission latch (`ignore_commands`) has no lifecycle

Priority: **020**. Effort: ~80 k tokens, 1–2 d.

Status: not started. Found while re-evaluating the `stopMove()` finding,
which is by design and now recorded in
`extras/doc/implemented/saleae_based_test_harness.md`; that item's own
conclusion was withdrawn and this is what replaced it.

## Verdict

A real defect, in the documented **low-level** API. `ignore_commands` is
a queue-admission latch with four writers in two layers, no reader, no
public way to clear it, and a refusal that reports success.

## The latch

`src/fas_queue/base.h:43`:

```cpp
// Commands are suspended during forceStopAndNewPosition()
volatile bool ignore_commands;
```

Read at exactly one place, `queue_add_entry.cpp:100`. Written at four:

| site | layer |
|---|---|
| `FastAccelStepper.cpp:437` `forceStop()` | high-level stop |
| `:448` `forceStopAndNewPosition()` | high-level stop |
| `:101` `fill_queue()` | ramp → queue bridge |
| `:647` `performOneStep()` | high-level single step |

So: **four writers, zero readers, no accessor, no public clear.** The
queue's own admission state is mutated from outside by both of its
neighbours, and the definition's comment names one of its two setters.

## Two consequences

**Inert on the path that sets it.** `fill_queue()` clears
`ignore_commands` at `:101` — *before* using it — on every pass where the
ramp is active. The ramp is stopped by its own state instead
(`force_immediate_stop` consumed once at `RampGenerator.cpp:255+`, and
`ramp_state != RAMP_STATE_IDLE`). So for a ramp user the gate is cleared
before it can refuse anything.

**Permanent on the path it must not affect.** `fill_queue()` returns at
`:87` — *before* the clear at `:101` — whenever the ramp is inactive,
which is permanently true for the documented `addQueueEntry()` user
(`FastAccelStepper.h:634-671`). `FastAccelStepper::addQueueEntry()`
(`fas_member/add_queue_entry.h`) never clears it. The only remaining
clear is `performOneStep()` at `:647`, reached through
`forwardStep()`/`backwardStep()` — high-level API.

So after `forceStop()` or `forceStopAndNewPosition()`, a low-level caller
can never queue again, and `queue_add_entry.cpp:100-108` writes the entry
fields, abandons them, does not advance `next_write_idx`, does not update
`queue_end`, and falls through to `return AQE_OK` at `:144`. The caller is
told the command was queued. Its position model then diverges silently.

## It is load-bearing for the planners — do not delete it

`FasNAxis::emergencyStop()` (`FasNAxis.h:404-413`) sets `_fault`, sets
`_feeding = false`, **and** calls `forceStop()` on every member, which
sets the queue latch. `FasNAxis::pump()` `:308` and `FasTimed::pump()`
`:218` poll `takeStopCause()` for the same purpose. That is three
independent latches, and the queue-level one is a deliberate backstop
against runaway pulses if a planner's own flag is wrong.

Which is why the consequence is not "the latch is broken" but **"the
latch has no key."** A planner that stops can never rearm it through any
public call. `FasNAxis::pump()` has exactly such a resume path at
`:319-322`:

```cpp
if (!_feeding && _head < _n_blk) { feeder_start(); feed_loop(); }
```

Its state machine would resume, and every `addQueueEntry()` would return
`AQE_OK` while queuing nothing. Currently latent, because `_fault` also
latches and blocks it — but latent only by coincidence, and the backstop
the planner installed is the one thing it can never disarm.

## Why not move it into the ramp generator

Considered and rejected. The flag gates **queue admission**, so the queue
is the correct owner of that state. What is missing is not the location
but the **interface**: a set/clear/query pair that the queue owns, so
that the four writers stop reaching into the field. Moving it into
`ramp_ro_s` would leave the planners — which never touch the ramp
generator — with no way to set it at all.

## The five changes

1. **Give it an interface.** In `protocol.h` beside `void forceStop();`
   (:48): `void setCommandAdmissionSuspended(bool)` and
   `bool isCommandAdmissionSuspended() const`. All four writers call it.
   It is a plain field on `StepperQueueBase`, so this needs no
   per-platform work.
2. **Add the public clear.** One `FastAccelStepper` method, so a caller
   can disarm what it armed. Without this the backstop stays one-way.
3. **Stop lying on refusal.** `queue_add_entry.cpp:100-108` returns
   `AQE_OK` for a dropped command. Add a non-retryable
   `AQE_COMMANDS_SUSPENDED` (classify it at `result_codes.h:39-42`
   alongside `aqeRetry()` / `aqeRetryImmediately()`) so a planner can
   tell "queued" from "refused" instead of silently desynchronising.
4. **Make enforcement total, or drop it.** Three holes:
   `addQueueEntry(NULL, start)` returns at `:14-22` *before* the gate, so
   the queue can still be started after an abort; `SET_DIRECTION_PIN_STATE`
   and `queue_end.dir = dir` run at `:63-79`, before the gate, so DIR is
   driven for a discarded command; and the pause accounting at `:130-143`
   advances `_last_pause_ticks` / `_nr_of_pauses` unconditionally, so
   statistics count commands that were never queued.
5. **Fix the contract at the definition.** `base.h:43` says "during
   forceStopAndNewPosition()": it omits `forceStop()` (`:437`), and
   "during" is false for every non-ramp user, for whom it is permanent.
   `extras/doc/driver_architecture.md:97` repeats the same wording.

## The test

**Abort, then rearm on the same connection.** No `CONFIG` in between.

The Saleae harness cannot currently express this, and the reason is worth
recording: `handle_config` (`common/saleae_app.cpp:826`) calls
`stepperConnectToPin()`, and `StepperQueue::_initVars()` (`queue_init.cpp`)
`memset`s the queue — so a per-scenario `CONFIG` silently re-arms the
latch. Every scenario reconnects, so the one case that is broken is the
one never run. `XSTOP` followed by `QCLR` → `QSEG` → `QRUN` on the same
connection should queue steps and does not.

`QCLR` does not help either: `stop_all()` (`saleae_app.cpp:739-749`)
calls `stopMove()`, which sets no latch — so `QCLR` neither sets nor
clears `ignore_commands`.

## Platform note: Pico destroys a count it could keep exactly

Not this item's subject, but found alongside it and relevant to any stop
API.

`StepperQueue::forceStop()` (`pd_pico/pico_queue.cpp:154-169`) does
`pio_sm_clear_fifos(pio, sm)` — which discards RX as well as TX — and
then `pos_offset = 0`. RX is where the performed-step count lives, so the
stop throws away the exact figure that makes Pico the most exact platform
in the library, and `getCurrentPosition()` afterwards reads an empty FIFO
(the `pio_sm_get()` at `:199` runs *before* the `pio_sm_is_rx_fifo_empty()`
test at `:200`).

Discarding TX is the desirable half — Pico is the only platform that
drops in-flight hardware work — but the count should be read inside the
halt, before the clear:

```cpp
pio_sm_set_enabled(pio, sm, false);
int32_t performed = <drain RX>;            // exact: SM halted, count final
pio_sm_clear_fifos(pio, sm);
...
pos_offset = <true position> - performed;  // not 0
```

The convention already exists — `queue_add_entry.cpp:60` does
`pos_offset = queue_end.pos - getCurrentStepCount()`.

And this is the only platform where it can be made exact: `pio_sm_set_enabled(false)`
halts the state machine synchronously and RX already holds the count of
what was emitted, so the read/stop window is genuinely closable. RMT keeps
playing already-transmitted symbols; I2S keeps playing DMA blocks. On
Pico, "abort now, know exactly where you stopped, lose nothing" is
*provable* rather than approximate — which makes the caller-supplied
`new_pos` redundant here and necessary everywhere else. That asymmetry is
itself part of why the stop API needs documenting per platform.

## References

- `extras/doc/engine_sources.md:70-71` — describes the transient latch
- `extras/doc/driver_architecture.md:97`, `:292-307`
- `extras/tests/saleae_based/AGENTS.md:210-220`
- `common/saleae_app.cpp:677-703` — 27 lines explaining which of three
  stops reaches the queue, which is this defect's symptom
- [130](130_position_pipeline_estimate.md) — the separate question of
  `getCurrentPosition()` *leading* the pin on a pipelined driver
