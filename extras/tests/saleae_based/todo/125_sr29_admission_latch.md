# SR_29 — admission latch after forceStop

**Status:** not implemented (skipped in all platform runs; not in white paper)

**Context:** When a force stop is issued, the queue sets an **admission latch** that refuses subsequent `addQueueEntry()` calls until `QCLR` re-arms it via `resumeCommands()`.

From AGENTS.md:
> That latch is what XSTOP sets too, so `QCLR` re-arms it (`resumeCommands()`):
> without that, a `XSTOP` -> `QCLR` -> `QSEG` -> `QRUN` on one connection
> queues nothing while reporting success. `CONFIG` also re-arms, via the
> `_initVars()` memset.

**What implementing it means:**
- Define a scenario that exercises the admission latch behavior:
  1. Fill the queue with a program.
  2. Issue a force stop (`XSTOP` or `STOP` with `forceStop()` semantics).
  3. Attempt to queue a new segment *before* `QCLR`.
  4. Verify the new segment is **refused** (the admission latch is active).
  5. Send `QCLR` to re-arm.
  6. Verify the next segment is **accepted**.
- This tests a subtle but critical invariant: the queue must reject commands after a force stop, and only re-arm after explicit clearance.

**Dependencies:**
- Driver: any (RMT, MCPWM/PCNT, I2S).
- Channel config: `1ch`.
- Needs a board where the admission latch behavior can be measured at the pin level.

**References:**
- AGENTS.md: discussion of the admission latch and `resumeCommands()` (§5.3).
- White paper: no definition; SR_29 is not in `white_paper_saleae_test_harness.md`.
- **Action item:** Define the scenario program and expected serial replies before implementation.

**Relation to other tests:**
- SR_30 tests the same XSTOP path but measures the *capture outcome* (no steps after stop).
- SR_29 tests the *queue-level behavior* (refusal/re-acceptance) that makes SR_30's outcome possible.