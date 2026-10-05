# SR_28 — forceStop() (middle stop variant)

**Status:** not implemented (skipped in all platform runs; not in white paper)

**Context:** The library has three stop methods, differing in *how* they stop and *what happens to the position*:

| Stop method | Saleae command | Queue after stop | Position |
|---|---|---|---|
| `stopMove()` | `STOP` (SR_25) | Queue runs out normally | Kept |
| **`forceStop()`** | **?** (SR_28?) | Queue runs out (abrupt) | **Kept** |
| `forceStopAndNewPosition()` | `XSTOP` (SR_30) | Queue emptied | **Lost** |

**SR_25** covers `stopMove()`. **SR_30** covers `forceStopAndNewPosition()`.
**`forceStop()` is the missing middle ground** — abrupt stop but position is kept, and the queue runs out (unlike SR_30 where it is emptied).

**What implementing it means:**
- Define a scenario that sends `STOP` (the library's `forceStop()` command).
- Verify the capture shows an abrupt stop (no deceleration ramp), position is preserved, and the queue runs out (no partial pulse, but unlike SR_30 the queue is not explicitly emptied by the stop).
- The program and stop logic would be similar to SR_25/SR_30 but with different expected outcomes.

**Dependencies:**
- Driver: any (RMT, MCPWM/PCNT, I2S).
- Channel config: `1ch`.
- Needs a board that supports `forceStop()` semantics (all ESP32 drivers do).

**References:**
- AGENTS.md: "The library has three stops... forceStop() stops abruptly but lets the queue run out, so the position is kept" (§5.3).
- AGENTS.md: "Neither of the other two stops has a scenario" — referring to `stopMove()` (now SR_25) and `forceStop()` (this one).
- White paper: no definition; SR_28 is not in `white_paper_saleae_test_harness.md`.
- **Action item:** Define the scenario program and expected capture characteristics before implementation.