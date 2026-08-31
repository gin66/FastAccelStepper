# FastAccelStepper SAMD51 Port

Fork-local context for the SAMD51 pulse driver (`src/pd_samd/`). Upstream FastAccelStepper
vocabulary (command queue, ramp generator, ticks) applies unchanged; this glossary covers only
the terms this port adds.

## Language

**Pulse driver**:
The platform-specific layer that executes command-queue entries in hardware (upstream term,
directory prefix `pd_`). This port's pulse driver is one TCC instance per stepper.
_Avoid_: backend, HAL

**Queue TCC**:
The TCC instance claimed by one stepper queue. It generates every step pulse in hardware
(normal PWM: `PER` = step interval, `CC` = pulse width) and its overflow interrupt advances
the queue. Claimed dynamically from the step pin's `g_APinDescription` PWM mux; one stepper
per TCC because `PER` is shared across all channels of an instance.
_Avoid_: step timer (ambiguous with the ramp tick), PWM channel (a channel is not the unit
of allocation — the instance is)

**Ramp tick**:
The periodic ~4 ms interrupt that runs the upstream ramp generator (`manageSteppers()`) to
refill the command queues. Driven by a plain TC instance (default TC3, override
`FAS_SAMD_RAMP_TC`), never by a TCC.
_Avoid_: engine task, background loop

**Command boundary**:
The overflow interrupt at which the current queue entry's steps are exhausted and the next
entry's interval is staged into `PERBUF` (and DIR toggled if requested). All queue-advance
logic runs here; mid-entry overflows only decrement the step counter.

**Pause entry**:
A queue entry with `steps == 0`, executed as one TCC period with `CCBUF = 0` so the step
output stays low. Pauses chain to represent intervals longer than the 16-bit tick ceiling
(4.096 ms at 16 MHz).
_Avoid_: delay command (upstream docs say "pause")
