# SAMD51 pulse driver: TCC normal PWM with one overflow interrupt per step

The SAMD51 port generates step pulses with the step pin's TCC in normal PWM (buffered
`PERBUF` = next step interval, `CCBUF` = pulse width), taking one overflow interrupt per
step that decrements the current queue entry and, at command boundaries, stages the next
entry's interval and direction. Pulse-edge timing is therefore hardware-exact regardless of
interrupt latency; the ISR only has to finish within one step interval, giving ~50 µs of
slack at typical speeds and a ~200 ksteps/s ceiling — far above stepper physics.

## Considered options

- **TCC free-running + Event System into a TC event counter (one interrupt per command,
  ESP32 MCPWM/PCNT-style).** Zero per-step CPU load, but the command-boundary overrun race
  (counter reaches N while the TCC is already emitting the next pulse at the old period)
  is the hardest part of the ESP32 port and would be re-invented on new silicon. Costs a
  TC + EVSYS channel per stepper. Rejected for v1; the peripheral setup is a superset of
  the chosen design, so it remains an upgrade path if multi-stepper high-rate support is
  ever needed.
- **DMA-fed per-step period updates (RMT-style).** Requires expanding queue commands into
  per-step period buffers — an extra layer no other port needs — to eliminate ISR load
  that is already negligible (~0.2% CPU for one motor). Rejected.
- **The SAM Due port's design (pin-change interrupt per step + PWM peripheral).** Known
  problematic upstream (audit lists 4 critical issues; needs sprinkled busy-waits).
  Explicitly not used as the template; the AVR port is.
