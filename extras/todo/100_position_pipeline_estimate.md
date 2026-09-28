# Estimate getCurrentPosition() for a pipelined driver

Priority: **100** — low. The documented remedy is a pulse counter.
This is only for callers who cannot attach one.

Status: not started.

## State

`getCurrentPosition()` counts steps the driver has already accepted.
On ESP32 RMT those steps can still be sitting in the symbol block, so
at high speed the returned position leads the step pin by the pipeline
depth. The method does not guess how far that block has played out.

A later estimate can subtract the steps whose tick duration has not
yet elapsed, using the command durations already handed to the driver
and a time base. No driver feedback. `src/` stays free of 64-bit
counters. Until that exists, `attachToPulseCounter()` /
`readPulseCounter()` is the position of the pulses already sent.
