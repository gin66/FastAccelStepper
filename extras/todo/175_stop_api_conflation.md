# 172 stopMove() / forceStop() / forceStopAndNewPosition() — three APIs, one harness conflated two

## Priority

**LOW** — harness bug that was found, fixed, and documented.  Worth recording
for future reference.

## Finding

The harness's `STOP` was `stopMove()` **plus zeroing the feeder cursor** — a
hybrid matching neither documented behaviour.  The library has three:

| API | contract |
|-----|----------|
| `stopMove()` | a flag for the ramp's **next** command. Must **not** truncate queued motion. |
| `forceStop()` | `ignore_commands = true`; nothing further *added*, queue drains (~20 ms). |
| `forceStopAndNewPosition()` | aborts everything queued — no further step issued. |

So the number reported earlier — 7655 steps left on `i2s_direct`, 7608 on
`rmt`, both just under the 8160 a 32-deep queue of 255-step commands
holds — was **the harness's own arithmetic, not a library guarantee.**

## Fix

`STOP` is now `stopMove()` alone, and `ESTOP` is `forceStop()`.  Because the
expected outcomes are *opposite*, they are two scenarios rather than one with
a flag; `CONTRASTING_PAIRS` declares them so the anti-duplication test
accepts the shared waveform.  SR_25 asserts `stopMove()` did **not** truncate;
SR_29 asserts `forceStop()` drained within the queue bound.

## Marker channel

A new `MARK <ch>` command designates an analyzer channel **no stepper owns**,
and the firmware flips its level when it processes a stop, so the instant is
on the waveform.  Placement is load-bearing: sent after `QRUN` it costs two
serial round-trips (~0.25 s each) before the stop, so on any driver whose move
was shorter than that the stop arrived after the move had finished.

Verified on hardware, rmt, same program, opposite assertions:

| | steps emitted | after the stop marker | verdict |
|---|---|---|---|
| SR_25 `stopMove()` | **20000 / 20000** | 8099 | **passed** — did not truncate, as required |
| SR_29 `forceStop()` | 18615 / 20000 | 7464 (bound 8160) | **passed** — stopped adding |
