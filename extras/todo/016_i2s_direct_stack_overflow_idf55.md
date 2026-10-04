# 016 i2s_direct is unstable on ESP-IDF 5.5 — nondeterministic refusals and a stack overflow

## Priority

**HIGH** — a driver that connects is not reliably usable, and one of the ways
it fails is memory corruption. Same SDK and probably the same root cause as
[015](015_rmt_panics_on_esp_idf_5_5.md); tracked separately because the
debugging path is different and neither was found before the release matrix.

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

## Why the two are tracked separately anyway

015 crashes in the RMT path with a decoded backtrace pointing into
`StepperISR_rmt_v2.cpp`; 016 has no backtrace yet and points at
`I2sManager::create()`. If they share a root cause, one of the two is the
mistaken one. Closing 016 on its own evidence — a debug build, an
`i2s_new_channel()` trace, a stack watermark check — is what settles it.

## Note on what this is not

`QUEUE_I2S_DIRECT` is 3 and the hardware allows 2. The constant is the subject
of [070](070_i2s_direct_channels.md) and is unchanged by this item: n = 3
failing on IDF 6.1 is 070, not this. What is new here is that on IDF 5.5.3 the
**refusal path itself is not reliable** — the sweep is supposed to answer "where
is the limit" and on this SDK it answers a different thing each time.

## What is left to find

- Why n = 2 fails on 5.5.3 and passes on 6.1. Diff `I2sManager::create()` and
  the channel-config it passes between the two SDKs; the classic cause is a
  struct that grew a field in the I2S driver and is initialised
  positionally.
- Whether the stack overflow is a real stack overrun (check
  `uxTaskGetStackHighWaterMark()` on the main task at the point of the sweep)
  or corruption left by an earlier allocation — the IDF 5.5 RMT defect above is
  the obvious first suspect and the cheapest thing to rule out.
- Whether 5.4 is affected, as in 015.

## Regression test once fixed

```bash
python3 scripts/run_matrix.py --targets idf-6.13.0 --force
```

`idf-6.13.0 / scale:i2s_direct` must be `pass` at n = 1…2 and a clean `refused`
with the IDF's own error for every n above, on every run. Run it three times:
the defect is intermittent, so a single clean pass is not evidence.