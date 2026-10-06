# Saleae harness — RP2350 (Pico 2) platform matrix

- **Generated:** 2026-10-06 21:41  _(re-measured in full on the current firmware: catalogue + `scale` sweep, no replay)_
- **Board:** RP2350 (Pico 2), native USB, GPIO2..GPIO9 = D0..D7
- **Firmware:** arduino-pico, driver `pio`; tag key `rpipico2_arduino_pio_pio2_dir`
- **Scenarios:** 27 recorded

## Catalogue

| scenario | verdict | steps hw/exp | note |
|---|---|---|---|
| SR_01 | PASS | 8/8 | period 25.0 us, 0 long / 0 short gaps |
| SR_02 | PASS | 255/255 | period 10.0 us, 0 long / 0 short gaps |
| SR_03 | PASS | 8/8 | period 200.0 us, 0 long / 0 short gaps |
| SR_04 | PASS | 4/4 | period 4095.9375 us, 0 long / 0 short gaps |
| SR_05 | PASS | 16/16 | period 12.5 us, 0 long / 0 short gaps |
| SR_06 | PASS | 4/4 | period 100.0 us, 0 long / 0 short gaps |
| SR_07 | PASS | 2000/2000 | period 10.0 us, 0 long / 0 short gaps |
| SR_08 | PASS | 4000/4000 | period 10.0 us, 0 long / 0 short gaps |
| SR_09 | PASS | 10/10 | pause 800.0 us, gap 40.0 vs 840.0 us |
| SR_10 | PASS | 40/40 |  |
| SR_11 | PASS | 40/40 | dir 01 vs 01, steps [20, 20] |
| SR_12 | PASS | 30/30 | dir 101 vs 101, steps [10, 10, 10] |
| SR_13 | PASS | 0 | 0 pulses for a rejected command (8 requested) |
| SR_14 | PASS(reported) | - | skew 172.25 us (reported) |
| SR_15 | PASS(reported) | - | skew 50.0 us (reported) |
| SR_16 | PASS | - | A 9.9881 us; B 9.9881 us |
| SR_17 | SKIP | - | skipped: driver not in this build |
| SR_18 | SKIP | - | skipped: driver not in this build |
| SR_19 | SKIP | - | skipped: driver not in this build |
| SR_20 | SKIP | - | skipped: driver not in this build |
| SR_21 | PASS | 200/200 | period 10.0 us, 0 long / 0 short gaps |
| SR_23 | SKIP | - | skipped: driver not in this build |
| SR_25 | PASS | 4080/4080 | period 10.0 us, 0 long / 0 short gaps |
| SR_26 | PASS | 2/2 | pause 4095.9375 us, gap 8191.75 vs 8191.875 us |
| SR_27 | PASS | 1/1 | period 200.0 us, 0 long / 0 short gaps |
| SR_30 | PASS | 1272/1272 | stopped at 1272 of 16320, partial pulses 0; `POS` asserted against the wire (delta 0-3) |
| SR_31 | PASS | - | 8 stepper(s) on pio (searched down from 8); 0 count(s) above it refused |

## Parallel stepper count (`scale`)

Each row is one point of the count sweep: what every stepper measured, not just whether the run passed.

| driver list | n | pins | steppers: period x steps | spread us | result |
|---|---|---|---|---|---|
| pio | 1 | nodir | A 9.9841usx64/64 | 0.0 | passed |
| pio+pio | 2 | nodir | A 9.9881usx64/64 B 9.9881usx64/64 | 0.0 | passed |
| pio+pio+pio | 3 | nodir | A 9.9881usx64/64 B 9.9881usx64/64 C 9.9881usx64/64 | 0.0 | passed |
| pio+pio+pio+pio | 4 | nodir | A 9.9881usx64/64 B 9.9841usx64/64 C 9.9881usx64/64 D 9.9841usx64/64 | 0.004 | passed |
| pio+pio+pio+pio+pio | 5 | nodir | A 9.9881usx64/64 B 9.9841usx64/64 C 9.9881usx64/64 D 9.9841usx64/64 E 9.9881usx64/64 | 0.004 | passed |
| pio+pio+pio+pio+pio+pio | 6 | nodir | A 9.9881usx64/64 B 9.9881usx64/64 C 9.9841usx64/64 D 9.9841usx64/64 E 9.9881usx64/64 F 9.9841usx64/64 | 0.004 | passed |
| pio+pio+pio+pio+pio+pio+pio | 7 | nodir | A 9.9881usx64/64 B 9.9881usx64/64 C 9.9841usx64/64 D 9.9841usx64/64 E 9.9841usx64/64 F 9.9881usx64/64 G 9.9881usx64/64 | 0.004 | passed |
| pio+pio+pio+pio+pio+pio+pio+pio | 8 | nodir | A 9.9881usx64/64 B 9.9881usx64/64 C 9.9881usx64/64 D 9.9881usx64/64 E 9.9881usx64/64 F 9.9841usx64/64 G 9.9841usx64/64 H 9.9881usx64/64 | 0.004 | passed |

## Driver capability

Read from the board by the `DRIVERS` command, not from a host table. A host table could only be a copy of the library's declared `QUEUES_*` constants, and those count allocations rather than working steppers: `QUEUES_MCPWM_PCNT` is 6, the board does allocate six, and only one of them runs.

| target | drivers this build accepts | i2s multiplexer |
|---|---|---|
| rpipico2 / arduino / sdk latest | pio, timer | not reported (no I2S on this target) |

## Closed: `POS` after `XSTOP`

SR_30 passes on the wire (`steps_after_stop = 0`, queue discarded). Its `POS`
after `XSTOP` was `0` in 15 of 40 runs while the wire carried the full step
count — the PIO's non-blocking position push leaves stale samples in the RX
FIFO, and the read used one as current. Fixed in `src/pd_pico/pico_queue.cpp`
and now asserted by the harness (`check_commanded_position`): `POS 0` in 0 of 70
runs after the fix. See
[`extras/doc/implemented/pico_position_read_returns_zero.md`](../../../doc/implemented/pico_position_read_returns_zero.md).
