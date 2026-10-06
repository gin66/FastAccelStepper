# Saleae harness — AVR (Arduino Nano, ATmega328P) platform matrix

- **Generated:** 2026-10-06 22:44  _(measured on hardware: catalogue + `scale` sweep, no replay)_
- **Board:** Arduino Nano, ATmega328P (16 MHz, 2 KB SRAM). Fixed cable: Saleae D0..D7 = Nano D12, D11, D10, D9, D5, D4, D3, D2
- **Firmware:** Arduino / `timer` (Timer1); tag key `nanoatmega328_arduino_timer_timer2_dir`
- **Scenarios:** 28 recorded (SR_00–16, 21, 25–27, 30, 31; ESP32-only drivers skip; SR_22/24/28/29 not implemented)

## Catalogue

| scenario | verdict | steps hw/exp | note |
|---|---|---|---|
| SR_00 | PASS | - | 8/8 channels at 1 Hz, commanded duty, cable verified |
| SR_01 | PASS | 8/8 | period 20.0 us, 0 long / 0 short gaps |
| SR_02 | PASS | 255/255 | period 20.0 us, 0 long / 0 short gaps |
| SR_03 | PASS | 8/8 | period 40.0 us, 0 long / 0 short gaps |
| SR_04 | PASS | 4/4 | period 4095.9375 us, 0 long / 0 short gaps |
| SR_05 | PASS | 16/16 | period 20.0 us, 0 long / 0 short gaps |
| SR_06 | PASS | 4/4 | period 20.0 us, 0 long / 0 short gaps |
| SR_07 | PASS | 2000/2000 | period 20.0 us, 0 long / 0 short gaps |
| SR_08 | PASS | 4000/4000 | period 20.0 us, 0 long / 0 short gaps |
| SR_09 | PASS | 10/10 | pause 400.0 us, gap 20.0 vs 420.0 us |
| SR_10 | PASS | 40/40 |  |
| SR_11 | PASS | 40/40 | dir 01 vs 01, steps [20, 20] |
| SR_12 | PASS | 30/30 | dir 101 vs 101, steps [10, 10, 10] |
| SR_13 | PASS | 0 | 0 pulses for a rejected command (8 requested) |
| SR_14 | PASS | - | skew 0.0 us (reported) |
| SR_15 | PASS | - | skew 0.0 us (reported) |
| SR_16 | PASS | - | A 26.5992 us; B 26.5992 us |
| SR_17 | SKIP | - | skipped: driver(s) this build does not have: rmt, mcpwm_pcnt |
| SR_18 | SKIP | - | skipped: driver(s) this build does not have: mcpwm_pcnt |
| SR_19 | SKIP | - | skipped: driver(s) this build does not have: mcpwm_pcnt |
| SR_20 | SKIP | - | skipped: driver(s) this build does not have: mcpwm_pcnt |
| SR_21 | PASS | 200/200 | period 20.0 us, 0 long / 0 short gaps |
| SR_23 | SKIP | - | skipped: driver(s) this build does not have: i2s_direct |
| SR_25 | PASS | 3570/3570 | period 20.0 us, 0 long / 0 short gaps |
| SR_26 | PASS | 2/2 | pause 4095.9375 us, gap 8183.75 vs 8191.875 us |
| SR_27 | PASS | 1/1 | period 40.0 us, 0 long / 0 short gaps |
| SR_30 | PASS | 1025/1025 | stopped at 1025 of 16320, partial pulses 0 |
| SR_31 | PASS | - | 2 stepper(s) on timer (searched down from 8); 6 count(s) above it refused |

## Parallel stepper count (`scale`)

Each row is one point of the count sweep: what every stepper measured, not just whether the run passed.

| driver list | n | pins | steppers: period x steps | spread us | result |
|---|---|---|---|---|---|
| timer | 1 | nodir | A 19.9802usx64/64 | 0.0 | passed |
| timer+timer | 2 | nodir | A 26.5952usx64/64 B 26.5952usx64/64 | 0.0 | passed |
| timer+timer+timer | 3 | nodir | - | - | refused (ERR CONFIG n=3 max=2 slots=8 chans=8/1) |
| timer+timer+timer+timer | 4 | nodir | - | - | refused (ERR CONFIG n=4 max=2 slots=8 chans=8/1) |
| timer+timer+timer+timer+timer | 5 | nodir | - | - | refused (ERR CONFIG n=5 max=2 slots=8 chans=8/1) |
| timer+timer+timer+timer+timer+timer | 6 | nodir | - | - | refused (ERR CONFIG n=6 max=2 slots=8 chans=8/1) |
| timer+timer+timer+timer+timer+timer+timer | 7 | nodir | - | - | refused (ERR CONFIG mode dir|nodir) |
| timer+timer+timer+timer+timer+timer+timer+timer | 8 | nodir | - | - | refused (ERR CONFIG mode dir|nodir) |

## Driver capability

Read from the board by the `DRIVERS` command, not from a host table. A host table could only be a copy of the library's declared `QUEUES_*` constants, and those count allocations rather than working steppers: `QUEUES_MCPWM_PCNT` is 6, the board does allocate six, and only one of them runs.

| target | drivers this build accepts | i2s multiplexer |
|---|---|---|
| nanoatmega328 / arduino / sdk latest | timer | not reported (no I2S on this target) |

## Notes

- **Fixed cable / step pins by identity.** On a 328P the step pins can only be Timer1's compare outputs (D9/D10), which this cable puts on analyzer channels 3 and 2. The firmware claims the compare pin by identity and MAP reports `steps=`/`dirs=`; the host reads those channels and ignores the stride. SR_00 proves all eight cable channels before anything else runs.
- **Two steppers is the ceiling.** `MAX_STEPPER` is 2 and Timer1 has two compare outputs, so `scale` passes n=1 and n=2 and every count from 3 up is refused (`ERR CONFIG n=3 max=2 …`).
- **Stops.** SR_25 (`stopMove`) keeps the queued motion (2553 steps after the marker of 3570 in the fill); SR_30 (`forceStopAndNewPosition`) empties the queue (steps_after_stop=0, queue_discarded=true). The stop instant is read from the marker edge on a free channel (marker=7).
- **Max count.** SR_31 probes down from the channel budget and the board accepts 2 in `nodir`; counts above it are refused by `CONFIG` with `max=2`.

