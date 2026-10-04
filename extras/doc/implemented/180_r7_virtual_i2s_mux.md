# 180 R7 — Virtual I2S Mux: 37-Channel Test Harness Extension — **as built**

> **Status: implemented and measured.** The design reference is
> `extras/tests/saleae_based/white_paper_180_r7_virtual_i2s_mux.md`. This file is
> what was actually built, what it measures on hardware, and the three places
> the design reference is wrong — each of which was found by measurement, and
> each of which changed the implementation.

## 1. What exists

| Piece | Where |
|---|---|
| VCD→VCD decoder, 8 channels in / 37 out | `scripts/i2s_mux_decoder.py` |
| Its tests (33, hardware-free) | `scripts/tests/test_i2s_mux_decoder.py` |
| Wire-protocol ground truth, measured | `scripts/probe_mux_bits.py` |
| Harness integration (map, decode, grid, tags) | `scripts/run_tests.py`, `scripts/harness.py` |
| Firmware: mux steppers, slot map, 3-channel bus | `common/saleae_app.cpp` |

Measured on an ESP32-DevKitC with the I2S mux on analyzer channels
**D5 (data) / D6 (bclk) / D7 (ws)**, a Saleae Logic at 24 MS/s.

**`--mode scale --driver i2s_mux --pin-mode nodir` sweeps 1…32 multiplexed
steppers. 29 of 32 stepper counts pass: every stepper's own step count and its own
period, through the decoded channel, with no mux-specific code in any evaluator.**
The three failures are a real driver finding — §5.

## 2. The bus takes the LAST three channels

Design decision, and the reason is that nothing else may move:

```
D0..D4   five stepper channels (step / dir of a physical driver)
D5,D6,D7 I2S data, bclk, ws
```

The bus is the **tail**, so the stepper channel map stays `stride * i` from
channel 0 with no offset in front of it, and a mux capture is a superset of a
physical one: decode it and D0..D4 keep their names. `SALEAE_BUS_BASE` in
`common/saleae_app.cpp` and the test-side `_bus_names()` default agree on that.

A mux stepper therefore **costs a bit of the 32-bit word and no analyzer
channel**, which is what removes the channel budget as its limit: `nodir` reaches
32, `dir` reaches 16 (a mux direction is a second bit of the *same* word).

`IMUX` takes no arguments for the same reason. The GPIO-to-channel table is per
board and per cable — the ESP32-DevKitC map is not the ESP32-S3 one — so naming
the bus pins from the host would mean naming this rig's wiring from something
that cannot see it, and a wrong bus decodes into 32 quiet channels.

## 3. Three things the design reference gets wrong

### 3.1 A slot is high for one bit clock, not for the frame

§3.3 says the bit is set "for `I2S_TICKS_PER_FRAME` (64 ticks = 4 µs),
producing a high pulse". Measured on the wire: **125 ns — one bclk period.** The
frame is the unit of *time*; a slot is one of 32 bits in it, so setting a slot
raises one bit for one bclk. The frame is the unit the receiving shift register
latches, which is why the decoder *widens* it back out to a frame: that is what
makes a decoded channel look like a GPIO step channel.

### 3.2 ≥ 8 MS/s is not enough; and 48 MS/s is unusable

§4.2 and §9.2 reason from "samples per bclk period" and get the arithmetic
wrong by 4–5×: 24 MS/s is **3** samples per 8 MHz bclk period, not 16. One
sample per bclk period (8 MS/s) is the Nyquist limit and recovers nothing.

Measured, one multiplexed stepper, 64 steps / 400 ticks, decoding the whole run:

| rate | samples/bclk | window | steps recovered | frames skipped |
|---|---|---|---|---|
| 16 MS/s | 2 | 2.0 s | 64 / 64 | 3543 (0.7 %) |
| **24 MS/s** | **3** | **2.0 s** | **64 / 64** | **393 (0.08 %)** |
| 48 MS/s | 6 | 0.18 ms | 0 / 64 | — (missed the run) |

**24 MS/s**, and not for the reason the arithmetic suggests. Three samples per
bit period is below the four Nyquist wants and is right anyway: nothing here
reconstructs the bit clock, it reads a value that is held for the whole cell.
48 MS/s resolves the clock better and is **worse** — this analyzer truncates an
eight-channel 48 MS/s capture to 0.18 ms, and a scenario lasts milliseconds, so
the capture covers a run that has not started. The floor is two-sided in
`harness.py` and both sides are tested.

### 3.3 The word goes out MSB first — confirmed, and worth having measured

`probe_mux_bits.py` puts four steppers on slots 0, 7, 16 and 31 at four
different periods and reads which bclk bit of each frame carries which slot:

| wire bit (0 = first bclk of the frame) | measured period | stepper | slot |
|---|---|---|---|
| 31 | 6.4 frames = 25.6 µs | 0 | 0 |
| 30 | 12.5 frames = 50 µs | 1 | 1 |
| 29 | 19 frames = 76 µs | 2 | 2 |
| 28 | 25 frames = 100 µs | 3 | 3 |

**MSB first, slot S = bit S of the word**, matching
`i2s_mux_slot_to_bit_pos()`. One bit of information, and a wrong guess would have
silently put every stepper's steps on another stepper's channel.

Distinct periods per stepper are what make this answerable: at one shared period
the pulses land on top of each other and a word with several bits set does not
say which bit belongs to which stepper.

## 4. The one non-obvious thing in the decoder

**Sample the data on the bclk rising edge.** Two plausible alternatives were
tried on hardware and both are wrong, silently — no exception, just a decode
that is confidently incorrect:

| sample point | result on the 24 MS/s 64-step capture |
|---|---|
| bclk rising edge | slot 0, 64 steps — **correct** |
| middle of the clock's high time | slots 0 *and* 1, 37 steps |
| last sample of the cell | slot **1**, 64 steps |

The middle-of-high-time rule needs a 50 % clock, and the ESP32 does not use one:
measured 4 samples high in 6 at 48 MS/s and 1 in 3 at 24 MS/s. The last-sample
rule lands on the next cell's first sample at 3 samples per cell.

The rising edge is right because of what the edges *mean* here: data is launched
half a cell before the observed bclk rising edge (on the 48 MS/s capture the data
cell runs 208…213 with bclk rises at 211 and 217), so every bit an observed
rising edge names has been stable for a full half cell.

Two consequences worth keeping: a **synthetic bus** is what pins this down —
`vcd_fixtures`-style rendering with known words, where a wrong sample point is
visible as a wrong word rather than as a mystery on hardware. And the earlier
lead that offset +2 "looked clean" was an artifact: +2 shifts the decode by one
slot, and the shifted slot happened to be clean.

## 5. Finding: an intermittent dropped step at 20+ slots

`--mode scale --driver i2s_mux --pin-mode nodir`, 64 steps at 400 ticks:

| n | result | steppers failing |
|---|---|---|
| 1–19 | all passed | — |
| **20** | failed | 16 of 20 — **slots 0–15** |
| 21–24 | passed | — |
| **25** | failed | 16 of 25 — **slots 0–15** |
| **26** | failed | 1 of 26 — slot 16 |
| 27–32 | all passed | — |

The signature is identical in every failing case: **63 of 64 steps, and one
47.96 µs inter-step period** — two 24 µs frame periods, i.e. one step missing
part-way through the run, not at either end. For n=20 and n=25 the same 16 slots
lose it at the same instant (288.257 ms into the capture, gap index 4), which is
a *shared* event rather than 16 independent ones; slots 16–31 are complete.

**Not reproducible on demand**: n=20 was re-measured three times consecutively
after the sweep and passed all three. So it is intermittent, not a bound.

Suspected mechanism, offered as a lead rather than a conclusion: the buffer is
DMA'd, and `i2s_fill_buffer_mux()` writes each slot with
`buf[frame_pos * 4 + byte_offset] |= bit_mask` — a read-modify-write on memory
the DMA is reading. Whether that loses bits on the low or the high half of the
word would depend on ordering, which is what the "low half" bias is consistent
with. I2S is the only driver of the three that streams from a DMA buffer, and
`AGENTS.md` already records its post-abort leak for that reason.

This is recorded, not worked around: the evaluator still fails the run, because a
step that went missing is a step that went missing.

## 6. Other defects found and fixed on the way

Both pre-existing, both found because the mux reaches what they were hiding.

| Defect | Effect | Fix |
|---|---|---|
| `linelen` is `uint8_t` in the serial line reader | A line over 255 characters wraps and **overwrites the head of the line buffer**, so a 271-character `CONFIG 32 i2s_mux,…` arrived as a mangled command and was reported `ERR unknown` — a complaint about the *command* when the *argument* had been lost. Invisible until the driver list got long enough; the `ARG2_MAX` ladder had guaranteed it never would. | `uint16_t`, +1 byte on AVR |
| `Pins.letters` was `sorted()`, and steppers are labelled A…Z then AA… | Past 26 steppers `sorted()` puts "A1" between "A" and "B", so stepper 27 got stepper 2's channel. Past Z, `ord(letter) - ord("A")` is meaningless and raises. | Insertion order (the map is built in stepper order) and `Pins.index_of()` |
| `QINFO_REPLY_FOR` sized for a one-digit stepper index | `SALEAE_QINFO_REPLY_MAX` is 15 bytes short at 32 steppers, so `QINFO` truncated mid-number and the host reported "no QINFO reply" with no hint the firmware ran out of buffer. Caught by the existing buffer test. | 18 bytes/stepper |
| A mode tag spelled the driver once per stepper | 32 mux steppers is 256 characters, which with the prefix and the decoder's `_37ch` suffix passes the 255-byte filename limit — and the capture is then silently not written, surfacing as "sr -> vcd conversion failed". | one name per distinct driver |
| `default_channel_map` labels stop at H | `IndexError: string index out of range` at 9 steppers. | `stepper_letter()`, 32 wide |

And two that were mine, caught by the existing tests rather than by inspection:
`send_imux` needed its arguments removed along with `IMUX`, and `read_map` had to
stop `sorted()`ing the channel map (§6).

## 7. Configuration

Both sources the design reference asks for, metadata first:

```
$comment
  I2S_MUX_DECODED source=capture_8ch.vcd channels=37
  bus: data=D5, bclk=D6, ws=D7
  passthrough: D0, D4
  slots: A=S0, B=S1
  stepper_count: 2
  pin_mode: nodir
$end
```

`DecoderConfig.resolve()` reads the VCD's own metadata and falls back to a JSON
file. The VCD is the artifact that was captured, so its metadata cannot have
drifted from it; a sidecar can. The decoded VCD carries the metadata forward, so
it is self-describing, and `signal_parser.load_vcd()` reads it back unchanged —
which is what makes a mux run re-evaluable without the hardware.

**A mux result is tagged `…_mux`, not `…_nodir`.** A physical 4-stepper `nodir`
run and a 4-slot mux run both read D0–D3 after decoding, and a result recorded
without the distinction says nothing about which produced it.

## 8. Frame-quantised periods

A multiplexed step can only start on a frame boundary, so its period is a **set**,
not a value: 400 ticks is 6, 6, 6, then 7 frames — 24, 24, 24, 28 µs, averaging
exactly 25. Against the harness's ordinary ±5 % band one period in four reads
12 % long, and `rate_adherence` reads the legal pattern as 16 % jitter; both would
fail a run that is exactly right.

`signal_parser.grid_period_defects()` derives the legal set from the grid rather
than hard-coding it (`floor((t + ticks)/F) - floor(t/F)` frames, so `q` or `q+1`
for `q = ticks // F`), and `check_periods()` is the single place that chooses
between the band and the grid — twelve call sites, each of which would otherwise
have to get it right. A period on any other frame count still fails, and the mean
is still held to one frame divided by the number of periods, so a driver running
steadily fast is caught while the grid's own quantisation is not.

## 9. Still open

- **The dropped step** (§5). Intermittent, 3 of 32 sweep points, not reproducible
  on demand. Needs either a longer soak or a host-side loopback to catch it.
- **DIR on the mux.** The firmware allocates it (two slots per stepper, 16 max)
  and `MAP` reports it, but no `dir`-mode sweep has been run against hardware.
- **`sync` on the mux.** Every stepper at its own period through the bus.
- **The `--imux` / `SR_00` ordering.** The mux comes up inside the same serial
  session as the CONFIG that needs it, because opening the port resets the ESP32
  and `initI2sMux()` cannot run twice or survive a reset. That is why
  `SR_00` passes with the mux up: each session is a fresh board.
- **`48 MS/s`** would resolve the bit clock at 6 samples/bit and is the right rate
  for a rig whose buffer is bigger than this clone's.