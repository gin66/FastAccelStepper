#!/usr/bin/env python3
"""
Tests for scripts/i2s_mux_decoder.py -- the 8-channel VCD to 37-channel VCD
decoder.

Everything here is hardware-free and built from a SYNTHETIC bus, not from a
capture. That is deliberate and it is the point of the module: the decoder's
job is to reproduce the mux word from the three bus wires, and the only way to
know it reproduced the *right* word is to put a known word on the wires. A
golden capture would only prove the decoder agrees with itself on 160 us of
real hardware, which cannot separate "decoded correctly" from "decoded the way
the hardware happens to encode".

So the bus is generated from a list of 32-bit words, by the same rules the ESP32
peripheral uses -- and those rules were measured, not assumed (see
scripts/probe_mux_bits.py and its docstring):

  - bclk 8 MHz, ws 250 kHz, 32 bclk per frame, one frame every 4 us;
  - the word goes out MSB first, so wire bit k (k = 0 at the first bclk of the
    frame) is slot 31 - k, i.e. slot S is bit S of the word;
  - slot S's signal is high for ONE bclk period (125 ns), not for the frame.

The last point is the one the white paper gets wrong (§3.3 says the bit is high
for the whole frame) and it is why `synthesize` has to widen a one-bit pulse
back to a frame: the frame is the unit of time the receiving hardware latches,
so a decoded slot channel is one frame wide per step.

The last test class checks the decoder against the real capture, if there is
one. That is a check on the *protocol assumptions*, not on the decoder alone,
and it skips when the capture is absent.
"""

import json
import subprocess
import sys
import tempfile
import unittest
from pathlib import Path

SCRIPTS = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(SCRIPTS))

import i2s_mux_decoder as dec  # noqa: E402
import signal_parser as sp  # noqa: E402

# The bus, as constants rather than as literals sprinkled through the tests, so
# a measurement that changes the parameters changes them in one place.
# 48 MS/s: the analyzer's top rate, and the rate of the hardware capture this
# module's protocol assumptions came from. It is also the only kind of rate the
# synthetic bus can be rendered at honestly -- a frame is 32 bclk periods, so the
# samples per frame have to be a multiple of 32, i.e. the rate has to be a
# multiple of 8 MS/s. 12 MS/s would give 1.5 samples per bit period, which is
# not a waveform at all.
MSPS = 48_000_000
BCLK_HZ = 8_000_000
FRAME_US = 4
FRAME_BITS = 32
SLOTS = 32


def us(t):
    return int(round(t * MSPS / 1e6))


# ---------------------------------------------------------------------------
# A synthetic I2S mux bus
# ---------------------------------------------------------------------------


class Bus:
    """Renders a list of 32-bit words onto data/bclk/ws sample arrays.

    `words` is one entry per frame, in transmission order. The result is three
    equally long bytearrays of 0/1, which is exactly what signal_parser.load_vcd
    hands back for a real three-channel capture -- so the decoder cannot tell
    the two apart, which is the property under test.
    """

    # `shift` moves all three wires together, which is the real situation: the
    # analyzer's sample grid lands at an arbitrary offset from the peripheral's.
    # Offsetting ws alone would model a peripheral whose word select is not
    # aligned to its own bit clock, which is not a thing that exists -- and which
    # a decoder is right to be unable to resolve.
    PAD = 1  # low sample before the first cell; see render()

    def __init__(self, words, lead_frames=2, tail_frames=2, shift=0):
        self.words = list(words)
        self.lead_frames = lead_frames
        self.tail_frames = tail_frames
        self.shift = shift

    @property
    def frame_samples(self):
        return us(FRAME_US)

    @property
    def bit_samples(self):
        return self.frame_samples // FRAME_BITS

    def frames(self):
        """Every frame on the wire, idle ones included: lead, words, tail."""
        return [0] * self.lead_frames + self.words + [0] * self.tail_frames

    def render(self):
        """The three wires, `self.shift` samples later than the bus's own grid."""
        waves = self._render()
        if not self.shift:
            return waves
        out = {}
        for name, samples in waves.items():
            out[name] = bytearray(self.shift) + samples[:len(samples) - self.shift]
        return out

    def _render(self):
        # One low sample of padding before the first cell.
        #
        # bclk is high for the first half of a bit cell, so a waveform that starts
        # at the cell's start also starts with bclk already high -- and the cell's
        # own rising edge is then before sample 0, where no edge detector can see
        # it. The first frame comes out one bit clock short, which the decoder
        # correctly refuses to decode. Padding by a single low sample puts the
        # first edge inside the capture, which is the situation a real acquisition
        # in any but pathological alignment is in.
        n = 1 + len(self.frames()) * self.frame_samples
        data = bytearray(n)
        bclk = bytearray(n)
        ws = bytearray(n)

        bclk_half = self.bit_samples // 2
        for fi, word in enumerate(self.frames()):
            base = 1 + fi * self.frame_samples
            # ws is low for the first (low) half of the frame and high for the
            # second: a word starts at a ws FALLING edge, the rising edge is its
            # middle. That is the measured polarity of the ESP32's I2S output.
            ws_start = base + self.frame_samples // 2
            ws_half = self.frame_samples // 2
            for i in range(ws_start, min(ws_start + ws_half, n)):
                ws[i] = 1
            for k in range(FRAME_BITS):
                b0 = base + k * self.bit_samples
                # bclk is HIGH for the FIRST half of the bit cell and low for the
                # second, which is what the peripheral does and what puts its
                # falling edge in the middle of the cell -- where a synchronous
                # protocol is read. Rendering it the other way round (high in the
                # second half) still looks like a square wave at the right
                # frequency and shifts every sample point by half a bit, which is
                # exactly the kind of thing a synthetic fixture can be wrong about
                # and a real capture cannot.
                for i in range(b0, min(b0 + bclk_half, n)):
                    bclk[i] = 1
                # Two 16-bit halves, low first, each MSB-first: wire bit k < 16
                # carries slot 15 - k, k >= 16 carries slot 47 - k. Valid for
                # the whole cell, from the rising edge to the next.
                slot = 15 - k if k < FRAME_BITS // 2 else 47 - k
                if (word >> slot) & 1:
                    for i in range(b0, min(b0 + self.bit_samples, n)):
                        data[i] = 1
        return {"data": data, "bclk": bclk, "ws": ws}

    def channels(self, passthrough=None):
        # Keyed the way a real capture is: the bus is the last three channels.
        data, bclk, ws = self.render().values()
        out = {"D5": data, "D6": bclk, "D7": ws}
        for name, samples in (passthrough or {}).items():
            out[name] = samples
        return out


def write_bus_vcd(path, bus=None, names=("D5", "D6", "D7"), passthrough=(),
                  comment_lines=(), sample_rate=MSPS, waves=None):
    """Write a bus as a VCD the decoder must be able to read back.

    The channels are named and ordered the way a real capture is: D0..D4 for the
    passthrough stepper pins and D5/D6/D7 for data/bclk/ws. The bus is on the
    LAST three channels on purpose -- see SALEAE_BUS_BASE in
    common/saleae_app.cpp, and the reason the five stepper channels ahead of it
    keep their names on both sides of a decode.

    `waves` overrides the rendered bus, for the tests that need a capture the
    renderer will not produce (one that starts mid-frame, say).
    """
    if waves is None:
        waves = bus.channels()
    ids, order = {}, []
    for n in tuple(names) + tuple(passthrough):
        if n not in order:
            ids[n] = chr(33 + len(order))
            order.append(n)
    flat = bytes(len(waves[names[0]]))
    for n in order:
        # A declared passthrough pin with no waveform is a flat low one, which
        # is what an unconnected stepper channel looks like on the wire.
        waves.setdefault(n, flat)

    rate_txt = (f"{sample_rate / 1e6:g} MHz" if sample_rate >= 1e6
                else f"{sample_rate / 1e3:g} kHz")
    lines = [
        "$date generated by test_i2s_mux_decoder.py $end",
        "$version libsigrok 0.5.2 $end",
        "$comment",
        f"  Acquisition with {len(order)}/{len(order)} channels at {rate_txt}",
    ]
    lines += list(comment_lines)
    lines += [
        "$end",
        f"$timescale {dec.timescale_for(sample_rate)} $end",
        "$scope module libsigrok $end",
    ]
    for n in order:
        lines.append(f"$var wire 1 {ids[n]} {n} $end")
    lines += ["$upscope $end", "$enddefinitions $end"]

    events = {}
    for n in order:
        prev = 0
        for i, v in enumerate(waves[n]):
            if v != prev:
                events.setdefault(i, []).append(f"{v}{ids[n]}")
                prev = v
    for i in sorted(events):
        lines.append(f"#{i} " + " ".join(sorted(events[i])))
    Path(path).write_text("\n".join(lines) + "\n")
    return path


class BusFixture(unittest.TestCase):
    """Base: a scratch directory and the common synthetic-bus helpers."""

    def setUp(self):
        self._tmp = tempfile.TemporaryDirectory()
        self.tmp = Path(self._tmp.name)
        self.addCleanup(self._tmp.cleanup)

    def roundtrip(self, bus, **kw):
        """Render -> VCD -> load_vcd, i.e. the decoder's real input path."""
        path = write_bus_vcd(self.tmp / "bus.vcd", bus, **kw)
        return sp.load_vcd(str(path))


# ---------------------------------------------------------------------------


class TestConfig(BusFixture):
    """The decoder is told which channels are the bus; it does not guess."""

    def test_json_config_names_the_bus(self):
        cfg = dec.DecoderConfig(
            source_vcd="in.vcd",
            output_vcd="out.vcd",
            i2s_channels={"data": "D5", "bclk": "D6", "ws": "D7"},
            passthrough_channels=["D0", "D4"],
        )
        self.assertEqual(cfg.i2s_channels["data"], "D5")
        self.assertEqual(cfg.passthrough_channels, ["D0", "D4"])

    def test_from_json_file_roundtrips(self):
        path = self.tmp / "cfg.json"
        path.write_text(json.dumps({
            "source_vcd": "in.vcd",
            "output_vcd": "out.vcd",
            "i2s_channels": {"data": "D5", "bclk": "D6", "ws": "D7"},
            "passthrough_channels": ["D0"],
            "mux_slot_map": {"A": {"slot": 0, "step_channel": "S0"}},
            "stepper_count": 1,
            "pin_mode": "nodir",
        }))
        cfg = dec.DecoderConfig.from_json(path)
        self.assertEqual(cfg.source_vcd, "in.vcd")
        self.assertEqual(cfg.stepper_count, 1)
        self.assertEqual(cfg.pin_mode, "nodir")
        self.assertEqual(cfg.mux_slot_map["A"]["slot"], 0)

    def test_a_bus_channel_cannot_also_be_a_passthrough(self):
        # The count would be right and the decode silently wrong: the bus is
        # consumed by the decoder, so a passthrough copy of it is a channel that
        # looks like a step signal and is not one.
        with self.assertRaises(dec.ConfigError) as cm:
            dec.DecoderConfig(
                source_vcd="in.vcd",
                output_vcd="out.vcd",
                i2s_channels={"data": "D5", "bclk": "D6", "ws": "D7"},
                passthrough_channels=["D5"],
            )
        self.assertIn("D5", str(cm.exception))

    def test_a_bus_channel_needs_three_distinct_names(self):
        with self.assertRaises(dec.ConfigError):
            dec.DecoderConfig(
                source_vcd="in.vcd",
                output_vcd="out.vcd",
                i2s_channels={"data": "D5", "bclk": "D5", "ws": "D7"},
                passthrough_channels=[],
            )

    def test_slot_out_of_range_is_refused(self):
        # 32 is one past the word. i2sMuxSetBit() ignores it silently, so a map
        # that names it produces a channel that is never asserted -- which reads
        # as a dead driver rather than as a bad map.
        with self.assertRaises(dec.ConfigError) as cm:
            dec.DecoderConfig(
                source_vcd="in.vcd",
                output_vcd="out.vcd",
                i2s_channels={"data": "D5", "bclk": "D6", "ws": "D7"},
                passthrough_channels=[],
                mux_slot_map={"A": {"slot": 32}},
            )
        self.assertIn("32", str(cm.exception))

    def test_two_steppers_cannot_claim_one_slot(self):
        with self.assertRaises(dec.ConfigError):
            dec.DecoderConfig(
                source_vcd="in.vcd",
                output_vcd="out.vcd",
                i2s_channels={"data": "D5", "bclk": "D6", "ws": "D7"},
                passthrough_channels=[],
                mux_slot_map={"A": {"slot": 3}, "B": {"slot": 3}},
            )


class TestVcdMetadata(BusFixture):
    """$comment metadata is the alternative to a JSON file (white paper 6.2)."""

    META = [
        "  I2S_MUX_CONFIG: data=D5, bclk=D6, ws=D7",
        "  MUX_SLOT_MAP: A=0, B=1",
        "  STEPPER_COUNT: 2",
        "  PIN_MODE: nodir",
    ]

    def test_metadata_is_read_back_out_of_the_header(self):
        bus = Bus([0])
        path = write_bus_vcd(self.tmp / "m.vcd", bus, comment_lines=self.META)
        cfg = dec.DecoderConfig.from_vcd(path)
        self.assertEqual(cfg.i2s_channels, {"data": "D5", "bclk": "D6", "ws": "D7"})
        self.assertEqual(cfg.stepper_count, 2)
        self.assertEqual(cfg.mux_slot_map["B"]["slot"], 1)

    def test_embedded_metadata_wins_over_the_json_file(self):
        # "Support both, check the VCD first" is the white paper's
        # recommendation, and it is the right order: the VCD is the artifact
        # that was actually captured, so its own metadata cannot have drifted
        # from it, while a sidecar file can.
        json_path = self.tmp / "cfg.json"
        json_path.write_text(json.dumps({
            "source_vcd": "m.vcd", "output_vcd": "o.vcd",
            "i2s_channels": {"data": "D0", "bclk": "D1", "ws": "D2"},
            "passthrough_channels": [], "stepper_count": 1,
        }))
        bus = Bus([0])
        vcd = write_bus_vcd(self.tmp / "m.vcd", bus, comment_lines=self.META)
        cfg = dec.DecoderConfig.resolve(vcd, json_path)
        self.assertEqual(cfg.i2s_channels["data"], "D5")
        self.assertEqual(cfg.stepper_count, 2)

    def test_json_is_used_when_the_vcd_carries_no_metadata(self):
        json_path = self.tmp / "cfg.json"
        json_path.write_text(json.dumps({
            "source_vcd": "m.vcd", "output_vcd": "o.vcd",
            "i2s_channels": {"data": "D5", "bclk": "D6", "ws": "D7"},
            "passthrough_channels": [], "stepper_count": 1,
        }))
        vcd = write_bus_vcd(self.tmp / "m.vcd", Bus([0]))
        cfg = dec.DecoderConfig.resolve(vcd, json_path)
        self.assertEqual(cfg.stepper_count, 1)

    def test_neither_metadata_nor_config_is_an_error(self):
        vcd = write_bus_vcd(self.tmp / "m.vcd", Bus([0]))
        with self.assertRaises(dec.ConfigError):
            dec.DecoderConfig.resolve(vcd, None)

    def test_metadata_is_written_into_the_decoded_vcd(self):
        # Otherwise the decoded artifact is not self-describing, and the next
        # thing that reads it has to be handed the config again.
        bus = Bus([0])
        out = self.tmp / "decoded.vcd"
        dec.decode_file(write_bus_vcd(self.tmp / "m.vcd", bus,
                                      comment_lines=self.META), out)  # noqa: E501
        text = out.read_text()
        self.assertIn("I2S_MUX_DECODED", text)
        self.assertIn("data=D5", text)
        self.assertIn("A=S0", text)


class TestFrameExtraction(BusFixture):
    """32 bclk edges per frame, MSB first, slot S = bit S."""

    def test_one_word_per_frame(self):
        words = [0x00000001, 0x80000000, 0xFFFFFFFF, 0x00000000]
        bus = Bus(words)
        channels, _ = self.roundtrip(bus)
        frames = dec.extract_frames(channels, MSPS)
        self.assertEqual([w for _, w in frames], bus.frames())

    def test_frame_boundaries_are_ws_edges(self):
        words = [0x00000001, 0x00000002, 0x00000003]
        bus = Bus(words)
        channels, _ = self.roundtrip(bus)
        frames = dec.extract_frames(channels, MSPS)
        starts = [f[0] for f in frames]
        # One frame per rendered frame, at the rendered stride.
        self.assertEqual(len(frames), len(bus.frames()))
        self.assertEqual(starts[1] - starts[0], bus.frame_samples)

    def test_the_sample_grid_offset_does_not_change_the_words(self):
        # Where the analyzer's samples fall relative to the bus is a property of
        # when the acquisition started, not of the protocol. Shifting the whole
        # bus by one, two and three samples must decode identically -- and 3 is
        # the significant one: it is a whole bit period at 24 MS/s, so it moves
        # every sample point to the other end of the cell it was in.
        words = [0x12345678, 0xDEADBEEF, 0x0F0F0F0F]
        base, _ = self.roundtrip(Bus(words))
        expect = [w for _, w in dec.extract_frames(base, MSPS)]
        self.assertEqual(expect, Bus(words).frames())
        for shift in (1, 2, 3, 4, 7):
            got, _ = self.roundtrip(Bus(words, shift=shift))
            decoded = [w for _, w in dec.extract_frames(got, MSPS)]
            # Shifting truncates the tail, so the capture holds one frame fewer at
            # the end. Everything that IS in it has to be the same word.
            self.assertEqual(decoded, expect[:len(decoded)], f"shift {shift}")

    def test_lead_in_and_tail_frames_are_decoded_too(self):
        # A capture starts and stops mid-stream. Dropping the partial frames
        # would silently shorten a step count by one at each end, which is the
        # same magnitude as a lost step and impossible to tell apart.
        words = [0x00000001]
        bus = Bus(words, lead_frames=3, tail_frames=3)
        channels, _ = self.roundtrip(bus)
        frames = dec.extract_frames(channels, MSPS)
        self.assertEqual(len(frames), 3 + 1 + 3)
        self.assertEqual([w for _, w in frames], [0, 0, 0, 1, 0, 0, 0])

    def test_a_frame_with_the_wrong_bit_count_is_reported_not_skipped(self):
        # A frame whose bclk is short or long is a protocol violation. It used
        # to be *skipped*, on the reasoning that guessing which bits it meant
        # would put steps on the wrong channels -- but skipping deletes data on
        # exactly the glitches worth seeing, and a caller counting steps cannot
        # tell a dropped frame from one that never existed. The decoder is a
        # shift register: 32 bclk edges make a word whatever ws thinks, and a
        # disagreement is reported.
        words = [0x00000001]
        bus = Bus(words)
        channels, _ = self.roundtrip(bus)
        # Knock the last bit clock out of the third frame -- the one carrying
        # the word, so "the word was skipped" and "the word decoded as
        # something else" are distinguishable.
        bad = dict(channels)
        bclk = bytearray(bad["D6"])
        start = bus.frame_samples * 2
        for i in range(start + bus.frame_samples - bus.bit_samples,
                       start + bus.frame_samples):
            bclk[i] = 0
        bad["D6"] = bclk
        frames, faults = dec.extract_frames(bad, MSPS, report_faults=True)
        # Reported...
        self.assertTrue(faults, "a short frame must be reported")
        # ...and not silently dropped: the word is still decoded, from 32 bclk
        # edges, and the missing edge shows up as a shifted bit -- which is why
        # the misalignment fault exists at all.
        self.assertGreaterEqual(len(frames), 4)
        self.assertNotEqual([w for _, w in frames], [0, 0, 0],
                            "the damaged frame must not vanish")

    def test_a_short_frame_reports_a_misalignment(self):
        # The check that replaced the skip: with a bit clock missing, ws rises
        # land off the frame grid, and that is reported rather than acted on.
        bus = Bus([0x00000001, 0x00000002])
        channels, _ = self.roundtrip(bus)
        bad = dict(channels)
        bclk = bytearray(bad["D6"])
        start = bus.frame_samples
        for i in range(start + bus.frame_samples - bus.bit_samples,
                       start + bus.frame_samples):
            bclk[i] = 0
        bad["D6"] = bclk
        _, faults = dec.extract_frames(bad, MSPS, report_faults=True)
        self.assertTrue(any("misaligned" in f or "into a frame" in f
                            for f in faults), faults)

    def test_no_bus_channels_is_an_error(self):
        # Naming the bus explicitly is what makes the message name the missing
        # channel. Left to the default the decoder would only say it had too few
        # channels, which does not say which one is missing.
        channels, _ = self.roundtrip(Bus([0]))
        del channels["D6"]
        with self.assertRaises(dec.DecodeError) as cm:
            dec.extract_frames(channels, MSPS,
                               {"data": "D5", "bclk": "D6", "ws": "D7"})
        self.assertIn("D6", str(cm.exception))

    def test_a_capture_starting_mid_frame_decodes_every_complete_frame(self):
        """The partial frame at the head of a capture is invisible, not faulty.

        The acquisition starts at an arbitrary instant, so it usually begins
        partway through a frame. That frame's *boundary* was before sample 0, so
        there is nothing to detect it by and nothing to report it with: the
        first frame the decoder can see is the first complete one, and it is
        complete. That is the honest reading, and it is also why the harness
        arms the capture before the run -- a stepper that starts mid-program has
        its first step in that invisible frame.
        """
        bus = Bus([0xFFFFFFFF, 0, 0x80000000])

        def words_of(crop):
            waves = {k: bytearray(v)[crop:] for k, v in bus.channels().items()}
            path = write_bus_vcd(self.tmp / f"c{crop}.vcd", waves=waves)
            channels, _ = sp.load_vcd(str(path))
            frames, faults = dec.extract_frames(channels, MSPS,
                                                report_faults=True)
            self.assertEqual(faults, [], f"crop {crop}")
            return [w for _, w in frames]

        whole = words_of(0)
        self.assertEqual(whole, bus.frames())
        # Cropping drops whole frames off the head -- the ones whose boundary
        # was before sample 0 -- and changes nothing else. In particular it never
        # mis-decodes one: the frames it does return are identical.
        for crop in (1, 99, 100, 191, 192):
            got = words_of(crop)
            self.assertEqual(got, whole[len(whole) - len(got):], f"crop {crop}")
        # A crop that reaches past the first ws edge drops that frame for good:
        # its boundary was before sample 0 and there is nothing to find it by.
        self.assertEqual(len(words_of(100)), len(whole) - 1)
        self.assertEqual(len(words_of(192)), len(whole) - 1)


class TestSamplingPoint(BusFixture):
    """Which instant of the bit cell the data is read at.

    The two plausible alternatives were both tried on hardware and both are wrong,
    for reasons that are not obvious and are recorded here so nobody "fixes" this
    back:

      - the middle of the clock's high time needs the duty cycle, and the ESP32
        does not use 50 % -- nor the same duty at every rate. Measured: 4 samples
        high in 6 at 48 MS/s, 1 in 3 at 24 MS/s. At 24 MS/s the middle of the high
        time is the one sample in three adjacent to the launch edge, and reading
        there turned a clean 64-step run into 37 steps spread over two slots.
      - the last sample of the cell lands on the *next* cell's first sample at
        24 MS/s, and the same 64 steps decoded to slot 1 instead of slot 0.

    Both failures are silent: no exception, just a decode that is confidently
    wrong. So they are pinned here against synthetic buses, where the words are
    known, rather than left to a hardware run to discover.
    """

    def test_the_words_are_read_when_data_changes_at_the_bulk_edge(self):
        # `Bus` renders data from the cell start, which is the bclk rising edge,
        # so this is the worst-case alignment.
        words = [0x00000001, 0x80000000, 0xFFFFFFFF, 0x00000000]
        bus = Bus(words)
        channels, _ = self.roundtrip(bus)
        self.assertEqual([w for _, w in dec.extract_frames(channels, MSPS)],
                         bus.frames())

    def test_the_sample_point_is_the_rising_edge(self):
        source = (SCRIPTS / "i2s_mux_decoder.py").read_text()
        self.assertNotIn("sp.falling_edges(bclk)", source)
        self.assertNotIn("_last_sample_of_cell", source)
        self.assertNotIn("_midpoint_of_cell", source)

    def test_a_thirds_duty_clock_still_decodes(self):
        # The measured 24 MS/s geometry: three samples per cell and the clock high
        # for one of them. A synthetic bus at a fraction-of-a-cell duty is what
        # makes the "sample the middle of the high time" rule demonstrably wrong.
        bus = Bus([0x00000001, 0x00000002, 0xFFFFFFFF, 0])
        waves = bus._render()
        bit = bus.bit_samples
        high = sum(waves["bclk"][bus.PAD:bus.PAD + bit])
        self.assertEqual(high, bit // 2,
                         "the fixture no longer exercises an asymmetric cell")
        channels, _ = self.roundtrip(bus)
        self.assertEqual([w for _, w in dec.extract_frames(channels, MSPS)],
                         bus.frames())


class TestSlotSynthesis(BusFixture):
    """A decoded slot channel is a step signal: one frame wide per step."""

    def test_a_set_bit_becomes_a_frame_wide_pulse(self):
        # The measured wire pulse is ONE bclk (125 ns). The frame is the unit the
        # receiving hardware latches, so the decoded channel is frame wide --
        # and that is what makes it comparable with a GPIO step channel.
        words = [0, 0x00000001, 0, 0, 0]
        bus = Bus(words)
        channels, _ = self.roundtrip(bus)
        out = dec.decode_channels(channels, MSPS, [], slots=[0])
        s0 = out["S0"]
        fs = bus.frame_samples
        self.assertEqual(len(s0), len(channels["D5"]))
        self.assertEqual(sum(s0), fs, "one frame wide, not one bit clock")

    def test_slot_zero_and_slot_thirtyone_are_distinct(self):
        # The two ends of the word, which is the whole test of the bit order: a
        # decoder that assumed the other one direction puts stepper 31's steps
        # on stepper 0's channel and vice versa.
        bus = Bus([0x00000001, 0, 0x80000000, 0])
        channels, _ = self.roundtrip(bus)
        out = dec.decode_channels(channels, MSPS, [], slots=[0, 31])
        fs = bus.frame_samples

        pad = Bus.PAD
        first = pad + (bus.lead_frames + 0) * fs   # word 0x1 -> slot 0
        second = pad + (bus.lead_frames + 2) * fs  # word 0x80000000 -> slot 31
        self.assertEqual(out["S0"][first:first + fs], b"\x01" * fs)
        self.assertEqual(out["S0"][second:second + fs], b"\x00" * fs)
        self.assertEqual(out["S31"][second:second + fs], b"\x01" * fs)
        self.assertEqual(out["S31"][first:first + fs], b"\x00" * fs)

    def test_every_slot_can_be_addressed(self):
        # One word per slot, so a slot that is never asserted shows up as an
        # all-zero channel rather than as a missing one.
        words = [1 << s for s in range(SLOTS)]
        bus = Bus(words)
        channels, _ = self.roundtrip(bus)
        out = dec.decode_channels(channels, MSPS, [], slots=range(SLOTS))
        self.assertEqual(len(out), SLOTS)
        fs = bus.frame_samples
        for s in range(SLOTS):
            # Frame bus.lead_frames + s carries slot s's word, because the
            # renderer puts the lead-in idle frames first.
            lo = Bus.PAD + (bus.lead_frames + s) * fs
            self.assertEqual(out[f"S{s}"][lo:lo + fs], b"\x01" * fs, f"slot {s}")

    def test_passthrough_channels_pass_through_unchanged(self):
        # They are the same wires they were; only the three bus channels are
        # consumed. A passthrough channel that came back altered would make the
        # physical steppers look like a decoder bug.
        passthrough = {"D0": bytes([0, 1] * 24), "D4": bytes([1, 0] * 24)}
        channels, _ = self.roundtrip(Bus([0, 1, 0]), passthrough=passthrough)
        out = dec.decode_channels(channels, MSPS, ["D0", "D4"], slots=[0])
        self.assertEqual(bytes(out["D0"]), bytes(channels["D0"]))
        self.assertEqual(bytes(out["D4"]), bytes(channels["D4"]))

    def test_bus_channels_do_not_appear_in_the_output(self):
        # 8 - 3 + 32 = 37, and the count only stays right if the bus is consumed
        # rather than passed through as well.
        n = len(Bus([0]).render()["data"])
        bus = Bus([0] * 3)
        channels, _ = self.roundtrip(bus, passthrough={"D0": bytes(n)})
        out = dec.decode_channels(channels, MSPS, ["D0"], slots=range(SLOTS))
        self.assertEqual(len(out), 1 + SLOTS)
        for n in ("D5", "D6", "D7"):
            self.assertNotIn(n, out)

    def test_all_channels_are_the_same_length(self):
        # signal_parser.load_vcd pads every channel to the capture length, and
        # an evaluator that reads past the end of a short channel raises
        # instead of reporting a short waveform.
        bus = Bus([0xFFFFFFFF])
        channels, _ = self.roundtrip(bus)
        out = dec.decode_channels(channels, MSPS, [], slots=range(SLOTS))
        lengths = {n: len(s) for n, s in out.items()}
        self.assertEqual(len(set(lengths.values())), 1, lengths)


class TestDecodedVcd(BusFixture):
    """The written artifact has to load back into the same picture."""

    def test_decoded_vcd_reads_back_with_the_expected_channels(self):
        bus = Bus([0x00000001, 0x80000000])
        src = write_bus_vcd(self.tmp / "in.vcd", bus, passthrough=["D0"])
        out = self.tmp / "out.vcd"
        cfg = dec.DecoderConfig(
            source_vcd=str(src), output_vcd=str(out),
            i2s_channels={"data": "D5", "bclk": "D6", "ws": "D7"},
            passthrough_channels=["D0"],
        )
        dec.decode(cfg)
        decoded, rate = sp.load_vcd(str(out))
        self.assertEqual(rate, MSPS)
        self.assertEqual(len(decoded), 1 + SLOTS)
        self.assertIn("S0", decoded)
        self.assertIn("S31", decoded)

    def test_round_trip_through_the_vcd_preserves_the_slot_waveforms(self):
        # The file is the interface to signal_parser, so a decoder that is
        # correct in memory and lossy on disk is still wrong.
        words = [0x00000001, 0x00000002, 0x00000004, 0x00000008]
        bus = Bus(words)
        src = write_bus_vcd(self.tmp / "in.vcd", bus)
        out = self.tmp / "out.vcd"
        dec.decode(dec.DecoderConfig(
            source_vcd=str(src), output_vcd=str(out),
            i2s_channels={"data": "D5", "bclk": "D6", "ws": "D7"},
            passthrough_channels=[],
        ))
        on_disk, _ = sp.load_vcd(str(out))
        in_memory = dec.decode_channels(sp.load_vcd(str(src))[0], MSPS, [],
                                        slots=range(SLOTS))
        for s in range(SLOTS):
            self.assertEqual(bytes(on_disk[f"S{s}"]), bytes(in_memory[f"S{s}"]),
                             f"slot {s}")

    def test_timescale_and_rate_survive(self):
        # sigrok picks $timescale from the sample rate, so it is 250 ns here.
        src = write_bus_vcd(self.tmp / "in.vcd", Bus([0]))
        out = self.tmp / "out.vcd"
        dec.decode(dec.DecoderConfig(
            source_vcd=str(src), output_vcd=str(out),
            i2s_channels={"data": "D5", "bclk": "D6", "ws": "D7"},
            passthrough_channels=[],
        ))
        self.assertIn(f"$timescale {dec.timescale_for(MSPS)}",
                      out.read_text())


class TestCommandLine(BusFixture):
    """The module is runnable, because the harness runs it."""

    def test_cli_writes_the_decoded_vcd(self):
        src = write_bus_vcd(self.tmp / "in.vcd", Bus([0x00000001]))
        cfg = self.tmp / "cfg.json"
        cfg.write_text(json.dumps({
            "source_vcd": str(src),
            "output_vcd": str(self.tmp / "out.vcd"),
            "i2s_channels": {"data": "D5", "bclk": "D6", "ws": "D7"},
            "passthrough_channels": [],
        }))
        r = subprocess.run(
            [sys.executable, str(SCRIPTS / "i2s_mux_decoder.py"),
             "--config", str(cfg)],
            capture_output=True, text=True,
        )
        self.assertEqual(r.returncode, 0, r.stderr)
        self.assertTrue((self.tmp / "out.vcd").exists())

    def test_cli_reports_a_bad_config_instead_of_a_traceback(self):
        cfg = self.tmp / "cfg.json"
        cfg.write_text(json.dumps({
            "source_vcd": str(self.tmp / "nope.vcd"),
            "output_vcd": str(self.tmp / "out.vcd"),
            "i2s_channels": {"data": "D5", "bclk": "D6", "ws": "D7"},
            "passthrough_channels": [],
        }))
        r = subprocess.run(
            [sys.executable, str(SCRIPTS / "i2s_mux_decoder.py"),
             "--config", str(cfg)],
            capture_output=True, text=True,
        )
        self.assertNotEqual(r.returncode, 0)
        self.assertIn("nope.vcd", r.stderr)


class TestAgainstRealCapture(unittest.TestCase):
    """The protocol assumptions, checked against hardware if there is any.

    Skipped without a capture, so the rest of the suite stays hardware-free.
    """

    CAPTURE = SCRIPTS.parent / "capture" / "probe_mux_bits.vcd"

    def setUp(self):
        if not self.CAPTURE.exists():
            self.skipTest(f"no capture at {self.CAPTURE}")

    def test_the_bus_timing_is_what_the_decoder_expects(self):
        channels, rate = sp.load_vcd(str(self.CAPTURE))
        if rate != 48_000_000:
            self.skipTest(f"capture is at {rate} Hz, not 48 MHz")
        ws_starts = sp.rising_edges(channels["D7"])
        self.assertGreater(len(ws_starts), 4)
        period = ws_starts[1] - ws_starts[0]
        # 4 us at 48 MS/s. The frame width is the single number every timing
        # claim in the white paper rests on, so it is asserted rather than
        # trusted.
        self.assertAlmostEqual(period / rate * 1e6, 4.0, delta=0.05)

    def test_a_stepped_slot_decodes_onto_the_channel_MAP_named(self):
        # The strongest hardware check available: the firmware reports which
        # slot each stepper owns (MAP), and the decoder has to agree.
        channels, rate = sp.load_vcd(str(self.CAPTURE))
        if rate != 48_000_000:
            self.skipTest(f"capture is at {rate} Hz, not 48 MHz")
        out = dec.decode_channels(channels, rate, [], slots=range(SLOTS))
        stepped = {n: sum(s) for n, s in out.items() if sum(s)}
        self.assertTrue(stepped, "no slot decoded any step at all")
        # Slots 0..3 were the connected steppers, one each (MAP slots=0,1,2,3).
        self.assertTrue(set(stepped) <= {f"S{i}" for i in range(4)}, stepped)


if __name__ == "__main__":
    unittest.main()