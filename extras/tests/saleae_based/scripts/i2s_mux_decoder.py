#!/usr/bin/env python3
"""
i2s_mux_decoder.py -- turn a captured I2S bus into step signals.

The ESP32 I2S multiplexer drives up to 32 steppers from **three** wires: one
32-bit word goes out on every I2S frame, one bit per stepper. That is how a
32-stepper run fits an eight-channel logic analyzer, and this module is the
half of that claim that has to be true: it reads the three bus wires and
produces 32 step channels, so an 8-channel capture becomes a 37-channel one
(8 - 3 + 32) that the existing signal parser and evaluators read unchanged.

    8-channel VCD  ->  5 passthrough stepper pins  ->  \
                       32 synthesized mux slots    ->   37-channel VCD

The three bus channels are **consumed**: they do not appear in the output. They
are the decoded representation of the five passthrough channels' unchanged
copies, in the same way S0..S31 are the decoded representation of the bus.

The protocol, as measured rather than assumed
----------------------------------------------
`scripts/probe_mux_bits.py` establishes all of this on hardware, and its
docstring records how. The parts that matter here:

* bclk 8 MHz, ws 250 kHz, 32 bclk per frame, so one frame every 4 us.
* The word goes out as two 16-bit halves, the **low half (slots 0-15) first**,
  each half MSB-first; ws is LOW for the first half and HIGH for the second, so
  a word starts at a ws *falling* edge. Wire bit k < 16 therefore carries slot
  15 - k and wire bit k >= 16 carries slot 47 - k, which makes slot S bit S of
  the word, matching `i2s_mux_slot_to_bit_pos()` in
  src/pd_esp32/i2s_constants.h.
* A set bit is high for **one bclk period** (125 ns) -- not for the frame.
  White paper §3.3 says the bit is high for the whole frame and that is wrong;
  `i2s_fill_buffer_mux()` sets one bit of one 32-bit word, and the word is the
  unit the receiving shift register latches. So `synthesize()` widens a one-bit
  pulse back out to the frame: that is the unit of time a step occupies, and it
  is what makes a decoded slot channel comparable with a GPIO step channel.

Sample rate matters more than the white paper's table says. Resolving an 8 MHz
bclk needs at least 4 samples per bit period, i.e. **>= 32 MS/s**; 12 MS/s is
below Nyquist for it and the bit values are not recoverable. §4.2 of the white
paper calls 8 MS/s "adequate" -- that is one sample per bclk period. 48 MS/s is
this analyzer's top rate and gives 6.
"""

import argparse
import json
import re
import sys
from pathlib import Path
from typing import Dict, Iterable, List, Optional, Sequence, Tuple

sys.path.insert(0, str(Path(__file__).resolve().parent))

import signal_parser as sp  # noqa: E402

# One bit per stepper, and one bit per stepper per frame.
SLOTS = 32
FRAME_BITS = 32

# The bus is the LAST three analyzer channels, so with no configuration the
# decoder can still tell which three they are. The firmware fixes the same
# three (SALEAE_BUS_BASE in common/saleae_app.cpp); the default here exists so
# that a capture with no metadata and no config file still decodes, and
# DecoderConfig makes the choice explicit everywhere it matters.
BUS_TAIL = 3
DATA, BCLK, WS = "data", "bclk", "ws"


class ConfigError(Exception):
    """The decoder was told something that cannot be true."""


class DecodeError(Exception):
    """The capture does not contain what the decoder was asked to decode."""


# ---------------------------------------------------------------------------
# Configuration
# ---------------------------------------------------------------------------


class DecoderConfig:
    """Which channels are the bus, which pass through, and which slot is whose.

    Every field the decoder needs is here and nothing is inferred from the
    capture, because the failure this exists to prevent is a *silently wrong*
    decode: a bus channel named wrongly produces 32 quiet channels, which reads
    as a driver that emits nothing.
    """

    __slots__ = ("source_vcd", "output_vcd", "i2s_channels",
                 "passthrough_channels", "mux_slot_map", "stepper_count",
                 "pin_mode", "source_comment", "out_comment")

    def __init__(self, source_vcd=None, output_vcd=None,
                 i2s_channels: Optional[Dict[str, str]] = None,
                 passthrough_channels: Optional[Iterable[str]] = None,
                 mux_slot_map: Optional[Dict[str, dict]] = None,
                 stepper_count: Optional[int] = None,
                 pin_mode: Optional[str] = None,
                 source_comment: Optional[List[str]] = None,
                 out_comment: Optional[List[str]] = None):
        self.source_vcd = str(source_vcd) if source_vcd is not None else None
        self.output_vcd = str(output_vcd) if output_vcd is not None else None
        self.i2s_channels = dict(i2s_channels or
                                 {DATA: "D5", BCLK: "D6", WS: "D7"})
        self.passthrough_channels = list(passthrough_channels or [])
        self.mux_slot_map = {k: dict(v) for k, v in (mux_slot_map or {}).items()}
        self.stepper_count = stepper_count
        self.pin_mode = pin_mode
        self.source_comment = list(source_comment or [])
        self.out_comment = list(out_comment or [])
        self._validate()

    # -- construction -------------------------------------------------------

    @classmethod
    def from_json(cls, path) -> "DecoderConfig":
        raw = json.loads(Path(path).read_text())
        return cls(
            source_vcd=raw.get("source_vcd"),
            output_vcd=raw.get("output_vcd"),
            i2s_channels=raw.get("i2s_channels"),
            passthrough_channels=raw.get("passthrough_channels"),
            mux_slot_map=raw.get("mux_slot_map"),
            stepper_count=raw.get("stepper_count"),
            pin_mode=raw.get("pin_mode"),
        )

    @classmethod
    def from_vcd(cls, path) -> Optional["DecoderConfig"]:
        """The config embedded in a VCD's $comment block, or None.

        The alternative to a sidecar file (white paper §6.2). Self-describing
        and impossible to mismatch against its own capture, which is why it is
        preferred; the file is the fallback for a VCD that has neither.
        """
        lines = read_comment(Path(path))
        return cls._from_comment(lines, str(path))

    @classmethod
    def _from_comment(cls, lines: Sequence[str], source: str):
        # Digits are in the key class because the canonical key is
        # I2S_MUX_CONFIG: `[A-Za-z_]+` stops at the "2" and the whole line is
        # then unreadable -- which is how this came to be found: a key pattern
        # that silently matches nothing reports "no metadata here", the same as
        # a VCD that genuinely has none.
        fields = {}
        for line in lines:
            m = re.match(r"\s*([A-Za-z0-9_]+)\s*:\s*(.*)$", line)
            if m:
                fields.setdefault(m.group(1).lower(), []).append(m.group(2))

        def one(*names):
            for n in names:
                if n in fields:
                    return fields[n][-1]
            return None

        bus_txt = one("i2s_mux_config", "bus", "i2s_channels")
        if bus_txt is None:
            return None
        i2s = {}
        for item in bus_txt.split(","):
            if "=" in item:
                k, _, v = item.partition("=")
                k = k.strip().lower()
                if k in (DATA, BCLK, WS):
                    i2s[k] = v.strip()

        passthrough = None
        pt_txt = one("passthrough", "passthrough_channels")
        if pt_txt:
            passthrough = [t.strip() for t in pt_txt.split(",") if t.strip()]

        slots = {}
        slot_txt = one("mux_slot_map", "slots")
        if slot_txt:
            for item in slot_txt.split(","):
                if "=" not in item:
                    continue
                k, _, v = item.partition("=")
                k, v = k.strip(), v.strip()
                # Both spellings are accepted: `A=3` (a slot number, what the
                # firmware's MAP reports) and `A=S3` (the decoded channel,
                # what this decoder writes back out).
                if v.startswith("S") and v[1:].isdigit():
                    slots[k] = {"slot": int(v[1:]), "step_channel": v}
                elif v.isdigit():
                    slots[k] = {"slot": int(v), "step_channel": f"S{v}"}

        count_txt = one("stepper_count")
        mode_txt = one("pin_mode")
        return cls(
            source_vcd=source,
            i2s_channels=i2s or None,
            passthrough_channels=passthrough,
            mux_slot_map=slots or None,
            stepper_count=int(count_txt) if count_txt and count_txt.isdigit()
            else None,
            pin_mode=mode_txt.strip() if mode_txt else None,
            source_comment=list(lines),
        )

    @classmethod
    def resolve(cls, vcd_path, config_path=None) -> "DecoderConfig":
        """Embedded metadata first, then the sidecar (white paper §6.2).

        The VCD is the artifact that was actually captured, so its own metadata
        cannot have drifted from it; a sidecar file can. Hence the order.
        """
        embedded = cls.from_vcd(vcd_path)
        if embedded is not None:
            cfg = embedded
        elif config_path is not None:
            cfg = cls.from_json(config_path)
        else:
            raise ConfigError(
                f"{vcd_path} carries no I2S_MUX_CONFIG metadata and no config "
                f"file was given. The bus channels cannot be guessed: a "
                f"misnamed one decodes into 32 quiet channels, which reads as a "
                f"driver that emits nothing."
            )
        if config_path is not None and embedded is not None:
            side = cls.from_json(config_path)
            for name in ("source_vcd", "output_vcd"):
                if getattr(cfg, name) is None:
                    setattr(cfg, name, getattr(side, name))
        return cfg

    # -- validation ---------------------------------------------------------

    def _validate(self):
        for role in (DATA, BCLK, WS):
            if role not in self.i2s_channels:
                raise ConfigError(f"i2s_channels is missing {role!r}")
        names = [self.i2s_channels[r] for r in (DATA, BCLK, WS)]
        if len(set(names)) != 3:
            raise ConfigError(
                f"the three bus roles must name three different channels, got "
                f"{', '.join(names)}: a channel cannot carry data and the bit "
                f"clock at once"
            )
        for role, name in zip((DATA, BCLK, WS), names):
            for other in self.passthrough_channels:
                if other == name:
                    raise ConfigError(
                        f"{name} is both the I2S {role} channel and a "
                        f"passthrough channel. The bus is consumed by the "
                        f"decoder, so a passthrough copy of it would be a "
                        f"channel that looks like a step signal and is not."
                    )
        if len(set(self.passthrough_channels)) != len(self.passthrough_channels):
            dupes = sorted({c for c in self.passthrough_channels
                            if self.passthrough_channels.count(c) > 1})
            raise ConfigError(f"passthrough channels repeated: {', '.join(dupes)}")

        seen = {}
        for letter, entry in sorted(self.mux_slot_map.items()):
            slot = entry.get("slot")
            if not isinstance(slot, int) or slot < 0 or slot >= SLOTS:
                raise ConfigError(
                    f"stepper {letter} names mux slot {slot!r}; a slot is "
                    f"0..{SLOTS - 1}. The mux word is {SLOTS} bits wide, and "
                    f"i2sMuxSetBit() ignores anything wider by returning "
                    f"quietly -- a channel that is never asserted reads as a "
                    f"dead driver rather than as a bad map."
                )
            if slot in seen:
                raise ConfigError(
                    f"steppers {seen[slot]} and {letter} both claim mux slot "
                    f"{slot}. A step signal and a direction signal are both one "
                    f"bit of the same word, so they compete for it."
                )
            seen[slot] = letter
            if entry.get("step_channel") is None:
                entry["step_channel"] = f"S{slot}"

        if self.stepper_count is None and self.mux_slot_map:
            self.stepper_count = len(self.mux_slot_map)

    # -- derived ------------------------------------------------------------

    @property
    def bus_names(self) -> List[str]:
        return [self.i2s_channels[r] for r in (DATA, BCLK, WS)]

    def slots(self) -> List[int]:
        """Which mux slots to synthesize: the named ones, or all 32."""
        if not self.mux_slot_map:
            return list(range(SLOTS))
        return sorted({e["slot"] for e in self.mux_slot_map.values()})

    def channel_map(self) -> Dict[str, dict]:
        """The evaluator-facing map: stepper letter -> step/dir channels.

        The same shape as the harness's MAP (white paper §5.0), so a decoded
        capture is read by the same `Pins` object as a physical one.
        """
        out = {}
        for letter, entry in sorted(self.mux_slot_map.items()):
            out[letter] = {
                "step": entry.get("step_channel", f"S{entry['slot']}"),
                "slot": entry["slot"],
            }
            if entry.get("dir_channel"):
                out[letter]["dir"] = entry["dir_channel"]
        return out

    def comment_lines(self) -> List[str]:
        """The $comment block of the decoded VCD, so it describes itself."""
        lines = list(self.out_comment)
        lines.append(
            f"  I2S_MUX_DECODED source={Path(self.source_vcd).name} "
            f"channels={len(self.passthrough_channels) + SLOTS}"
        )
        lines.append(
            "  bus: " + ", ".join(f"{r}={self.i2s_channels[r]}"
                                  for r in (DATA, BCLK, WS))
        )
        if self.passthrough_channels:
            lines.append("  passthrough: " + ", ".join(self.passthrough_channels))
        cmap = self.channel_map()
        if cmap:
            lines.append("  slots: " + ", ".join(
                f"{letter}={v['step']}" for letter, v in cmap.items()))
        if self.stepper_count is not None:
            lines.append(f"  stepper_count: {self.stepper_count}")
        if self.pin_mode:
            lines.append(f"  pin_mode: {self.pin_mode}")
        return lines


# ---------------------------------------------------------------------------
# Decoding
# ---------------------------------------------------------------------------


def _bus_names(channels: Dict[str, Sequence[int]],
               i2s: Optional[Dict[str, str]] = None) -> Tuple[str, str, str]:
    if i2s is not None:
        return i2s[DATA], i2s[BCLK], i2s[WS]
    names = sorted(channels)
    if len(names) < BUS_TAIL:
        raise DecodeError(
            f"only {len(names)} channels to decode and the bus needs "
            f"{BUS_TAIL}: {', '.join(names)}"
        )
    # The last three, in analyzer-channel order: the firmware puts the bus on
    # channels 5/6/7 (SALEAE_BUS_BASE) precisely so the stepper channels ahead
    # of them keep their names.
    return tuple(names[-BUS_TAIL:])  # type: ignore[return-value]


def extract_frames(channels: Dict[str, Sequence[int]], sample_rate_hz: int,
                   i2s: Optional[Dict[str, str]] = None,
                   report_faults: bool = False):
    """The 32-bit mux word of every I2S frame on the bus.

    This is a shift register, which is what the receiving hardware is: a
    74HC595 shifts one bit in per **bclk rising edge** and after 32 of them the
    word is complete. So the decode is driven by bclk edges and a bit counter,
    and by nothing else.

    Framing and bit order are what the hardware does, measured on the wire:

    * The 32-bit word goes out as two 16-bit halves, each MSB-first, the **low
      half (slots 0-15) first**. The word-select line marks the halves: ws is
      LOW for the first half and HIGH for the second, so a word starts at a ws
      *falling* edge and a ws rising edge sits at its middle, 16 bits in.
    * Consequently the 32 collected bits are the word with its halves swapped:
      `word = (ser >> 16) | ((ser & 0xffff) << 16)`, which makes bit S of the
      finished word slot S -- matching `i2s_mux_slot_to_bit_pos()`.

    What that rules out, deliberately:

    * **No sample rate, no grid, no period.** `sample_rate_hz` is accepted and
      unused. Nothing here knows how long a bit is meant to be, so a capture at
      any rate decodes identically. A frame is 32 bclk edges because the shift
      register says so, not because anything was measured.
    * **ws is a check, not a frame delimiter.** The previous version sliced the
      bclk edges between consecutive ws edges and *discarded* any frame whose
      count was not 32. That silently deletes data on exactly the glitches worth
      seeing, and a caller counting steps cannot tell a dropped frame from one
      that never existed. Here ws edges are checked against the counter and a
      mismatch is *reported*; the word is emitted either way.
    * **No frame is ever dropped.** Every group of 32 bclk edges yields a word.

    The one thing edges cannot decide is where the first group starts: a capture
    that opens mid-word would decode every later word offset by a constant
    number of bits. So the first ws *fall* anchors the phase -- the partial
    group containing it is the only thing discarded. After the anchor the
    counter alone decides.

    The sample point is the bclk rising edge itself, i.e. the level present at
    the edge. Two plausible alternatives were tried and both are wrong on this
    hardware, for measurable reasons:

      - "sample the middle of the high time, so the value has settled". The
        ESP32 does not run bclk at 50 %, and not at the same duty at every rate:
        measured 4 samples high in 6 at 48 MS/s, 1 in 3 at 24 MS/s. At 24 MS/s
        the middle of the high time is the one sample in three next to the launch
        edge, and reading it there turned a clean 64-step run into 37 steps.
      - "sample the last sample of the cell". At 24 MS/s that is the next cell's
        first sample: the same run decoded to slot 1 instead of slot 0.

    The edge is right because of what the edges *mean*: data is launched half a
    cell before the observed rising edge (on a 48 MS/s capture the data cell runs
    208..213 with bclk rises at 211 and 217), so the bit a rising edge names is
    already stable for a full half cell.

    Returns [(first_bclk_sample, word), ...].
    """
    data_name, bclk_name, ws_name = _bus_names(channels, i2s)
    for role, name in ((DATA, data_name), (BCLK, bclk_name), (WS, ws_name)):
        if name not in channels:
            raise DecodeError(
                f"the capture has no {role} channel {name!r} "
                f"(channels: {', '.join(sorted(channels))})"
            )
    data = channels[data_name]
    bclk_rises = sp.rising_edges(channels[bclk_name])
    ws_rises = sp.rising_edges(channels[ws_name])
    ws_falls = sp.falling_edges(channels[ws_name])

    if not ws_falls:
        raise DecodeError(f"the word-select channel {ws_name!r} never falls; "
                          f"there is no I2S bus in this capture")
    if not bclk_rises:
        return ([], ["no bclk rising edges: the bus is not clocking"]) \
            if report_faults else []

    # Index of the first bclk edge at or after each ws edge.
    fall_at = [_lower_bound(bclk_rises, f) for f in ws_falls]
    rise_at = [_lower_bound(bclk_rises, r) for r in ws_rises]

    # A word starts at a ws FALL (low half first), and the capture opens
    # mid-word almost by definition. Which word is the first COMPLETE one
    # depends on where it opened:
    #
    # - during the HIGH half: the first ws edge is a fall and the word
    #   containing it is partial; the first complete word starts at that fall.
    # - during the LOW half: the first ws edge is a rise. If the whole low
    #   half is present (16 bclk edges before the rise), the capture opened
    #   exactly on a word boundary and the word starting at the first bclk
    #   edge is complete. Otherwise that word is partial too, and the first
    #   complete one again starts at the first fall.
    #
    # The partial group is the only thing ever dropped, and only because its
    # boundary is invisible: an edge that happened before sample 0 cannot be
    # reported. After the anchor the counter alone decides.
    if ws_falls[0] < ws_rises[0] or rise_at[0] != FRAME_BITS // 2:
        anchor = fall_at[0]
    else:
        anchor = 0
    frames: List[Tuple[int, int]] = []
    faults: List[str] = []
    word = 0
    nbits = 0
    group_start = anchor

    for i, edge in enumerate(bclk_rises):
        if i < anchor:
            continue
        word = (word << 1) | (1 if data[edge] else 0)
        nbits += 1
        if nbits == FRAME_BITS:
            # The two halves arrive low-first, each MSB-first, so the collected
            # word is the true one with its halves swapped. Swapping back makes
            # bit S of the word slot S.
            frames.append(
                (bclk_rises[group_start], (word >> 16) | ((word & 0xFFFF) << 16)))
            word = 0
            nbits = 0
            group_start = i + 1

    if nbits:
        faults.append(f"{nbits} trailing bit(s) with no frame boundary")
    if bclk_rises[-1] < ws_falls[-1]:
        faults.append("capture ended before the last ws fall; the trailing "
                      "frame is incomplete")

    # ws as a check only: falls on a word boundary (offset 0), rises at the
    # middle (offset 16). A mismatch is reported, never acted on -- the words
    # above were emitted regardless.
    for fall, at in zip(ws_falls, fall_at):
        if at < anchor or at >= len(bclk_rises):
            continue
        off = (at - anchor) % FRAME_BITS
        if off:
            faults.append(
                f"ws fall at sample {fall}: bclk edge {at} is {off} bit(s) into "
                f"a frame, not on a boundary; words from here on may be "
                f"misaligned")
    for rise, at in zip(ws_rises, rise_at):
        if at < anchor or at >= len(bclk_rises):
            continue
        off = (at - anchor) % FRAME_BITS
        if off != FRAME_BITS // 2:
            faults.append(
                f"ws rise at sample {rise}: bclk edge {at} is {off} bit(s) into "
                f"a frame, not at the half (16); words from here on may be "
                f"misaligned")

    if report_faults:
        return frames, faults
    return frames


def _lower_bound(sorted_values: Sequence[int], target: int) -> int:
    lo, hi = 0, len(sorted_values)
    while lo < hi:
        mid = (lo + hi) // 2
        if sorted_values[mid] < target:
            lo = mid + 1
        else:
            hi = mid
    return lo


def synthesize(frames: Sequence[Tuple[int, int]], slots: Iterable[int],
               n_samples: int) -> Dict[str, bytearray]:
    """Turn (frame start, word) pairs into one channel per mux slot.

    A set bit is high for the whole frame, which is the unit the receiving
    hardware latches and the unit a step occupies -- see the module docstring.
    The wire pulse is one bclk period; widening it here is what makes a decoded
    channel look like a GPIO step channel to the evaluators.

    Only the requested slots are built. All 32 on a long capture is one byte per
    sample per slot, so a 10 s capture at 8 MS/s would be 2.5 GB; a caller that
    names four slots from MAP pays for four.
    """
    out = {f"S{s}": bytearray(n_samples) for s in slots}
    # A frame ends where the next one starts, and the last one runs to the end
    # of the capture -- otherwise a step in the final frame is silently half as
    # long as every other one, which is a duty-cycle error with no edge to
    # explain it.
    starts = [f[0] for f in frames]
    for i, (start, word) in enumerate(frames):
        end = starts[i + 1] if i + 1 < len(starts) else n_samples
        if end <= start:
            continue
        run = bytes([1]) * (end - start)
        for s in slots:
            if (word >> s) & 1:
                out[f"S{s}"][start:end] = run
    return out


def decode_channels(channels: Dict[str, Sequence[int]], sample_rate_hz: int,
                    passthrough: Iterable[str],
                    slots: Optional[Iterable[int]] = None,
                    i2s: Optional[Dict[str, str]] = None) -> Dict[str, bytearray]:
    """The decoded channel set: the passthrough pins plus one per mux slot.

    Every channel is the same length as the capture, because the evaluators
    index them and a short one raises instead of reporting a short waveform.
    """
    names = sorted(channels)
    n = max((len(channels[c]) for c in names), default=0)
    out: Dict[str, bytearray] = {}
    for name in passthrough:
        samples = channels.get(name)
        if samples is None:
            raise DecodeError(
                f"passthrough channel {name!r} is not in the capture "
                f"(channels: {', '.join(names)})"
            )
        series = bytearray(n)
        series[:min(n, len(samples))] = samples[:n]
        out[name] = series
    frames = extract_frames(channels, sample_rate_hz, i2s)
    out.update(synthesize(frames, list(range(SLOTS)) if slots is None else slots, n))
    return out


# ---------------------------------------------------------------------------
# VCD writing
# ---------------------------------------------------------------------------


def timescale_for(sample_rate_hz: int) -> str:
    """A `$timescale` for a one-tick-per-sample VCD at this rate.

    `signal_parser.load_vcd()` recovers the sample period from
    `$timescale * $comment rate`, so the two have to agree or every timestamp
    lands on the wrong sample. Emitting exactly 1e9/rate ns guarantees they do.
    """
    ns = 1e9 / float(sample_rate_hz)
    for unit, factor in (("s", 1e9), ("ms", 1e6), ("us", 1e3), ("ns", 1)):
        value = ns / factor
        if value >= 1 and abs(value - round(value)) < 1e-9:
            return f"{int(round(value))} {unit}"
    # Not a whole number of any ns-or-larger unit (48 MS/s is 20.8333 ns).
    # parse_timescale() reads a float, so one tick is still one sample.
    return f"{ns:.6f} ns"


def _var_ids(names: Sequence[str]) -> Dict[str, str]:
    return {n: chr(33 + i) for i, n in enumerate(names)}


def write_vcd(path, channels: Dict[str, Sequence[int]], sample_rate_hz: int,
              comment: Sequence[str] = (), module: str = "i2s_mux") -> Path:
    """One-tick-per-sample VCD, in the form `signal_parser.load_vcd` reads.

    Scalar `$var`s and space-separated changes on a shared timestamp line,
    which is what sigrok emits and what load_vcd's fast path parses. A vector
    `$var` would be silently unreadable -- its `b1010 #` change lines do not
    match the scalar pattern -- so the 32 slots are 32 scalars here, not one bus.
    """
    names = sorted(channels)
    ids = _var_ids(names)
    rate_txt = (f"{sample_rate_hz / 1e6:g} MHz" if sample_rate_hz >= 1e6
                else f"{sample_rate_hz / 1e3:g} kHz")
    lines = [
        "$date generated by i2s_mux_decoder.py $end",
        "$version libsigrok 0.5.2 $end",
        "$comment",
        f"  Acquisition with {len(names)}/{len(names)} channels at {rate_txt}",
    ]
    lines += list(comment)
    lines += [
        "$end",
        f"$timescale {timescale_for(sample_rate_hz)} $end",
        f"$scope module {module} $end",
    ]
    for n in names:
        lines.append(f"$var wire 1 {ids[n]} {n} $end")
    lines += ["$upscope $end", "$enddefinitions $end"]

    events: Dict[int, List[str]] = {}
    for n in names:
        prev = 0
        for i, v in enumerate(channels[n]):
            if v != prev:
                events.setdefault(i, []).append(f"{v}{ids[n]}")
                prev = v
    for i in sorted(events):
        lines.append(f"#{i} " + " ".join(sorted(events[i])))

    out = Path(path)
    out.write_text("\n".join(lines) + "\n")
    # The extent sidecar capture.py also writes. A VCD holds only value changes,
    # so a file whose channels go quiet at the end does not say how long the
    # capture was: the last edge is not the end. load_vcd() honours this file
    # for exactly that reason, and without it a decoded capture whose final
    # frames are all-zero comes back truncated -- which reads as "the run
    # stopped early" rather than as "the recording ended".
    n_samples = max((len(channels[n]) for n in names), default=0)
    out.with_suffix(".meta").write_text(json.dumps({
        "samples": n_samples,
        "sample_rate": sample_rate_hz,
        "source": "i2s_mux_decoder",
    }) + "\n")
    return out


def read_comment(path) -> List[str]:
    """The lines of a VCD's `$comment` block, without the `$end` markers."""
    lines: List[str] = []
    inside = False
    for line in Path(path).read_text().splitlines():
        if line.startswith("$comment"):
            inside = True
            continue
        if inside:
            if line.startswith("$end"):
                break
            lines.append(line)
    return lines


# ---------------------------------------------------------------------------
# Entry points
# ---------------------------------------------------------------------------


def decode(cfg: DecoderConfig, source_rate_hz: Optional[int] = None) -> Path:
    """Decode cfg.source_vcd into cfg.output_vcd and return the output path."""
    if not cfg.source_vcd:
        raise ConfigError("no source_vcd")
    if not cfg.output_vcd:
        raise ConfigError("no output_vcd")
    channels, rate = sp.load_vcd(cfg.source_vcd, source_rate_hz)
    out = decode_channels(channels, rate, cfg.passthrough_channels,
                          cfg.slots(), cfg.i2s_channels)
    return write_vcd(cfg.output_vcd, out, rate, cfg.comment_lines())


def decode_file(source_vcd, output_vcd, config_path=None,
                source_rate_hz: Optional[int] = None) -> Path:
    """Resolve the config for `source_vcd` and write `output_vcd`.

    The output path is an argument rather than something to be found in the
    config: it is the one field a caller always knows, and a config that was
    written for a previous run would otherwise send this run's capture
    somewhere else.
    """
    cfg = DecoderConfig.resolve(source_vcd, config_path)
    if output_vcd is not None:
        cfg.output_vcd = str(output_vcd)
    return decode(cfg, source_rate_hz)


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(
        description="Decode an ESP32 I2S mux bus capture into step channels.")
    ap.add_argument("--config", help="JSON decoder config (see README)")
    ap.add_argument("--source", help="source VCD (overrides the config)")
    ap.add_argument("--output", help="decoded VCD (overrides the config)")
    ap.add_argument("--sample-rate", type=int,
                    help="source sample rate in Hz, if not in the VCD")
    args = ap.parse_args(argv)

    if args.source:
        cfg = DecoderConfig.resolve(args.source, args.config)
    elif args.config:
        cfg = DecoderConfig.from_json(args.config)
    else:
        ap.error("need --source or --config")

    if args.source and not cfg.source_vcd:
        cfg.source_vcd = args.source
    if args.output:
        cfg.output_vcd = args.output
    if not cfg.output_vcd:
        ap.error("no output VCD: pass --output or set output_vcd in the config")

    try:
        out = decode(cfg, args.sample_rate)
    except (ConfigError, DecodeError) as exc:
        print(f"i2s_mux_decoder: {exc}", file=sys.stderr)
        return 2
    print(out)
    return 0


if __name__ == "__main__":
    sys.exit(main())