#!/usr/bin/env python3
"""
signal_parser.py — Signal reconstruction and timing metrics (white paper §2.2).

Turns a capture into per-channel edges and the metrics the SR_xx tests are
judged on:

  pulse_width_us        step high time / low time
  inter_step_period_us  time between successive step pulses (rising → rising)
  frequency_hz          inverse of the average inter-step period
  duty_cycle_percent    high time / (high time + low time)
  step_count            number of step pulses (rising edges)
  glitch_count          pulses shorter than half the minimum command tick
  dir_to_first_step_us  time from a DIR change to the next step pulse
  cross_channel_skew_us spread of the first step pulse across channels

Captures are recorded as .sr (sigrok srzip: one packed byte per sample) and
evaluated from the VCD sigrok derives from it — a VCD holds only value changes,
so it stays compact and can be inspected in GTKWave.

No third-party dependencies. analyze_csv.py (SR_00) and future SR_01+ scenarios
build on this module.
"""

from __future__ import annotations

import re
import zipfile
from dataclasses import dataclass, field
from typing import Dict, List, Optional, Sequence, Tuple

# Minimum command tick at 16 MHz = 62.5 ns; glitch filter = half of that.
MIN_CMD_TICKS_NS = 62.5
GLITCH_FILTER_NS = MIN_CMD_TICKS_NS / 2.0

Edge = Tuple[int, int]  # (sample index, level after the edge)

# A VCD value change: a level digit followed by the signal identifier.
VCD_CHANGE = re.compile(r"([01])([!\"#$%&'()*+,\-./0-9:;<=>?@A-Z\[\]\\^_`a-z{|}~]+)")


def parse_rate(text: str) -> int:
    """Parse '1 MHz', '4kHz', '1000000 Hz' into Hz."""
    t = text.strip()
    for suffix, mult in (("MHz", 1_000_000), ("kHz", 1_000), ("Hz", 1)):
        if t.endswith(suffix):
            return int(float(t[: -len(suffix)]) * mult)
    return int(float(t))


def load_csv(filepath: str) -> Tuple[Dict[str, List[int]], int]:
    """Load a sigrok CSV capture.

    Returns (channels, sample_rate_hz). Comment lines start with ';', the
    sample rate is read from '; Samplerate:'. Channels are named D0..Dn.
    """
    sample_rate = 20000
    channels: Optional[List[List[int]]] = None

    with open(filepath, "r") as f:
        for raw in f:
            if not raw or raw[0] == ";":
                if raw.startswith("; Samplerate:"):
                    sample_rate = parse_rate(raw.split(":", 1)[1])
                continue

            parts = raw.strip().split(",")
            numeric = all(p in ("0", "1") for p in parts)
            if not numeric:
                continue  # column header row ("logic,logic,...")

            if channels is None:
                channels = [[] for _ in parts]
            if len(parts) != len(channels):
                continue

            for i, p in enumerate(parts):
                channels[i].append(1 if p == "1" else 0)

    if channels is None:
        raise ValueError(f"no samples found in {filepath}")

    return {f"D{i}": ch for i, ch in enumerate(channels)}, sample_rate


def load_sr(filepath: str) -> Tuple[Dict[str, List[int]], int]:
    """Load a sigrok native capture (.sr / srzip).

    The samples are stored as raw unitsize-byte little-endian samples split
    across 'logic-1-N' ZIP members; bit i of a sample is channel D(i). This is
    far more compact and far faster to record than CSV, so high-rate captures
    are not truncated by the output stage.
    """
    with zipfile.ZipFile(filepath) as z:
        meta = z.read("metadata").decode(errors="replace")
        sample_rate = 20000
        unitsize = 1
        capturefile = "logic-1"
        nprobes = 0
        for line in meta.splitlines():
            line = line.strip()
            if line.startswith("samplerate="):
                sample_rate = parse_rate(line.split("=", 1)[1])
            elif line.startswith("unitsize="):
                unitsize = int(line.split("=", 1)[1])
            elif line.startswith("capturefile="):
                capturefile = line.split("=", 1)[1].strip()
            elif line.startswith("total probes="):
                nprobes = int(line.split("=", 1)[1])

        prefix = capturefile + "-"
        chunks = [n for n in z.namelist()
                  if n.startswith(prefix) and n[len(prefix):].isdigit()]
        chunks.sort(key=lambda n: int(n[len(prefix):]))
        raw = b"".join(z.read(n) for n in chunks)

    if nprobes == 0:
        nprobes = unitsize * 8

    channels = {f"D{i}": [] for i in range(nprobes)}
    n_samples = len(raw) // unitsize
    for s in range(n_samples):
        sample = raw[s * unitsize:(s + 1) * unitsize]
        for i in range(nprobes):
            byte = sample[i // 8]
            channels[f"D{i}"].append((byte >> (i % 8)) & 1)
    return channels, sample_rate


VCD_UNITS_NS = {"s": 10 ** 9, "ms": 10 ** 6, "us": 10 ** 3, "ns": 1,
                "ps": 1e-3, "fs": 1e-6}


def parse_timescale(text: str) -> float:
    """Parse a VCD '$timescale 1 us' value into nanoseconds."""
    parts = text.strip().split()
    mult = float(parts[0]) if parts else 1.0
    unit = parts[-1] if len(parts) > 1 else "ns"
    return mult * VCD_UNITS_NS[unit]


def load_vcd(filepath: str) -> Tuple[Dict[str, List[int]], int]:
    """Load a VCD (value changes only) and expand it back to samples.

    sigrok-cli derives the VCD from the .sr capture, so the sample rate is
    recovered from $timescale and the sample spacing. Returns the same
    (channels, sample_rate_hz) shape as load_sr/load_csv.
    """
    names: Dict[str, str] = {}          # vcd id code -> channel name
    changes: Dict[str, List[Tuple[int, int]]] = {}
    timescale_ns: Optional[float] = None
    time = 0
    n_samples = 0

    with open(filepath, "r") as f:
        for raw in f:
            line = raw.strip()
            if not line:
                continue

            if line.startswith("$timescale"):
                timescale_ns = parse_timescale(line.split(None, 1)[1]
                                              .rsplit("$end", 1)[0])
                continue
            if line.startswith("$var"):
                fields = line.split()
                names[fields[3]] = fields[4]
                continue
            if line.startswith("$"):
                continue  # date/version/comment/scope/enddefinitions

            if not line.startswith("#"):
                body = line
            else:
                # A timestamp may share its line with value changes.
                head, _, body = line.partition(" ")
                time = int(head[1:])
                if not body:
                    continue
            # Changes are '<0|1><id>', either space-separated (sigrok) or
            # concatenated.
            if " " in body:
                pairs = VCD_CHANGE.findall(body)
            else:
                pairs = [(body[i], body[i + 1])
                         for i in range(0, len(body) - 1, 2)]
            for level, code in pairs:
                if code in names:
                    changes.setdefault(names[code], []).append((time, int(level)))

    if timescale_ns is None:
        raise ValueError(f"{filepath} has no $timescale")

    for ch in changes.values():
        if ch:
            n_samples = max(n_samples, ch[-1][0])

    # Timestamps are whole samples; derive the rate from that spacing.
    if not changes or not any(changes.values()):
        raise ValueError(f"no value changes found in {filepath}")
    sample_ns = min(ch[1][0] - ch[0][0]
                    for ch in changes.values()
                    if len(ch) > 1 and ch[1][0] > ch[0][0])
    ticks_per_sample = max(1, int(round(sample_ns / timescale_ns)))
    sample_rate = int(round(1e9 / (timescale_ns * ticks_per_sample)))

    channels: Dict[str, List[int]] = {}
    for name in names.values():
        series: List[int] = []
        level = 0
        for time, value in changes.get(name, []):
            series.extend([level] * (time // ticks_per_sample - len(series)))
            series.append(value)
            level = value
        series.extend([level] * (n_samples // ticks_per_sample + 1 - len(series)))
        channels[name] = series

    return channels, sample_rate


def load_capture(filepath: str) -> Tuple[Dict[str, List[int]], int]:
    """Load a capture by extension: .sr (srzip), .vcd or .csv."""
    ext = str(filepath).lower()
    if ext.endswith(".sr"):
        return load_sr(filepath)
    if ext.endswith(".vcd"):
        return load_vcd(filepath)
    return load_csv(filepath)


def detect_edges(samples: Sequence[int]) -> List[Edge]:
    """Return (sample index, level after the edge) for every transition."""
    edges: List[Edge] = []
    prev: Optional[int] = None
    for idx, value in enumerate(samples):
        if prev is not None and value != prev:
            edges.append((idx, value))
        prev = value
    return edges


def rising_edges(samples: Sequence[int]) -> List[int]:
    """Sample indices of rising edges."""
    return [idx for idx, level in detect_edges(samples) if level == 1]


def falling_edges(samples: Sequence[int]) -> List[int]:
    """Sample indices of falling edges."""
    return [idx for idx, level in detect_edges(samples) if level == 0]


@dataclass
class ChannelMetrics:
    edge_count: int = 0
    step_count: int = 0
    high_widths_us: List[float] = field(default_factory=list)
    low_widths_us: List[float] = field(default_factory=list)
    inter_step_us: List[float] = field(default_factory=list)
    glitch_count: int = 0
    frequency_hz: float = 0.0
    duty_cycle_percent: float = 0.0
    avg_high_us: float = 0.0
    avg_low_us: float = 0.0
    max_pulse_width_us: float = 0.0


def channel_metrics(samples: Sequence[int], sample_rate_hz: int) -> ChannelMetrics:
    """Compute the per-channel timing metrics."""
    us_per_sample = 1_000_000.0 / sample_rate_hz
    m = ChannelMetrics()

    edges = detect_edges(samples)
    m.edge_count = len(edges)

    # Classify each inter-edge interval by the level it represents: a rising
    # edge starts a HIGH interval, a falling edge starts a LOW one.
    for i in range(len(edges) - 1):
        width_us = (edges[i + 1][0] - edges[i][0]) * us_per_sample
        if edges[i][1] == 1:
            m.high_widths_us.append(width_us)
        else:
            m.low_widths_us.append(width_us)

    m.step_count = len(rising_edges(samples))

    risings = rising_edges(samples)
    for i in range(1, len(risings)):
        m.inter_step_us.append((risings[i] - risings[i - 1]) * us_per_sample)

    glitch_threshold_us = GLITCH_FILTER_NS / 1000.0
    m.glitch_count = sum(
        1 for w in (m.high_widths_us + m.low_widths_us) if w < glitch_threshold_us
    )

    if m.inter_step_us:
        avg_inter = sum(m.inter_step_us) / len(m.inter_step_us)
        m.frequency_hz = 1_000_000.0 / avg_inter if avg_inter > 0 else 0.0

    # Duty from averages: a capture rarely holds equal counts of high and low
    # intervals, and summing would bias the result.
    m.avg_high_us = (
        sum(m.high_widths_us) / len(m.high_widths_us) if m.high_widths_us else 0.0
    )
    m.avg_low_us = (
        sum(m.low_widths_us) / len(m.low_widths_us) if m.low_widths_us else 0.0
    )
    total = m.avg_high_us + m.avg_low_us
    m.duty_cycle_percent = (m.avg_high_us / total * 100.0) if total > 0 else 0.0

    all_widths = m.high_widths_us + m.low_widths_us
    m.max_pulse_width_us = max(all_widths) if all_widths else 0.0
    return m


def dir_to_first_step_us(
    dir_samples: Sequence[int], step_samples: Sequence[int], sample_rate_hz: int
) -> List[float]:
    """For every DIR change, time until the next step pulse (us)."""
    us_per_sample = 1_000_000.0 / sample_rate_hz
    dir_changes = detect_edges(dir_samples)
    steps = rising_edges(step_samples)

    delays: List[float] = []
    j = 0
    for change_idx, _ in dir_changes:
        while j < len(steps) and steps[j] <= change_idx:
            j += 1
        if j < len(steps):
            delays.append((steps[j] - change_idx) * us_per_sample)
    return delays


def first_step_sample(step_samples: Sequence[int]) -> Optional[int]:
    steps = rising_edges(step_samples)
    return steps[0] if steps else None


def cross_channel_skew_us(
    step_channels: Dict[str, Sequence[int]], sample_rate_hz: int
) -> float:
    """Max spread between the first step pulse of each channels (us)."""
    firsts = [
        first_step_sample(ch)
        for ch in step_channels.values()
        if first_step_sample(ch) is not None
    ]
    if len(firsts) < 2:
        return 0.0
    return (max(firsts) - min(firsts)) * (1_000_000.0 / sample_rate_hz)
