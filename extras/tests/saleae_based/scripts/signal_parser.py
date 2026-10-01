#!/usr/bin/env python3
"""
signal_parser.py — Signal reconstruction and timing metrics (white paper §2.2).

Turns a sigrok CSV capture into per-channel edges and the metrics the SR_xx
tests are judged on:

  pulse_width_us        step high time / low time
  inter_step_period_us  time between successive step pulses (rising → rising)
  frequency_hz          inverse of the average inter-step period
  duty_cycle_percent    high time / (high time + low time)
  step_count            number of step pulses (rising edges)
  glitch_count          pulses shorter than half the minimum command tick
  dir_to_first_step_us  time from a DIR change to the next step pulse
  cross_channel_skew_us spread of the first step pulse across channels

No third-party dependencies. analyze_csv.py (SR_00) and future SR_01+ scenarios
build on this module.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import Dict, List, Optional, Sequence, Tuple

# Minimum command tick at 16 MHz = 62.5 ns; glitch filter = half of that.
MIN_CMD_TICKS_NS = 62.5
GLITCH_FILTER_NS = MIN_CMD_TICKS_NS / 2.0

Edge = Tuple[int, int]  # (sample index, level after the edge)


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
