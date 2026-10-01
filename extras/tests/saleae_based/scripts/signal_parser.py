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


# sigrok writes the acquisition rate into $comment, e.g.
#   Acquisition with 8/8 channels at 1 MHz
VCD_COMMENT_RATE = re.compile(r"channels at ([0-9.]+)\s*([kMG]?Hz)", re.I)


def load_vcd(filepath: str,
             sample_rate_hz: Optional[int] = None
             ) -> Tuple[Dict[str, List[int]], int]:
    """Load a VCD (value changes only) and expand it back to samples.

    The sample rate comes from sigrok's `$comment` ("Acquisition with 8/8
    channels at 1 MHz"), which is authoritative. Pass `sample_rate_hz` to
    override it; only if both are missing does it fall back to inferring the
    spacing from the changes, which is approximate because a capture need not
    contain a shortest-interval pair.

    Returns the same (channels, sample_rate_hz) shape as load_sr/load_csv.
    """
    names: Dict[str, str] = {}          # vcd id code -> channel name
    changes: Dict[str, List[Tuple[int, int]]] = {}
    timescale_ns: Optional[float] = None
    comment_rate_hz: Optional[int] = None
    in_comment = False
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
            if in_comment:
                # sigrok writes "$comment\n  Acquisition ...\n$end".
                if line.startswith("$end"):
                    in_comment = False
                else:
                    m = VCD_COMMENT_RATE.search(line)
                    if m:
                        comment_rate_hz = int(
                            float(m.group(1)) * parse_rate("1" + m.group(2)))
                continue
            if line.startswith("$comment"):
                in_comment = True
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

    if not changes or not any(changes.values()):
        raise ValueError(f"no value changes found in {filepath}")

    if sample_rate_hz is None:
        sample_rate_hz = comment_rate_hz
    if sample_rate_hz:
        ticks_per_sample = max(
            1, int(round(1e9 / (timescale_ns * sample_rate_hz))))
    else:
        # Last resort: infer the spacing from the changes. Approximate, since
        # the capture may not contain the shortest interval.
        sample_ns = min(ch[1][0] - ch[0][0]
                        for ch in changes.values()
                        if len(ch) > 1 and ch[1][0] > ch[0][0])
        ticks_per_sample = max(1, int(round(sample_ns / timescale_ns)))
        sample_rate_hz = int(round(1e9 / (timescale_ns * ticks_per_sample)))

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

    return channels, sample_rate_hz


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

    risings = rising_edges(samples)
    m.step_count = len(risings)
    for i in range(1, len(risings)):
        m.inter_step_us.append((risings[i] - risings[i - 1]) * us_per_sample)

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


def period_defects(periods_us, expected_us, tol_frac=0.05, tol_abs=0.5):
    """Compare measured inter-step periods against the commanded one.

    A short period means two commanded steps arrived as one pulse; a long one
    means a step never arrived (a missing step shows up as a single gap of
    roughly twice the period). Both are defects with no tolerance: a spurious or
    swallowed pulse is a failure, not a statistic to be traded off.
    """
    tol = max(abs(expected_us) * tol_frac, tol_abs)
    short = [p for p in periods_us if p < expected_us - tol]
    long_ = [p for p in periods_us if p > expected_us + tol]
    return {
        "expected_period_us": expected_us,
        "tolerance_us": tol,
        "periods_measured": len(periods_us),
        "short_periods_us": [round(p, 4) for p in short[:16]],
        "long_periods_us": [round(p, 4) for p in long_[:16]],
        "n_short": len(short),
        "n_long": len(long_),
        "ok": not short and not long_,
    }


def rate_adherence(periods_us, commanded_period_us, tol_frac=0.02,
                   tol_abs=0.25):
    """How closely the emitted step rate follows the commanded rate.

    On architectures where a timer compare interrupt calls an ISR that sets the
    step pin, the pin edge happens *inside* the ISR. The ISR entry and body
    therefore eat into the period, so the achieved rate sags below the
    commanded one at high rates -- and the sag grows with the number of active
    steppers, because two ISRs cost more than one. None of that is visible in
    `ticks`, in `getCurrentPosition()`, or to any PC-side test.

    Distinguishes three things a plain average would hide:
      * `sag_pct`    -- systematic: every period is long, the rate is low.
      * `jitter_pct` -- per-step spread about the mean.
      * `n_out_of_tolerance` -- individual steps that miss by more than either.

    The default tolerance is deliberately tight (2 %), because ISR cost is the
    thing being characterized and a 5 % band would hide exactly the effect under
    study. Callers may pass their own.
    """
    if len(periods_us) < 2:
        # A single-step command has no inter-step period at all, so there is
        # nothing to compare against and nothing to fault. Reporting a failure
        # here would reject a perfectly good one-step capture -- and one step is
        # a real boundary, not a degenerate case: it takes a different branch in
        # the ISR than a multi-step entry does. So report it as not measurable
        # and pass.
        return {"commanded_period_us": commanded_period_us,
                "commanded_rate_hz": 1e6 / commanded_period_us,
                "periods_measured": len(periods_us),
                "measurable": False,
                "ok": True,
                "reason": "fewer than 2 inter-step periods: rate adherence "
                          "cannot be measured for a single-step command"}

    mean = sum(periods_us) / len(periods_us)
    lo, hi = min(periods_us), max(periods_us)
    tol = max(abs(commanded_period_us) * tol_frac, tol_abs)
    out = [p for p in periods_us
           if abs(p - commanded_period_us) > tol]

    return {
        "commanded_period_us": commanded_period_us,
        "commanded_rate_hz": 1e6 / commanded_period_us,
        "periods_measured": len(periods_us),
        "measurable": True,
        "mean_period_us": round(mean, 4),
        "min_period_us": round(lo, 4),
        "max_period_us": round(hi, 4),
        "rate_mean_hz": round(1e6 / mean, 2),
        "rate_max_hz": round(1e6 / lo, 2),
        "rate_min_hz": round(1e6 / hi, 2),
        # Positive sag = slower than commanded.
        "sag_pct": round(100.0 * (mean - commanded_period_us)
                         / commanded_period_us, 3),
        "jitter_pct": round(100.0 * (hi - lo) / commanded_period_us, 3),
        "tolerance_us": round(tol, 4),
        "n_out_of_tolerance": len(out),
        "worst_deviation_us": round(
            max((abs(p - commanded_period_us) for p in periods_us), default=0.0),
            4),
        "ok": not out,
    }


def step_count_defects(measured, expected):
    """A spurious pulse gives extra steps; a swallowed one gives missing steps."""
    return {
        "steps_expected": expected,
        "steps_measured": measured,
        "missing_steps": max(0, expected - measured),
        "extra_steps": max(0, measured - expected),
        "ok": measured == expected,
    }


def dir_to_first_step_us(
    dir_samples: Sequence[int], step_samples: Sequence[int], sample_rate_hz: int
) -> List[float]:
    """For every DIR change, time until the next step pulse (us).

    A step that lands in the *same* sample as the DIR change yields 0.0, which
    is the honest reading: the separation is smaller than one sample, not
    absent. Skipping such a step and matching the next one instead would report
    a full step period and hide the Pico case entirely, where the PIO sets DIR
    and STEP from adjacent instructions.
    """
    us_per_sample = 1_000_000.0 / sample_rate_hz
    dir_changes = detect_edges(dir_samples)
    steps = rising_edges(step_samples)

    delays: List[float] = []
    j = 0
    for change_idx, _ in dir_changes:
        while j < len(steps) and steps[j] < change_idx:
            j += 1
        if j < len(steps):
            delays.append((steps[j] - change_idx) * us_per_sample)
    return delays


def step_high_intervals(step_samples: Sequence[int]) -> List[Tuple[int, int]]:
    """Sample index ranges where the step pin is high, as (rise, fall)."""
    intervals: List[Tuple[int, int]] = []
    start: Optional[int] = None
    for idx, level in detect_edges(step_samples):
        if level:
            start = idx
        elif start is not None:
            intervals.append((start, idx))
            start = None
    if start is not None:
        intervals.append((start, len(step_samples) - 1))
    return intervals


def dir_changes_during_step_high(
    dir_samples: Sequence[int], step_samples: Sequence[int], sample_rate_hz: int
) -> List[dict]:
    """Every DIR change that happens while the STEP pin is high.

    A stepper driver latches the direction on the STEP edge, so a DIR
    transition inside the pulse window is not merely untidy: the driver can
    decode the new direction for that step, and the transition itself can
    glitch the DIR input while the coil is being driven. The library never
    intends to do it -- `Stepper_ToggleDirection()` and `Stepper_One()` are
    ordered so the direction settles first -- so any occurrence is a defect.

    This holds for every capture, not just the direction-change scenario, so it
    is applied as a global invariant rather than per test.

    A DIR change exactly on the rise or the fall sample is the boundary, not a
    violation: those are the two edges where a direction change belongs.
    """
    us_per_sample = 1_000_000.0 / sample_rate_hz
    intervals = step_high_intervals(step_samples)
    out: List[dict] = []
    for idx, level in detect_edges(dir_samples):
        for rise, fall in intervals:
            if rise < idx < fall:
                out.append({
                    "sample": idx,
                    "at_us": round(idx * us_per_sample, 4),
                    "into_high_us": round((idx - rise) * us_per_sample, 4),
                    "high_width_us": round((fall - rise) * us_per_sample, 4),
                    "dir_level": level,
                })
                break
    return out


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
