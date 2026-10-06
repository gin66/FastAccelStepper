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

import json
import re
import zipfile
from dataclasses import dataclass, field
from pathlib import Path
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
        # the capture may not contain the shortest interval. A capture whose
        # channels never change carries no interval at all, and the only honest
        # answer there is one tick per sample rather than a division by zero.
        gaps = [ch[1][0] - ch[0][0]
                for ch in changes.values()
                if len(ch) > 1 and ch[1][0] > ch[0][0]]
        sample_ns = min(gaps) if gaps else timescale_ns
        ticks_per_sample = max(1, int(round(sample_ns / timescale_ns)))
        sample_rate_hz = int(round(1e9 / (timescale_ns * ticks_per_sample)))

    # A VCD records value changes only, so a line that stays flat after the last
    # edge contributes no further entries and the file's last timestamp is not
    # the end of the capture. capture.py writes the true sample count to a .meta
    # sidecar; honouring it is what lets a test distinguish "the run stopped"
    # from "the recording ran out" -- the two look identical in a bare VCD.
    # SR_25 hit exactly this: the clone's 64 MSample buffer ended 2.5 s after the
    # last step, and without the extent every capture looked like it stopped the
    # instant its final pulse landed.
    declared = read_capture_meta(filepath)
    if declared:
        # The sidecar counts samples; n_samples is in VCD ticks. Converting here
        # rather than later keeps the one place that knows about ticks_per_sample
        # being the one that has to get it right.
        n_samples = max(n_samples,
                        (int(declared["samples"]) - 1) * ticks_per_sample)
    total = n_samples // ticks_per_sample + 1

    channels: Dict[str, Sequence[int]] = {}
    for name in names.values():
        # bytearray, not list: a full-length 24 MS/s capture is 96M samples, and
        # a Python list of ints would cost ~770 MB per channel against 96 MB
        # here. Indexing, slicing and len() all behave the same for the
        # analysers, which only ever read.
        series = bytearray()
        level = 0
        for time, value in changes.get(name, []):
            series.extend(bytes([level]) * (time // ticks_per_sample
                                            - len(series)))
            series.append(value)
            level = value
        series.extend(bytes([level]) * (total - len(series)))
        channels[name] = series

    return channels, sample_rate_hz


def read_capture_meta(vcd_path) -> Optional[Dict[str, int]]:
    """The true sample count capture.py recorded beside a VCD, if present."""
    sidecar = Path(str(vcd_path).rsplit(".", 1)[0] + ".meta")
    try:
        data = json.loads(sidecar.read_text())
    except (OSError, ValueError):
        return None
    if isinstance(data, dict) and "samples" in data:
        return data
    return None


def describe(values: Sequence[float], digits: int = 4) -> Dict[str, Optional[float]]:
    """Summarise a distribution of timings.

    A single average hides the thing a characterization run is looking for. If a
    driver holds a fixed pulse width, min == max and that is the finding; if it
    jitters, the spread is the finding; if one pulse in ten thousand is short,
    only the minimum says so. Averaging first would report all three cases as
    the same number, which is why the report carries min/max/spread rather than
    a mean and a pass mark.

    `median` is reported alongside the mean because a long tail pulls the mean
    without moving the typical value, and for inter-step periods that difference
    is usually the defect itself.
    """
    if not values:
        return {"n": 0, "min": None, "max": None, "mean": None,
                "median": None, "spread": None, "stdev": None}
    ordered = sorted(values)
    n = len(ordered)
    mean = sum(ordered) / n
    mid = n // 2
    median = (ordered[mid] if n % 2
              else (ordered[mid - 1] + ordered[mid]) / 2.0)
    variance = sum((v - mean) ** 2 for v in ordered) / n
    return {
        "n": n,
        "min": round(ordered[0], digits),
        "max": round(ordered[-1], digits),
        "mean": round(mean, digits),
        "median": round(median, digits),
        "spread": round(ordered[-1] - ordered[0], digits),
        "stdev": round(variance ** 0.5, digits),
    }


def stepper_metrics(samples: Sequence[int], sample_rate_hz: int) -> Dict:
    """Per-stepper measurements for one channel, in report-ready form.

    Reports the *distribution* of each timing rather than one number: see
    `describe` for why. No glitch count and no pass mark on pulse width -- the
    driver sets the width, its value is a property of the silicon rather than
    something the library promises, and a threshold invented here would be a
    number with no source behind it.
    """
    m = channel_metrics(samples, sample_rate_hz)
    return {
        "edge_count": m.edge_count,
        "step_count": m.step_count,
        "pulse_high_us": describe(m.high_widths_us),
        "pulse_low_us": describe(m.low_widths_us),
        "inter_step_us": describe(m.inter_step_us),
        "duty_cycle_percent": round(m.duty_cycle_percent, 4),
        "max_pulse_width_us": round(m.max_pulse_width_us, 4),
    }


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
class EdgeMetrics:
    """What a list of step edges says, without the waveform around them.

    Separate from `ChannelMetrics` because the two answer different questions.
    Channel metrics are properties of the *pin* over the whole capture -- pulse
    width, duty, the level it idles at -- and a capture's idle stretch before
    the move is part of that. The step count and the inter-step periods are
    properties of the *move*, and the move is a bounded stretch of the capture,
    so a caller measuring a commanded program restricts the edges first and asks
    for these (`edge_metrics`) rather than the whole capture's metrics.
    """
    step_count: int = 0
    inter_step_us: List[float] = field(default_factory=list)


def edge_metrics(rising: Sequence[int], sample_rate_hz: int) -> EdgeMetrics:
    """Step count and inter-step periods of an explicit list of step edges."""
    m = EdgeMetrics(step_count=len(rising))
    us_per_sample = 1_000_000.0 / sample_rate_hz
    m.inter_step_us = [(rising[i] - rising[i - 1]) * us_per_sample
                       for i in range(1, len(rising))]
    return m


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


def channel_metrics(samples: Sequence[int], sample_rate_hz: int,
                    rising: Optional[Sequence[int]] = None) -> ChannelMetrics:
    """Compute the per-channel timing metrics.

    `rising` supplies the step edges for a caller that already has them -- a
    multiplexed capture holds 12 million samples per channel, so a second full
    pass to rediscover the edges is not free.
    """
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

    risings = rising_edges(samples) if rising is None else list(rising)
    em = edge_metrics(risings, sample_rate_hz)
    m.step_count = em.step_count
    m.inter_step_us = em.inter_step_us

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


def grid_period_defects(periods_us, expect_ticks, ticks_per_s, grid_ticks):
    """Periods against a command whose step instants are quantised to a frame.

    A driver that can only emit a step on a frame boundary does not emit one
    period, it emits a *set* of them. The ESP32 I2S multiplexer is the case: a
    step pulse is one frame (64 ticks = 4 us) high and the pulse is placed in the
    frame containing its instant, so a 400-tick command is not a steady 25 us --
    it is 6, 6, 6, then 7 frames, i.e. 24, 24, 24, 28 us, averaging 25 exactly.
    Judged against a +-5% band around 25 us, one period in four reads as "long"
    and the run fails while measuring precisely what it exists to describe.

    The legal set is derived from the grid and the command rather than
    hand-written: the frame holding a step is `floor(t / grid)`, so consecutive
    steps are `floor((t + ticks) / grid) - floor(t / grid)` frames apart, which
    can only be `q` or `q + 1` frames for `q = ticks // grid`, and only those two
    when `ticks` is not a whole number of frames. That is a check, not a
    rationalisation: a period on any other frame count fails, and so does one
    that is not on a frame boundary at all.

    The mean is still held to the commanded period, to a tolerance of one frame
    divided by the number of periods measured plus the sample quantisation. The
    grid does not move the mean -- the extra frame is paid every fourth step, not
    on every step -- so any drift is the driver losing or adding time, and it
    accumulates: over 63 periods a single steady extra frame is already 1.6 %.
    """
    frame_us = grid_ticks * 1e6 / ticks_per_s
    expect_us = expect_ticks * 1e6 / ticks_per_s
    q = expect_ticks // grid_ticks
    legal_frames = [q] if expect_ticks % grid_ticks == 0 else [q, q + 1]
    # A quarter frame: enough for the capture's own sample quantisation, far too
    # little to admit a neighbouring frame count (which is 4 us away).
    slack = frame_us * 0.25

    off_grid = [p for p in periods_us
                if min(abs(p - f * frame_us) for f in legal_frames) > slack]
    mean = sum(periods_us) / len(periods_us) if periods_us else None
    # How far the mean may sit from the commanded period, and why that is not a
    # flat tolerance: the grid pays its extra frame every so often, so over N
    # periods the accumulated difference is at most one frame divided by N. A
    # driver running steadily fast accumulates instead, and by 63 periods one
    # extra frame is already 1.6 % -- visible where a fixed band is not. The
    # quarter-frame term is the capture's own sample quantisation.
    mean_slack = frame_us / max(1, len(periods_us)) + slack
    mean_off = (mean is not None and abs(mean - expect_us) > mean_slack)
    return {
        "expected_period_us": expect_us,
        "grid_us": frame_us,
        "legal_periods_us": [f * frame_us for f in legal_frames],
        "periods_measured": len(periods_us),
        "off_grid_periods_us": off_grid[:16],
        "n_off_grid": len(off_grid),
        "mean_period_us": mean,
        "mean_tolerance_us": mean_slack,
        "ok": not off_grid and not mean_off,
    }


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
    dir_samples: Sequence[int], step_samples: Sequence[int], sample_rate_hz: int,
    steps: Optional[Sequence[int]] = None,
) -> List[float]:
    """For every DIR change, time until the next step pulse (us).

    A step that lands in the *same* sample as the DIR change yields 0.0, which
    is the honest reading: the separation is smaller than one sample, not
    absent. Skipping such a step and matching the next one instead would report
    a full step period and hide the Pico case entirely, where the PIO sets DIR
    and STEP from adjacent instructions.

    `steps` overrides the step edges, for a caller that has already restricted
    them to the commanded move: matching a DIR change against a step that
    happened before the move would report a delay to a pulse that has nothing
    to do with the direction change.
    """
    us_per_sample = 1_000_000.0 / sample_rate_hz
    dir_changes = detect_edges(dir_samples)
    steps = rising_edges(step_samples) if steps is None else list(steps)

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
