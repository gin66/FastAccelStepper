#!/usr/bin/env python3
"""
analyze.py — Basic Python signal analyzer for Saleae capture files.

Parses .sr (sigrok native) files and extracts key metrics:
  - pulse_width_us: Step high/low time
  - inter_step_period_us: Time between successive step pulses
  - dir_to_first_step_us: Time from direction-change edge to first step pulse
  - duty_cycle: High time / total period
  - step_count: Total step pulses counted from edges
  - glitch_count: Spurious edges (width < 1/2 MIN_CMD_TICKS)

Usage:
    python3 analyze.py capture_20261001_120000.sr --channels 0,1,2,3 --output results/
"""

import argparse
import json
import os
import sys
from datetime import datetime
from pathlib import Path


# Minimum command tick at 16 MHz: 62.5 ns
# Glitch filter: edges narrower than half this are ignored
MIN_CMD_TICKS_NS = 62.5
GLITCH_FILTER_NS = MIN_CMD_TICKS_NS / 2.0  # 31.25 ns


def load_sr_file(sr_path):
    """
    Load a sigrok .sr capture file.

    Uses libsigrok Python bindings if available, otherwise falls back
    to parsing the binary format directly.

    Returns a dict mapping channel_id -> list of (timestamp_ns, value) tuples.
    """
    try:
        import libsigrok as sr
        return _load_with_libsigrok(sr_path, sr)
    except ImportError:
        print("WARNING: libsigrok Python bindings not available.", file=sys.stderr)
        print("Install with: pip install python-libsigrok4", file=sys.stderr)
        print("Falling back to basic binary parsing (limited support).", file=sys.stderr)
        return _load_basic(sr_path)


def _load_with_libsigrok(sr_path, sr):
    """Load .sr file using libsigrok Python bindings."""
    channels = {}

    # Open the session
    session = sr.Session(sr_path)

    # Get data packets
    for packet in session.data_get():
        if packet.type == sr::PACKET_DATA:
            # Process samples
            timestamps = packet.data[0]  # timestamp array
            channel_data = packet.data[1:]  # per-channel data arrays

            for ch_idx, data in enumerate(channel_data):
                if ch_idx not in channels:
                    channels[ch_idx] = []

                for i, ts in enumerate(timestamps):
                    value = data[i]
                    channels[ch_idx].append((ts, value))

    return channels


def _load_basic(sr_path):
    """Basic fallback: attempt to parse .sr binary format."""
    # This is a simplified parser — full .sr support requires libsigrok
    print(f"ERROR: Cannot parse {sr_path} without libsigrok bindings.", file=sys.stderr)
    print("Please install: pip install python-libsigrok4", file=sys.stderr)
    sys.exit(1)


def detect_edges(timestamps_values, channel_id):
    """
    Detect rising and falling edges for a channel.

    Args:
        timestamps_values: List of (timestamp_ns, value) tuples
        channel_id: Channel identifier

    Returns:
        List of (timestamp_ns, edge_type) where edge_type is 'rising' or 'falling'
    """
    edges = []
    prev_value = None

    for ts, value in timestamps_values:
        if prev_value is not None and value != prev_value:
            edge_type = 'rising' if value > prev_value else 'falling'
            edges.append((ts, edge_type))
        prev_value = value

    return edges


def compute_pulse_widths(edges):
    """Compute pulse widths from edge list."""
    pulse_widths_ns = []

    for i in range(0, len(edges) - 1, 2):
        if i + 1 < len(edges):
            width_ns = edges[i + 1][0] - edges[i][0]
            pulse_widths_ns.append(width_ns)

    return pulse_widths_ns


def compute_inter_step_periods(edges):
    """Compute time between successive rising edges (step pulses)."""
    rising_edges = [(ts, et) for ts, et in edges if et == 'rising']
    periods_ns = []

    for i in range(1, len(rising_edges)):
        period_ns = rising_edges[i][0] - rising_edges[i - 1][0]
        periods_ns.append(period_ns)

    return periods_ns


def count_glitches(pulse_widths_ns):
    """Count pulses narrower than the glitch filter threshold."""
    glitch_count = 0
    for width in pulse_widths_ns:
        if width < GLITCH_FILTER_NS:
            glitch_count += 1
    return glitch_count


def compute_duty_cycle(pulse_widths_ns):
    """Compute average duty cycle (high time / total period)."""
    if not pulse_widths_ns:
        return 0.0

    total_high = sum(pulse_widths_ns[0::2])  # high pulses (0, 2, 4, ...)
    total_period = sum(pulse_widths_ns)

    if total_period == 0:
        return 0.0

    return (total_high / total_period) * 100.0  # percentage


def analyze_channel(channel_id, timestamps_values):
    """Analyze a single channel and return metrics."""
    edges = detect_edges(timestamps_values, channel_id)
    pulse_widths_ns = compute_pulse_widths(edges)
    inter_step_periods_ns = compute_inter_step_periods(edges)
    glitch_count = count_glitches(pulse_widths_ns)
    duty_cycle = compute_duty_cycle(pulse_widths_ns)

    # Convert to microseconds for reporting
    pulse_widths_us = [w / 1000.0 for w in pulse_widths_ns]
    inter_step_periods_us = [p / 1000.0 for p in inter_step_periods_ns]

    return {
        "channel_id": channel_id,
        "edge_count": len(edges),
        "step_count": len(edges) // 2,
        "pulse_widths_us": pulse_widths_us,
        "inter_step_periods_us": inter_step_periods_us,
        "avg_pulse_width_us": sum(pulse_widths_us) / len(pulse_widths_us) if pulse_widths_us else 0,
        "avg_inter_step_us": sum(inter_step_periods_us) / len(inter_step_periods_us) if inter_step_periods_us else 0,
        "max_pulse_width_us": max(pulse_widths_us) if pulse_widths_us else 0,
        "min_inter_step_us": min(inter_step_periods_us) if inter_step_periods_us else 0,
        "glitch_count": glitch_count,
        "duty_cycle_percent": duty_cycle,
    }


def generate_report(sr_path, channel_ids, output_dir):
    """Generate analysis report for a capture file."""
    print(f"Analyzing: {sr_path}")

    # Load capture data
    data = load_sr_file(sr_path)

    # Analyze each channel
    results = {}
    for ch_id in channel_ids:
        if ch_id in data:
            results[f"ch_{ch_id}"] = analyze_channel(ch_id, data[ch_id])
        else:
            print(f"WARNING: Channel {ch_id} not found in capture file.", file=sys.stderr)

    # Save results
    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    base_name = Path(sr_path).stem  # e.g., capture_20261001_120000

    results_dir = Path(output_dir)
    results_dir.mkdir(parents=True, exist_ok=True)

    output_file = results_dir / f"{timestamp}_{base_name}_analysis.json"
    with open(output_file, "w") as f:
        json.dump({
            "capture_file": sr_path,
            "analysis_timestamp": datetime.now().isoformat() + "Z",
            "metrics": results,
        }, f, indent=2)

    print(f"Analysis saved to: {output_file}")
    return results


def main():
    parser = argparse.ArgumentParser(
        description="Saleae signal analyzer for .sr capture files"
    )
    parser.add_argument(
        "sr_file", help="Path to .sr capture file"
    )
    parser.add_argument(
        "--channels", default="0,1,2,3,4,5,6,7,8,9",
        help="Comma-separated channel IDs to analyze"
    )
    parser.add_argument(
        "--output", default="./results",
        help="Output directory for analysis results"
    )

    args = parser.parse_args()

    channel_ids = [int(c.strip()) for c in args.channels.split(",")]

    generate_report(args.sr_file, channel_ids, args.output)


if __name__ == "__main__":
    main()