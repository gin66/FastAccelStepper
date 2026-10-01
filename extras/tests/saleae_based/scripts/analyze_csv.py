#!/usr/bin/env python3
"""
analyze_csv.py — SR_00 evaluation of a capture (.sr / .vcd / .csv).

SR_00 (connection verification) expects all 8 identification pins to toggle at
exactly 1 Hz, each with a distinct asymmetric duty (5 %..40 %). Since no channel
is 50 %, a mis-wired or inverted channel reads back as the complement duty and
is flagged. The transition count must match the commanded square wave exactly:
a spurious pulse is an extra edge and is a failure, never a tolerated count.

The signal reconstruction itself lives in signal_parser.py; this module only
applies the SR_00 expectations and writes the JSON result.
"""

import json
import os
import sys
from datetime import datetime
from pathlib import Path

import signal_parser as sp

# Channel -> GPIO mapping (white paper §3.3, ESP32-DevKitC).
GPIO_MAP = {
    "D0": "GPIO 2",
    "D1": "GPIO 0",
    "D2": "GPIO 4",
    "D3": "GPIO 16",
    "D4": "GPIO 17",
    "D5": "GPIO 5",
    "D6": "GPIO 18",
    "D7": "GPIO 19",
}

EXPECTED_FREQ_HZ = 1.0
FREQ_TOLERANCE_HZ = 0.05
DUTY_TOLERANCE_PCT = 1.5
PERIOD_MS = 1000

EXPECTED_HIGH_MS = {
    "D0": 50,
    "D1": 100,
    "D2": 150,
    "D3": 200,
    "D4": 250,
    "D5": 300,
    "D6": 350,
    "D7": 400,
}

# A measured width may differ from the commanded one by at most 2 ms (the
# firmware updates its pins on a 1 ms loop); anything else is an extra or a
# missing transition.
EXPECTED_WIDTH_TOL_US = 2000.0

EXPECTED_DUTY = {
    "D0": 5.0,
    "D1": 10.0,
    "D2": 15.0,
    "D3": 20.0,
    "D4": 25.0,
    "D5": 30.0,
    "D6": 35.0,
    "D7": 40.0,
}


def evaluate_sr00(channels, sample_rate_hz):
    """Return (all_passed, per_channel_results) for an SR_00 capture."""
    all_passed = True
    channel_results = {}

    for ch_name in sorted(channels.keys()):
        metrics = sp.channel_metrics(channels[ch_name], sample_rate_hz)
        exp_duty = EXPECTED_DUTY.get(ch_name)

        freq_ok = abs(metrics.frequency_hz - EXPECTED_FREQ_HZ) <= FREQ_TOLERANCE_HZ
        duty_ok = exp_duty is not None and \
            abs(metrics.duty_cycle_percent - exp_duty) <= DUTY_TOLERANCE_PCT
        inverted = exp_duty is not None and \
            abs(metrics.duty_cycle_percent - (100.0 - exp_duty)) <= DUTY_TOLERANCE_PCT
        # A spurious pulse shows up as an edge whose interval is neither the
        # commanded high time nor the commanded low time. Every measured width
        # must be one of exactly those two; there is no tolerance for an extra
        # or missing transition.
        exp_high = EXPECTED_HIGH_MS.get(ch_name)
        exp_low = (PERIOD_MS - exp_high) if exp_high is not None else None
        widths_ok = True
        bad_widths = []
        if exp_high is not None:
            # channel_metrics reports microseconds.
            hi_us, lo_us = exp_high * 1000.0, exp_low * 1000.0
            for w in metrics.high_widths_us:
                if abs(w - hi_us) > EXPECTED_WIDTH_TOL_US:
                    widths_ok = False
                    bad_widths.append(round(w, 1))
            for w in metrics.low_widths_us:
                if abs(w - lo_us) > EXPECTED_WIDTH_TOL_US:
                    widths_ok = False
                    bad_widths.append(round(w, 1))
        edges_ok = widths_ok
        passed = freq_ok and duty_ok and not inverted and widths_ok

        all_passed = all_passed and passed
        channel_results[ch_name] = {
            "gpio": GPIO_MAP.get(ch_name, ch_name),
            "frequency_hz": round(metrics.frequency_hz, 2),
            "duty_cycle_percent": round(metrics.duty_cycle_percent, 1),
            "expected_duty_percent": exp_duty,
            "inverted": inverted,
            "edge_count": metrics.edge_count,
            "expected_high_ms": exp_high,
            "expected_low_ms": exp_low,
            "unexpected_widths_ms": bad_widths[:8],
            "widths_ok": widths_ok,
            "passed": passed,
        }

    return all_passed, channel_results


def print_report(channels, sample_rate_hz, results):
    for ch_name in sorted(channels.keys()):
        r = results[ch_name]
        metrics = sp.channel_metrics(channels[ch_name], sample_rate_hz)
        print(f"=== {ch_name} ({r['gpio']}) ===")
        print(f"  Edges:        {metrics.edge_count}")
        print(f"  Edges:        {r['edge_count']} "
              f"(high {r['expected_high_ms']} ms / low {r['expected_low_ms']} ms)")
        if r["unexpected_widths_ms"]:
            print(f"  Unexpected:   {r['unexpected_widths_ms']} ms")
        print(f"  Step count:   {metrics.step_count}")
        print(f"  Frequency:    {r['frequency_hz']:.2f} Hz "
              f"(expected {EXPECTED_FREQ_HZ:.1f})")
        if r["expected_duty_percent"] is not None:
            print(f"  Duty cycle:   {r['duty_cycle_percent']:.1f}% "
                  f"(expected {r['expected_duty_percent']:.0f}%)")
        else:
            print(f"  Duty cycle:   {r['duty_cycle_percent']:.1f}%")
        if r["inverted"]:
            status = "✗ FAIL (inverted duty)"
        elif not r["widths_ok"]:
            status = "✗ FAIL (unexpected transition)"
        elif not r["passed"]:
            status = "✗ FAIL"
        else:
            status = "✓ PASS"
        print(f"  SR_00:        {status}")
        print()


def main():
    capture_file = sys.argv[1] if len(sys.argv) > 1 else "capture.sr"
    output_dir = sys.argv[2] if len(sys.argv) > 2 else "./results"

    print(f"Loading: {capture_file}")
    channels, sample_rate = sp.load_capture(capture_file)
    print(f"Channels: {sorted(channels.keys())}")
    print(f"Sample rate: {sample_rate} Hz")
    print(f"Samples per channel: {len(next(iter(channels.values())))}")
    print()

    all_passed, channel_results = evaluate_sr00(channels, sample_rate)
    print_report(channels, sample_rate, channel_results)

    os.makedirs(output_dir, exist_ok=True)
    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    results = {
        "capture_file": capture_file,
        "timestamp": datetime.now().isoformat() + "Z",
        "sample_rate_hz": sample_rate,
        "sr_00_passed": all_passed,
        "channels": channel_results,
    }
    output_file = Path(output_dir) / f"{timestamp}_sr00_analysis.json"
    with open(output_file, "w") as f:
        json.dump(results, f, indent=2)

    print(f"Results saved to: {output_file}")
    print(f"\n=== SR_00 Overall: {'PASS ✓' if all_passed else 'FAIL ✗'} ===")
    return 0 if all_passed else 1


if __name__ == "__main__":
    sys.exit(main())
