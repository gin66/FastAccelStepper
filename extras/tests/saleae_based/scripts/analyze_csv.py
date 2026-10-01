#!/usr/bin/env python3
"""
analyze_csv.py — Analyze CSV capture from sigrok-cli directly.

Reads CSV output from sigrok-cli and computes:
  - pulse_width_us: Step high/low time
  - inter_step_period_us: Time between successive step pulses
  - frequency_hz: Signal frequency
  - duty_cycle: High time / total period
  - glitch_count: Spurious edges (width < 0.5 samples)
"""

import sys
import csv
import json
from datetime import datetime
from pathlib import Path


def load_csv(filepath):
    """Load CSV capture from sigrok-cli."""
    channels = {}
    
    # Read all lines once
    with open(filepath, 'r') as f:
        lines = f.readlines()
    
    # Parse header and sample rate
    header_line = None
    sample_rate = 20000  # default
    for line in lines:
        line = line.strip()
        if line.startswith(';'):
            if line.startswith('; Samplerate:'):
                rate_str = line.split(':')[1].strip()
                if 'MHz' in rate_str:
                    sample_rate = int(float(rate_str.replace('MHz', '')) * 1_000_000)
                elif 'kHz' in rate_str:
                    sample_rate = int(float(rate_str.replace('kHz', '')) * 1000)
            continue
        header_line = line
        break
    
    if not header_line:
        print("ERROR: No data found in CSV")
        return None
    
    channel_names = header_line.split(',')
    print(f"Channels: {channel_names}")
    print(f"Sample rate: {sample_rate} Hz")
    print()
    
    # Parse data rows
    num_channels = len(channel_names)
    indexed_names = [f'D{i}' for i in range(num_channels)]
    for i in range(num_channels):
        channels[indexed_names[i]] = []
    
    reader = csv.reader(lines)
    for row_num, row in enumerate(reader):
        if not row or len(row) != num_channels:
            continue
        for ch_idx in range(num_channels):
            try:
                value = int(row[ch_idx])
                channels[indexed_names[ch_idx]].append((row_num, value))
            except ValueError:
                pass
    
    return channels, sample_rate


def detect_edges(ch_data, ch_name):
    """Detect rising and falling edges for a channel."""
    edges = []
    prev_value = None
    
    for sample_num, value in ch_data:
        if prev_value is not None and value != prev_value:
            edge_type = 'rising' if value > 0 else 'falling'
            edges.append((sample_num, edge_type))
        prev_value = value
    
    return edges


def compute_metrics(edges, sample_rate_hz):
    """Compute metrics from edge list."""
    us_per_sample = 1_000_000 / sample_rate_hz

    # Classify each inter-edge interval by the level it represents:
    # a rising edge starts a HIGH interval, a falling edge starts a LOW one.
    high_us = []
    low_us = []
    for i in range(len(edges) - 1):
        width_us = (edges[i + 1][0] - edges[i][0]) * us_per_sample
        if edges[i][1] == 'rising':
            high_us.append(width_us)
        else:
            low_us.append(width_us)

    pulse_widths_us = high_us + low_us

    # Inter-step periods (rising to next rising)
    inter_step_periods_us = []
    rising_edges = [(ts, et) for ts, et in edges if et == 'rising']
    for i in range(1, len(rising_edges)):
        period_samples = rising_edges[i][0] - rising_edges[i - 1][0]
        inter_step_periods_us.append(period_samples * us_per_sample)

    # Glitch count (pulses < 1 sample)
    glitch_count = sum(1 for pw in pulse_widths_us if pw < 1.0 * us_per_sample)

    # Duty cycle: average high time / (average high + average low time).
    # Use averages, not sums: a capture window rarely holds the same number
    # of high and low intervals, and summing would bias the duty.
    total_high = sum(high_us)
    total_low = sum(low_us)
    avg_high = total_high / len(high_us) if high_us else 0
    avg_low = total_low / len(low_us) if low_us else 0
    total_time = avg_high + avg_low
    duty_cycle = (avg_high / total_time * 100) if total_time > 0 else 0

    # Frequency
    avg_inter_step_us = sum(inter_step_periods_us) / len(inter_step_periods_us) if inter_step_periods_us else 0
    frequency_hz = 1_000_000 / avg_inter_step_us if avg_inter_step_us > 0 else 0

    return {
        'edge_count': len(edges),
        'step_count': len(edges) // 2,
        'pulse_widths_us': pulse_widths_us,
        'high_widths_us': high_us,
        'low_widths_us': low_us,
        'inter_step_periods_us': inter_step_periods_us,
        'avg_high_us': total_high / len(high_us) if high_us else 0,
        'avg_low_us': total_low / len(low_us) if low_us else 0,
        'avg_pulse_width_us': sum(pulse_widths_us) / len(pulse_widths_us) if pulse_widths_us else 0,
        'avg_inter_step_us': avg_inter_step_us,
        'frequency_hz': frequency_hz,
        'duty_cycle_percent': duty_cycle,
        'glitch_count': glitch_count,
    }


def main():
    csv_file = sys.argv[1] if len(sys.argv) > 1 else 'capture.csv'
    output_dir = sys.argv[2] if len(sys.argv) > 2 else './results'
    
    print(f"Loading: {csv_file}")
    result = load_csv(csv_file)
    
    if not result:
        print("ERROR: No channels loaded")
        return
    
    channels, sample_rate = result
    
    if not channels:
        print("ERROR: No channels loaded")
        return
    
    num_channels = len(channels)
    print(f"Samples per channel: {len(list(channels.values())[0])}")
    print()
    
    # Map channel names to GPIO pins (white paper Section 3.3)
    gpio_map = {
        'D0': 'GPIO 2',
        'D1': 'GPIO 0',
        'D2': 'GPIO 4',
        'D3': 'GPIO 16',
        'D4': 'GPIO 17',
        'D5': 'GPIO 5',
        'D6': 'GPIO 18',
        'D7': 'GPIO 19',
    }

    # SR_00 simple_test: all channels 1 Hz with a distinct, asymmetric duty.
    # No channel is 50 %, so an inverted channel reads back as the complement
    # duty and can be flagged explicitly.
    expected_freq_hz = 1.0
    freq_tolerance_hz = 0.05
    duty_tolerance_pct = 1.5
    expected_duty = {
        'D0': 5.0,
        'D1': 10.0,
        'D2': 15.0,
        'D3': 20.0,
        'D4': 25.0,
        'D5': 30.0,
        'D6': 35.0,
        'D7': 40.0,
    }

    all_passed = True
    channel_results = {}

    for ch_name in sorted(channels.keys()):
        edges = detect_edges(channels[ch_name], ch_name)
        metrics = compute_metrics(edges, sample_rate)

        gpio = gpio_map.get(ch_name, ch_name)
        freq = metrics['frequency_hz']
        duty = metrics['duty_cycle_percent']
        exp_duty = expected_duty.get(ch_name)

        freq_ok = abs(freq - expected_freq_hz) <= freq_tolerance_hz
        duty_ok = exp_duty is not None and abs(duty - exp_duty) <= duty_tolerance_pct
        inverted = exp_duty is not None and \
            abs(duty - (100.0 - exp_duty)) <= duty_tolerance_pct
        passed = (metrics['glitch_count'] == 0 and freq_ok and
                  duty_ok and not inverted)

        print(f"=== {ch_name} ({gpio}) ===")
        print(f"  Edges:        {metrics['edge_count']}")
        print(f"  Step count:   {metrics['step_count']}")
        print(f"  Frequency:    {freq:.2f} Hz (expected {expected_freq_hz:.1f})")
        if exp_duty is not None:
            print(f"  Duty cycle:   {duty:.1f}% (expected {exp_duty:.0f}%)")
        else:
            print(f"  Duty cycle:   {duty:.1f}%")
        print(f"  Glitches:     {metrics['glitch_count']}")

        if not passed:
            all_passed = False
        if inverted:
            status = "✗ FAIL (inverted duty)"
        elif not freq_ok:
            status = "✗ FAIL (frequency)"
        elif not duty_ok:
            status = "✗ FAIL (duty)"
        elif metrics['glitch_count'] != 0:
            status = "✗ FAIL (glitches)"
        else:
            status = "✓ PASS"
        print(f"  SR_00:        {status}")
        print()

        channel_results[ch_name] = {
            'gpio': gpio,
            'frequency_hz': round(freq, 2),
            'duty_cycle_percent': round(duty, 1),
            'expected_duty_percent': exp_duty,
            'inverted': inverted,
            'glitch_count': metrics['glitch_count'],
            'passed': passed,
        }

    # Save results
    os.makedirs(output_dir, exist_ok=True)
    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")

    results = {
        'capture_file': csv_file,
        'timestamp': datetime.now().isoformat() + 'Z',
        'sample_rate_hz': sample_rate,
        'sr_00_passed': all_passed,
        'channels': channel_results,
    }
    
    output_file = Path(output_dir) / f"{timestamp}_sr00_analysis.json"
    with open(output_file, 'w') as f:
        json.dump(results, f, indent=2)
    
    print(f"Results saved to: {output_file}")
    print(f"\n=== SR_00 Overall: {'PASS ✓' if all_passed else 'FAIL ✗'} ===")


if __name__ == "__main__":
    import os
    main()
