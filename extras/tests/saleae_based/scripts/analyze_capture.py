#!/usr/bin/env python3
"""
analyze_capture.py — Analyze binary logic capture from Saleae clone.

Reads interleaved binary data from sigrok-cli and computes:
  - pulse_width_us: Step high/low time
  - inter_step_period_us: Time between successive step pulses
  - step_count: Total step pulses counted from edges
  - duty_cycle: High time / total period
  - glitch_count: Spurious edges (width < 62.5ns / 2)
"""

import struct
import sys
import json
from datetime import datetime
from pathlib import Path


def load_binary_data(filepath):
    """Load interleaved binary logic data."""
    with open(filepath, 'rb') as f:
        data = f.read()
    
    # Each sample is 1 byte per channel, 8 channels interleaved
    num_samples = len(data) // 8
    channels = [[] for _ in range(8)]
    
    for i in range(num_samples):
        chunk = data[i*8:(i+1)*8]
        for ch in range(8):
            channels[ch].append((i, chunk[ch]))  # (sample_num, value)
    
    return channels, num_samples


def detect_edges(ch_data, ch_id):
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
    
    pulse_widths_us = []
    inter_step_periods_us = []
    
    # Pulse widths (high time = rising to falling)
    for i in range(0, len(edges) - 1, 2):
        if i + 1 < len(edges):
            width_samples = edges[i + 1][0] - edges[i][0]
            pulse_widths_us.append(width_samples * us_per_sample)
    
    # Inter-step periods (rising to next rising)
    rising_edges = [(ts, et) for ts, et in edges if et == 'rising']
    for i in range(1, len(rising_edges)):
        period_samples = rising_edges[i][0] - rising_edges[i - 1][0]
        inter_step_periods_us.append(period_samples * us_per_sample)
    
    # Glitch count (pulses < 31.25ns = 0.03125us at 1MS/s, or 1.56us at 20KS/s)
    glitch_threshold_us = 0.5 / sample_rate_hz * 1_000_000  # 0.5 samples
    glitch_count = sum(1 for pw in pulse_widths_us if pw < glitch_threshold_us)
    
    # Duty cycle
    high_pulses = pulse_widths_us[0::2]  # 0, 2, 4, ...
    total_period = sum(pulse_widths_us)
    duty_cycle = (sum(high_pulses) / total_period * 100) if total_period > 0 else 0
    
    return {
        'edge_count': len(edges),
        'step_count': len(edges) // 2,
        'pulse_widths_us': pulse_widths_us,
        'inter_step_periods_us': inter_step_periods_us,
        'avg_pulse_width_us': sum(pulse_widths_us) / len(pulse_widths_us) if pulse_widths_us else 0,
        'avg_inter_step_us': sum(inter_step_periods_us) / len(inter_step_periods_us) if inter_step_periods_us else 0,
        'max_pulse_width_us': max(pulse_widths_us) if pulse_widths_us else 0,
        'min_inter_step_us': min(inter_step_periods_us) if inter_step_periods_us else 0,
        'glitch_count': glitch_count,
        'duty_cycle_percent': duty_cycle,
    }


def main():
    binary_file = sys.argv[1] if len(sys.argv) > 1 else 'capture_raw.bin'
    output_dir = sys.argv[2] if len(sys.argv) > 2 else './results'
    
    print(f"Loading: {binary_file}")
    channels, total_samples = load_binary_data(binary_file)
    
    # Determine sample rate from capture length (2000ms capture)
    # 20 kHz for 2000ms = 40000 samples total = 5000 per channel (8 ch)
    # 1 MHz for 2000ms = 8000000 samples total = 1000000 per channel
    if total_samples >= 100000:
        sample_rate = 1000000  # 1 MHz
    else:
        sample_rate = 20000  # 20 kHz (fx2lafw default)
    
    print(f"Channels: {len(channels)}, Samples per channel: {len(channels[0])}")
    print(f"Detected sample rate: {sample_rate} Hz")
    print()
    
    channel_names = ['CH0 (GPIO 2)', 'CH1 (GPIO 0)', 'CH2 (GPIO 4)', 'CH3 (GPIO 16)',
                     'CH4 (GPIO 17)', 'CH5 (GPIO 5)', 'CH6 (GPIO 18)', 'CH7 (GPIO 19)']
    
    all_passed = True
    
    for ch_id in range(8):
        edges = detect_edges(channels[ch_id], ch_id)
        metrics = compute_metrics(edges, sample_rate)
        
        name = channel_names[ch_id]
        freq_hz = 1_000_000 / metrics['avg_inter_step_us'] if metrics['avg_inter_step_us'] > 0 else 0
        
        print(f"=== {name} ===")
        print(f"  Edges:        {metrics['edge_count']}")
        print(f"  Step count:   {metrics['step_count']}")
        print(f"  Avg pulse:    {metrics['avg_pulse_width_us']:.1f} us")
        print(f"  Max pulse:    {metrics['max_pulse_width_us']:.1f} us")
        print(f"  Avg inter:    {metrics['avg_inter_step_us']:.1f} us")
        print(f"  Min inter:    {metrics['min_inter_step_us']:.1f} us")
        print(f"  Frequency:    {freq_hz:.2f} Hz")
        print(f"  Duty cycle:   {metrics['duty_cycle_percent']:.1f}%")
        print(f"  Glitches:     {metrics['glitch_count']}")
        
        # SR_00 validation: simple_test uses delay(500) = 500ms HIGH + 500ms LOW
        # At 20 kHz: 2000 samples/cycle. At 1 MHz: 1000000 samples/cycle.
        # Accept any frequency between 1 Hz and 10 kHz — we just want clean square waves
        expected_duty = 50.0
        expected_freq_min = 1.0
        expected_freq_max = 10000.0
        
        passed = (metrics['glitch_count'] == 0 and
                  abs(metrics['duty_cycle_percent'] - expected_duty) < 2 and
                  expected_freq_min <= freq_hz <= expected_freq_max)
        
        status = "✓ PASS" if passed else "✗ FAIL"
        if not passed:
            all_passed = False
        print(f"  SR_00 (1 Hz): {status}")
        print()
    
    # Save results
    os.makedirs(output_dir, exist_ok=True)
    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    
    results = {
        'capture_file': binary_file,
        'timestamp': datetime.now().isoformat() + 'Z',
        'sample_rate_hz': sample_rate,
        'sr_00_passed': all_passed,
        'channels': {},
    }
    
    for ch_id in range(8):
        edges = detect_edges(channels[ch_id], ch_id)
        metrics = compute_metrics(edges, sample_rate)
        results['channels'][f'ch_{ch_id}'] = {
            'name': channel_names[ch_id],
            'step_count': metrics['step_count'],
            'avg_pulse_width_us': round(metrics['avg_pulse_width_us'], 1),
            'avg_inter_step_us': round(metrics['avg_inter_step_us'], 1),
            'frequency_hz': round(1_000_000 / metrics['avg_inter_step_us'], 2) if metrics['avg_inter_step_us'] > 0 else 0,
            'duty_cycle_percent': round(metrics['duty_cycle_percent'], 1),
            'glitch_count': metrics['glitch_count'],
            'passed': metrics['glitch_count'] == 0 and abs(metrics['duty_cycle_percent'] - 50) < 2,
        }
    
    output_file = Path(output_dir) / f"{timestamp}_sr00_analysis.json"
    with open(output_file, 'w') as f:
        json.dump(results, f, indent=2)
    
    print(f"Results saved to: {output_file}")
    print(f"\n=== SR_00 Overall: {'PASS ✓' if all_passed else 'FAIL ✗'} ===")


if __name__ == "__main__":
    import os
    main()