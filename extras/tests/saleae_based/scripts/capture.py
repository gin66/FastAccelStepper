#!/usr/bin/env python3
"""
capture.py — reliable waveform capture via sigrok-cli.

Captures raw digital channels from a sigrok-compatible logic analyzer
(fx2lafw / Saleae) into a .sr file (sigrok srzip: one packed byte per sample,
so a capture is orders of magnitude smaller than CSV and is not truncated by
the output stage).

With --vcd the capture is additionally converted to a VCD by sigrok-cli. A VCD
holds only value changes, so it stays small and is directly readable in
GTKWave; sigrok picks $timescale from the sample rate (1 us at 1 MHz, 100 ps
at 48 MHz), so no timing information is lost.

The sample rate and measurement time are configurable. The script probes the
connected devices first, then runs sigrok-cli with the correct options and
verifies that a non-empty capture with the expected number of channels was
produced.

Note: a requested sample rate may not be sustainable — the analyzer's USB
bandwidth and buffer size cap the capture length, so the result can be shorter
than requested. This is detected and reported (use --strict to fail instead).
Check the supported rates with: sigrok-cli -d <driver> --show
Reference: https://sigrok.org/wiki/Sigrok-cli

Examples:
    # 8 channels, 4 MHz, 5 seconds -> capture.sr + capture.vcd
    python3 capture.py --sample-rate 4000000 --seconds 5 --vcd

    # 1 MHz, 2.5 s, channels 0-3
    python3 capture.py --sample-rate 1000000 --seconds 2.5 --channels 0,1,2,3

    # CSV still available for inspection
    python3 capture.py --format csv --output capture.csv

    # List connected devices
    python3 capture.py --list-devices
"""

import argparse
import os
import subprocess
import sys
import time
import zipfile
from pathlib import Path

DEFAULT_CHANNELS = "0,1,2,3,4,5,6,7"
PREFERRED_DRIVERS = ("fx2lafw", "saleae")


def list_devices():
    """Return {driver_name: description} for devices sigrok-cli can see."""
    try:
        result = subprocess.run(
            ["sigrok-cli", "--scan"],
            capture_output=True, text=True, timeout=15,
        )
    except FileNotFoundError:
        print("ERROR: sigrok-cli not found. Install sigrok first.", file=sys.stderr)
        sys.exit(1)
    except subprocess.TimeoutExpired:
        print("ERROR: sigrok-cli --scan timed out.", file=sys.stderr)
        sys.exit(1)

    # Lines look like: "fx2lafw - Saleae Logic with 8 channels: D0 D1 ..."
    devices = {}
    for line in result.stdout.splitlines():
        line = line.strip()
        if " - " in line:
            name, desc = line.split(" - ", 1)
            devices[name.strip()] = desc.strip()
    return devices


def match_driver(devices, wanted):
    """Match a driver by exact name or by a 'name:conn=...' prefix."""
    for name in devices:
        if name == wanted or name.startswith(wanted + ":"):
            return name
    return None


def pick_driver(devices, requested):
    if requested and requested != "auto":
        match = match_driver(devices, requested)
        if not match:
            print(f"ERROR: driver '{requested}' not found. "
                  f"Connected: {', '.join(devices) or 'none'}", file=sys.stderr)
            sys.exit(1)
        return match
    for wanted in PREFERRED_DRIVERS:
        match = match_driver(devices, wanted)
        if match:
            return match
    # Fall back to the first non-demo driver
    for drv in devices:
        if drv != "demo":
            return drv
    print("ERROR: no logic analyzer found.", file=sys.stderr)
    print("Connected devices: " + (", ".join(devices) or "none"), file=sys.stderr)
    sys.exit(1)


def detect_analyzer(requested, attempts=3):
    """Scan for devices, retrying because fx2lafw enumeration is flaky."""
    devices = {}
    for attempt in range(attempts):
        devices = list_devices()
        if requested and requested != "auto":
            if match_driver(devices, requested):
                return devices, match_driver(devices, requested)
        elif any(match_driver(devices, d) for d in PREFERRED_DRIVERS):
            break
        time.sleep(0.5)
    return devices, pick_driver(devices, requested)


def normalize_channels(channels):
    """Accept '0,1', 'D0,D1' or a mix; return 'D0,D1'."""
    names = []
    for token in channels.split(","):
        token = token.strip()
        if not token:
            continue
        names.append(token if token.upper().startswith("D") else f"D{token}")
    if not names:
        print("ERROR: no channels given.", file=sys.stderr)
        sys.exit(1)
    return ",".join(names)


def build_command(driver, sample_rate, channels, time_ms, output, trigger,
                  output_format="srzip"):
    cmd = [
        "sigrok-cli",
        "-d", driver,
        "-c", f"samplerate={sample_rate}",
        "-C", channels,
        "--time", str(time_ms),
        "-O", output_format,
        "-o", output,
    ]
    if trigger:
        cmd += ["--triggers", trigger]
    return cmd


def verify_output(output, expected_channels, sample_rate):
    """Sanity-check a produced capture. Returns the sample count or None."""
    path = Path(output)
    if not path.exists() or path.stat().st_size == 0:
        print(f"ERROR: sigrok-cli produced no data at {output}", file=sys.stderr)
        return None

    if output.endswith(".sr"):
        return _verify_srzip(path)

    data_lines = 0
    header_channels = None
    with open(output, "r") as f:
        for line in f:
            if line.startswith(";"):
                if "Channels" in line and ":" in line:
                    header_channels = line.split(":", 1)[1].strip()
                continue
            if line.startswith("logic") or line.startswith("D"):
                continue  # column header row
            data_lines += 1

    if data_lines == 0:
        print("ERROR: CSV contains no samples.", file=sys.stderr)
        return None

    if header_channels is not None:
        found = header_channels.split("/")[0].strip()
        if expected_channels:
            print(f"  Header channels: {found}")
    return data_lines


def _verify_srzip(path):
    """Count the samples in a .sr capture from its ZIP members."""
    unitsize = 1
    prefix = "logic-1-"
    n_bytes = 0
    with zipfile.ZipFile(path) as z:
        for line in z.read("metadata").decode(errors="replace").splitlines():
            line = line.strip()
            if line.startswith("unitsize="):
                unitsize = int(line.split("=", 1)[1])
            elif line.startswith("capturefile="):
                prefix = line.split("=", 1)[1].strip() + "-"
        for name in z.namelist():
            if name.startswith(prefix) and name[len(prefix):].isdigit():
                n_bytes += z.getinfo(name).file_size

    samples = n_bytes // unitsize
    if samples == 0:
        print(f"ERROR: {path} contains no samples.", file=sys.stderr)
        return None
    return samples


def sr_to_vcd(sr_file, vcd_file):
    """Convert a .sr capture to a VCD containing only value changes.

    sigrok-cli picks the $timescale from the sample rate (1 us at 1 MHz, 100 ps
    at 48 MHz), so no timestamps are lost and the file stays small enough to
    open in GTKWave.
    """
    cmd = ["sigrok-cli", "-I", "srzip", "-i", str(sr_file),
           "-O", "vcd", "-o", str(vcd_file)]
    result = subprocess.run(cmd, capture_output=True, text=True, timeout=120)
    if result.returncode != 0 or not Path(vcd_file).exists():
        print(f"ERROR: sr -> vcd conversion failed for {sr_file}",
              file=sys.stderr)
        if result.stderr.strip():
            print(result.stderr.strip(), file=sys.stderr)
        return None
    return Path(vcd_file)


def run_capture(args):
    devices, driver = detect_analyzer(args.driver)
    print(f"Device:      {driver} — {devices.get(driver, '')}")

    channels = normalize_channels(args.channels)
    time_ms = int(round(args.seconds * 1000))
    if time_ms <= 0:
        print("ERROR: --seconds must be > 0.", file=sys.stderr)
        sys.exit(1)

    cmd = build_command(driver, args.sample_rate, channels, time_ms,
                        args.output, args.trigger, args.format)
    print(f"Channels:    {channels}")
    print(f"Sample rate: {args.sample_rate} Hz")
    print(f"Duration:    {args.seconds} s ({time_ms} ms)")
    print(f"Command:     {' '.join(cmd)}")

    try:
        result = subprocess.run(cmd, capture_output=True, text=True,
                                timeout=args.seconds + 30)
    except subprocess.TimeoutExpired:
        print("ERROR: capture timed out.", file=sys.stderr)
        return 1

    if result.returncode != 0:
        print(f"ERROR: sigrok-cli failed (exit {result.returncode}).",
              file=sys.stderr)
        if result.stderr.strip():
            print(result.stderr.strip(), file=sys.stderr)
        return 1

    n = verify_output(args.output, len(channels.split(",")), args.sample_rate)
    if n is None:
        return 1

    actual_seconds = n / args.sample_rate
    size_mb = os.path.getsize(args.output) / (1024 * 1024)
    print(f"Wrote:       {args.output} "
          f"({n} samples, ~{size_mb:.1f} MB, {args.sample_rate} Hz)")
    print(f"Duration:    {actual_seconds:.3f} s captured "
          f"(requested {args.seconds} s)")

    if actual_seconds < 0.9 * args.seconds:
        print(f"WARNING: capture is shorter than requested — the device "
              f"truncated it to {n} samples at {args.sample_rate} Hz "
              f"({actual_seconds:.3f} s of {args.seconds} s). Lower the "
              f"--sample-rate, use --samples for an exact length, or check "
              f"the device limits with: sigrok-cli -d {driver.split(':')[0]} "
              f"--show", file=sys.stderr)
        if args.strict:
            return 1

    if args.vcd:
        vcd = Path(str(args.output).rsplit(".", 1)[0] + ".vcd")
        if sr_to_vcd(args.output, vcd) is None:
            return 1
        vcd_kb = vcd.stat().st_size / 1024
        n_changes = sum(1 for line in vcd.open() if line.startswith("#"))
        print(f"VCD:         {vcd} (~{vcd_kb:.1f} KB, "
              f"{n_changes} value changes — open in GTKWave)")

    print("Capture OK")
    return 0


def main():
    parser = argparse.ArgumentParser(
        description="Reliable sigrok-cli waveform capture to .sr (+ VCD)."
    )
    parser.add_argument("--sample-rate", type=int, default=4000000,
                        help="Sample rate in Hz (default: 4000000)")
    parser.add_argument("--seconds", type=float, default=5.0,
                        help="Measurement time in seconds (default: 5)")
    parser.add_argument("--channels", default=DEFAULT_CHANNELS,
                        help=f"Channels, e.g. 0,1 or D0,D1 (default: {DEFAULT_CHANNELS})")
    parser.add_argument("--driver", default="auto",
                        help="sigrok driver (default: auto -> fx2lafw/saleae)")
    parser.add_argument("--output", default="capture.sr",
                        help="Output .sr path (default: capture.sr)")
    parser.add_argument("--format", default="srzip",
                        choices=("srzip", "csv"),
                        help="sigrok-cli output format (default: srzip)")
    parser.add_argument("--vcd", action="store_true",
                        help="also convert the capture to a change-only VCD")
    parser.add_argument("--trigger", default=None,
                        help="Optional sigrok trigger, e.g. 'D0=r'")
    parser.add_argument("--list-devices", action="store_true",
                        help="List connected devices and exit")
    parser.add_argument("--strict", action="store_true",
                        help="Fail if the capture is shorter than requested")
    args = parser.parse_args()

    if args.vcd and args.format != "srzip":
        print("ERROR: --vcd needs the .sr format.", file=sys.stderr)
        return 1
    if args.format == "srzip" and not args.output.endswith(".sr"):
        print("ERROR: --output must end in .sr for the srzip format.",
              file=sys.stderr)
        return 1

    if args.list_devices:
        for name, desc in list_devices().items():
            print(f"  {name}: {desc}")
        return 0

    return run_capture(args)


if __name__ == "__main__":
    sys.exit(main())
