#!/usr/bin/env python3
"""Read the PROBE line from the pico_stepper_count firmware over USB serial."""

import argparse
import re
import sys
import time

import serial


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--port", default="/dev/cu.usbmodem4101")
    parser.add_argument("--baud", type=int, default=115200)
    parser.add_argument("--seconds", type=float, default=6.0)
    parser.add_argument(
        "--expect",
        type=int,
        default=None,
        help="exit non-zero unless the probe reports this count",
    )
    args = parser.parse_args()

    deadline = time.time() + args.seconds
    last = None
    seen = None
    with serial.Serial(args.port, args.baud, timeout=0.5) as port:
        while time.time() < deadline:
            line = port.readline().decode("ascii", "replace").strip()
            if not line.startswith("PROBE"):
                continue
            if line != seen:
                print(line)
                seen = line
            last = line

    if last is None:
        print("no PROBE line received on " + args.port, file=sys.stderr)
        return 1
    match = re.search(r"connected=(\d+)", last)
    if match is None:
        print("unparsable PROBE line: " + last, file=sys.stderr)
        return 1
    count = int(match.group(1))
    if args.expect is not None and count != args.expect:
        print(
            "expected connected=%d, got %d" % (args.expect, count),
            file=sys.stderr,
        )
        return 2
    return 0


if __name__ == "__main__":
    sys.exit(main())
