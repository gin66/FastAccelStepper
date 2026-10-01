#!/usr/bin/env python3
"""
control.py — send newline commands to the Saleae test app and read replies.

The firmware (common/saleae_app.cpp) speaks a tiny text protocol over the
serial console:

    SR00                      SR_00 connection self-test
    SR01 <steps> <speed_us>   constant-speed move; replies "DONE <pos>"
    POS                       replies "POS <position>"
    STOP                      stop move / self-test

Examples:
    python3 control.py --port /dev/cu.usbserial-0001 --send "STOP" --read 0.5
    python3 control.py --port /dev/cu.usbserial-0001 --send "SR01 400 400" --read 2
"""

import argparse
import sys
import time

import serial


def main():
    p = argparse.ArgumentParser(description="Saleae test app serial control.")
    p.add_argument("--port", default="/dev/cu.usbserial-0001")
    p.add_argument("--baud", type=int, default=115200)
    p.add_argument("--send", help="commands separated by ';', e.g. 'STOP;SR01 400 400'")
    p.add_argument("--read", type=float, default=1.0,
                   help="seconds to read replies after sending")
    p.add_argument("--settle", type=float, default=0.2,
                   help="pause after opening the port")
    args = p.parse_args()

    try:
        ser = serial.Serial(args.port, args.baud, timeout=0.1)
    except serial.SerialException as e:
        print(f"ERROR: cannot open {args.port}: {e}", file=sys.stderr)
        return 1

    with ser:
        time.sleep(args.settle)
        ser.reset_input_buffer()

        if args.send:
            for cmd in args.send.split(";"):
                cmd = cmd.strip()
                if not cmd:
                    continue
                ser.write((cmd + "\n").encode())
                time.sleep(0.05)

        deadline = time.time() + args.read
        out = b""
        while time.time() < deadline:
            data = ser.read(64)
            if data:
                out += data

    sys.stdout.write(out.decode(errors="replace"))
    return 0


if __name__ == "__main__":
    sys.exit(main())
