#!/usr/bin/env -S uv run --with pyserial python
"""seq_01b: Forward run, force stop, two moves, verify final position @1100.

Identical to seq_01a -- provides a second run to check repeatability.
"""

import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from serial_session import SerialSession

MOTOR = os.environ.get("MOTOR", "M1")
DEVICE = sys.argv[1] if len(sys.argv) > 1 else "ttyUSB0"
if not DEVICE.startswith("/"):
    DEVICE = "/dev/" + DEVICE

LOG = os.path.splitext(os.path.abspath(__file__))[0] + ".log"

try:
    with SerialSession(DEVICE, motor=MOTOR, log_file=LOG) as s:
        s.wait_for_boot()

        s.send(
            f"{MOTOR} p7,-32767,32767 H25000 A10000 f w1000 X W pc R100 w100 W R1000 w1000 W"
        )

        s.wait_for(rf">> {MOTOR}: @1100 \[1100\]")
    print("PASS")
except Exception as e:
    print(f"FAIL: {e}", file=sys.stderr)
    sys.exit(1)
