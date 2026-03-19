#!/usr/bin/env -S uv run --with pyserial python
"""seq_01c: Forward run, very short force stop, single step, verify @1.

Steps:
  1. Attach pulse counter p7
  2. Run forward at 25000 steps/s, accel 10000
  3. After only 10 ms, force-stop (motor barely moved)
  4. Clear counter, wait 20 ms, move +1 step
  5. Expected final position: 1
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

        s.send(f"{MOTOR} p7,-32767,32767 H25000 A10000 f w10 X W pc w 20 R1 W")

        s.wait_for(rf">> {MOTOR}: @1 \[1\]")
    print("PASS")
except Exception as e:
    print(f"FAIL: {e}", file=sys.stderr)
    sys.exit(1)
