#!/usr/bin/env -S uv run --with pyserial python
"""seq_07a: Move +54 then -54, verify return to origin @0.

Steps:
  1. Attach pulse counter p7
  2. Move +54 steps at 40 us/step, accel 1000
  3. Move -54 steps (return to origin)
  4. Expected final position: 0
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

        s.send(f"{MOTOR} p7,-32767,32767 V40 A1000 R54 W R-54 W")

        s.wait_for(rf">> {MOTOR}: @0 \[0\]")
    print("PASS")
except Exception as e:
    print(f"FAIL: {e}", file=sys.stderr)
    sys.exit(1)
