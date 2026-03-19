#!/usr/bin/env -S uv run --with pyserial python
"""seq_06: Low speed (40 us/step) move, verify final position @54.

Steps:
  1. Attach pulse counter p7
  2. Move +54 steps at 40 us/step (25000 steps/s), accel 1000000
  3. Wait for completion
  4. Expected final position: 54
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

        s.send(f"{MOTOR} p7,-32767,32767 V40 A1000000 R54 W")

        s.wait_for(rf">> {MOTOR}: @54 \[54\]")
    print("PASS")
except Exception as e:
    print(f"FAIL: {e}", file=sys.stderr)
    sys.exit(1)
