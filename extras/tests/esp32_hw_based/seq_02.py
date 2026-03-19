#!/usr/bin/env -S uv run --with pyserial python
"""seq_02: High-speed long move, verify final position @30000.

Steps:
  1. Attach pulse counter p7
  2. Move +30000 steps at 30000 steps/s, accel 100000
  3. Verify final position is exactly 30000
  4. Check pulse counter sync throughout the move
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

        s.send(f"{MOTOR} p7,-32767,32767 H30000 A100000 R30000")

        s.wait_for(rf">> {MOTOR}: @30000 \[30000\]")

        s.check_pcnt_sync()
    print("PASS")
except Exception as e:
    print(f"FAIL: {e}", file=sys.stderr)
    sys.exit(1)
