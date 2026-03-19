#!/usr/bin/env -S uv run --with pyserial python
"""seq_05: Force stop during acceleration, then move 10 steps, verify @10.

Steps:
  1. Move +1000 steps at 25000 steps/s, accel 10000
  2. After only 200 ms (motor still accelerating), force-stop
  3. Wait for motor to fully stop
  4. Attach pulse counter p7
  5. Print status (?), move +10 steps
  6. Expected final position: 10
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

        s.send(f"{MOTOR} H25000 A10000 R1000 w200 X W p7,-32767,32767 ? R10 w1000")

        s.wait_for(rf">> {MOTOR}: @10 \[10\]")
    print("PASS")
except Exception as e:
    print(f"FAIL: {e}", file=sys.stderr)
    sys.exit(1)
