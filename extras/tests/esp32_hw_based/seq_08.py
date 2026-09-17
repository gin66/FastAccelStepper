#!/usr/bin/env -S uv run --with pyserial python
"""seq_08: Inverted pulse-counter attach, forward run, return to 0.

Runs on every motor/driver (MCPWM/PCNT, RMT, I2S). Command:

  A1000000 V100 pi7 f w1000 P0 W

Counts falling STEP edges. MCPWM must not toggle DIR while STEP is still
high (#370); the old-DIR MIN_CMD_TICKS pause covers that.
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

        s.send(f"{MOTOR} A1000000 V100 pi7 f w1000 P0 W")

        s.wait_for(rf">> {MOTOR}: @0 \[0\]")

        s.check_pcnt_sync()
    print("PASS")
except Exception as e:
    print(f"FAIL: {e}", file=sys.stderr)
    sys.exit(1)
