#!/usr/bin/env -S uv run --with pyserial python
"""seq_03: Run built-in test sequences with pulse counter verification.

Loops over several StepperDemo test sequences (t-mode), each using
the pulse counter to verify position accuracy throughout the test.

Each iteration:
  1. Software reset the ESP32 (keeps serial open)
  2. Wait for boot and config selection
  3. Attach pulse counter p7, enter test mode, run sequence
  4. Wait for "test passed"
  5. Check pulse counter sync in the log
"""

import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from serial_session import SerialSession

MOTOR = os.environ.get("MOTOR", "M1")
DEVICE = sys.argv[1] if len(sys.argv) > 1 else "ttyUSB0"
if not DEVICE.startswith("/"):
    DEVICE = "/dev/" + DEVICE

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
SEQUENCES = [13, 1, 2, 3, 4, 6, 7, 10, 11]
PASS = "test passed"

try:
    with SerialSession(DEVICE, motor=MOTOR) as s:
        for seq in SEQUENCES:
            log_file = os.path.join(SCRIPT_DIR, f"seq_03_{seq:02d}.log")

            s.software_reset()
            s.clear_log(new_log_file=log_file)

            s.wait_for_boot()

            s.send(f"{MOTOR} p7,-32767,32767 t {MOTOR} {seq:02d} R")

            s.wait_for(PASS, timeout=300)

            s.check_pcnt_sync()

            print(f"  sequence {seq:02d}: PASS")
    print("PASS")
except Exception as e:
    print(f"FAIL: {e}", file=sys.stderr)
    sys.exit(1)
