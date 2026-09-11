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
from datetime import datetime

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from serial_session import SerialSession

MOTOR = os.environ.get("MOTOR", "M1")
DUT = os.environ.get("DUT", "flashed")
CFG = os.environ.get("STEPPER_CONFIG", "0")
CFG_TAG = " config=1" if CFG == "1" else ""
DEVICE = sys.argv[1] if len(sys.argv) > 1 else "ttyUSB0"
if not DEVICE.startswith("/"):
    DEVICE = "/dev/" + DEVICE

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
TEST_ALL_LOG = os.path.join(SCRIPT_DIR, "test_all.log")
SEQUENCES = [13, 14, 1, 2, 3, 4, 6, 7, 10, 11]
PASS = "test passed"


def log_all(seq_name, result):
    line = (
        f"{datetime.now():%Y-%m-%d %H:%M:%S}  {DUT}  "
        f"{MOTOR} {seq_name}{CFG_TAG} {result}\n"
    )
    print(line, end="")
    with open(TEST_ALL_LOG, "a") as fh:
        fh.write(line)


current = None
try:
    with SerialSession(DEVICE, motor=MOTOR) as s:
        for seq in SEQUENCES:
            current = seq
            name = f"seq_03_{seq:02d}"
            log_file = os.path.join(SCRIPT_DIR, f"{name}.log")

            s.software_reset()
            s.clear_log(new_log_file=log_file)

            s.wait_for_boot()

            s.send(f"{MOTOR} p7,-32767,32767 t {MOTOR} {seq:02d} R")

            s.wait_for(PASS, timeout=300)

            s.check_pcnt_sync()

            log_all(name, "passed")
    print("PASS")
except Exception as e:
    if current is not None:
        log_all(f"seq_03_{current:02d}", "FAILED")
    print(f"FAIL: {e}", file=sys.stderr)
    sys.exit(1)
