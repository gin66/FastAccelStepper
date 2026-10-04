#!/usr/bin/env python3
"""
probe_mux_bits.py -- one-off ground truth for the I2S mux wire bit order.

Deliberately NOT part of the harness. The decoder needs to know which bclk bit
inside a frame carries which bit of the 32-bit _mux_state word, and that is a
property of the ESP32 I2S peripheral and its std-mode slot configuration, not
something the white paper can settle. So it is measured.

Four mux steppers and four different periods, one per stepper:

    stepper 0 -> slot 0, 400 ticks   (25.0 us)
    stepper 1 -> slot 1, 800 ticks   (50.0 us)
    stepper 2 -> slot 2, 1200 ticks  (75.0 us)
    stepper 3 -> slot 3, 1600 ticks  (100.0 us)

Distinct periods, so every high frame on the data line is attributable to one
stepper. A run with all four at one speed cannot answer the question: the pulses
land on top of each other and a word with several bits set does not say which bit
belongs to which stepper.

Sample rate. The bus runs at 8 MHz bclk, so the rate that matters is *samples
per bclk period*: 48 / 8 = 6 at 48 MS/s, 3 at 24 MS/s, 1.5 at 12 MS/s. Anything
below 32 MS/s aliases the bit clock and the bit values are not recoverable, which
is a stronger requirement than the white paper's ">= 8 MS/s" -- that number is
one sample per bclk period, the Nyquist limit. 48 MS/s is the analyzer's top
rate and is used here.

Timing. No trigger: this analyzer's hardware trigger is unreliable, and the
analyzer truncates an 8-channel 48 MS/s capture to ~7680 samples (160 us), so
the run has to still be going when sigrok-cli finishes enumerating the device.
One segment of 65025 steps at 400 ticks is 1.6 s of pulses, which covers it.

Run it with the bus wired to channels D5/D6/D7 (data/bclk/ws).
"""

import subprocess
import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))

import serial  # noqa: E402

PORT = "/dev/tty.usbserial-0001"
RATE = 48_000_000
MILLIS = 20
OUT = Path(__file__).resolve().parent.parent / "capture"

# slot index == position in this list; the period is what tells the pulses apart.
PERIODS = [400, 800, 1200, 1600]
# 65535 is the uint16_t ceiling on a segment's step count; at 400 ticks that is
# 1.64 s of pulses per stepper.
STEPS = 65535
DRIVERS = ["i2s_mux"] * len(PERIODS)


def serial_cmd(ser, cmd, wait=1.5):
    ser.reset_input_buffer()
    ser.write((cmd + "\n").encode())
    time.sleep(wait)
    return ser.read_all().decode("utf8", "replace").strip()


def main():
    OUT.mkdir(parents=True, exist_ok=True)
    sr = OUT / "probe_mux_bits.sr"
    vcd = OUT / "probe_mux_bits.vcd"

    ser = serial.Serial(PORT, 115200, timeout=0.2)
    time.sleep(3)
    print(serial_cmd(ser, "IMUX"))
    print(serial_cmd(ser, f"CONFIG {len(DRIVERS)} {','.join(DRIVERS)} nodir", 3))
    print(serial_cmd(ser, "MAP", 3))
    print(serial_cmd(ser, "QINFO", 2))
    print(serial_cmd(ser, "QCLR", 1.5))
    # The indexed QSEG gives each stepper its own program, which is the only way
    # to give four steppers four periods out of one queue layer.
    for idx, ticks in enumerate(PERIODS):
        print(serial_cmd(ser, f"QSEG {idx} {STEPS} {ticks} 1", 0.6))

    print(serial_cmd(ser, "QRUN 0xFFFFFFFF", 0.2))
    cap = subprocess.Popen(
        [
            "sigrok-cli", "-d", "fx2lafw", "-c", f"samplerate={RATE}",
            "-C", "D0,D1,D2,D3,D4,D5,D6,D7", "--time", str(MILLIS),
            "-O", "srzip", "-o", str(sr),
        ],
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
    )
    out, _ = cap.communicate(timeout=300)
    print(out[-300:])
    ser.close()

    subprocess.run(
        ["sigrok-cli", "-I", "srzip", "-O", "vcd", "-i", str(sr), "-o", str(vcd)],
        check=True,
    )
    print("wrote", vcd)


if __name__ == "__main__":
    main()