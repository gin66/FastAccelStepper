"""Shared serial session helper for ESP32 StepperDemo hardware tests.

The ESP32 resets automatically when the serial port is opened due to
DTR/RTS toggling by most USB-serial adapters (CH340, CP2102, etc.).
Opening the port twice (as grabserial does) causes a double reset,
leaving the ESP32 in an unpredictable state.

This module keeps a single serial connection open for the entire test,
avoiding that problem entirely.

Usage in test scripts::

    sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
    from serial_session import SerialSession

    with SerialSession("/dev/ttyUSB0", motor="M1", log_file="test.log") as s:
        s.wait_for_boot()
        s.send("H25000 A10000 R1000 W")
        s.wait_for(r">> M1: @1000 \\[1000\\]")
"""

import os
import re
import sys
import time

import serial


class SerialSession:
    """Manages a single serial connection to the ESP32 StepperDemo.

    Opens the serial port once.  If the ESP32 resets on open (the
    common case), ``wait_for_boot()`` handles the boot sequence.
    If it does *not* reset, ``wait_for_boot()`` falls back to a
    software reset via the ``reset`` command.

    Args:
        device:
            Serial port path, e.g. ``"/dev/ttyUSB0"`` or
            ``"/dev/cu.usbserial-0001"``.
        baud:
            Baud rate (default 115200).
        motor:
            StepperDemo motor identifier, e.g. ``"M1"`` or ``"M9"``.
        config:
            Pin configuration number sent at boot.  Set via the
            ``STEPPER_CONFIG`` environment variable, falling back to
            ``"0"`` (Test-HW).  ``"1"`` selects Driver-Types.
        log_file:
            Optional path to write all received output to.
        default_timeout:
            Default timeout in seconds for ``wait_for()`` calls.
    """

    def __init__(
        self,
        device,
        baud=115200,
        motor="M1",
        config=None,
        log_file=None,
        default_timeout=30.0,
    ):
        self.device = device
        self.baud = baud
        self.motor = motor
        self.config = (
            config if config is not None else os.environ.get("STEPPER_CONFIG", "0")
        )
        self.default_timeout = default_timeout
        self.log_file = log_file
        self.log = []
        self._ser = None
        self._log_fh = None

    def __enter__(self):
        self.open()
        return self

    def __exit__(self, *args):
        self.close()

    def open(self):
        """Open the serial port.

        May trigger an ESP32 auto-reset depending on the USB-serial
        adapter.  Call ``wait_for_boot()`` afterwards.
        """
        self._ser = serial.Serial(self.device, self.baud, timeout=0.1)
        if self.log_file:
            self._log_fh = open(self.log_file, "w")

    def close(self):
        """Close the serial port and any open log file."""
        if self._ser and self._ser.is_open:
            self._ser.close()
            self._ser = None
        if self._log_fh:
            self._log_fh.close()
            self._log_fh = None

    def _log_line(self, line):
        """Append a received line to the in-memory log, stdout, and log file."""
        self.log.append(line)
        print(line)
        if self._log_fh:
            self._log_fh.write(line + "\n")
            self._log_fh.flush()

    def _read_lines(self, timeout):
        """Yield complete lines from the serial port.

        Keeps reading until *timeout* seconds elapse with no new data.
        Lines are terminated by ``\\n`` or ``\\r``.  Partial lines are
        buffered until a terminator arrives.
        """
        deadline = time.monotonic() + timeout
        line = ""
        while True:
            now = time.monotonic()
            if self._ser and self._ser.in_waiting:
                ch = self._ser.read(1).decode("ascii", errors="replace")
                if ch in ("\n", "\r"):
                    stripped = line.strip()
                    if stripped:
                        self._log_line(stripped)
                        yield stripped
                    line = ""
                else:
                    line += ch
                deadline = time.monotonic() + timeout
            elif now >= deadline:
                break
            else:
                time.sleep(0.01)

    def wait_for(self, pattern, timeout=None):
        """Block until a line matching *pattern* is received.

        Args:
            pattern:
                A regular-expression string to search for in each
                received line.
            timeout:
                Maximum seconds to wait (default ``default_timeout``).

        Returns:
            The first matching line.

        Raises:
            RuntimeError:
                If the pattern is not seen within *timeout* seconds.
        """
        if timeout is None:
            timeout = self.default_timeout
        pat = re.compile(pattern)
        for line in self._read_lines(timeout):
            if pat.search(line):
                return line
        raise RuntimeError(f"Timeout after {timeout}s waiting for: {pattern}")

    def wait_for_boot(self):
        """Wait for the ESP32 to boot and reach the StepperDemo prompt.

        Handles two cases:

        1. **Hardware reset on open** -- the ESP32 reboots automatically
           when the serial port is opened.  We wait for the pin
           configuration prompt, send the config number, and wait for
           ``LOG start`` to confirm full boot.
        2. **No hardware reset** -- the ESP32 is already running.
           We fall back to a software reset via the ``reset`` command
           and then proceed as in case 1.
        """
        try:
            self.wait_for("Press number to select", timeout=10)
        except RuntimeError:
            self.send("reset")
            self.log.clear()
            self.wait_for("Press number to select", timeout=15)

        time.sleep(0.05)
        self.send(self.config)

        self.wait_for("LOG start")
        time.sleep(0.1)

    def send(self, command):
        """Send a command string to the ESP32.

        A trailing newline is appended automatically.
        """
        self._ser.write((command + "\n").encode())
        self._ser.flush()

    def software_reset(self):
        """Perform a software reset of the ESP32.

        Sends ``x`` first to exit test mode if active, then ``reset``
        which calls ``ESP.restart()``.  The serial connection stays
        open, so no DTR/RTS toggle occurs.
        Call ``wait_for_boot()`` afterwards to wait for the reboot.
        """
        self._ser.write(b"x\n")
        self._ser.flush()
        time.sleep(0.05)
        self.send("reset")
        self.log.clear()

    def clear_log(self, new_log_file=None):
        """Clear the in-memory log and optionally switch log file.

        Useful for test loops (e.g. seq_03) where each iteration
        needs a fresh log.
        """
        self.log.clear()
        if new_log_file is not None:
            if self._log_fh:
                self._log_fh.close()
            self.log_file = new_log_file
            self._log_fh = open(new_log_file, "w") if new_log_file else None

    def check_pattern(self, pattern):
        """Verify that *pattern* was seen in any logged line.

        Raises:
            RuntimeError:
                If the pattern was never seen.
        """
        pat = re.compile(pattern)
        for line in self.log:
            if pat.search(line):
                return True
        raise RuntimeError(f"Pattern not found in log: {pattern}")

    def check_pcnt_sync(self, tolerance=66):
        """Validate pulse counter synchronisation.

        Python equivalent of ``judge_pcnt_sync.awk``.  Checks that
        every motor status line has the API-reported position matching
        the hardware pulse counter reading within *tolerance* steps.

        Only meaningful after a ``p<n>`` command has attached a pulse
        counter.

        Raises:
            RuntimeError:
                If any reading exceeds the tolerance.
        """
        motor_num = self.motor[1]

        running_re = re.compile(rf"^M[{motor_num}]:\s+@(-?\d+)\s+\[(-?\d+)\]")
        selected_re = re.compile(rf"^>> {self.motor}:\s+@(-?\d+)\s+\[(-?\d+)\]")

        for line in self.log:
            m = running_re.match(line)
            if m:
                api = int(m.group(1)) % 32767
                pcnt = int(m.group(2))
                if pcnt < 0:
                    pcnt += 32767
                delta = abs(api - pcnt)
                if delta > 32767 - delta:
                    delta = 32767 - delta
                if tolerance < delta < 32767 - tolerance:
                    raise RuntimeError(
                        f"Pcnt sync error: {line} "
                        f"(api={api}, pcnt={pcnt}, delta={delta})"
                    )

            m = selected_re.match(line)
            if m:
                api = int(m.group(1)) % 32767
                pcnt = int(m.group(2))
                if pcnt < 0:
                    pcnt += 32767
                if api != pcnt:
                    raise RuntimeError(
                        f"Pcnt sync error: {line} (api={api}, pcnt={pcnt})"
                    )

        return True
