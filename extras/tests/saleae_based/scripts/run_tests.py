#!/usr/bin/env python3
"""
run_tests.py — addQueueEntry() characterization orchestrator.

Drives the Saleae harness: it programs a bounded segment list on the DUT,
kicks it off, captures the step/dir pins, and evaluates the measured waveform.

Everything the DUT is asked to do is expressed in **timer ticks**, because
`stepper_command_s.ticks` is a raw 16-bit queue period. The DUT's tick rate is
therefore not an assumption — it is read back from the firmware with `QINFO`
and stored in every result. Never hardcode 16 MHz: it differs per platform
(Teensy, for example, prescaled) and AVR's speed floor additionally depends on
the number of connected steppers.

A tag key ({arch}_{driver}_{channel_config}) indexes results so an already
passed test is skipped on the next run, and so the same measurement on
different silicon stays separate.

Two *generic modes* generate runs from a rule rather than naming them, and both
are here rather than in the front-end because a mode is a measurement, not a
command line:

  * `scale`  -- counts 1..channel-budget on one named driver, all steppers
    running the same period in parallel.
  * `sync`   -- every driver-list combination on a board with more than one
    driver, each stepper given its *own* period so adherence is checkable.

Neither names an architecture. `scale` is the single run an AVR board needs
(one driver, two steppers); `sync` applies to the ESP32 family, and an
architecture that grows a second driver needs no new code path here, only a
second name in the driver list.

Usage:
    python3 scripts/run_tests.py --list
    python3 scripts/run_tests.py --tag-key esp32_idf5_3_0_mcpwm_pcnt_2ch
    python3 scripts/run_tests.py --tag-key ... --tests SR_01,SR_05
    python3 scripts/run_tests.py --tag-key ... --force
    python3 scripts/run_tests.py --mode scale --driver rmt --pin-mode nodir
    python3 scripts/run_tests.py --mode sync --arch esp32
"""

import argparse
import dataclasses
import json
import os
import re
import subprocess
import sys
import time
from datetime import datetime
from pathlib import Path

SCRIPTS = Path(__file__).resolve().parent
sys.path.insert(0, str(SCRIPTS))

HARNESS = SCRIPTS.parent  # extras/tests/saleae_based


def anchor_path(value):
    """A relative --capture-dir/--results-dir resolves against the harness.

    The defaults are the bare names `capture` and `results`, which the shell
    then resolves against the working directory. So the same command typed in
    the harness root and in scripts/ produced two capture directories, two
    result directories and two tag_index.json files that know nothing about each
    other -- and a report built from one of them was silently missing every
    result recorded in the other. One home per artifact, chosen here rather
    than per invocation: an absolute path still wins, so a caller that wants a
    scratch directory can still ask for one.
    """
    p = Path(value)
    return p if p.is_absolute() else HARNESS / p

import analyze_csv  # noqa: E402
import capture as cap  # noqa: E402
import i2s_mux_decoder as muxdec  # noqa: E402
import signal_parser as sp  # noqa: E402

# SR_01..SR_30 are the implemented catalogue plus SR_00; the range is the
# catalogue's numbering, and every id in it that SCENARIOS does not define is
# reported "not implemented" rather than silently missing.
ALL_TESTS = ["SR_00"] + [f"SR_{i:02d}" for i in range(1, 32)]

# Analyzer channel -> stepper, derived from what the DUT reports in MAP rather
# than assumed here.
#
# The old hardcoded A=D0, B=D2, C=D4, D=D6 is only right in `dir` mode with four
# steppers, and it silently measured the wrong channel in every other case: in
# `nodir` mode stepper B is D1, not D2. So the map is built from the count and
# stride the firmware reports, and carried in the result record (todo R4).
#
# The default is the 4-stepper `dir` map, which is what every scenario in
# SCENARIOS below uses until a `nodir` one exists; a run that actually connected
# a different shape replaces it from MAP before evaluating.
# Stepper labels. A..Z then A1..F1, which is 32 and is as far as the I2S mux
# word goes. Not a plain alphabet: `eval_sync` used to derive a stepper's index
# as `ord(letter) - ord("A")`, which stops meaning anything at Z and raises
# above it -- and a 32-stepper mux run is exactly the case that reaches it. The
# index now comes from the map's own order.
MUX_SLOT_WIDTH = 32

# One I2S frame, in stepper ticks: src/pd_esp32/i2s_constants.h
# I2S_TICKS_PER_FRAME. A multiplexed step pulse is exactly one frame high and can
# only start on a frame boundary, which is what makes its period a set rather
# than a value. Kept here as a named constant with its source rather than read
# from the firmware, because QINFO does not report it and a second copy in a
# header the harness cannot include is the cheaper of the two.
I2S_TICKS_PER_FRAME = 64


def stepper_letter(index):
    """The label for stepper `index`: A..Z, then AA, AB, ...

    Excel-column order, matching stepper order. The labels are read back out of
    the channel map by `Pins.letters`, which uses the map's own order rather than
    sorting -- see there -- so this only has to be distinct and stable.
    """
    if index >= MUX_SLOT_WIDTH:
        raise RuntimeError(
            f"{index + 1} steppers: the harness labels at most "
            f"{MUX_SLOT_WIDTH}, which is the width of the I2S mux word")
    out = ""
    index += 1
    while index:
        index, rem = divmod(index - 1, 26)
        out = chr(ord("A") + rem) + out
    return out


def default_channel_map(count=4, stride=2):
    """{'A': {'step': 'D0', 'dir': 'D1'}, ...} for a count/stride pair.

    Sorted by label, and `Pins.index_of()` is the label -> position map that
    depends on it, so the two must agree on what "in order" means.
    """
    out = {}
    for i in range(count):
        entry = {"step": f"D{i * stride}"}
        if stride > 1:
            entry["dir"] = f"D{i * stride + 1}"
        out[stepper_letter(i)] = entry
    return out


def step_channels(chan_map):
    return {name: entry["step"] for name, entry in chan_map.items()}


def dir_channels(chan_map):
    return {name: entry["dir"] for name, entry in chan_map.items()
            if "dir" in entry}


class Pins:
    """Which analyzer channel is which stepper, and which has a direction pin.

    The map is **configuration, not a constant**, and this class is what makes
    that structural rather than a convention: an evaluator receives the `Pins`
    for the run it is judging and reads channels through it, so there is no
    module-level channel table for a stale one to be read through by accident.

    It existed as two globals (`STEP_CHANNELS`, `DIR_CHANNELS`) that `evaluate()`
    overwrote per run. That works one run at a time and quietly breaks the moment
    anything holds two maps at once -- a fixture and a mode run, or two results
    in the same process. It also meant the *wrong* map was not an error but a
    silent substitution: with `nodir`, stepper B is `D1` and not `D2`, so an
    evaluator reading the `dir` table measures a quiet pin and reports zero
    steps, which reads as a dead driver rather than as a wrong map.

    `letters()` is the stepper order, A..H, and `step_of`/`dir_of` look one up.
    `dir_of` returns None in `nodir`, where no direction pin exists at all.
    """

    __slots__ = ("map", "step", "dir")

    def __init__(self, chan_map):
        self.map = dict(chan_map)
        self.step = step_channels(self.map)
        self.dir = dir_channels(self.map)

    @classmethod
    def default(cls):
        """The 4-stepper `dir` map, for callers with nothing better to go on.

        Every scenario in SCENARIOS that uses more than one stepper connects the
        `dir` shape, and 1ch scenarios need only A, so this is correct for the
        catalogue. It is a *fallback*, not the harness's idea of a channel map;
        anything that knows what it connected should use `for_scenario()` or
        the board's own map.
        """
        return cls(default_channel_map(4, 2))

    @classmethod
    def for_scenario(cls, scenario):
        """The map for the configuration a named scenario connects.

        Derived from the scenario's own CONFIGS entry rather than assumed, so a
        1-stepper scenario is not handed a 4-stepper map and then reported as
        having three silent steppers. The pin mode comes from
        `scenario_pin_mode()`, which is `dir` for every scenario but the
        max-count one.

        For a probe scenario the CONFIGS count is only the *widest* the pin mode
        allows, not the count the board agreed to connect, so the map is built at
        that width. A hardware run does not use this map at all -- `measure()`
        reads the one the board prints -- so this is the fixture and report path,
        and its job there is to be wide enough that a fixture which puts every
        stepper on its own channel is read at all.
        """
        config = SCENARIOS[scenario][0]
        mode = scenario_pin_mode(scenario)
        count = CONFIGS[config][0]
        if scenario in PROBE_SCENARIOS:
            count = MAX_STEPPERS_PER_MODE[mode]
        return cls(default_channel_map(count, CHANNELS_PER_STEPPER[mode]))

    @property
    def count(self):
        return len(self.step)

    @property
    def letters(self):
        """Stepper names, in stepper order.

        The map's own order, NOT `sorted()`. The map is built in stepper order --
        by `default_channel_map()`, and by `read_map()` when it substitutes the
        mux slots -- and dicts keep insertion order, so that order is stepper
        order exactly.

        Sorting cannot express it past Z. The I2S mux reaches 32 steppers, so the
        labels run A..Z and then need a second block; there is no labelling of 32
        names whose *lexicographic* order is the stepper order ("AA" sorts between
        "A" and "B"), and this is why the second block exists at all -- before it,
        stepper 27 was 'Z' and stepper 28 did not exist.
        """
        return list(self.step)

    def step_of(self, letter):
        return self.step[letter]

    def dir_of(self, letter):
        """The direction channel for `letter`, or None in `nodir`."""
        return self.dir.get(letter)

    def step_wave(self, channels, letter):
        """The step channel's samples, or None when it was not captured.

        A capture can legitimately omit a channel -- a scenario that only needs
        two of eight, or an analyzer that dropped one -- and that has to be
        distinguishable from a channel that was captured and stayed quiet.
        """
        ch = self.step.get(letter)
        return channels.get(ch) if ch is not None else None

    def dir_wave(self, channels, letter):
        ch = self.dir.get(letter)
        return channels.get(ch) if ch is not None else None

    def items(self):
        """(letter, step channel) pairs, in stepper order."""
        return [(letter, self.step[letter]) for letter in self.letters]

    def index_of(self, letter):
        """`letter`'s position in stepper order.

        Not `ord(letter) - ord("A")`: that is an alphabet assumption, and it is
        wrong twice over past Z -- it is meaningless there and raises beyond it.
        The I2S mux reaches 32 steppers, which is past both.
        """
        return self.letters.index(letter)

    def missing(self, channels):
        """Steppers the board connected whose channel the capture lacks.

        This is an **incomplete capture**, not a quiet stepper, and the two have
        to be distinguishable: the map says the board connected stepper B, so if
        the capture has no `D2` then nothing was measured for B. Reporting that
        as "B emitted 0 steps" invents a defect, and *passing* it invents a
        result. Either way the measurement is not there, so a run that hits this
        fails with the reason rather than answering a question nobody asked.
        """
        return [letter for letter in self.letters
                if self.step[letter] not in channels]


# Every analyzer channel, in order. A capture has to enable all of them or a
# scenario's own channels come back missing.
STEP_CHANNEL_ORDER = [f"D{i}" for i in range(8)]

# Width of the I2S multiplexer's word: 32 slots, and a step bit and a direction
# bit are the same kind of thing -- one bit of it each. Used to bound a
# multiplexed stepper's direction slot, which MAP does not report.
MUX_SLOT_COUNT = 32


def is_mux_driver(driver):
    """A driver whose steppers are bits of the 32-bit I2S word, not pins.

    Two things follow from it and both are used below: a multiplexed stepper
    costs a *slot* and not one of the eight analyzer channels, so its count is
    bounded by the word and not by `CHANNELS`; and the three bus channels are
    spoken for, so the analyzer has five left for anything physical.

    `startswith` rather than equality because a run may name several drivers at
    once and the harness joins them with `+` (`rmt+i2s_mux`).
    """
    return (driver or "").startswith("i2s_mux")

# How many steppers each pin mode can carry: 8 channels, 2 per stepper with a
# direction pin and 1 without (white paper 3.3/10.1). The firmware caps the
# count at the smaller of this and the platform's own stepper limit.
CHANNELS = len(STEP_CHANNEL_ORDER)
CHANNELS_PER_STEPPER = {"dir": 2, "nodir": 1}
MAX_STEPPERS_PER_MODE = {
    mode: CHANNELS // per for mode, per in CHANNELS_PER_STEPPER.items()
}

# The I2S bus takes the *last* three channels, deliberately, so the stepper map
# stays `stride * i` from channel 0 with no offset in front of it and a mux
# capture is a superset of a physical one. It is the same constant as
# SALEAE_BUS_COUNT in the firmware, and the only reason the host knows it is
# that the mux bus is this harness's own wiring rather than an argument.
BUS_CHANNELS = 3

# The pin mode a *max-count* run connects in. `nodir`, because `dir` spends two
# channels per stepper and stops at 4 -- below the 6 queues MCPWM/PCNT has on
# ESP32 -- so a `dir` max-count run would report the analyzer's channel count as
# the driver's queue count. See `scenario_pin_mode()`.
MAX_COUNT_PIN_MODE = "nodir"

# Default capture rate. 4 MS/s is the practical minimum to resolve a pulse a
# few us wide at 16 MHz (see README).
DEFAULT_RATE = 4_000_000

# How deep QFILL is asked to fill the queue, in entries, and how many queue-fulls
# of steps the stop scenarios run on top of that.
#
# 16 entries, fixed rather than taken from QINFO's QUEUE_LEN, so the waveform is
# the same on every board: QUEUE_LEN is 16 on AVR and 32 on ESP32 and Pico, and
# a scenario whose fill depth is a board constant characterizes the board as much
# as the stop. 16 * 255 = 4080 steps, which is a whole queue on AVR and half of
# one on ESP32 -- enough that a driver which emits a different number says so.
QUEUE_FILL_ENTRIES = 16
QUEUE_FILL_STEPS = QUEUE_FILL_ENTRIES * 255
# Four times the fill, so a forceStop() that stopped nothing emits three quarters
# of the program and truncation cannot be mistaken for a capture that ended.
# Four segments is within QE_MAX_SEG (8) and comes to ~80 ms at the speed floor
# of a 16 MHz tick, so the capture window stays close to the one SR_25 has used.
QUEUE_FILL_ROUNDS = 4

# What elapses between the capture being armed and QRUN being written: the arm
# sleep, plus one serial round-trip per setup command after the capture starts
# (QFILL, for the scenarios that fill). The capture has to cover that as well as
# the program, or the program is cut off by the end of the recording and the
# scenario measures the capture rather than the driver.
#
# It did. SR_25 measured 12007 of 16320 steps with the last pulse on the final
# sample of the recording: the run started 0.54 s into a 0.66 s window because
# the 0.3 s arm sleep and QFILL's round-trip are inside it, and only ~0.11 s was
# left of a 0.163 s program. `truncated_by_stop` read true and the whole run
# looked like a stop that had worked.
SETUP_AFTER_CAPTURE_S = 0.3 + 0.25

# How long the SR_00 1 Hz pattern runs before its capture starts, so the window
# holds whole cycles only. One period plus a margin for the loop's granularity;
# the pattern keeps running during the capture, so nothing is lost by starting
# it early.
SR00_SETTLE_S = 1.5


# ---------------------------------------------------------------------------
# Serial
# ---------------------------------------------------------------------------


def native_usb_port(port):
    """True for a port that is the MCU's own USB, not a USB-serial bridge.

    RP2040/RP2350 and other native-USB parts do **not** reset the target when
    the host opens the port, and there is no DTR/RTS toggle to force one (the
    only reset the arduino-pico core offers is the 1200 bps jump to the UF2
    bootloader). So the firmware's `READY`, printed once at boot, is already
    gone by the time the host reconnects -- and a real reset would re-enumerate
    USB and lose it all over again.

    The distinction is load-bearing: the `PING` fallback in `open_board()` must
    only apply where a reset is impossible. On a bridge board (ESP32-DevKitC,
    AVR nano) a missing READY means the DTR/RTS toggle did not fire, and
    probing past that would silently measure a stale board -- the exact failure
    `open_board()` exists to make loud. macOS names native ports `cu.usbmodem*`,
    Linux `ttyACM*`; a bridge is `usbserial*` / `SLAB_USBtoUART` / `ttyUSB*`.
    """
    base = os.path.basename(port)
    return base.startswith(("cu.usbmodem", "usbmodem", "ttyACM"))


def open_board(port, baud, timeout=6.0, require_ready=True):
    """Open the serial port (which resets the board) and wait for READY.

    **Every run starts here, and the reset is not optional.** The board's
    firmware trips its own task watchdog while completely idle -- measured on
    ESP-IDF 5.5.3: `open_board()`, then nothing at all, produces six
    `task_wdt` lines within ~5.3 s and an IDLE0 backtrace, with no CONFIG, no
    program and no capture involved. `saleae_hal_serial_read()` is a
    zero-timeout `uart_read_bytes()`, so `app_main` spins and IDLE0 starves.

    That is a firmware property, and it is the reason a reset precedes each
    test rather than once per invocation: a watchdog panic *inside* a capture
    window ends the run early, and SR_00 then reports a truncated first pulse
    (measured: D0's first high 19.3 ms where 50 ms was commanded, with the
    pairs summing to exactly one 1000 ms period) as eight dead pins. The
    pre-check that exists to catch a bad cable was reporting a firmware panic.

    `require_ready` makes that failure loud. Opening the port does not
    *guarantee* a reset -- the DTR/RTS toggle does not always fire, which is
    the same reason a flash occasionally comes up in the wrong boot mode -- so
    the old loop simply fell through after `timeout` and handed back a stale
    board that answered commands from the previous test. It now raises, because
    a run against an unreset board measures that board's leftovers.

    Native USB (RP2040/RP2350) is the one board this cannot work on, because
    the port open does not reset it at all and no toggle can. There the reset is
    requested over the protocol instead -- see `_open_native_usb()`.
    """
    import serial
    if native_usb_port(port):
        return _open_native_usb(port, baud, timeout, require_ready)
    ser = serial.Serial(port, baud, timeout=0.1)
    deadline = time.time() + timeout
    buf = b""
    while time.time() < deadline:
        data = ser.read(256)
        if data:
            buf += data
            if b"READY" in buf:
                break
    if require_ready and b"READY" not in buf:
        ser.close()
        raise BoardError(
            f"{port} did not report READY within {timeout:.0f}s, so the board "
            f"was not reset (the DTR/RTS toggle does not always fire). "
            f"Measuring a stale board would report the previous run's state "
            f"as this one's. Saw {len(buf)} byte(s) of boot output; unplug and "
            f"replug the board, or pass a longer --baud-timeout.")
    ser.reset_input_buffer()
    return ser


def _open_native_usb(port, baud, timeout, require_ready):
    """Reset a native-USB board over the protocol, then wait for `PING`.

    RP2040/RP2350 do not reset on port open and expose no DTR/RTS reset, so the
    reset is requested with the `RESET` command (firmware `rp2040.reboot()`).
    That is not cosmetic: a queue is allocated once and the engine has no
    release, so a board that was **not** reset refuses a *different* CONFIG --
    which is every second scenario in a catalogue or a `scale` sweep. On a
    bridge board the port-open reset is what clears that; this is its stand-in.

    The chip re-enumerates USB when it reboots, so the boot-time `READY` is a
    race and is not awaited. `PING` proves the board came back and is the right
    firmware. A port that does not answer within `max(timeout, 10)` raises,
    because a run against a board that did not reset measures its leftovers.
    """
    import serial

    def try_open():
        try:
            return serial.Serial(port, baud, timeout=0.1)
        except (OSError, serial.SerialException):
            return None

    # Ask for the reboot. The reply is best-effort: USB drops as the chip
    # resets, so it may be cut off before it is read.
    ser = try_open()
    if ser is not None:
        try:
            ser.write(b"RESET\n")
            ser.flush()
        except (OSError, serial.SerialException):
            pass
        time.sleep(0.5)
        try:
            ser.close()
        except OSError:
            pass

    # Wait for the port to come back and answer PING.
    window = max(timeout, 10.0)
    deadline = time.time() + window
    buf = b""
    ser = None
    while time.time() < deadline:
        ser = try_open()
        if ser is None:
            time.sleep(0.1)
            continue
        try:
            ser.write(b"PING\n")
        except (OSError, serial.SerialException):
            ser.close()
            ser = None
            time.sleep(0.1)
            continue
        end = time.time() + 1.0
        while time.time() < end:
            data = ser.read(256)
            if data:
                buf += data
                if b"OK PING" in buf:
                    ser.reset_input_buffer()
                    return ser
        ser.close()
        ser = None

    if ser is not None:
        ser.close()
    if require_ready:
        raise BoardError(
            f"{port} did not answer PING within {window:.0f}s of a RESET, so "
            f"the board did not reset or is not running the harness firmware. "
            f"Saw {len(buf)} byte(s); if the port name changed across the "
            f"reboot, pass the new one. A stale board would report the "
            f"previous run's state as this one's.")
    ser = try_open()
    if ser is None:
        raise BoardError(f"{port} is not present after RESET")
    ser.reset_input_buffer()
    return ser


# Markers the firmware prints that mean "something went wrong in the firmware",
# as opposed to a line the host sent being refused. A test that measures a pin
# must not read one of these as a dead wire, and the way to guarantee that is to
# fail the run on the spot rather than leave it to the waveform to explain.
FIRMWARE_FAULT_MARKERS = (
    "task_wdt",
    "Guru Meditation",
    "assert failed",
    "Backtrace:",
    "abort() was called",
    "Panic",
)


def firmware_fault(reply):
    """The firmware-fault markers in `reply`, or None. See FIRMWARE_FAULT_MARKERS."""
    hits = [m for m in FIRMWARE_FAULT_MARKERS if m in reply]
    return hits or None


class BoardError(RuntimeError):
    """The board is not in a state this measurement can be taken in.

    Distinct from a scenario `failed` on the record: this one means *no*
    measurement was made, so it must not be recorded as a verdict about the
    driver. A reset that did not happen is the case that motivated it.
    """


def send_line(ser, line, settle=0.05):
    ser.write((line + "\n").encode())
    time.sleep(settle)


def drain(ser, seconds=0.3):
    buf = b""
    deadline = time.time() + seconds
    while time.time() < deadline:
        data = ser.read(4096)
        if data:
            buf += data
    return buf.decode(errors="replace")


def reply_of(ser, line):
    send_line(ser, line)
    return drain(ser, 0.2)


# maxall is the largest per-stepper floor, which is what a shared program has
# to be planned against. maxspeed is the first stepper's own floor and is kept
# for the grammar's sake. Both are read because the firmware prints one floor
# per stepper comma-separated: parsing only the first field is right for one
# stepper and silently wrong for more, because the run then either plans too
# fast (the firmware refuses) or, worse, plans against a number that is really
# several concatenated together.
# `maxall` leads the reply so it cannot be the field a buffer overruns, and the
# per-stepper floors are indexed after it. Parsing the *first* field alone would
# have been wrong for any run with more than one stepper, which is exactly the
# shape the two generic modes produce.
QINFO_RE = re.compile(r"tps=(\d+) mincmd=(\d+) qlen=(\d+) maxall=(\d+)"
                      r"(?: maxspeed\d+=(\d+))*")
# `ch=` may be empty, a pin list, or the single `-` a build without
# SUPPORT_SELECT_DRIVER_TYPE emits (it gives up the pin list). The `-` has to be
# consumed by the class -- with `[\d,]*` it was left in the string, and because
# it sits immediately after `ch=` with no whitespace, every following optional
# field (bus/slots/dslots/`marker`) failed to match against it. `marker` then
# defaulted to -1 and SR_25/SR_30 reported "no marker channel free" on every
# non-ESP32 board even though MARK had just succeeded.
MAP_RE = re.compile(r"MAP count=(\d+) mode=(\w+) stride=(\d+) ch=([\d,-]*)"
                      r"(?:\s+bus=([\d,-]+))?"
                      r"(?:\s+slots=([\d,-]*))?"
                      r"(?:\s+dslots=([\d,-]*))?"
                      r"(?:\s+marker=(\d+))?")
# "OK DRIVERS mux=0 rmt=1 rmt=1 mcpwm_pcnt=1 i2s_direct=1 i2s_mux=1
#  mux_init=0". The driver fields are read as a name=value scan rather than by
# position, because the set of names is build-dependent: an AVR build emits only
# `timer`, a Pico only `timer` and `pio`. A positional parse would read a
# different field as each driver on each target.
DRIVERS_RE = re.compile(r"OK DRIVERS\s+mux=(\d)(.*?)(?:\s+mux_init=(\d))?\s*$",
                        re.M)
# "OK QFILL q=14" -- the queue depth the board actually reached, which is not
# necessarily the one asked for: QUEUE_LEN is 16 on AVR and 32 on ESP32, and the
# firmware keeps QE_ROOM_RESERVE entries free for a driver's DIR-drain pause.
QFILL_RE = re.compile(r"OK QFILL q=(\d+)")


def read_drivers(ser):
    """What this build accepts: ({driver: bool}, mux_init) or raises.

    The board is asked rather than consulted from a table. The host used to keep
    DRIVER_MAXS, a hand-maintained copy of the library's declared QUEUES_*
    constants. It was not wrong about what it counted -- this board really does
    allocate six MCPWM queues, and refuses the seventh at CONFIG -- but a queue
    count is not a health check: only one of those six runs, and no constant can
    express the difference. A host cannot detect that class of error, because
    only the hardware knows the answer, so the copy has to go rather than be
    corrected.

    Queue *counts* are not reported by the firmware and are not asked for here.
    They live in pd_config.h behind headers the public API does not expose, and
    a library accessor added for a test harness would be test scaffolding in the
    product. They are also not needed: `scale` sweeps to the analyzer's channel
    budget and the point the board refuses *is* the measured bound.
    """
    text = ""
    for _ in range(5):
        text = reply_of(ser, "DRIVERS")
        m = DRIVERS_RE.search(text.strip())
        if m:
            present = {k: v == "1" for k, v in re.findall(r"(\w+)=(\d)", m.group(2))}
            return present, m.group(3) == "1"
        time.sleep(0.1)
    raise RuntimeError(f"no DRIVERS reply, got: {text!r}")


def send_imux(ser):
    """Bring the I2S multiplexer up. Returns True on success.

    `initI2sMux()` has to precede any mux stepper and cannot run twice, so it is
    issued once here rather than retried per scenario. On a board where no
    multiplexer is wired this is simply never called, and `DRIVERS` reports
    i2s_mux present but not up -- which is the distinction that stops a planner
    from offering a driver every CONFIG will refuse.

    No arguments. The bus is the last three analyzer CHANNELS and their GPIOs
    come out of the firmware's own channel table, so naming the pins from here
    would mean naming *this* rig's wiring from a host that has no way to know it
    -- and the failure would be a capture that decodes into the wrong 32 slots.
    See SALEAE_BUS_BASE in common/saleae_app.cpp.
    """
    text = reply_of(ser, "IMUX")
    return text.lstrip().startswith("OK IMUX")


def read_map(ser):
    """Read the DUT's channel map: which analyzer channel is which stepper.

    The host must not assume it. `dir` spends two channels per stepper (step,
    dir) and `nodir` one, so 4 steppers is D0,D2,D4,D6 in the first case and
    D0,D1,D2,D3 in the second -- and a host that guessed wrong measures a quiet
    pin and reports zero steps, which reads as a driver that emits nothing.

    A multiplexed stepper is not on a channel at all: its step signal is one bit
    of the 32-bit word the I2S bus carries, so MAP reports the slot and the
    channel it becomes is `S<slot>` in the DECODED capture. Handing the
    evaluators `D0` for such a stepper would measure the pin of whichever
    physical stepper happens to own it.

    Returns (channel_map, pins) where channel_map is the letter -> {step, dir}
    dict the evaluators index and pins is the GPIO behind each channel, for the
    record.
    """
    for _ in range(5):
        text = reply_of(ser, "MAP")
        m = MAP_RE.search(text)
        if m:
            count, mode, stride = int(m.group(1)), m.group(2), int(m.group(3))
            # `-` (no pin list) and empty fields are skipped: this build may
            # report `ch=-`, so a bare int() on it would raise.
            pins = [int(p) for p in m.group(4).split(",") if p.isdigit()]
            # bus= is "-" when the multiplexer is not up; the fields are only
            # present on an ESP32 build, hence the None guards.
            bus_field = m.group(5)
            bus = [int(b) for b in bus_field.split(",")] if bus_field and bus_field != "-" else []
            slot_field = m.group(6)
            slots = ([int(s) if s != "-" else None
                      for s in slot_field.split(",")]
                     if slot_field else [])
            # The direction bit of a multiplexed stepper, same indexing as
            # `slots`. Reported by the firmware rather than inferred, because the
            # host's `step_slot + 1` was right only while allocation stayed
            # gapless -- 0/1, 2/3, ... in stepper order -- which is a property of
            # this firmware's cursor, not a fact the reply carried.
            dslot_field = m.group(7)
            dslots = ([int(s) if s != "-" else None
                       for s in dslot_field.split(",")]
                      if dslot_field else [])

            chan_map = default_channel_map(count, stride)
            # Insertion order, not sorted(). Past 26 steppers the two differ --
            # sorted() puts "A1" right after "A", the build order puts it after
            # "Z" -- and the mismatch hands slot 11 to the letter that owns slot
            # 16. Every stepper then gets another stepper's channel and the
            # result says so, which is the one thing a channel map exists to
            # prevent. See Pins.letters.
            letters = list(chan_map)
            # Physical channels are handed out in order to the steppers that
            # own a wire, and a multiplexed stepper does not take one -- so this
            # cursor is NOT the stepper index. `CONFIG 3 i2s_mux,i2s_mux,rmt
            # nodir` puts the physical stepper on channel 0, not channel 2, and a
            # map built from the index would read D2: a channel the bus uses.
            chan_used = 0
            for j, name in enumerate(letters):
                # One entry per STEPPER, not per channel: the firmware reports
                # `slots[i].mux_slot`, which is the step bit stepper i claimed,
                # and `-` for a stepper that is on a GPIO. Its own comment says
                # the field "lines up with the stepper letters one to one", and
                # `nodir` is the only mode where one-per-stepper and
                # one-per-channel coincide -- which is exactly why every mux
                # measurement made so far was `nodir` and looked right.
                #
                # Reading it as one-per-channel (this indexed `slots[j*stride]`)
                # ran off the end for every stepper past the first in `dir`, so
                # a mux stepper was given a GPIO channel and measured on a pin
                # that carries somebody else's steps: `sync --imux` reported 0 of
                # 64 for a stepper the capture shows stepping 64 times on S0.
                step_slot = slots[j] if j < len(slots) else None
                if step_slot is None:
                    if chan_used + stride > count * stride:
                        raise RuntimeError(
                            f"stepper {name} maps to a channel outside the "
                            f"reported {count * stride}")
                    chan_map[name] = {"step": STEP_CHANNEL_ORDER[chan_used]}
                    if stride > 1:
                        chan_map[name]["dir"] = STEP_CHANNEL_ORDER[chan_used + 1]
                    chan_used += stride
                    continue
                chan_map[name] = {"step": f"S{step_slot}"}
                if stride > 1:
                    # Direction on the mux is a second bit of the same word,
                    # and MAP reports it in `dslots`. This used to be
                    # `step_slot + 1`, which was right only because CONFIG resets
                    # the slot cursor and connects the steppers in order, so the
                    # pairs came out gapless: 0/1, 2/3, ... in stepper order.
                    # That is a property of the allocator, not a fact the reply
                    # carried, so a host reading it was right by coincidence and
                    # silently wrong the moment the cursor was ever resumed.
                    if not dslots:
                        raise RuntimeError(
                            f"stepper {name} is multiplexed and in {mode} mode, "
                            f"so its direction bit is needed, but this MAP reply "
                            f"has no dslots= field -- the firmware predates it. "
                            f"Reflashing is required; the alternative "
                            f"(step_slot + 1) is the assumption this field "
                            f"exists to remove.")
                    dir_slot = dslots[j] if j < len(dslots) else None
                    if dir_slot is None:
                        raise RuntimeError(
                            f"stepper {name} has a direction pin in {mode} mode "
                            f"(step slot {step_slot}) but MAP reports no "
                            f"direction bit for it")
                    if not 0 <= dir_slot < MUX_SLOT_COUNT:
                        raise RuntimeError(
                            f"stepper {name} has a direction bit at {dir_slot}, "
                            f"outside the {MUX_SLOT_COUNT}-bit word MAP reports "
                            f"on")
                    chan_map[name]["dir"] = f"S{dir_slot}"
            # marker=-1 means no channel is designated as the event marker.
            marker = int(m.group(8)) if m.group(8) else -1
            physical = chan_used
            return chan_map, {"mode": mode, "stride": stride, "pins": pins,
                              "marker": marker, "bus": bus, "slots": slots,
                              "dslots": dslots,
                              "physical_channels": physical}
        time.sleep(0.1)
    raise RuntimeError(f"no MAP reply, got: {text!r}")


def read_qinfo(ser):
    """Read the DUT's tick rate and queue limits.

    Returns a dict with ticks_per_s, min_cmd_ticks, queue_len and the
    per-stepper speed floor. These are the numbers every expectation below is
    derived from.
    """
    for _ in range(5):
        text = reply_of(ser, "QINFO")
        m = QINFO_RE.search(text)
        if m:
            per_stepper = [int(v) for v in
                           re.findall(r"maxspeed\d+=(\d+)", m.group(0))]
            return {
                "ticks_per_s": int(m.group(1)),
                "min_cmd_ticks": int(m.group(2)),
                "queue_len": int(m.group(3)),
                # The fastest period legal for *every* connected stepper --
                # the largest of the per-stepper floors. Every builder asks for
                # "the fastest period this program may use", and for a program
                # all steppers walk that is the max, not stepper A's own. The
                # per-stepper values stay available for a builder that has to
                # address one stepper specifically.
                "max_speed_ticks": max(per_stepper, default=int(m.group(4))),
                "max_speed_all_ticks": int(m.group(4)),
                "max_speed_per_stepper": per_stepper,
            }
        time.sleep(0.1)
    raise RuntimeError(f"no QINFO reply, got: {text!r}")


# SR_00's own capture window, in seconds. Two full cycles of a 1 Hz pattern with
# every channel at a different duty, so each one shows a complete high and a
# complete low and the width check has something to measure.
SR00_SECONDS = 2.5

# Scenarios whose subject needs the STOP instant *on the waveform*. Sending STOP
# and then inferring when it landed from "the pulses ceased" cannot work on this
# rig: the capture the host requests is not the capture it gets (24 MHz
# truncates 0.7 s to ~458 ms), so a quiet tail is not evidence of a stop.
SCENARIO_MARKERS = {"SR_25", "SR_30"}


def marker_channel_for(count, stride, channels=8, bus_channels=0):
    """The highest analyzer channel no stepper owns, or None if there is none.

    The marker has to be readable without ambiguity against a stepper's own
    edges, so it cannot share a channel. At 8 steppers in `nodir` every channel
    is taken and there is no marker to be had -- which is a real limit of this
    approach, and the reason MARK is refused rather than quietly overwriting a
    step pin.

    `bus_channels` is what the I2S multiplexer costs: three analyzer channels
    that carry the bus and belong to no stepper, but whose edges are the bus
    protocol rather than an event, so a marker there is unreadable. It counts
    against the budget for MARK even though it counts for nothing else -- which
    is exactly why it is a parameter and not a constant here.
    """
    used = count * stride + bus_channels
    if used >= channels:
        return None
    return channels - 1


def stop_after_for(scenario, segments, info):
    """When to issue the stop, in seconds after QRUN, or None.

    A fraction of the *fill's* duration, derived from the DUT's own tick rate and
    period -- not a wall-clock constant. The queue is filled before the start and
    nothing is added after it, so the run is exactly `QUEUE_FILL_STEPS` steps
    long whatever the program behind it says, and the stop has one requirement:
    land inside that. A fixed 1 ms met it on two of three drivers and not on the
    third -- i2s_direct takes longer than that to produce its first step, being a
    DMA stream rather than a per-entry compare, and its marker edge came 16 us
    *before* the first pulse. The scenario then measured a stop that interrupted
    nothing, which the `stop_interrupted_the_run` guard rejected.

    Scaled, the same delay is ~10 ms on this DUT: 25% into a 40.8 ms fill, about
    a thousand steps in, and far outside any driver's start latency. Scaling is
    also what makes it portable -- 1 ms is 200 steps at 5 us and 25 at 40 us, so
    a constant tuned on one driver is a different depth on the next.

    The floor exists because a stop issued in the same instant as QRUN can land
    before the move starts, which measures nothing at all.
    """
    if scenario not in STOP_AFTER:
        return None
    fill_s = (QUEUE_FILL_STEPS * segments[0][1]
              / float(info["ticks_per_s"]))
    return max(STOP_AFTER_MIN, fill_s * STOP_AFTER[scenario])


# The scenarios whose subject IS a queue rejection. SR_13 programs a command
# below MIN_CMD_TICKS on purpose, to pin that nothing is emitted; everywhere else
# such a rejection is a setup failure.
REJECTION_SCENARIOS = {"SR_13"}


def unprogrammable(segments, info):
    """The first entry the queue must reject, or None.

    addQueueEntry() bounds the whole command: `ticks * steps` (or `ticks` for a
    pause) must reach MIN_CMD_TICKS, and QINFO told us that number before the
    run. Checking here means an illegal scenario is refused *before* a capture
    is started, instead of after -- when all that is left to say is that the pin
    emitted nothing, which reads as a dead driver and is not the case.

    This is not hypothetical bookkeeping. SR_05 asked for 16 steps at a period
    floor of 160 ticks because its builder used `max(max_speed_ticks, 160)`
    rather than legal_ticks(): 2560 ticks against a MIN_CMD_TICKS of 3200. RMT's
    floor of 640 made the same expression give 10240, so the scenario passed for
    years and broke the moment it was run on a driver whose floor is lower.
    """
    for steps, ticks, _ in segments:
        if ticks * (steps if steps else 1) < info["min_cmd_ticks"]:
            return {"steps": steps, "ticks": ticks,
                    "us": round(ticks * (steps if steps else 1)
                                / info["ticks_per_s"] * 1e6, 1),
                    "min_cmd_ticks": info["min_cmd_ticks"]}
    return None


def program(ser, segments, info=None):
    """Send QCLR followed by one QSEG per segment. Returns False on error."""
    if info is not None:
        bad = unprogrammable(segments, info)
        if bad is not None:
            print(f"    not programmable: {bad['steps']} x {bad['ticks']} ticks "
                  f"= {bad['us']} us < MIN_CMD_TICKS ({bad['min_cmd_ticks']} "
                  f"ticks). The queue would reject it; not capturing.")
            return False
    text = reply_of(ser, "QCLR")
    if "OK QCLR" not in text:
        return False
    for steps, ticks, count_up in segments:
        text = reply_of(ser, f"QSEG {steps} {ticks} {1 if count_up else 0}")
        if "OK QSEG" not in text:
            print(f"    QSEG {steps} {ticks} rejected: {text.strip()}")
            return False
    return True


def program_per_stepper(ser, programs, info=None):
    """QCLR, then the 4-argument QSEG for each stepper's own program.

    The shared 3-argument form cannot express two different periods, so a
    scenario that needs them (SR_15, and every `sync` run) has to send one
    indexed list per stepper. Split out of run_hardware's inline version so both
    runners send the same thing in the same order.
    """
    if info is not None:
        for idx in sorted(programs):
            bad = unprogrammable(programs[idx], info)
            if bad is not None:
                print(f"    stepper {idx} not programmable: {bad['steps']} x "
                      f"{bad['ticks']} ticks = {bad['us']} us < "
                      f"MIN_CMD_TICKS ({bad['min_cmd_ticks']} ticks)")
                return False
    text = reply_of(ser, "QCLR")
    if "OK QCLR" not in text:
        return False
    for idx in sorted(programs):
        for steps, ticks, count_up in programs[idx]:
            line = f"QSEG {idx} {steps} {ticks} {1 if count_up else 0}"
            if "OK QSEG" not in reply_of(ser, line):
                print(f"    {line} rejected")
                return False
    return True


# Scenarios that must know what the queue holds when the stop lands, so the
# board fills it deliberately (QFILL, start = false) before QRUN instead of
# leaving the depth to the feeder.
SCENARIO_FILL = {"SR_25", "SR_30"}


def fill_queue(ser, mask, entries=QUEUE_FILL_ENTRIES):
    """QFILL the queue and return the depth the board reached, or 0.

    The depth reached is not the depth asked for: QUEUE_LEN is 16 on AVR and 32
    on ESP32, and the firmware keeps two entries free for a driver's DIR-drain
    pause, so a request for 16 can only be met in full on some boards. The
    number is the board's, and the evaluator bounds the drain with it rather
    than with a host-side assumption.
    """
    m = QFILL_RE.search(reply_of(ser, f"QFILL {mask} {entries}"))
    return int(m.group(1)) if m else 0


# ---------------------------------------------------------------------------
# Capture
# ---------------------------------------------------------------------------


def decode_mux_capture(capture_file, channels, sample_rate, pin_map,
                       chan_map=None):
    """Turn a mux run's 8 channels into its 37, and say what it was decoded from.

    The three bus channels are consumed and 32 slot channels appear in their
    place, so the evaluators and every metric in signal_parser.py work unchanged
    on the result -- which is the whole point: the alternative is teaching eight
    channel-specific metrics about a multiplexed driver.

    `source_vcd` is the change-only VCD of the capture, not the .sr: a .sr is a
    zip archive and `signal_parser.load_vcd` does not read one.

    `pin_map["bus"]` is the board's own answer to which channels carry the bus
    and which bit of the word each stepper owns, read from MAP. Nothing here is
    guessed: a bus channel named wrongly decodes into 32 quiet channels, which
    reads as a driver that emits nothing rather than as a bad map.

    The decoded VCD is written next to the capture so a run can be re-evaluated
    without the hardware, and it is returned in the result as `decoded_from` --
    without it, "which waveform did you measure" has no answer for a mux run,
    because the waveform in the capture is three wires and the one the evaluators
    read was computed from them.
    """
    bus = pin_map["bus"]
    # The physical steppers ride along. They are named by the same map the
    # evaluators get, so this cannot disagree with what they will read: take
    # the entries that are NOT slot channels.
    passthrough = []
    for entry in (chan_map or {}).values():
        for role in ("step", "dir"):
            name = entry.get(role)
            if name and not name.startswith("S") and name not in passthrough:
                passthrough.append(name)
    cfg = muxdec.DecoderConfig(
        source_vcd=str(capture_file),
        output_vcd=str(Path(str(capture_file).rsplit(".", 1)[0] + "_37ch.vcd")),
        i2s_channels={"data": f"D{bus[0]}", "bclk": f"D{bus[1]}",
                      "ws": f"D{bus[2]}"},
        passthrough_channels=sorted(passthrough),
        mux_slot_map={
            letter: {"slot": entry["slot"],
                     "step_channel": entry["step"],
                     "dir_channel": entry.get("dir")}
            for letter, entry in _mux_map_with_slots(pin_map).items()},
        stepper_count=len(pin_map.get("slots") or []),
        pin_mode=pin_map.get("mode"),
        include_bus=True,
        out_comment=[f"  captured_from: {Path(capture_file).name}"],
    )
    faults = []
    out = muxdec.decode(cfg, sample_rate, faults)
    decoded, rate = sp.load_vcd(str(out))
    # Only the slots this run used, so the decoded file is 5 + n channels rather
    # than 37. The count is recorded in the result either way; what a reader wants
    # is the waveform of the steppers that were actually connected.
    return decoded, rate, {"vcd": str(out), "source": str(capture_file),
                           "bus_channels": bus,
                           "channels": len(decoded),
                           # How many frames the decoder could not place on the
                           # bus's own frame grid. Recorded rather than acted on:
                           # 24 MS/s over an 8 MHz bclk is three samples per bit,
                           # and in `dir` the data line toggles every frame, so
                           # faults are expected there and gating on them would
                           # fail every `dir` mux run for a property of the
                           # analyzer. What it buys is that a red run can say
                           # "this capture decoded N misaligned words" next to
                           # the step count that read it as a driver defect.
                           "frame_faults": len(faults),
                           "frame_fault_examples": faults[:8]}


def _mux_map_with_slots(pin_map):
    """letter -> {slot, step, dir} for the multiplexed steppers only.

    One entry per *stepper*, from `slots` and `dslots` -- the same indexing
    `read_map()` uses, and for the same reason. This used to index
    `slots[j * stride]`, treating the field as one entry per channel, which is
    the misreading that made `read_map()` hand a multiplexed stepper a GPIO
    channel in `dir` mode; it survived here because the decoder path and the
    evaluator path disagreed about what the field meant.
    """
    stride = pin_map.get("stride", 2)
    slots = pin_map.get("slots") or []
    dslots = pin_map.get("dslots") or []
    letters = "ABCDEFGH"
    out = {}
    for j, slot in enumerate(slots):
        if slot is None:
            continue
        letter = letters[j] if j < len(letters) else f"S{j}"
        entry = {"slot": slot, "step": f"S{slot}"}
        if stride > 1 and j < len(dslots) and dslots[j] is not None:
            entry["dir"] = f"S{dslots[j]}"
        out[letter] = entry
    return out


def start_capture(output, seconds, rate):
    devices, driver = cap.detect_analyzer("auto")
    # All eight, not just the ones this scenario reads: SR_00 needs every channel
    # and a nodir 8-stepper run needs all of them too, so a narrowed list would
    # have to be correct per scenario rather than always.
    channels = ",".join(STEP_CHANNEL_ORDER)
    cmd = cap.build_command(driver, rate, channels,
                            int(seconds * 1000), output, None, "srzip")
    return subprocess.Popen(cmd, stdout=subprocess.DEVNULL,
                            stderr=subprocess.DEVNULL)


def load_capture_for_eval(capture_file, with_vcd=False):
    """Load a recorded .sr as channels + rate, via its change-only VCD.

    `with_vcd` adds the VCD path, which the mux decoder needs: it reads a VCD, and
    handing it the .sr makes it decode a zip archive. The VCD is the change-only
    form of the same capture, so deriving it here is what keeps the decoder on
    the same artifact the evaluators are reading rather than on a second
    conversion of the same bytes.
    """
    vcd_file = cap.sr_to_vcd(capture_file,
                             Path(str(capture_file).rsplit(".", 1)[0] + ".vcd"))
    if vcd_file is not None:
        channels, rate = sp.load_vcd(str(vcd_file))
        return (channels, rate, str(vcd_file)) if with_vcd else (channels, rate)
    channels, rate = sp.load_capture(str(capture_file))
    return (channels, rate, None) if with_vcd else (channels, rate)


def sub_min_entries(segments, info, programs=None):
    """Queue entries shorter than the DUT's MIN_CMD_TICKS, in microseconds.

    Some drivers emit them and some silently discard them, and the difference
    is invisible from the outside: the pin is simply quiet afterwards. Naming
    them in the result turns "measured zero steps" into "measured zero steps,
    and here is a command this driver does not honour".
    """
    entries = segments if segments is not None else [
        seg for segs in programs.values() for seg in segs]
    return [{"steps": st, "ticks": tk,
             "us": round(tk * (st if st else 1) / info["ticks_per_s"] * 1e6, 1)}
            for st, tk, _ in entries
            if tk * (st if st else 1) < info["min_cmd_ticks"]]


def scenario_seconds(segments, ticks_per_s):
    """Exact duration of a program: sum of ticks*steps (or ticks for a pause).

    There is no ramp here, so this is exact rather than an estimate.
    """
    total = 0
    for steps, ticks, _ in segments:
        total += ticks * (steps if steps else 1)
    return total / float(ticks_per_s)


# ---------------------------------------------------------------------------
# Scenarios
# ---------------------------------------------------------------------------
#
# Each scenario returns (config, segments_fn, mask). segments_fn takes the
# QINFO limits so it can address the exact boundaries (the speed floor,
# ticks == 65535) without hardcoding platform constants.


def seg_period(n, ticks, count_up=True):
    return [(n, ticks, count_up)]


def legal_ticks(info, steps, wanted):
    """Raise `wanted` until the command is one the firmware will accept.

    addQueueEntry() bounds the *whole command*, not the period:
    command_rate_ticks = ticks * steps (for steps > 1) must be >= MIN_CMD_TICKS,
    or it returns ErrorTicksTooLow and emits nothing. Confirmed on the ESP32:
    QSEG 2 640 1 is refused, QSEG 2 1600 1 is accepted.

    So a scenario cannot pick a period freely -- with few steps the fastest
    legal period is MIN_CMD_TICKS/steps. Sending an illegal command describes a
    waveform the hardware never produces, so every builder clamps through here.
    """
    floor = info["min_cmd_ticks"]
    need = -(-floor // steps) if steps > 1 else floor
    # And under the 16-bit ticks field, which is a property of the command and
    # not of the board. Two max_speed floors concatenated by a sloppy QINFO
    # parse once produced 808080 here, and the firmware's rejection came back
    # saying the ticks were out of range -- true, and about a number no scenario
    # had asked for. Clamping makes that class of mistake fail as an
    # implausibly slow run instead of as a confusing refusal.
    return min(65535, max(wanted, need, 160))


def sc_period_exact(info):
    # Comfortably fast, but well inside the 16-bit range.
    return seg_period(8, legal_ticks(info, 8, info["max_speed_ticks"]))


def sc_steps_per_command(info):
    return seg_period(255, legal_ticks(info, 255, info["max_speed_ticks"]))


# SR_15 is the one scenario the shared command program cannot express: it needs
# two steppers at *different* periods. The firmware's 4-argument QSEG form takes
# a stepper index for exactly this. Defined here, once, so the runner that sends
# the commands and the evaluator that checks the result cannot disagree about
# who runs at what speed.
SR_15_RATIO = 2


# The scenarios that cannot be expressed with the shared 3-argument QSEG and so
# need one indexed program per stepper. SR_15 is the only one, and it is the
# only one because it is the only one that gives two steppers *different*
# periods.
#
# This is a set rather than a probe because the runner has to answer "does this
# scenario need per-stepper programs?" *before* it has a QINFO dict, and
# per_stepper_programs() needs one. Asking it with a None info and comparing the
# answer crashed on SR_15 -- a scenario the catalogue has carried since R1 and
# which nothing had actually run, because the harness path that reaches it had
# never worked.
PER_STEPPER_SCENARIOS = {"SR_15"}


def needs_per_stepper(scenario):
    return scenario in PER_STEPPER_SCENARIOS


# The scenarios whose stepper *count* is not known before the board is asked, and
# whose pin mode is therefore not `dir`. SR_31 is the only one.
#
# A set for the same reason PER_STEPPER_SCENARIOS is one: `measure()` has to
# answer "does this scenario need a probe?" before it has a serial port, and the
# probe needs the driver name, which only the caller has.
PROBE_SCENARIOS = {"SR_31"}

# Scenario -> pin mode. Everything absent is `dir`; see `scenario_pin_mode()`.
#
# SR_31's mode is `nodir` and the entry says so in one place, so that a fifth
# caller of `config_wire()` cannot reintroduce a `dir` CONFIG for it. Its
# direction-pin coverage is SR_10..SR_12's job anyway -- the subject here is how
# many steppers connect, and a direction pin per stepper would halve that count
# for the analyzer rather than for the driver.
SCENARIO_PIN_MODE = {"SR_31": MAX_COUNT_PIN_MODE}


def per_stepper_programs(scenario, info):
    """{stepper index: segments}, for scenarios needing per-stepper speeds.

    None for every other scenario: they all share one program, and the
    3-argument QSEG form is what they use.
    """
    if not needs_per_stepper(scenario):
        return None
    # Derive the slow stepper from the *clamped* fast period, not from
    # max_speed_ticks * ratio. legal_ticks() has a 160-tick floor, and the
    # ESP32's real RMT floor is 80 -- so asking for 80 and for 160 both return
    # 160, and the ratio collapses to 1.0. This scenario's entire subject is
    # that each stepper keeps *its own* period, so a builder that hands the
    # firmware two identical periods measures nothing and reports success.
    #
    # It went unnoticed because the fixture DUT's floor is 640, where the ratio
    # survives by arithmetic accident -- 640 and 1280 clear the floor. The
    # recorded result was therefore never a statement about the real chip.
    fast = legal_ticks(info, 200, info["max_speed_ticks"])
    slow = legal_ticks(info, 200, fast * SR_15_RATIO)
    return {0: [(200, fast, True)], 1: [(200, slow, True)]}


def sc_sync_independent_speeds(info):
    """SR_15: both steppers start together, then each runs at its own period.

    Returns stepper A's program only, because that is the list shape every
    scenario uses and the evaluator derives B's from SR_15_RATIO. What the test
    actually asserts is that the arm stayed aligned *and* the periods stayed
    independent -- a synchronized start that dragged both steppers onto one speed
    would satisfy the first and fail the second.
    """
    return per_stepper_programs("SR_15", info)[0]


def sc_emergency_stop(info):
    """SR_25/SR_30: a stop 1 ms into a run whose queue was filled before it.

    Two things make the measurement well defined, and neither of them was true
    of the 20000-step program this replaces.

    The queue is filled to a *reported* depth by QFILL before QRUN, with
    start = false, and the stop lands 1 ms later. So the steps that can follow
    the marker are the ones the board said it was holding, rather than whatever
    the feeder had got through in a quarter of a 20000-step program. Measured on
    rmt before this change: 7464 steps after the marker against a bound of
    8160, close enough to the queue's whole capacity that the number described
    the feeder rather than forceStop().

    The program is four segments of 4080 steps -- 16320 of them, four times the
    fill. That is what leaves room for the two opposite outcomes to be opposite:
    a forceStop() that stopped nothing emits nearly all of it, and one that
    stopped adding cannot exceed the queue's capacity however long it is left.
    """
    t = legal_ticks(info, QUEUE_FILL_STEPS, info["max_speed_ticks"])
    return [(QUEUE_FILL_STEPS, t, True)] * QUEUE_FILL_ROUNDS


def sc_rmt_buffer_split(info):
    """SR_21: a long RMT run, where a hardware buffer split can land.

    RMT V1 arms a command by splitting its hardware buffer. If the split is
    placed at the wrong step, exactly one inter-step gap comes out wrong --
    which eval_period_defects already reports, because it flags any gap outside
    tolerance rather than only the average. 200 steps rather than SR_01's 8,
    so the split boundary has somewhere to land: a split near the start of a
    short run is not distinguishable from normal first-step latency.
    """
    t = legal_ticks(info, 200, info["max_speed_ticks"])
    return [(200, t, True)]


def sc_i2s_timing(info):
    """SR_23: the I2S step output, which is buffered audio rather than a timer.

    I2S emits steps from a DMA-fed sample stream, so nothing about its timing
    comes from a compare register the way RMT and MCPWM do. That makes it worth
    a separate run: a period error here would point at the sample rate rather
    than at the ramp maths.
    """
    t = legal_ticks(info, 64, info["max_speed_ticks"])
    return [(64, t, True)]


def sc_pause_ticks_max(info):
    """SR_26: a pause of exactly 65535 ticks.

    The white paper notes the 16-bit boundary applies to a pause's tick field
    as well as to ticks*steps, and the two are separate fields -- `ticks` for a
    pause, `ticks * steps` for a move. A move at 65535 ticks is SR_04; this
    pins the pause path, which a move does not exercise. The pause is 4.0965 ms
    at 16 MHz -- 65535 ticks, not 65535 * something larger -- and it brackets
    two single steps, so the measurable gap between them is 8.1919 ms.

    Deviates from the white paper's two-segment form by adding a step *after* the
    pause. A pause is silence, and silence can only be measured between two
    pulses: with one step before the pause and none after there is no second
    edge to measure against, and the gap is unobservable rather than wrong. The
    trailing step is what makes the 16-bit boundary measurable at all.
    """
    return [(1, 65535, True), (0, 65535, True), (1, 65535, True)]


def sc_single_step(info):
    # steps == 1 is not just the low end of the SR_02 sweep: it takes the other
    # branch in the ISR (`e->steps > 1` is false, so the read pointer advances
    # and the next entry is stepped in the same interrupt), and it is the only
    # command that produces no inter-step period at all.
    return seg_period(1, legal_ticks(info, 1, info["max_speed_ticks"]))


def sc_ticks_min(info):
    """The speed floor: the *largest* ticks the firmware will accept.

    min_cmd_ticks is the floor, not max_speed_ticks. max_speed_ticks is the
    fastest legal speed (the smallest ticks), so a scenario that claims to test
    the floor while using it tests the opposite end of the range -- and lands on
    the same 640 ticks as SR_01, which is why the two were indistinguishable.
    """
    return seg_period(8, info["min_cmd_ticks"])


def sc_ticks_max(info):
    return seg_period(4, 65535)


def sc_multi_stepper_timing(info):
    """Both steppers stepping the same period, for the SR_16 comparison.

    The question SR_16 asks is not "does B step" but "does B perturb A": the
    same channel is compared between a 1ch run and this 2ch run. On AVR the
    answer is structural -- the speed floor rises once a second stepper is
    attached -- so a sweep done only at 1ch would report a speed the board
    cannot sustain with two steppers connected. The measurement that settles it
    is the period of channel A here versus channel A alone.
    """
    t = legal_ticks(info, 64, info["max_speed_ticks"])
    return [(64, t, True)]


def sc_sync_cross_driver(info):
    """Two steppers on different drivers, started together.

    RMT and MCPWM+PCNT arm through entirely different hardware, so this is the
    hardest start-alignment case: one channel is driven by a buffered
    peripheral filling symbols ahead of time, the other by a timer compare with
    a hardware counter watching the pin. The offset is reported rather than
    gated on -- see eval_sync_start and white paper 1.3.
    """
    t = legal_ticks(info, 200, info["max_speed_ticks"])
    return [(200, t, True)]


def sc_dir_change_both_ways(info):
    """Reverse phase then forward phase: the same change, both directions.

    A direction change costs a DIR drain before the next step. Doing it in both
    directions shows the drain is symmetric rather than an artefact of one
    transition, and that the dir pin returns to where it started.
    """
    t = legal_ticks(info, 20, info["max_speed_ticks"])
    return [(20, t, False), (20, t, True)]


def sc_multi_step_direction(info):
    """Three phases: forward, reverse, forward. Two direction changes."""
    t = legal_ticks(info, 10, info["max_speed_ticks"])
    return [(10, t, True), (10, t, False), (10, t, True)]


def sc_ticks_error_rejected(info):
    """A command the firmware must refuse, and emit nothing for.

    8 steps at (max_speed - 1) ticks: period * steps lands below MIN_CMD_TICKS,
    so addQueueEntry() returns ErrorTicksTooLow. The point of the test is the
    second half -- a rejected command must produce no pulse at all. A rejection
    that still stepped would move the motor by 8 steps the caller never asked
    for.
    """
    fast = max(info["max_speed_ticks"] - 1, 1)
    # Keep it below MIN_CMD_TICKS/steps, which is the actual rejection rule.
    while 8 * fast >= info["min_cmd_ticks"] and fast > 1:
        fast -= 1
    return [(8, fast, True)]


def sc_mcpwm_overrun_after_255(info):
    """255 pulses, a gap, then exactly one. The MCPWM/PCNT overrun case.

    The PCNT high limit is re-armed from the live counter value on every
    command, and a `steps == 1` command issued straight after a full 255-step
    run is where a stale or mis-computed limit shows up: the expected result is
    exactly one pulse, and a lost or duplicated pulse here is a real defect
    rather than a timing tolerance.
    """
    t = legal_ticks(info, 255, info["max_speed_ticks"])
    # The trailing single step cannot use the same period as the run. A
    # steps == 1 command is bounded by ticks alone, so it needs at least
    # MIN_CMD_TICKS -- the white paper's `QSEG 1 <max>` with max = 640 ticks is
    # refused outright by addQueueEntry(). Hence legal_ticks() here.
    t1 = legal_ticks(info, 1, info["max_speed_ticks"])
    return [(255, t, True), (0, legal_ticks(info, 1, 6400), True), (1, t1, True)]


def sc_mcpwm_overrun_boundary(info):
    """The same shape at a smaller first command, for a one-off probe.

    SR_18 pins the full 255 case; this runs the trailing single step on its own
    so a failure can be attributed to the boundary rather than to the sweep.
    """
    t = legal_ticks(info, 200, info["max_speed_ticks"])
    t1 = legal_ticks(info, 1, info["max_speed_ticks"])
    return [(200, t, True), (1, t1, True)]


def sc_pause_after_full_command(info):
    """255 pulses, a pause, then another 255.

    Complements SR_18 by making the *second* command the large one, so a limit
    left over from the pause is exercised in the other direction.
    """
    t = legal_ticks(info, 255, info["max_speed_ticks"])
    return [(255, t, True), (0, legal_ticks(info, 1, 6400), True),
            (255, t, True)]


def sc_pulse_high_time(info):
    # 16 steps is enough to measure a stable high time and still short.
    #
    # legal_ticks, not max(max_speed, 160): addQueueEntry bounds the *whole*
    # command, so 16 steps need ticks*16 >= MIN_CMD_TICKS. On rmt the floor
    # is 640 and the old `max(..., 160)` gave 16*640 = 10240 by accident; on
    # i2s_direct the floor is 80, so it gave 16*160 = 2560, below MIN_CMD_TICKS,
    # and the queue rejected the command. The scenario measured a rejected
    # command as "hardware emitted 0 of 16 steps" -- a statement about the pin
    # that was simply false. See legal_ticks().
    return seg_period(16, legal_ticks(info, 16, max(info["max_speed_ticks"],
                                                     160)))


def sc_trailing_wait(info):
    t = legal_ticks(info, 2, info["max_speed_ticks"])
    return [(2, t, True), (2, t, True)]


def sc_long_run(info):
    return seg_period(2000, legal_ticks(info, 2000,
                                         max(info["max_speed_ticks"], 160)))


def sc_queue_full(info):
    return seg_period(4000, legal_ticks(info, 4000, info["max_speed_ticks"]))


def sc_pause(info):
    # Legal for the 5-step legs: 5*t must clear MIN_CMD_TICKS, which on a driver
    # with a low speed floor the bare max_speed_ticks does not.
    t = legal_ticks(info, 5, max(info["max_speed_ticks"], 160))
    pause = min(65535, t * 20)
    return [(5, t, True), (0, pause, True), (5, t, True)]


def sc_dir_change(info):
    t = max(info["max_speed_ticks"], 160)
    return [(20, t, True), (20, t, False)]


def sc_sync_start(info):
    t = legal_ticks(info, 2000, info["max_speed_ticks"])
    return seg_period(2000, t)


def sc_max_stepper_count(info):
    """SR_31: one shared program, every stepper at the same period.

    Deliberately the same program a `scale` point sends, and deliberately *one*
    program: the question is whether each of the board's maximum steppers gets
    every step it was promised, and a per-stepper program would let a driver
    that matched its own command while dragging the others off pace look
    correct. `eval_scale` judges each stepper against this one command.

    The step count is `SCALE_STEPS` rather than something of its own, so the
    max-count run and a `scale` sweep at the same count measure the same
    waveform and a difference between them is a difference in the count.
    """
    return seg_period(SCALE_STEPS,
                      legal_ticks(info, SCALE_STEPS, info["max_speed_ticks"]))


# Scenario config -> stepper count and driver list.
#
# The driver list is explicit on every architecture, including the ones with a
# single native driver, where it simply repeats that driver (`timer` for
# AVR/SAM, `pio` for Pico). There is no "auto" entry and no shorthand, because
# the firmware now refuses an unspecified driver instead of falling back to the
# library's automatic choice -- a result that does not record which driver
# produced it characterizes nothing. `native` stands for "whatever single driver
# this architecture has", which the caller names.
CONFIGS = {
    "1ch": (1, "native"),
    "2ch": (2, "native"),
    "mcpwm": (1, ("mcpwm_pcnt",)),
    "mixed_rmt_mcpwm": (2, ("rmt", "mcpwm_pcnt")),
    "i2s": (1, ("i2s_direct",)),
    # SR_31's. The count here is the widest `nodir` allows, which is the *top of
    # the probe's search* and not what the run connects: the board picks a count
    # at or below it (`MaxCountProbe`). It has to be a CONFIGS entry anyway so
    # that the scenario table, `driver_tag()` and `Pins.for_scenario()` all keep
    # resolving a config key rather than learning about a second kind.
    "nodir_max": (MAX_STEPPERS_PER_MODE[MAX_COUNT_PIN_MODE], "native"),
}

# The pulse driver of an architecture that has only one, used to expand the
# "native" spec above. ESP32 has several, so the default here is the one the
# measured baseline was taken on and any other run passes its own.
DEFAULT_NATIVE_DRIVER = "rmt"


def per_stepper_builder(scenario):
    """(info) -> {stepper: segments} for a scenario, or None.

    A thin adapter so the runner can ask "does this scenario need per-stepper
    programs?" without knowing that per_stepper_programs() needs a QINFO dict to
    compute them -- which it does not have until the board is wired.
    """
    if not needs_per_stepper(scenario):
        return None
    return lambda info: per_stepper_programs(scenario, info)


def config_wire_drivers(drivers, pin_mode="dir"):
    """The CONFIG line for an explicit driver list and pin mode.

    The one place a CONFIG line is built from a driver list rather than a
    CONFIGS key, because the two generic modes generate their own lists:
    `scale` needs [d]*n and `sync` needs an arbitrary combination. Everything
    else goes through config_wire() so there is one grammar, not two.
    """
    return f"CONFIG {len(drivers)} {','.join(drivers)} {pin_mode}"


def config_wire_for(config, native_driver=DEFAULT_NATIVE_DRIVER,
                    pin_mode="dir"):
    """The CONFIG line for a CONFIGS key, in an explicit pin mode.

    Kept beside config_wire rather than inside it so the existing three-argument
    form -- used by run_hardware.py, sweep.py and the tests -- keeps working
    unchanged, and so a mode can ask for the same named config in `nodir`
    without either caller having to rebuild the line itself.
    """
    return config_wire_drivers(config_drivers(config, native_driver), pin_mode)


def config_drivers(config, native_driver=DEFAULT_NATIVE_DRIVER):
    """Resolve a scenario config to one driver name per stepper.

    Always exactly as many drivers as steppers: the firmware refuses a list
    whose length is not the count, because a silently reduced run makes the
    capture look like a driver problem. Asserting it here catches the mistake at
    the table instead of as an ERR on the board.
    """
    count, spec = CONFIGS[config]
    drivers = [native_driver] * count if spec == "native" else list(spec)
    assert len(drivers) == count, \
        f"{config}: {count} steppers but {len(drivers)} drivers"
    return drivers


def config_wire(config, native_driver=DEFAULT_NATIVE_DRIVER):
    """The CONFIG line this scenario sends, in the firmware's grammar."""
    drivers = config_drivers(config, native_driver)
    return f"CONFIG {len(drivers)} {','.join(drivers)} dir"


def driver_tag(config, native_driver=DEFAULT_NATIVE_DRIVER):
    """The driver(s) a scenario's CONFIG selects, as one tag component."""
    return "+".join(config_drivers(config, native_driver))


def scenario_pin_mode(scenario, default="dir"):
    """The pin mode a named scenario connects in.

    `dir` for everything but SR_31, and that is a decision rather than an
    oversight: `dir` spends two of the eight analyzer channels per stepper and
    so stops at 4, which is *below* the queue count of the driver with the most
    queues this harness drives (MCPWM/PCNT's 6 on ESP32). A `dir` max-count run
    would therefore measure the analyzer and call it the driver -- the same
    confusion `RELEASE_SCALE_PIN_MODE = "nodir"` exists to avoid, and it is why
    this is a table rather than a constant.
    """
    return SCENARIO_PIN_MODE.get(scenario, default)


def scenario_wire(scenario, native_driver=DEFAULT_NATIVE_DRIVER):
    """The CONFIG line a named scenario sends, in that scenario's own pin mode.

    The single place a scenario's wiring is spelled out, because the pin mode
    stopped being the same for all of them. A caller that builds the line itself
    from `config_wire()` gets a `dir` CONFIG for a `nodir` scenario, which the
    firmware accepts -- and which then scores stepper B against D2 instead of D1.
    """
    return config_wire_for(SCENARIOS[scenario][0], native_driver,
                           scenario_pin_mode(scenario))


def max_stepper_count_bound(driver, pin_mode=MAX_COUNT_PIN_MODE):
    """(count_max, why) for a max-count run: the top of the probe's search.

    The search starts here and walks *down*, so this only has to be an upper
    bound; the board picks the answer. It is derived from the two budgets that
    are the host's to know and from nothing else:

    - a multiplexed stepper is one bit of the 32-bit word, so it costs a slot
      and not a channel -- 32 in `nodir`, 16 in `dir`;
    - a physical stepper is on the wire, so it costs `stride` channels of the
      eight -- 8 in `nodir`, 4 in `dir`.

    What the driver can actually *allocate* is deliberately not consulted. It is
    a header constant (`QUEUES_MCPWM_PCNT` and friends), and a host table of
    them is a belief about an SDK rather than a fact about the board in front of
    it -- which is why the catalogue asks the board instead (see
    `MaxCountProbe`) and `harness.DRIVER_MAXS` only ever cross-checks a sweep.
    """
    per = CHANNELS_PER_STEPPER[pin_mode]
    if is_mux_driver(driver):
        why = (f"the 32-bit I2S mux word ({per} slot(s) per stepper); a "
               f"multiplexed stepper costs a slot and not one of the "
               f"{CHANNELS} analyzer channels, {BUS_CHANNELS} of which carry "
               f"the bus")
        return MUX_SLOT_COUNT // per, why
    why = (f"the analyzer channel budget ({CHANNELS} channels, {per} per "
           f"stepper in {pin_mode})")
    return MAX_STEPPERS_PER_MODE[pin_mode], why


class MaxCountProbe:
    """The CONFIG line and QRUN mask at the largest count the board accepts.

    How many steppers a driver drives is the one quantity in this harness that
    the host cannot know: it is a hardware fact, not a constant in a header, and
    the constants disagree in both directions. `QUEUES_MCPWM_PCNT` is 6 and the
    board really allocates six, so a host table is right -- and `harness`'s own
    `scale_bound()` says of the same constant that the number of queues which
    *run* is a different question the table cannot answer. Meanwhile the
    analyzer's channel budget can be the smaller of the two and neither one is
    the answer.

    So the count is asked for, by asking the board: descending from
    `max_stepper_count_bound()`, one CONFIG per count, and the first one that
    *actually connects that many steppers* is the maximum. Descending rather
    than ascending because the refusals are the interesting half of the answer
    -- `ERR connect step 6` at n=7 names MCPWM/PCNT's sixth queue,
    `ERR CONFIG n=9 needs 9 channels` names the analyzer -- and because it needs
    no assumption about where the limit is.

    **Acceptance is MAP, never the reply.** Measured, not hypothesized: a CONFIG
    refused partway through `connect_stepper()` leaves the steppers it did
    connect in place and `slot_count` at that partial count, so the *next* CONFIG
    short-circuits to `OK CONFIG n=6 mode=nodir already` -- success, for a
    configuration that was never established. A host that trusts the reply reads
    that as "7 is fine" and records 7 for a board running six steppers. That is
    exactly what the first `--tests SR_31` run did, and the run passed, so
    nothing downstream would have noticed. `read_map()` is the board's own count
    and the only one that cannot be stale.

    **Every attempt is preceded by a board reset**, because of the same trap: a
    failed CONFIG leaves the half-connected state that makes the next reply
    meaningless. Re-opening the serial port resets the ESP32, which is the
    mechanism `ensure_mux()` already depends on, so it is not a new assumption
    about this board. What it buys is *provenance*, not reachability -- and that
    is worth being precise about, because the obvious reading is the opposite.
    Measured on the board, with MAP-based acceptance, all three of these reach
    the same count:

    - reset between attempts: 8 refused, 7 refused, 6 CONFIGured and connected;
    - no reset: 8 refused, 7 answers `OK ... already` for the six the first
      attempt left behind, and 6 is accepted because MAP says six;
    - no reopen at all: the same, since the accepted state is a leftover.

    The difference is that without the reset the accepted state comes from a
    *refused* attempt. That is still the right answer, and only because every
    attempt is the same request -- one driver, one pin mode. Both halves of that
    are asserted. Were two attempts allowed to differ, MAP matching on count
    would be a coincidence and the reset would be load-bearing again.

    It is one CONFIG *per attempt and no more*: the accepted attempt is the run's
    own CONFIG, and `find()` hands back the reply it already has so the caller
    does not connect the same steppers twice.

    Why `nodir` and not `dir`: see `scenario_pin_mode()`.

    One consequence worth naming: a run's tag key records the `--pin-mode` and
    `--count` the *invocation* asked for, which is what every scenario in one
    run shares -- so an SR_31 result inside a `--pin-mode dir` run is tagged
    `dir` while having connected `nodir` steppers. That is the tag scheme's
    existing shape (SR_14 is tagged `1` and connects two), and the record
    itself is unambiguous: `pin_mode` and `channel_map` above come from the
    board's own MAP.
    """

    def __init__(self, driver, pin_mode=MAX_COUNT_PIN_MODE):
        self.driver = driver
        self.pin_mode = pin_mode
        self.bound, self.bound_reason = max_stepper_count_bound(driver,
                                                                 pin_mode)

    def driver_wire(self):
        """The CONFIG line to hand `ensure_mux()` before the probe runs.

        Only the driver name matters to that decision -- whether the run needs
        the multiplexer -- so the count in it is the search bound and nothing
        more. It is never sent.
        """
        return config_wire_drivers([self.driver] * self.bound, self.pin_mode)

    def wire_for(self, count):
        """The CONFIG line for `count` steppers on this driver."""
        return config_wire_drivers([self.driver] * count, self.pin_mode)

    def find(self, ser, reopen=None):
        """Search downward for the largest count the board really connects.

        `reopen()` returns a fresh serial port for the next attempt and is
        called between attempts, not before the first -- the caller has already
        opened one, and for a driver that accepts the bound outright it is never
        called at all.

        Returns `(wire, mask, detail, reply, ser)`. `ser` is the live port,
        because `reopen()` may have replaced it and the caller goes on to use it
        for the whole run. `reply` is the accepted CONFIG's own reply -- which is
        what `measure()` needs, so it does not send the line again -- or the last
        refusal, so the caller reports a `refused` with the board's own words.
        """
        detail = {
            "probed_driver": self.driver,
            "pin_mode": self.pin_mode,
            "search_bound": self.bound,
            "bound_reason": self.bound_reason,
            # Every count tried above the one accepted, with the board's reason
            # for each. This is the measurement as much as the count below it:
            # "mcpwm_pcnt stops at 6" and "rmt stops at 8 because the analyzer
            # ran out of channels" are different answers with the same number.
            "refused_above": [],
            # Set only if a CONFIG ever answered OK while connecting fewer
            # steppers than it was asked for. A firmware lie, and it belongs in
            # the record rather than in a comment: a reader seeing
            # `max_stepper_count: 6` next to `ok_but_short` can tell the
            # difference between "the board refuses 7" and "the board said yes
            # and did not".
            "ok_but_short": [],
        }
        last = None
        for count in range(self.bound, 0, -1):
            wire = self.wire_for(count)
            reply = reply_of(ser, wire)
            connected = len(read_map(ser)[0])
            last = (wire, reply)
            if connected == count:
                detail["max_stepper_count"] = count
                detail["probe_attempts"] = self.bound - count + 1
                return wire, (1 << count) - 1, detail, reply, ser
            if "OK CONFIG" in reply and connected != count:
                # Named, not folded into refused_above: the board answered yes.
                detail["ok_but_short"].append(
                    {"n": count, "reply": reply.strip(), "connected": connected})
            else:
                detail["refused_above"].append({"n": count,
                                                "reply": reply.strip()})
            if reopen is not None:
                ser.close()
                ser = reopen()
        # Nothing from the bound down to one. Report the smallest attempt rather
        # than the largest: its refusal is the one that names why even a single
        # stepper could not connect.
        #
        # `probe_attempts` and `max_stepper_count` are set on both paths, not
        # just the accepting one, because the caller reads them to *print* the
        # probe before it decides whether the run continues. A key present only
        # on success is a KeyError on exactly the run whose transcript matters
        # most.
        wire, reply = last
        detail["max_stepper_count"] = 0
        detail["probe_attempts"] = self.bound
        # `refused_above` already ends with n=1, which is the smallest attempt
        # and the refusal that names why even one stepper could not connect. No
        # extra entry for it: an n=0 line would read as a CONFIG for zero
        # steppers, which is not a thing this harness sends.
        return wire, 0, detail, reply, ser


def probe_for(scenario, driver):
    """The `MaxCountProbe` a scenario needs, or None.

    None for every scenario but the max-count one, which is the only one whose
    stepper count is not known before the board is asked. Kept beside
    `PER_STEPPER_SCENARIOS` for the same reason that table is beside SCENARIOS:
    a property of a scenario that is not one of its four table fields.
    """
    if scenario not in PROBE_SCENARIOS:
        return None
    return MaxCountProbe(driver, scenario_pin_mode(scenario))


# ---------------------------------------------------------------------------
# The two generic modes
# ---------------------------------------------------------------------------
#
# The scenario table below is a *catalogue*: hand-picked cases, each with a name
# worth keeping. The modes are the other half -- they generate runs from a rule,
# so coverage scales with the hardware instead of with someone's patience.
#
# Neither mode names an architecture. That is the point: `scale` is the whole
# cross-architecture matrix (one run on an AVR, four on an ESP32 in `dir`), and
# `sync` applies to any board that has more than one driver to choose from. A
# driver nobody has connected yet -- `i2s_mux` today -- is one flag away and
# needs no new code path, because a driver is a *name* in a list and nothing
# here switches on it.


class ModeRun:
    """One generated run: a wire, a program, an evaluator, and a label.

    Deliberately not a scenario id. A scenario is a named case that appears in
    the catalogue and in the report; a generated run is one point of a sweep,
    and giving each one an id would put 4 or 10 rows of near-identical entries
    into SCENARIOS, where `scale` at 3 steppers and `scale` at 7 steppers would
    differ by a loop variable and nothing else.
    """

    def __init__(self, label, drivers, pin_mode, builder, evaluator, mask,
                 goal, per_stepper_builder=None, steps=None, ticks=None):
        self.label = label
        self.drivers = list(drivers)
        self.pin_mode = pin_mode
        self.builder = builder
        self.evaluator = evaluator
        self.mask = mask
        self.goal = goal
        self.per_stepper_builder = per_stepper_builder
        # Recorded when a builder ignores the QINFO limits and was told the
        # period outright. None means "read it from the board", which is the
        # normal case and the only trustworthy one.
        self.steps = steps
        self.ticks = ticks

    @property
    def count(self):
        return len(self.drivers)

    @property
    def wire(self):
        return config_wire_drivers(self.drivers, self.pin_mode)

    @property
    def tag(self):
        """A filename- and tag-key-safe label. '+' and ',' would both leak.

        One name per *distinct* driver, not one per stepper. Spelling the driver
        out N times says nothing -- a `scale` run connects N of the same driver
        by construction, and `sync` connects two -- and it stops being a filename
        at all: 32 mux steppers is 32 x 8 characters, which with the tag prefix
        and the `_37ch` suffix the decoder appends overruns the 255-byte limit,
        and the capture is then silently not written.
        """
        distinct = []
        for d in self.drivers:
            if d not in distinct:
                distinct.append(d)
        return "+".join(distinct) + self.pin_mode + f"n{self.count}"


# How long a `scale` run's program is. Long enough for the period to be
# measurable several times over -- one inter-step period is a single sample
# interval at 4 MS/s and says nothing -- and short enough that 8 steppers
# still fit the analyzer's 64 MSample buffer (see white paper 10).
SCALE_STEPS = 64


def scale_plan(driver, pin_mode, count_max, steps=SCALE_STEPS, ticks=None):
    """Every count 1..count_max on one driver, all steppers in parallel.

    One shared program, so the question each run answers is the one `scale` is
    for: does this driver still emit the commanded period once N steppers are
    attached, and does each one get every step it was promised? A per-stepper
    program would answer a different question (SR_15's), and asking it here
    would conflate "does adding steppers slow it down" with "does each stepper
    keep its own speed".

    The upper bound is `count_max`, which the caller has already bounded by
    min(driver max, channel budget) -- so the plan is legal before anything is
    run, and a sweep point that the firmware would refuse never gets measured.
    """
    plan = []
    for count in range(1, count_max + 1):
        def builder(info, count=count):
            t = ticks if ticks else legal_ticks(info, steps,
                                                info["max_speed_ticks"])
            return seg_period(steps, t)

        plan.append(ModeRun(
            label=f"scale_{driver}_{pin_mode}_n{count}",
            drivers=[driver] * count,
            pin_mode=pin_mode,
            builder=builder,
            # Every stepper's own count and period, judged against the shared
            # command: that is exactly what "N in parallel on one driver" has to
            # mean, and it is the same check SR_16 makes for two steppers.
            evaluator=eval_scale,
            mask=(1 << count) - 1,
            goal=f"{count} stepper(s) on {driver} in parallel",
            steps=steps,
            ticks=ticks,
        ))
    return plan


# The per-stepper period ratios `sync` runs at, and why more than one.
#
# Adherence is only checkable if each stepper was given a *different* period:
# a start that dragged every stepper onto one speed would satisfy a first-step
# test and a shared-period test, and would be reported as a perfect sync. The
# ratios are distinct per stepper so a collapse is visible in the measurement
# rather than inferred from its absence.
SYNC_RATIOS = (1, 2, 3)


def sync_plan(drivers, pin_mode, count=2, steps=SCALE_STEPS):
    """One run per driver-list combination, each stepper at its own period.

    Every combination *with repetition* of `count` steppers drawn from
    `drivers`, so the plan contains the same-driver pairs (rmt+rmt) as well as
    the cross-driver ones (rmt+mcpwm_pcnt). That is deliberate and is the whole
    reason to enumerate rather than hand-pick: the same-driver pair is the
    baseline the cross-driver skew is only interpretable against, and R1
    recorded a finding that had exactly this shape and was wrong -- an earlier
    firmware overwrote the parsed driver list with its automatic choice, so the
    "cross-driver" run was RMT+RMT and agreed with the same-driver number to
    four decimal places. Identical numbers across supposedly different
    configurations is the signature to look for, so the plan has to contain
    both.

    `count` defaults to 2 because skew needs two steppers to have a skew at
    all. Three or more is allowed and adds combinations rather than changing
    the question.
    """
    from itertools import combinations_with_replacement

    if len(drivers) < count:
        raise ValueError(f"sync needs {count} drivers, {drivers} has "
                         f"{len(drivers)}")
    plan = []
    for combo in combinations_with_replacement(drivers, count):
        ratios = SYNC_RATIOS[:count]

        def per_stepper(info, ratios=ratios, steps=steps):
            base = legal_ticks(info, steps, info["max_speed_ticks"])
            return {i: seg_period(steps,
                                  legal_ticks(info, steps, base * r), True)
                    for i, r in enumerate(ratios)}

        # segments is stepper A's program: it sets the capture window and the
        # shared-program fields. Every stepper's real expectation goes to the
        # evaluator through the very dict that was sent to the board, so the two
        # cannot disagree about what was commanded.
        plan.append(ModeRun(
            label=f"sync_{'+'.join(combo)}_{pin_mode}_n{count}",
            drivers=list(combo),
            pin_mode=pin_mode,
            builder=lambda info, ps=per_stepper: ps(info)[0],
            evaluator=lambda ch, rt_, segs, inf, pins, ps=per_stepper:
                eval_sync(ch, rt_, segs, inf, pins, ps(inf)),
            mask=(1 << count) - 1,
            goal=f"{'+'.join(combo)}: aligned start, each at its own period",
            per_stepper_builder=per_stepper,
            steps=steps,
        ))
    return plan


# test id -> (config, segment builder, mask, human name)
SCENARIOS = {
    "SR_01": ("1ch", sc_period_exact, 1, "inter-step period equals ticks"),
    "SR_02": ("1ch", sc_steps_per_command, 1, "255 steps in one command"),
    "SR_03": ("1ch", sc_ticks_min, 1, "at the speed floor"),
    "SR_04": ("1ch", sc_ticks_max, 1, "ticks = 65535 (16-bit max)"),
    "SR_05": ("1ch", sc_pulse_high_time, 1, "pulse high time"),
    "SR_06": ("1ch", sc_trailing_wait, 1, "trailing wait after last step"),
    "SR_07": ("1ch", sc_long_run, 1, "2000 steps, no underrun"),
    "SR_08": ("1ch", sc_queue_full, 1, "4000 steps, QueueFull retry"),
    "SR_09": ("1ch", sc_pause, 1, "pause command"),
    "SR_10": ("1ch", sc_dir_change, 1, "dir change -> first step"),
    "SR_11": ("1ch", sc_dir_change_both_ways, 1,
              "reverse then forward: the drain is symmetric"),
    "SR_12": ("1ch", sc_multi_step_direction, 1,
              "forward, reverse, forward: two direction changes"),
    # SR_13 is a negative test: the command is refused and nothing is emitted.
    "SR_13": ("1ch", sc_ticks_error_rejected, 1,
              "a command below MIN_CMD_TICKS must emit nothing"),
    "SR_14": ("2ch", sc_sync_start, 3, "2 steppers, synchronized start"),
    "SR_16": ("2ch", sc_multi_stepper_timing, 3,
              "does a second stepper perturb the first?"),
    # Cross-driver start alignment: one stepper on RMT, one on MCPWM+PCNT.
    "SR_17": ("mixed_rmt_mcpwm", sc_sync_cross_driver, 3,
              "synchronized start across two different drivers"),
    "SR_15": ("2ch", sc_sync_independent_speeds, 3,
              "aligned start, then each stepper at its own period"),
    "SR_27": ("1ch", sc_single_step, 1, "single step in one command"),
    "SR_21": ("1ch", sc_rmt_buffer_split, 1,
              "long RMT run: no gap at a buffer split"),
    "SR_23": ("i2s", sc_i2s_timing, 1, "I2S step output timing"),
    "SR_26": ("1ch", sc_pause_ticks_max, 1,
              "pause of exactly 65535 ticks (16-bit pause field)"),
    "SR_25": ("1ch", sc_emergency_stop, 1,
              "STOP a quarter into the fill: it still drains"),
    # The only stop with an observable effect on this harness, and therefore the
    # only stop scenario there is. `forceStopAndNewPosition()` empties the ring,
    # so whatever a driver had already handed to its hardware keeps stepping and
    # the rest never runs -- and that count is what tells the drivers apart.
    "SR_30": ("1ch", sc_emergency_stop, 1,
              "XSTOP a quarter into the fill: it is discarded"),
    # ESP32 MCPWM/PCNT only; the overrun needs the PCNT high-limit re-arm.
    "SR_18": ("mcpwm", sc_mcpwm_overrun_after_255, 1,
              "255 steps, gap, exactly 1 (PCNT limit re-arm)"),
    "SR_19": ("mcpwm", sc_mcpwm_overrun_boundary, 1,
              "200 steps then a single step at the boundary"),
    "SR_20": ("mcpwm", sc_pause_after_full_command, 1,
              "255, pause, 255 (large command on both sides)"),
    # The only scenario that asks how *many* steppers a driver drives, and the
    # mask of 0 says why: the count is not known when this table is written, so
    # `MaxCountProbe` finds it by descending CONFIG and sets the mask. Every
    # other entry has a literal here, which is the point -- a count that could be
    # wrong is the thing this scenario exists to measure, so it must not be a
    # literal anywhere, including here. `--mode scale` reaches the same numbers
    # but is not part of the characterisation set, is not tagged per test and
    # does not appear in a matrix row as its own result, which is why this
    # scenario exists (extras/todo/README.md → Done, item 183).
    "SR_31": ("nodir_max", sc_max_stepper_count, 0,
              "the driver's own maximum stepper count, each stepper measured"),
}


# ---------------------------------------------------------------------------
# Evaluation
# ---------------------------------------------------------------------------


def check_pin_invariants(channels, rate, pins):
    """Rules that must hold for *every* capture, whatever the scenario.

    Currently one: the direction pin must never change while the step pin is
    high. A driver latches the direction on the STEP edge, so a DIR transition
    inside the pulse window can make it decode the new direction for that step,
    and the transition itself can glitch the DIR input while the coil is being
    driven. The library orders `Stepper_ToggleDirection()` before
    `Stepper_One()` precisely so the direction settles first, so any occurrence
    is a defect.

    This is checked across all steppers, on every test, rather than only in the
    direction-change scenario: a DIR edge during a high STEP would be a bug
    anywhere in the program, and a per-scenario check would only catch it in
    the one scenario that happens to change direction.

    `nodir` has no direction pin, so there is nothing to check -- `dir_of` is
    None and the stepper is skipped rather than compared against itself.
    """
    per_stepper = {}
    total = 0
    for name, step_ch in pins.items():
        dir_wave = pins.dir_wave(channels, name)
        step_wave = pins.step_wave(channels, name)
        if step_wave is None or dir_wave is None:
            continue
        conflicts = sp.dir_changes_during_step_high(dir_wave, step_wave, rate)
        if conflicts:
            per_stepper[name] = conflicts
        total += len(conflicts)
    return {
        "n_dir_while_step_high": total,
        "dir_while_step_high": per_stepper,
        "ok": total == 0,
    }


def check_periods(periods_us, ticks, info, with_adherence=False):
    """(period detail, adherence) for one stepper's inter-step periods.

    One place decides whether a period is judged against a band or against the
    driver's frame grid, because the choice has to be right at every call site and
    there are a dozen of them. Getting it wrong at one is not cosmetic:
    `period_defects` at 400 ticks reports the mux's legal 28 us step as 12 % long,
    and `rate_adherence` reports the legal 6/6/6/7 frame pattern as 16 % jitter.
    Both would fail a run that is exactly right.

    On the grid the adherence tolerance is one frame rather than 2 %, because a
    single 7-frame step is 3 us off the commanded period by construction, and no
    percentage of 25 us covers that without also covering a real rate error.

    Called once per measurement where both are needed, and twice where a site only
    needs one -- the work is a few arithmetic operations over the period list, and
    a cache would be a second thing that can disagree with the decision above.
    """
    tps = info["ticks_per_s"]
    expect_us = ticks * 1e6 / tps
    grid = info.get("frame_grid_ticks")
    if grid:
        return (sp.grid_period_defects(periods_us, ticks, tps, grid),
                sp.rate_adherence(periods_us, expect_us, tol_frac=0.0,
                                  tol_abs=grid * 1e6 / tps))
    return (sp.period_defects(periods_us, expect_us),
            sp.rate_adherence(periods_us, expect_us) if with_adherence else None)


# How close a measured inter-step gap must sit to its commanded value for the
# pair to be *placed inside the commanded move*. Deliberately looser than
# check_periods(), because this rule locates and that one judges: an anchoring
# tolerance tighter than the judging one would refuse to anchor a run that
# check_periods() then has something to say about, and the run would fall back to
# measuring the whole capture. Loose is safe here because the only thing it has
# to separate is a gap the command could have produced from the gap to a pulse
# that was never commanded at all -- which is orders of magnitude apart. A rate
# error small enough to sit inside this band is left to check_periods(), which
# judges it on its own tolerance.
MOVE_ANCHOR_TOL = 0.25
# ...and an absolute floor, because a quarter of a commanded period can be less
# than the capture's own sample quantisation.
MOVE_ANCHOR_TOL_ABS_US = 1.0


def commanded_timeline(segments, info, limit=None):
    """The timeline a program commands, in ticks since the move started.

    A segment of `steps` at `ticks` occupies `steps * ticks`, and a pause
    (`steps == 0`) occupies `ticks`. Steps land at the *start* of their own tick,
    because the last step's pulse occupies the last tick of its segment -- which
    is what makes a scenario's duration `steps * ticks` rather than
    `(steps - 1) * ticks`.

    `limit` takes the first N commanded steps, which is what the evaluators that
    judge one segment's worth of steps need: they assert against
    `segments[0][0]` steps, so the window has to be the same N steps and not the
    whole program.
    """
    offsets = []
    last_ticks = segments[0][1] if segments else 0
    cursor = 0
    for steps, ticks, _up in segments:
        if steps > 0:
            want = steps if limit is None else min(steps, limit - len(offsets))
            for k in range(max(0, want)):
                offsets.append(cursor + k * ticks)
                # The last step's own tick count, which is the move's final
                # pulse duration and the last step *inside* the limit rather
                # than the last step of the program when the caller asked for
                # fewer.
                last_ticks = ticks
        if limit is not None and len(offsets) >= limit:
            break
        cursor += (steps if steps > 0 else 1) * ticks
    gaps = [offsets[i + 1] - offsets[i] for i in range(len(offsets) - 1)]
    return {
        "steps_ticks": offsets,
        "gaps_ticks": gaps,
        "last_ticks": last_ticks,
        "span_ticks": (offsets[-1] - offsets[0]) if len(offsets) > 1 else 0,
    }


def gap_is_legal(measured_us, gap_ticks, info):
    """Is this measured gap the one the command asked for, closely enough to
    place the move?

    On a grid the legal gaps are whole frames and the quarter-frame slack is the
    one `grid_period_defects()` uses, so a step can only be placed on a frame
    boundary its command could have produced. Off a grid it is a quarter of the
    commanded gap, and loose on purpose: it has to tell a gap the command could
    have produced from the gap to a pulse that was never commanded, which are
    orders of magnitude apart, and it must not refuse to anchor a driver whose
    steps arrive late. An ISR-driven architecture does sag systematically
    (`rate_adherence` measures the sag); `check_periods()` is what judges it, on
    its own tolerance.
    """
    tps = info["ticks_per_s"]
    grid = info.get("frame_grid_ticks")
    if grid:
        frame_us = grid * 1e6 / tps
        q = gap_ticks // grid
        frames = [q] if gap_ticks % grid == 0 else [q, q + 1]
        return min(abs(measured_us - f * frame_us) for f in frames) \
            <= frame_us * 0.25
    want_us = gap_ticks * 1e6 / tps
    return abs(measured_us - want_us) <= max(want_us * MOVE_ANCHOR_TOL,
                                             MOVE_ANCHOR_TOL_ABS_US)


def move_window(edges, rate, info, segments, expected):
    """(the step edges of the commanded move, what was outside it).

    A capture is not the move. It starts before the test is triggered over
    serial and outlives it, so it holds however long the host took to send the
    command and however long the board idled afterwards -- on the recorded
    `i2s_mux` `dir` captures, 285 ms of it before the first commanded step.
    Nothing the driver did is in that stretch, and counting it anyway reports a
    driver that emitted steps it did not emit, which is the failure this
    harness exists to catch. The plan fixes where the move is: no ramp is in
    the way (`addQueueEntry()` is driven directly), so the program says
    exactly how many steps come out and how far apart.

    The window is the **first contiguous run of `expected` steps whose gaps all
    match the command**. First, because an unexplained pulse next to the move
    cannot be told apart from the move's own first step by the gaps that follow
    it, and the earliest run that matches the command is the one the plan
    described; a pulse *after* the run is inside the move's span and is counted.

    Three gaps decide the search -- the first, the middle and the last -- and
    only a candidate that passes all of them is checked gap by gap. A window
    that begins on an unexplained pulse fails the very first gap by orders of
    magnitude, so the probe rejects it in constant time and the full check runs
    on the one candidate that can be the move.

    **Fails open.** No window found means the capture is measured whole, exactly
    as it was before this existed, and the record says so: a driver running at
    the wrong rate cannot place its own move, and the whole-capture measurement
    is the one that reports it.

    **The one case it cannot decide, resolved conservatively.** A pulse exactly
    one commanded period ahead of the move's first step has the same gaps behind
    it as the move does, so nothing in the waveform tells the two apart. The
    window takes the *earliest* candidate, which absorbs that pulse into the
    move -- and a move then holds one step more than the plan commands, which
    fails. That is the intended outcome: the capture holds a pulse the harness
    cannot account for, and a run that has seen one does not report the driver
    as clean on the strength of which of two equally-plausible readings it
    happened to prefer. A pulse further out, whose gap to the move is not one
    the command could produce, is outside the window and is recorded.
    """
    us = 1e6 / rate
    rec = {
        "expected_steps": expected,
        "measured_steps": len(edges),
        "anchored": False,
    }
    if expected <= 0 or len(edges) < expected:
        # Nothing to place: fewer steps than the plan commands is the
        # measurement, and the evaluators judge it.
        rec["steps_in_window"] = len(edges)
        rec["reason"] = ("the capture holds {} of the {} commanded steps, so "
                         "there is no move to place".format(len(edges), expected))
        return list(edges), rec

    tl = commanded_timeline(segments, info, limit=expected)
    gaps_ticks = tl["gaps_ticks"]
    if not gaps_ticks:
        return list(edges), rec
    n = expected - 1
    probe = sorted({0, n // 2, n - 1})

    start = None
    for j in range(len(edges) - expected + 1):
        ok = True
        for i in probe:
            g = (edges[j + i + 1] - edges[j + i]) * us
            if not gap_is_legal(g, gaps_ticks[i], info):
                ok = False
                break
        if not ok:
            continue
        if all(gap_is_legal((edges[j + i + 1] - edges[j + i]) * us,
                            gaps_ticks[i], info) for i in range(n)):
            start = j
            break

    if start is None:
        rec["steps_in_window"] = len(edges)
        rec["reason"] = ("no run of {} steps matches the commanded gaps; the "
                         "whole capture was measured".format(expected))
        return list(edges), rec

    first = edges[start]
    end = edges[start + expected - 1] + round(tl["last_ticks"] * rate
                                              / info["ticks_per_s"])
    # The move ends when its last commanded pulse ends, one commanded tick later
    # -- but a pulse that *continues* the move's rhythm past that is still the
    # driver stepping, so the boundary must not be drawn where such a pulse
    # lands. Measured on IDF 5.5.3: `mcpwm_pcnt` emits one step more than it was
    # given, at the commanded period, immediately after the run (the MCPWM/PCNT
    # overrun, `extras/doc/platforms/esp32.md`), and
    # at 24 MS/s that lands on the tick boundary to within a sample. So the same
    # rule the anchor uses is applied forwards: the window extends through every
    # edge whose gap from the one before it is one the command could have
    # produced. A pulse further out has no such gap -- 12 ms of idle in a mux
    # `dir` capture, 60 ms on a GPIO one -- and stays outside, recorded.
    last_gap = gaps_ticks[-1]
    i = start + expected
    while i < len(edges) and gap_is_legal((edges[i] - edges[i - 1]) * us,
                                          last_gap, info):
        i += 1
        end = edges[i - 1] + round(tl["last_ticks"] * rate / info["ticks_per_s"])
    inside = [e for e in edges if first <= e <= end]
    before = [e for e in edges if e < first]
    after = [e for e in edges if e > end]
    rec.update({
        "anchored": True,
        # The move's first step as a sample index, which is what the skew is
        # measured on: the earliest edge in the capture is not necessarily the
        # first step of the move.
        "first_step_sample": first,
        "window_start_us": round(first * us, 4),
        "window_end_us": round(end * us, 4),
        "commanded_span_us": round(tl["span_ticks"] * 1e6 / info["ticks_per_s"], 4),
        "steps_in_window": len(inside),
        "steps_outside": len(before) + len(after),
        # Offsets from the move's own start, so a pre-move pulse reads as how
        # far before the move it landed rather than as a wall-clock time.
        "outside_before_us": [round((e - first) * us, 4) for e in before[:16]],
        "outside_after_us": [round((e - first) * us, 4) for e in after[:16]],
    })
    return inside, rec


def moved_edges(wave, rate, info, segments, expected=None):
    """(step edges inside the commanded move, the window record).

    `expected` defaults to every step the program commands.
    """
    edges = sp.rising_edges(wave)
    if expected is None:
        expected = sum(n for n, _, _ in segments)
    return move_window(edges, rate, info, segments, expected)


def moved_metrics(wave, rate, info, segments, expected=None):
    """`channel_metrics` with the step count and periods of the move alone.

    The pin's own numbers -- pulse widths, duty, the idle level -- stay
    properties of the whole capture, because they are: a step is as wide before
    the move as during it. The count and the inter-step periods are not, so
    those two come from the window and the rest is untouched.
    """
    edges = sp.rising_edges(wave)
    if expected is None:
        expected = sum(n for n, _, _ in segments)
    inside, rec = move_window(edges, rate, info, segments, expected)
    m = sp.channel_metrics(wave, rate, rising=edges)
    em = sp.edge_metrics(inside, rate)
    avg = (sum(em.inter_step_us) / len(em.inter_step_us)
           if em.inter_step_us else 0.0)
    return dataclasses.replace(
        m,
        step_count=em.step_count,
        inter_step_us=em.inter_step_us,
        frequency_hz=(1e6 / avg) if avg > 0 else 0.0,
    ), rec

def evaluate(evaluator, channels, rate, segments, info, chan_map=None,
             extra=None, frame_grid_ticks=None):
    """Run an evaluator, then the global invariants.

    `evaluator` is an EVALUATORS key or a callable, so the two generic modes
    can pass their own without registering a scenario id per generated run --
    `scale` at 7 steppers is the same measurement as at 3, and giving it its own
    id would be 8 rows in SCENARIOS that differ only in a loop variable.

    Every result carries the invariant block, and a violation fails the test
    regardless of what the scenario's own checks concluded.

    `chan_map` is the board's own channel map, and the evaluator is given a
    `Pins` built from it rather than reading a module-level table. Omitting it
    falls back to the 4-stepper `dir` map, which is what every named catalogue
    scenario connects -- the fallback exists for those, not as the harness's
    idea of a channel map.

    `extra` is forwarded to an evaluator that needs more than the four
    standard arguments -- `eval_sync` needs the per-stepper programs it was
    sent, so its expectations cannot drift from what went on the wire.

    `frame_grid_ticks` says the run's step instants are quantised to a frame of
    that many ticks -- the I2S multiplexer, whose pulse is one frame high and
    can only start on a frame boundary. It is also placed in `info`, which is
    where an evaluator that wants it will look: threading a sixth argument through
    every evaluator to reach one of them is the kind of plumbing that makes the
    next evaluator forget it exists.
    """
    if frame_grid_ticks is not None:
        info = dict(info, frame_grid_ticks=frame_grid_ticks)
    pins = Pins(chan_map) if chan_map is not None else Pins.default()

    # A capture that does not carry every stepper the board connected cannot
    # answer the question. Checked here rather than in each evaluator because
    # every one of them would have to get it right, and one that skipped the
    # missing stepper would *pass* -- reporting a result for a stepper nothing
    # was measured about.
    missing = pins.missing(channels)
    if missing:
        wanted = [pins.step_of(letter) for letter in missing]
        return False, {
            "incomplete_capture": {
                "missing_steppers": missing,
                "missing_channels": wanted,
                "channel_map": pins.map,
                "captured": sorted(channels),
            },
            "invariants": {"ok": False, "n_dir_while_step_high": 0,
                           "dir_while_step_high": {},
                           "skipped": "capture lacks a connected stepper"},
        }

    fn = evaluator if callable(evaluator) else EVALUATORS[evaluator]
    args = (channels, rate, segments, info, pins)
    ok, detail = fn(*args, extra) if extra is not None else fn(*args)
    inv = check_pin_invariants(channels, rate, pins)
    detail = dict(detail)
    detail["invariants"] = inv
    return ok and inv["ok"], detail


def eval_period_exact(channels, rate, segments, info, pins):
    """Inter-step period must equal the commanded ticks, in microseconds."""
    ticks = segments[0][1]
    expect_us = ticks * 1e6 / info["ticks_per_s"]
    m, win = moved_metrics(pins.step_wave(channels, "A"), rate, info,
                           segments, segments[0][0])
    counts = sp.step_count_defects(m.step_count, segments[0][0])
    # ISR-driven architectures set the step pin from inside a timer interrupt,
    # so the achieved rate is systematically below the commanded one. That is
    # invisible to a step count and to a gross-period check.
    detail, adherence = check_periods(m.inter_step_us, ticks, info,
                                      with_adherence=True)
    return detail["ok"] and counts["ok"] and adherence["ok"], {
        "ticks": ticks,
        "ticks_per_s": info["ticks_per_s"],
        "period": detail,
        "steps": counts,
        "window": win,
        "adherence": adherence,
    }


def eval_step_count(channels, rate, segments, info, pins):
    ticks = segments[0][1]
    expect_us = ticks * 1e6 / info["ticks_per_s"]
    n = sum(steps for steps, _, _ in segments)
    step = pins.step_wave(channels, "A")
    m, win = moved_metrics(step, rate, info, segments, n)
    counts = sp.step_count_defects(m.step_count, n)
    detail, _ = check_periods(m.inter_step_us, ticks, info)
    return counts["ok"] and detail["ok"], {
        "ticks": ticks,
        "steps": counts,
        "window": win,
        "period": detail,
    }


def eval_scale(channels, rate, segments, info, pins):
    """`scale`: every stepper's own count and period, against the shared command.

    The assertion each stepper must satisfy is its own: the exact number of
    steps it was promised, and the exact period it was given. Judging all N
    against one another instead would let a driver that emitted the right
    number of steps on the wrong pin pass, which is the failure mode a
    channel-map mix-up produces.

    No threshold on how close together the steppers are. N steppers at one
    period *should* agree -- that is the claim -- but the tolerance that would
    make it an assertion has to come from the pulse timing the driver actually
    emits (RMT holds 15.54 us, I2S 2 us, measured), which is not known before
    the run. So each is asserted against its own command and the spread is
    reported, and the report is where a change in it is noticed.
    """
    t = segments[0][1]
    expect_us = t * 1e6 / info["ticks_per_s"]
    expected = sum(n for n, _, _ in segments)
    per_stepper = {}
    means = []
    ok = True
    for letter, ch_name in pins.items():
        if ch_name not in channels:
            continue
        m, win = moved_metrics(channels[ch_name], rate, info, segments,
                               expected)
        counts = sp.step_count_defects(m.step_count, expected)
        # A multiplexed stepper can only be stepped on a frame boundary, so its
        # period is one of two values rather than one; check_periods() reads the
        # grid out of `info` and judges it accordingly.
        detail, _ = check_periods(m.inter_step_us, t, info)
        ok = ok and counts["ok"] and detail["ok"]
        mean = (sum(m.inter_step_us) / len(m.inter_step_us)
                if m.inter_step_us else None)
        if mean is not None:
            means.append(mean)
        per_stepper[letter] = {
            "channel": ch_name,
            "steps": counts,
            "window": win,
            "period": detail,
            "mean_period_us": round(mean, 4) if mean is not None else None,
        }
    return ok and bool(per_stepper), {
        "ticks": t,
        "expected_steps": expected,
        "stepper_count": len(per_stepper),
        "per_stepper": per_stepper,
        # How far apart the steppers' mean periods are. Reported, not gated:
        # see the docstring.
        "period_spread_us": round(max(means) - min(means), 4)
                            if len(means) > 1 else 0.0,
    }


def skew_of_first_steps(firsts, rate):
    """Skew in us across the steppers' first steps, given as sample indices.

    max - min, so one straggler is not hidden by the others being close
    together.

    Takes the first steps rather than the channels: the first step of a capture
    is not the first step of the move, and on a capture whose pre-move idle
    holds an unexplained pulse the difference is milliseconds.
    """
    if len(firsts) < 2:
        return 0.0
    return (max(firsts.values()) - min(firsts.values())) * 1e6 / rate


def eval_sync(channels, rate, segments, info, pins, programs):
    """`sync`: first-step skew reported, per-stepper period adherence asserted.

    Two questions, and they are deliberately not the same check.

    **Skew is reported, never gated.** How closely several steppers actually
    begin is a property of the pulse driver and of what the processor happens
    to be doing at that instant, not a correctness property of the queue. RMT
    and MCPWM arm unrelated hardware; PCNT and an ISR-based driver step from an
    interrupt, so their offset grows with interrupt latency. A stepper that
    starts a few microseconds late has not malfunctioned, and a pass/fail on it
    would report a platform characteristic as a defect. The number is given in
    microseconds *and* in step periods, because on its own it is meaningless: a
    skew of 29 us is three quarters of a period at 640 ticks and three
    thousandths of one at 65535.

    **Adherence is asserted, per stepper, against its own commanded period.**
    `programs` is the *same dict the plan sent to the board*, so the
    expectation cannot drift from the command -- and because the plan gave each
    stepper a distinct period, a synchronized start that dragged them all onto
    one speed shows up as every stepper measuring at some *other* stepper's
    period. That is the whole reason the periods differ; checking them against
    a common expectation would pass exactly the case this exists to catch.

    The step counts are asserted too, on the same reasoning as SR_14: a
    swallowed or spurious step is a real defect on any platform.

    Every stepper is measured against **its own** program, including for the
    window: stepper A at 400 ticks and stepper B at 800 cannot be placed by the
    same gaps, and a shared one would anchor B on nothing and report its whole
    capture.
    """
    ok = True
    per_stepper = {}
    firsts = {}
    for letter, ch_name in pins.items():
        idx = pins.index_of(letter)
        wave = pins.step_wave(channels, letter)
        if wave is None or idx not in programs:
            continue
        segs = programs[idx]
        ticks = segs[0][1]
        expected = sum(n for n, _, _ in segs)
        expect_us = ticks * 1e6 / info["ticks_per_s"]
        m, win = moved_metrics(wave, rate, info, segs, expected)
        counts = sp.step_count_defects(m.step_count, expected)
        period, _ = check_periods(m.inter_step_us, ticks, info)
        ok = ok and counts["ok"] and period["ok"]
        if win["anchored"]:
            firsts[letter] = win["first_step_sample"]
        per_stepper[letter] = {
            "channel": ch_name,
            "ticks": ticks,
            "steps": counts,
            "period": period,
            "window": win,
            "mean_period_us": round(sum(m.inter_step_us)
                                    / len(m.inter_step_us), 4)
                              if m.inter_step_us else None,
        }
    skew_us = skew_of_first_steps(firsts, rate)
    return ok and bool(per_stepper), {
        "per_stepper": per_stepper,
        "first_step_skew_us": round(skew_us, 4),
        "first_step_us": {k: round(v * 1e6 / rate, 4) for k, v in
                          firsts.items()},
        # The ratio column is what makes the microseconds comparable. A skew is
        # meaningless without the period it is measured against.
        "skew_periods": round(
            skew_us / (per_stepper["A"]["mean_period_us"] or 1), 4)
            if "A" in per_stepper and per_stepper["A"]["mean_period_us"]
            else None,
    }


def eval_multi_stepper_periods(channels, rate, segments, info, pins):
    """Every stepper's own step count and period, for the SR_16 comparison.

    Asserts each stepper independently against the commanded period, and
    reports each one's mean and worst period so the two runs SR_16 compares --
    one stepper alone, then two -- can be set side by side. Whether attaching a
    second stepper perturbs the first is answered by comparing those two
    reports, not by a threshold here: the second stepper has to be present for
    the question to mean anything, so a within-tolerance check on both is the
    only thing that can be asserted in a single capture.
    """
    t = segments[0][1]
    expect_us = t * 1e6 / info["ticks_per_s"]
    expected = sum(n for n, _, _ in segments)
    per_stepper = {}
    ok = True
    for letter, ch_name in pins.items():
        if ch_name not in channels:
            continue
        m, win = moved_metrics(channels[ch_name], rate, info, segments,
                               expected)
        counts = sp.step_count_defects(m.step_count, expected)
        detail, _ = check_periods(m.inter_step_us, t, info)
        ok = ok and counts["ok"] and detail["ok"]
        per_stepper[letter] = {
            # Which channel this stepper was read on, so the record says the map
            # it used instead of leaving the reader to assume one.
            "channel": ch_name,
            "steps": counts,
            "period": detail,
            "window": win,
            "mean_period_us": round(sum(m.inter_step_us) / len(m.inter_step_us), 4)
                              if m.inter_step_us else None,
            "min_period_us": round(min(m.inter_step_us), 4) if m.inter_step_us else None,
            "max_period_us": round(max(m.inter_step_us), 4) if m.inter_step_us else None,
        }
    return ok and bool(per_stepper), {"ticks": t, "per_stepper": per_stepper}


def eval_direction_phases(channels, rate, segments, info, pins):
    """Step count per phase, and the dir pin's level for each.

    For SR_11 and SR_12 the claim is not about timing but about the dir pin
    tracking the commanded direction through every phase, and about the step
    count being exactly what was asked for. Both are asserted with no
    tolerance: a phase that steps the wrong number of times, or a dir pin that
    does not reach its commanded level, is a defect rather than a statistic.

    The direction is checked per phase and not only at the end of the capture.
    A final level plus a phase split constrains the counts but not the
    directions: a toggle that lands inside a phase leaves every count correct
    while the steps of that phase went out backwards, which is the failure a
    DIR drain that is shorter than the driver's read-ahead produces.
    """
    step = pins.step_wave(channels, "A")
    dir_ch = pins.dir_wave(channels, "A")
    expected_steps = sum(steps for steps, _, _ in segments)
    # Only the steps of the move are phases. An unexplained pulse in the
    # capture's pre-move idle is not a direction phase, and letting it in would
    # add a phase the plan never commanded.
    rises, win = moved_edges(step, rate, info, segments, expected_steps)
    counts = sp.step_count_defects(len(rises), expected_steps)

    # One phase per commanded segment, split at *every* dir change rather than
    # only at a rise: reverse-then-forward has a single rising edge but two
    # phases, and the first phase starts at the beginning of the capture.
    # Boundaries are the dir edges; a step on the same sample as an edge belongs
    # to the new phase, because the direction is set before the step is emitted.
    edges = sp.detect_edges(dir_ch)
    bounds = [e for e, _lvl in edges]
    phases = []
    # len(bounds) edges cut the capture into len(bounds) + 1 regions: the run
    # before the first edge, each run between edges, and the run after the last.
    for i in range(len(bounds) + 1):
        start = 0 if i == 0 else bounds[i - 1]
        end = bounds[i] if i < len(bounds) else len(step)
        phases.append(sum(1 for r in rises if start <= r < end))

    want_phases = sum(1 for n, _, _ in segments if n > 0)

    # Which direction each phase actually ran at. The phase split above already
    # forces the step *counts* to line up, but a phase can have the right count
    # and the wrong direction -- and POS cannot see that at all, because the
    # queue counts what it was told to do, not what the pin emitted. A driver
    # that toggles DIR while encoding rather than while playing lands here: the
    # edge moves into the middle of a phase, the counts still add up, and the
    # steps went out backwards.
    want_dirs = [1 if d else 0 for n, _, d in segments if n > 0]
    got_dirs = []
    dir_stable = True
    for i in range(len(bounds) + 1):
        start = 0 if i == 0 else bounds[i - 1]
        end = bounds[i] if i < len(bounds) else len(step)
        rs = [r for r in rises if start <= r < end]
        if not rs:
            continue
        levels = sorted({int(dir_ch[r]) for r in rs})
        if len(levels) != 1:
            dir_stable = False
        got_dirs.append(levels[0])
    dir_per_phase_ok = dir_stable and got_dirs == want_dirs

    # The dir level at the end has to match the last commanded direction.
    final_dir = dir_ch[-1]
    want_final = 1 if segments[-1][2] else 0
    dir_ok = (final_dir == want_final)

    # Regions with no steps are not phases of the motion. On hardware the dir
    # pin can settle to its starting level before the first step, which adds a
    # leading empty region; dropping empty regions keeps that from reading as a
    # defect while still requiring one non-empty region per commanded segment.
    phases = [p for p in phases if p > 0]
    want_counts = [n for n, _, _ in segments if n > 0]
    per_phase_ok = bool(phases) and phases == want_counts

    return (counts["ok"] and per_phase_ok and dir_ok and dir_per_phase_ok), {
        "steps": counts,
        "window": win,
        "phases": len(phases),
        "phases_expected": want_phases,
        "steps_per_phase": phases,
        "expected_steps": [n for n, _, _ in segments],
        "dir_edges": len(edges),
        "dir_per_phase": got_dirs,
        "expected_dir_per_phase": want_dirs,
        "dir_stable_within_phase": dir_stable,
        "final_dir": int(final_dir),
        "expected_final_dir": want_final,
    }


def eval_nothing_emitted(channels, rate, segments, info, pins):
    """SR_13: a rejected command must produce no pulse at all.

    The inverse of every other test in the suite. Here a non-empty capture is
    the failure, so this asserts the absence of steps rather than their count.
    It is the only test that checks the firmware refuses rather than that it
    performs, and a rejection that still stepped would move the motor by steps
    the caller never asked for.
    """
    step = pins.step_wave(channels, "A")
    rises = sp.rising_edges(step)
    expected_steps = sum(steps for steps, _, _ in segments)
    return len(rises) == 0, {
        "steps_measured": len(rises),
        "steps_that_would_have_been": expected_steps,
        "rejected_ticks": segments[0][1],
        "min_cmd_ticks": info["min_cmd_ticks"],
        "command_rate_ticks": segments[0][1] * max(expected_steps, 1),
    }


def eval_counts_and_gap(channels, rate, segments, info, pins):
    """Exact step count plus the presence of the commanded pause.

    Used for the phase-shaped scenarios (255 / gap / 1). The count is the point:
    these tests exist to catch a lost or duplicated pulse at a command
    boundary, and a count that is off by one is a defect with no tolerance.
    The inter-step period check is deliberately not applied across the pause,
    which is a stretch of silence and not a period.
    """
    expected = sum(steps for steps, _, _ in segments)
    m, win = moved_metrics(pins.step_wave(channels, "A"), rate, info, segments,
                           expected)
    counts = sp.step_count_defects(m.step_count, expected)
    pause_us = None
    gap_ok = True
    for steps, ticks, _up in segments:
        if steps == 0:
            period = segments[0][1]
            pause_us = (period + ticks) * 1e6 / info["ticks_per_s"]
            gap_ok = any(abs(w - pause_us) <= pause_us * 0.05 + 1.0
                         for w in m.inter_step_us)
            break
    return counts["ok"] and gap_ok, {
        "steps": counts,
        "window": win,
        "pause_found": gap_ok,
        "expected_gap_us": round(pause_us, 4) if pause_us else None,
    }


def eval_pulse_width(channels, rate, segments, info, pins):
    """The primary characterization output: high time and duty at one speed."""
    ticks = segments[0][1]
    expect_us = ticks * 1e6 / info["ticks_per_s"]
    m, win = moved_metrics(pins.step_wave(channels, "A"), rate, info,
                           segments, segments[0][0])
    counts = sp.step_count_defects(m.step_count, segments[0][0])
    detail, adherence = check_periods(m.inter_step_us, ticks, info,
                                      with_adherence=True)
    return counts["ok"] and detail["ok"] and adherence["ok"], {
        "ticks": ticks,
        "ticks_per_s": info["ticks_per_s"],
        "expected_period_us": round(expect_us, 4),
        "min_pulse_high_us": round(min(m.high_widths_us), 4)
                             if m.high_widths_us else None,
        "avg_high_us": round(m.avg_high_us, 4),
        "avg_low_us": round(m.avg_low_us, 4),
        "duty_percent": round(m.duty_cycle_percent, 2),
        "frequency_hz": round(m.frequency_hz, 2),
        "steps": counts,
        "window": win,
        "period": detail,
        "adherence": adherence,
    }


def eval_independent_speeds(channels, rate, segments, info, pins):
    """SR_15: aligned start, independent periods.

    Checks each stepper against *its own* commanded period, not against a
    common one. That distinction is the whole test: a start that pulled both
    steppers onto a single speed would look fine if the expectation were shared,
    and this is the scenario that would catch it.

    The first-step skew is measured and reported, not gated on, for the same
    reason as SR_14 and SR_17 -- how closely two drivers arm is a property of
    the hardware, not a correctness property of the queue. What must hold is
    that each stepper then keeps the speed it was given.
    """
    per = per_stepper_programs("SR_15", info)
    ok = True
    detail = {}
    for letter, idx in (("A", 0), ("B", 1)):
        ch_name = pins.step_of(letter)
        if ch_name not in channels:
            continue
        ticks = per[idx][0][1]
        expect_us = ticks * 1e6 / info["ticks_per_s"]
        expected = sum(n for n, _, _ in per[idx])
        m, win = moved_metrics(channels[ch_name], rate, info, per[idx],
                               expected)
        counts = sp.step_count_defects(m.step_count, expected)
        period, _ = check_periods(m.inter_step_us, ticks, info)
        ok = ok and counts["ok"] and period["ok"]
        detail[letter] = {
            "ticks": ticks,
            "steps": counts,
            "period": period,
            "window": win,
            "mean_period_us": round(sum(m.inter_step_us)
                                    / len(m.inter_step_us), 4)
                              if m.inter_step_us else None,
        }

    skew = None
    # The move's first step, not the first edge in the capture.
    firsts = {k: v["window"]["first_step_sample"] for k, v in detail.items()
              if isinstance(v, dict) and v["window"]["anchored"]}
    if len(firsts) > 1:
        skew = round(skew_of_first_steps(firsts, rate), 4)
    detail["first_step_skew_us"] = skew
    # Also in step periods. A skew of 29 us means nothing on its own -- it is
    # three quarters of a period at 640 ticks and three thousandths of one at
    # 65535, and only the ratio says whether the drivers actually started
    # together.
    period_us = per[0][0][1] * 1e6 / info["ticks_per_s"]
    detail["skew_periods"] = round(skew / period_us, 4) if skew and period_us \
        else None
    detail["speed_ratio"] = SR_15_RATIO
    return ok and len(detail) >= 2, detail


def requested_steps(segments):
    """Steps the program asks for in total.

    Not `segments[0][0]`: the stop scenarios program four identical segments, and
    reading the first would compare 16320 emitted steps against 4080 requested
    ones and call a complete run a four-fold oversupply.
    """
    return sum(steps for steps, _, _ in segments)


def eval_stop_move_contract(channels, rate, segments, info, pins,
                           marker_channel=None):
    """SR_25: `stopMove()` must NOT truncate already-queued motion.

    This is the opposite assertion to SR_30's, and it is the reason the two are
    separate scenarios rather than one with a flag. `stopMove()` only sets a
    flag for the ramp generator to consult when it asks for its *next* command;
    motion already in the queue is meant to run to completion. A harness that
    called it, then reported "the stop was ignored" when every step came out,
    would be calling the documented behaviour a defect.

    Earlier this scenario asserted the reverse -- that steps cease -- because the
    harness's STOP was a conflation: `stopMove()` *plus* zeroing the feeder
    cursor. That hybrid is a partial `forceStop()` wearing stopMove()'s name,
    and the residue it left (7655 steps on i2s_direct, 7608 on rmt, against a
    queue holding 32 * 255 = 8160) was the harness's own arithmetic rather than
    any guarantee the library makes.

    So the measurement here is that *nothing* was cut, witnessed on the marker
    channel rather than inferred from where the pulses ended.

    The expectation is the fill, not the program. QFILL queues `filled` entries
    and QRUN adds nothing afterwards, so the run is exactly `filled * 255` steps
    however long the program behind it is, and a stopMove() that truncated
    anything would leave a shortfall of whole entries that nothing else
    explains. Four queue-fills of program are still programmed: a run *longer*
    than the fill is then the detectable defect -- something queued after the
    start -- rather than something the harness has to be trusted not to have
    done.
    """
    # Through Pins, not `channels["D0"]`. A multiplexed stepper's step signal is
    # on no channel at all -- it is a bit of the I2S word, decoded to S<slot> --
    # so a hardcoded D0 raises KeyError on every mux run instead of measuring it.
    ch = pins.step_wave(channels, "A")
    m = sp.channel_metrics(ch, rate)
    t = segments[0][1]
    period, _ = check_periods(m.inter_step_us, t, info)
    steps = sp.rising_edges(ch)
    requested = requested_steps(segments)
    filled = info.get("queue_filled_entries", 0)

    base = {"requested_steps": requested, "steps_measured": len(steps),
            "steps": sp.step_count_defects(m.step_count, len(steps)),
            "period": period,
            # The depth QFILL reported, which is the whole run: QRUN on a filled
            # queue adds nothing after the start.
            "queue_filled_entries": filled}
    marker_at = _marker_edge(channels, rate, marker_channel)
    if marker_at is None:
        base["stop_measured"] = False
        base["reason"] = "no marker edge: the stop instant is not on the waveform"
        return False, base
    after = [i for i in steps if i > marker_at]
    before = len(steps) - len(after)
    # The whole run is the fill, and everything the marker interrupted came out
    # of it. One entry of slack below, for a driver that drops the entry a stop
    # landed inside.
    fill = filled * 255
    floor = max(0, fill - before - 255)
    # Same guard as SR_30: a marker ahead of the first pulse did not interrupt
    # anything, and a run that totals the fill without being interrupted is not
    # a measurement of stopMove().
    interrupted = 0 < before < fill and len(after) > 0
    base.update({"stop_measured": True, "marker_channel": marker_channel,
                 "steps_before_stop": before,
                 "steps_after_stop": len(after),
                 "filled_steps": fill,
                 # Nothing was cut: the queue drained, and the run is the fill.
                 "not_truncated": len(steps) >= fill,
                 # Reported: the marker interrupted the queue, and this is what
                 # was still in it at that instant.
                 "queued_at_stop_estimate": max(0, fill - before),
                 # And nothing beyond it: anything more was queued after the
                 # start, which stopMove() does not permit either.
                 "nothing_added_after_start": len(steps) <= fill})
    base["stop_interrupted_the_run"] = interrupted
    ok = (period["ok"] and interrupted and len(steps) >= floor
          and len(steps) <= fill
          and len(after) >= max(0, fill - before - 255))
    return ok, base


# How many steps may follow the marker before the queue counts as discarded.
# Two entries: the one the driver was executing when the stop landed, plus one it
# may already have handed to its hardware. Anything beyond that is a driver
# stepping commands nothing can reach any more, and it is the number that tells
# the drivers apart -- mcpwm_pcnt programs the next compare per entry, so it has
# nothing in flight, while a driver with its own transmit buffer can.
ABORT_TAIL_ENTRIES = 2


def eval_abort_queue(channels, rate, segments, info, pins, marker_channel=None):
    """SR_30: `forceStopAndNewPosition()` empties the queue.

    The third stop, and the only one whose waveform differs from the other two.
    `stopMove()` (SR_25) leaves the queue to drain, 4080 of 4080 steps. This one
    calls `q->forceStop()`, which
    per driver stops the timer or channel and does `read_idx = next_write_idx`,
    so the queued commands never run at all.

    That makes it the scenario that can rank drivers. What may still follow the
    marker is whatever each driver had already taken out of the queue: for
    mcpwm_pcnt, which re-arms the compare per entry, that is at most the entry in
    progress. A driver with a hardware transmit buffer can be further ahead, and
    the count is the measurement rather than a pass/fail about the stop.

    The gate is deliberately the same shape as SR_25's, mirrored: the run started
    (`0 < steps_before_stop`), it stopped short of the fill -- otherwise nothing
    was discarded -- and the tail is within `ABORT_TAIL_ENTRIES`.
    """
    # Through Pins, not `channels["D0"]`. A multiplexed stepper's step signal is
    # on no channel at all -- it is a bit of the I2S word, decoded to S<slot> --
    # so a hardcoded D0 raises KeyError on every mux run instead of measuring it.
    ch = pins.step_wave(channels, "A")
    m = sp.channel_metrics(ch, rate)
    t = segments[0][1]
    period, _ = check_periods(m.inter_step_us, t, info)
    steps = sp.rising_edges(ch)
    requested = requested_steps(segments)
    filled = info.get("queue_filled_entries", 0)
    fill = filled * 255

    edges = sp.detect_edges(ch)
    starts = [i for i, level in edges if level == 1]
    ends = [i for i, level in edges if level == 0]
    unterminated = 1 if starts and (not ends or starts[-1] > ends[-1]) else 0

    tail_bound = ABORT_TAIL_ENTRIES * 255
    marker_at = _marker_edge(channels, rate, marker_channel)
    base = {"requested_steps": requested, "steps_measured": len(steps),
            "unterminated_pulses": unterminated, "queue_filled_entries": filled,
            "filled_steps": fill, "abort_tail_bound_steps": tail_bound,
            "period": period,
            "steps": sp.step_count_defects(m.step_count, len(steps))}
    if marker_at is None:
        base.update({"stop_measured": False, "queue_discarded": False,
                     "reason": "no marker edge: the stop instant is not on the "
                               "waveform, so an abort cannot be separated from "
                               "the capture simply ending"})
        return False, base

    after = [i for i in steps if i > marker_at]
    before = len(steps) - len(after)
    last = steps[-1] if steps else None
    interrupted = 0 < before < fill
    base.update({
        "stop_measured": True,
        "marker_channel": marker_channel,
        "steps_before_stop": before,
        # The measurement: what the driver still emitted after the queue was
        # emptied. Small, and how small is the driver's own answer.
        "steps_after_stop": len(after),
        "tail_entries": round(len(after) / 255, 2),
        "stop_to_last_step_us": round((last - marker_at) / rate * 1e6, 3)
        if last is not None and last > marker_at else 0.0,
        "stop_interrupted_the_run": interrupted,
        # The queue was discarded: the run stopped short of what was filled.
        "queue_discarded": interrupted and len(steps) < fill,
    })
    ok = (period["ok"] and not unterminated and interrupted
          and len(steps) < fill and len(after) <= tail_bound)
    return ok, base


def _marker_edge(channels, rate, marker_channel):
    """Sample index of the first edge on the marker channel, or None."""
    if marker_channel is None or marker_channel < 0:
        return None
    mark = channels.get(STEP_CHANNEL_ORDER[marker_channel])
    if not mark:
        return None
    edges = sp.detect_edges(mark)
    return edges[0][0] if edges else None



def eval_pause(channels, rate, segments, info, pins):
    """A pause (steps=0) must produce exactly its tick count of silence.

    A pause far shorter than commanded means pulses arrived during it, which is
    a defect rather than a measurement.
    """
    ticks = segments[0][1]
    pause_ticks = segments[1][1]
    pause_us = pause_ticks * 1e6 / info["ticks_per_s"]
    expected_steps = sum(steps for steps, _, _ in segments)
    m, win = moved_metrics(pins.step_wave(channels, "A"), rate, info, segments,
                           expected_steps)
    # A pause shows up as one long inter-step period, not as a wide pulse: it
    # is a stretch of silence, so searching the high widths would be looking in
    # the wrong place entirely.
    #
    # The observed gap is longer than the pause itself. A command of n steps
    # spaced `ticks` apart occupies n*ticks, so the pause begins one full
    # period after the last step of the phase before it (avr_queue.cpp:192
    # schedules the next entry's first step one period later).
    expected_gap_us = (ticks + pause_ticks) * 1e6 / info["ticks_per_s"]
    ok = any(abs(w - expected_gap_us) <= expected_gap_us * 0.05 + 1.0
             for w in m.inter_step_us)
    counts = sp.step_count_defects(m.step_count, expected_steps)
    return ok and counts["ok"], {
        "ticks": ticks,
        "pause_ticks": pause_ticks,
        "pause_us": round(pause_us, 4),
        "expected_gap_us": round(expected_gap_us, 4),
        "measured_gaps_us": [round(w, 4) for w in m.inter_step_us
                             if w > m.avg_high_us + 1.0][:8],
        "pause_found": ok,
        "steps": counts,
        "window": win,
    }


def eval_dir_change(channels, rate, segments, info, pins):
    """Measure the delay from the dir edge to the first step of phase 2.

    The value is **reported, not gated on** (white paper 1.3): it is set by how
    the driver starts stepping, and the platforms differ by three orders of
    magnitude -- the Pico's PIO sets DIR and STEP from adjacent instructions
    (~50 ns), while ESP32 MCPWM/PCNT spends a whole MIN_CMD_TICKS pause (~200 us)
    settling the direction first. Asserting one number would assert a platform
    characteristic. What is asserted is that a delay exists at all, and that
    every commanded step arrived.

    `sample_us` is reported because this measurement can fall below the capture
    resolution: one sample at 4 MS/s is 250 ns, which cannot resolve a 50 ns
    Pico delay. A reading of 0 or 1 sample means "faster than we can see", not
    "zero".
    """
    expected_steps = sum(steps for steps, _, _ in segments)
    step = pins.step_wave(channels, "A")
    dir_ch = pins.dir_wave(channels, "A")
    # Matched against the move's steps only: a pulse in the pre-move idle would
    # otherwise be the "next step" for the first dir edge and report a delay to
    # a pulse that precedes the direction change entirely.
    rises, win = moved_edges(step, rate, info, segments, expected_steps)
    delays = sp.dir_to_first_step_us(dir_ch, step, rate, steps=rises)
    counts = sp.step_count_defects(len(rises), expected_steps)
    sample_us = 1e6 / rate
    dir_edges = sp.detect_edges(dir_ch)
    resolved = [d for d in delays if d >= sample_us]

    # A delay of 0.0 means DIR and STEP landed in the same sample: the
    # separation is real but smaller than this capture can resolve. That is the
    # Pico case, where the PIO sets both from adjacent instructions (~50 ns),
    # and it is a good capture, not a failure. It must be flagged rather than
    # silently reported as 0 us, or a Pico run reads as "no delay at all" and
    # nobody notices the measurement was never taken.
    ok = bool(delays) and counts["ok"]
    return ok, {
        "dir_to_first_step_us": [round(d, 4) for d in delays[:8]],
        "dir_to_first_step_min_us": round(min(delays), 4) if delays else None,
        "capture_rate_hz": rate,
        "sample_us": round(sample_us, 4),
        "below_capture_resolution": bool(delays) and not resolved,
        "dir_edges": len(dir_edges),
        "steps": counts,
        "window": win,
    }


def eval_sync_start(channels, rate, segments, info, pins):
    """Measure how well the steppers start together, and check the step counts.

    The skew is **measured and reported, not gated on**. `synchronizedStart()`
    asks each stepper to begin, but how closely they actually begin is a
    property of the pulse driver and of what the processor is doing at that
    instant, not a correctness property of the queue:

      * RMT and MCPWM arm their hardware compare units, so the offset is a
        fixed few microseconds.
      * PCNT and the AVR timer ISR step the pin from an interrupt, so the
        offset grows with interrupt latency and with how much other work the
        uC is doing.
      * With several drivers mixed, the steppers are not even on the same kind
        of timer.

    A stepper that starts a few microseconds late has not malfunctioned, and
    failing the test for it would report a platform characteristic as a bug.
    So the number goes into the result and is compared across architectures and
    drivers, where a regression *is* meaningful.

    The step counts, in contrast, are a hard requirement: a swallowed or
    spurious step is a real defect on any platform.
    """
    counts = {}
    firsts = {}
    windows = {}
    expected = segments[0][0]
    for name, ch in pins.items():
        if ch not in channels:
            continue
        edges, win = moved_edges(channels[ch], rate, info, segments, expected)
        counts[name] = len(edges)
        windows[name] = win
        if win["anchored"]:
            firsts[name] = win["first_step_sample"]
    skew_us = skew_of_first_steps(firsts, rate)
    period_us = segments[0][1] * 1e6 / info["ticks_per_s"]
    defects = {k: sp.step_count_defects(v, expected)
               for k, v in counts.items()}
    return all(d["ok"] for d in defects.values()), {
        "steps_per_stepper": defects,
        "window_per_stepper": windows,
        # Reported, not asserted. See the docstring.
        "first_step_skew_us": round(skew_us, 4),
        "skew_periods": round(skew_us / period_us, 4) if period_us else None,
        "period_us": round(period_us, 4),
        "first_step_us": {k: round(v * 1e6 / rate, 4)
                          for k, v in firsts.items()},
    }


# Scenarios where the host has to act while the run is in progress: the seconds
# to wait after QRUN before issuing STOP. Everything else runs untouched to
# completion. Kept out of the SCENARIOS tuple so the four-field shape every
# other scenario uses stays uniform.
# Scenario -> the command issued after the run has started. STOP is the
# library's `stopMove()`, whose contract is that it must NOT truncate queued
# motion; XSTOP is `forceStopAndNewPosition()`, which empties the queue.
#
# `forceStop()` is deliberately absent: on a queue this harness fills itself and
# then stops feeding, it is a no-op. Its only effect is `ignore_commands = true`,
# which refuses *later* addQueueEntry() calls, and by construction there are none
# -- so a scenario for it could not fail. `stopMove()` is weaker still: it sets
# a flag the ramp generator consults for its *next* command, and this harness
# drives addQueueEntry() directly and never runs one.
SCENARIO_STOP = {"SR_25": "STOP", "SR_30": "XSTOP"}
# Scenario -> how long after QRUN the stop is issued, in seconds.
#
# A stop a quarter of the way into the run -- the earlier rule -- measures the
# queue only if the feeder is still filling at that point, and that is a race
# between the host's serial loop and the driver's drain rather than a property
# of the stop. Measured on rmt: 7464 steps after the marker against a bound of
# 8160, from a 20000-step program the feeder was still working through. The same
# program with the stop 1 ms in leaves the queue as QFILL put it, so what the
# marker bounds is a queue state the board itself reported.
# Scenario -> where in the fill's duration the stop is issued, as a fraction.
# See stop_after_for() for why this is scaled rather than a constant.
STOP_AFTER = {"SR_25": 0.25, "SR_30": 0.25}
# ...and never sooner than this, or the stop can precede the start.
STOP_AFTER_MIN = 0.001

EVALUATORS = {
    "SR_01": eval_period_exact,
    "SR_02": eval_step_count,
    "SR_03": eval_step_count,
    "SR_04": eval_step_count,
    "SR_05": eval_pulse_width,
    "SR_06": eval_step_count,
    "SR_07": eval_step_count,
    "SR_08": eval_step_count,
    "SR_09": eval_pause,
    "SR_10": eval_dir_change,
    "SR_11": eval_direction_phases,
    "SR_12": eval_direction_phases,
    "SR_13": eval_nothing_emitted,
    "SR_14": eval_sync_start,
    "SR_16": eval_multi_stepper_periods,
    "SR_17": eval_sync_start,
    "SR_15": eval_independent_speeds,
    "SR_27": eval_period_exact,
    "SR_21": eval_period_exact,
    "SR_23": eval_period_exact,
    "SR_26": eval_pause,
    "SR_30": eval_abort_queue,
    "SR_25": eval_stop_move_contract,
    "SR_18": eval_counts_and_gap,
    "SR_19": eval_counts_and_gap,
    "SR_20": eval_counts_and_gap,
    # The max-count scenario, judged by the evaluator `scale` already uses: each
    # stepper's own step count and its own period, measured from the capture.
    # `eval_scale` is count-agnostic -- it walks the channel map it is handed --
    # so 2 steppers and 32 come through the same code, which is what lets the
    # catalogue have a max-count test without a second evaluator to keep in step.
    "SR_31": eval_scale,
}


# ---------------------------------------------------------------------------
# Runner
# ---------------------------------------------------------------------------


def run_sr00(tag_key, args):
    """Capture the SR_00 pin pattern. Not a queue test; it gates the rest."""
    capture_file = Path(args.capture_dir) / f"sr00_{tag_key}.sr"
    rate = args.sr00_sample_rate
    # SR_00's pattern is 1 Hz, so its window is set by the PATTERN and not by
    # `--seconds`: that flag sizes a queue scenario, which is tens of
    # milliseconds, and a 1 Hz pattern needs a couple of cycles to show one full
    # high and one full low on every channel. It ran on whatever the scenario
    # asked for, and at the short window a mux run legitimately wants it saw two
    # edges per channel and reported all eight as dead pins -- which is what a
    # dead cable looks like, so the pre-check was indistinguishable from the
    # fault it exists to catch.
    seconds = max(args.seconds, SR00_SECONDS)
    ser = open_board(args.port, args.baud)
    try:
        # Start the pattern BEFORE the capture, not during it. The evaluator
        # requires every width to be exactly the commanded high or low time,
        # and an interval that straddles the start of the capture window is a
        # fragment of one: measured 402 us / 998.8 ms where 50 / 950 ms were
        # commanded, reported as a spurious edge on a correctly wired channel.
        # Letting the pattern reach steady state first keeps the check strict
        # instead of teaching the evaluator to forgive the boundary.
        send_line(ser, "SR00")
        time.sleep(SR00_SETTLE_S)
        proc = start_capture(capture_file, seconds, rate)
        proc.wait()
        replies = drain(ser, 0.3)
    finally:
        send_line(ser, "STOP")
        ser.close()

    # A firmware fault is reported as one, before the waveform is interpreted.
    #
    # This check is the reason a task watchdog no longer reads as eight dead
    # pins. Measured: a watchdog panic inside this capture window truncates the
    # pattern mid-cycle, and the evaluator -- correctly, on the waveform it was
    # given -- reports every channel's first high time as a fragment of the
    # commanded one (D0 19.3 ms against 50 ms, the pairs summing to exactly one
    # 1000 ms period). "All eight channels are dead" is what that looks like,
    # and it is not what happened: the harness was measuring a panic.
    #
    # So the verdict is `error`, not `failed`, and it names the marker. `failed`
    # on SR_00 is a wiring statement, and it must not be reachable from a
    # firmware fault.
    fault = firmware_fault(replies)
    channels, sample_rate = load_capture_for_eval(capture_file)
    passed, channel_results = analyze_csv.evaluate_sr00(channels, sample_rate)
    if fault:
        return "error", {
            "sample_rate_hz": sample_rate,
            "channels": channel_results,
            "reply": replies.strip(),
            "firmware_fault": fault,
            "error": "the firmware faulted during the SR_00 capture window, so "
                     "the pin pattern was truncated and the waveform says "
                     "nothing about the wiring: " + ", ".join(fault),
        }
    return ("passed" if passed else "failed"), {
        "sample_rate_hz": sample_rate,
        "channels": channel_results,
        "reply": replies.strip(),
    }


def measure(tag_key, name, wire, mask, builder, evaluator, args,
            per_stepper_for=None, scenario=None, probe=None):
    """Wire the board, capture one measurement, evaluate it. One code path.

    `scenario` is the SR id when there is one, and None for a mode run. It is
    separate from `evaluator` because a mode run's evaluator is a closure, and
    SR_13's exemption below needs the *id* to be knowable, not the callable.

    Wire the board, capture one measurement, evaluate it. One code path.

    Everything measured here goes through this function -- the named scenarios
    in SCENARIOS and the runs the two generic modes generate alike. A mode with
    its own capture path could disagree with the scenarios about when sigrok is
    armed, how long the window has to be, or which channel map is correct, and
    nothing would say which of the two was right.

    `wire` is the whole CONFIG line, or None when a `probe` decides it.
    `evaluator` is (channels, rate, segments, info) -> (passed, detail), passed
    straight to evaluate() so the global pin invariants are checked for generated
    runs too.

    `probe` is a `MaxCountProbe` for the scenarios whose stepper count is not
    known until the board is asked. It runs *after* `ensure_mux()` and *before*
    the CONFIG that counts on it, because the multiplexer has to be up for a
    `CONFIG` naming `i2s_mux` to connect at all and the probe's first attempt is
    such a CONFIG. It replaces `wire` and `mask` and contributes its transcript
    to `detail`, so the result record says *why* the count is what it is and not
    only how many steppers stepped.

    Returns (status, detail). `status` is `refused` when the board would not
    accept the configuration at all -- which is a *result* for `scale`, whose
    question is where the limit is, and a wiring problem for a named scenario.
    """
    ser = open_board(args.port, args.baud)
    try:
        # ensure_mux() only reads the driver names off the wire, so a probed
        # run can hand it the probe's top-of-search line: the name is already
        # known even though the count is not.
        ensure_mux(ser, wire or probe.driver_wire(), args)
        probe_detail = None
        if probe is None:
            text = reply_of(ser, wire)
        else:
            # `ser` is rebound: a probe that had to try more than one count
            # reopened the port between attempts, because a CONFIG refused
            # partway leaves the board half-configured and the next reply then
            # describes the *previous* attempt. Closing and reopening resets the
            # ESP32, which is the same mechanism `ensure_mux()` already relies on
            # to bring the multiplexer up at all.
            def reopen():
                nonlocal ser
                ser.close()
                ser = open_board(args.port, args.baud)
                return ser

            wire, mask, probe_detail, text, ser = probe.find(ser, reopen)
            print(f"    probe: {probe_detail['max_stepper_count']} stepper(s) is "
                  f"the most {probe_detail['probed_driver']} connects in "
                  f"{probe_detail['pin_mode']} (searched down from "
                  f"{probe_detail['search_bound']} in "
                  f"{probe_detail['probe_attempts']} attempt(s))")
            for refused in probe_detail["refused_above"]:
                print(f"      n={refused['n']}: {refused['reply']}")
            for short in probe_detail["ok_but_short"]:
                # The board said yes and connected fewer than it was asked for.
                # That is a firmware defect, so it is named as one here rather
                # than being quietly counted as a refusal -- and it is a count
                # the probe did NOT believe, which is the whole reason it is
                # reported at all.
                print(f"      n={short['n']}: {short['reply']}")
                print(f"      ^ answered OK but connected {short['connected']}"
                      f" stepper(s); not believed")

        def with_probe(failure):
            """A failure record carrying what the probe found, when it ran.

            A setup failure *after* a successful probe would otherwise be the one
            record with no count in it -- and for SR_31 the count is the
            measurement, so it belongs in every record the run produces.
            """
            return dict(failure, **(probe_detail or {}))

        # Whether the board took the configuration at all. For a probed run the
        # probe has already decided -- by MAP, not by the reply -- so the reply
        # is not consulted again: a board that answered `OK ... already` for a
        # count it never connected has to read as refused here, or the run would
        # proceed against steppers nobody asked for.
        accepted = (probe_detail["max_stepper_count"] > 0
                    if probe_detail is not None else "OK CONFIG" in text)
        if not accepted:
            return "refused", with_probe({"error": text.strip(), "wire": wire})
        # The channel map comes from the board, not from the host's assumption:
        # `dir` and `nodir` put different channels on different steppers, so
        # evaluating a capture against the wrong map reads a quiet pin and calls
        # it zero steps.
        chan_map, pin_map = read_map(ser)
        info = read_qinfo(ser)

        # Put the STOP instant on the waveform, for the scenarios that need it.
        #
        # Here, before QRUN, and that placement is load-bearing rather than
        # tidy. Sent after QRUN it cost two serial round-trips (~0.25 s each)
        # that sat between the start of the move and the STOP, so on a driver
        # whose move was shorter than that -- i2s_direct and rmt at a floor of
        # 80 ticks both run 20000 steps in 0.1 s -- STOP arrived after the move
        # had finished and the marker edge fell past the end of the delivered
        # capture. The run then reported a complete 20000-step move and "no
        # marker edge", which reads as "STOP was never processed".
        #
        # MARK is configuration, like CONFIG, so it belongs in the setup phase.
        marker = None
        if scenario in SCENARIO_MARKERS:
            want = marker_channel_for(len(chan_map), pin_map.get("stride", 2),
                                      bus_channels=len(pin_map.get("bus") or ()))
            mark_reply = reply_of(ser, f"MARK {want if want is not None else 'none'}")
            if "OK MARK" not in mark_reply:
                print(f"    MARK refused: {mark_reply.strip()}")
            pin_map = read_map(ser)[1]
            marker = pin_map.get("marker", -1)
            if marker < 0:
                print(f"    no marker channel free for {scenario}: every "
                      f"channel belongs to a stepper, so the stop instant "
                      f"cannot be put on the waveform")

        segments = builder(info)
        programs = per_stepper_for(info) if per_stepper_for else None
        # The legality pre-check is skipped for the scenarios whose subject is a
        # rejection: SR_13 programs a command below MIN_CMD_TICKS on purpose, so
        # refusing it here would rob the scenario of the thing it exists to see,
        # and it would "pass" by never reaching the queue.
        check = None if scenario in REJECTION_SCENARIOS else info
        if programs:
            if not program_per_stepper(ser, programs, check):
                return "failed", with_probe({"error": "QSEG rejected",
                                             "segments": segments})
        elif not program(ser, segments, check):
            return "failed", with_probe({"error": "QSEG rejected",
                                         "segments": segments})

        seconds = scenario_seconds(segments, info["ticks_per_s"]) \
            + (SETUP_AFTER_CAPTURE_S if scenario in SCENARIO_FILL else 0.0) \
            + 0.5
        rate = args.sample_rate
        capture_file = Path(args.capture_dir) / f"{name}_{tag_key}.sr"

        proc = start_capture(capture_file, seconds, rate)
        time.sleep(0.3)  # let sigrok-cli start sampling
        # Fill the queue before the run, for the scenarios that stop into it.
        # QFILL goes here rather than with the rest of the setup because it is
        # what makes the start deterministic: QRUN alone prefills half a queue
        # and tops it up from the main loop, so the depth a millisecond later is
        # a race between the loop and the drain. Sent after the capture is
        # running, so a refusal is inside the capture rather than a silent gap.
        if scenario in SCENARIO_FILL:
            filled = fill_queue(ser, mask)
            if filled <= 0:
                proc.kill()
                return "error", with_probe(
                    {"error": "QFILL did not reach the queue",
                     "requested_entries": QUEUE_FILL_ENTRIES,
                     "reply": drain(ser, 0.2).strip()})
            info["queue_filled_entries"] = filled
        # settle=0 on both lines below, and the reason is the whole point of the
        # scenario. `send_line` sleeps 50 ms *after* writing, so with the default
        # the stop was written 51 ms after QRUN and the marker landed on step
        # 5610 of a 10 us program -- 56 ms in, not the 1 ms the scenario asks
        # for, by which time the feeder had replaced most of the filled queue.
        # Measured, not inferred: that run reported 5596 steps before the marker
        # against a fill of 4080. run_hardware's `send()` waits *after* the write
        # and so never had this problem; the two disagreed by 50x on when the
        # stop landed.
        send_line(ser, f"QRUN {mask}", settle=0.0)
        # STOP, for the scenarios whose subject is stopping.
        #
        # This was missing entirely from measure(): the only STOP in this file
        # was SR_00's cleanup, and STOP_AFTER was honoured only by
        # run_hardware.py. So a catalogue run through this orchestrator never
        # issued one, on any driver -- SR_25 asserts "pulses cease when STOP is
        # issued" while no STOP was issued, and the 20000-step move simply ran
        # to completion. That reads as "STOP does not work on this driver",
        # which is not what was being measured.
        stop_after = stop_after_for(scenario, segments, info)
        if stop_after:
            time.sleep(stop_after)
            send_line(ser, SCENARIO_STOP.get(scenario, "STOP"), settle=0.0)
        proc.wait()
        replies = drain(ser, 0.4)
        # A firmware fault inside a capture window truncates the run, and the
        # waveform then reports a step count or a period that is short for a
        # reason that has nothing to do with the driver. Same reasoning as
        # SR_00's, and the same verdict: `error`, naming the marker, never
        # `failed`. Measured -- a task watchdog fires on this board while idle,
        # so this is reachable without anything being wrong with the driver.
        fault = firmware_fault(replies)
        # SR_13 is excluded because the rejection is its *subject*: it exists to
        # assert that a command below MIN_CMD_TICKS emits nothing, and the queue
        # saying `ERR QE step0 rc=-1` is that scenario passing, not failing. The
        # set is asserted against the catalogue in the tests, so a second
        # rejection scenario cannot be added without deciding the same question.
        #
        # `ERR QE ... rc=-1` arriving *now* is the queue rejecting a command --
        # almost always ticks*steps below MIN_CMD_TICKS. It is a setup failure,
        # not a measurement, and reporting it as one produces a falsehood: the
        # evaluator then counts zero pulses on a pin that carries every legal
        # move perfectly. Caught here because QSEG only acknowledges the parse;
        # the real addQueueEntry() call happens in qe_pump() from the main loop,
        # after QRUN, so the syntax check in program() cannot see it.
        if fault and scenario not in REJECTION_SCENARIOS:
            print(f"    firmware fault: {', '.join(fault)} -- a setup failure, "
                  f"not a measurement")
            return "error", with_probe(
                {"error": "the firmware faulted during the capture window",
                 "firmware_fault": fault,
                 "firmware_reply": replies.strip(),
                 "segments": segments,
                 "capture": str(capture_file)})
        if "ERR QE" in replies and scenario not in REJECTION_SCENARIOS:
            detail = with_probe(
                {"error": "queue rejected the command",
                 "firmware_reply": replies.strip(),
                 "segments": segments,
                 "entries_below_min_cmd_ticks": sub_min_entries(
                     segments if not programs else None, info, programs)})
            print(f"    {replies.strip().splitlines()[0]} -- a setup failure, "
                  f"not a measurement")
            return "error", detail
        send_line(ser, "POS")
        replies += drain(ser, 0.2)
    finally:
        send_line(ser, "QCLR")
        ser.close()

    channels, sample_rate, source_vcd = load_capture_for_eval(capture_file,
                                                              with_vcd=True)
    # A multiplexed run has to be decoded before anything can read it: stepper
    # A's step signal is one bit of a 32-bit word on three shared wires, and it
    # is on no channel at all. The 8-channel capture becomes a 37-channel one
    # (8 - 3 + 32) and the evaluators see ordinary step channels from there on.
    decoded_from = None
    if pin_map.get("bus"):
        if source_vcd is None:
            return "failed", with_probe(
                {"error": "mux run needs a VCD to decode; the capture has none",
                 "capture": str(capture_file)})
        decoded = decode_mux_capture(source_vcd, channels, sample_rate,
                                     pin_map, chan_map)
        channels, sample_rate, decoded_from = decoded
    # The marker channel is an optional 6th argument, so it goes through `extra`
    # rather than being appended for every evaluator. `extra` is the *value*,
    # forwarded positionally: eval_sync's 6th parameter is the programs dict and
    # gets it the same way.
    # A multiplexed run is judged on the frame grid it is emitted on. The grid is
    # a library constant (I2S_TICKS_PER_FRAME), it is a property of the driver
    # rather than of the board, and it travels in `info` so that every evaluator
    # sees it -- a scenario-specific special case would be one more thing to keep
    # in step with the two above it.
    if pin_map.get("bus"):
        info = dict(info, frame_grid_ticks=I2S_TICKS_PER_FRAME)
    passed, detail = evaluate(evaluator, channels, sample_rate, segments, info,
                              chan_map, extra=marker,
                              frame_grid_ticks=info.get("frame_grid_ticks"))
    detail.update({
        "capture": str(capture_file),
        "channel_map": chan_map,
        "pin_map": pin_map,
        "decoded_from": decoded_from,
        "sample_rate_hz": sample_rate,
        "capture_seconds_requested": round(seconds, 3),
        "segments": segments,
        "per_stepper_segments": programs,
        "reply": replies.strip(),
    })
    # How the stepper count was arrived at, for the scenarios that had to ask.
    # `max_stepper_count` is the measurement this scenario exists for, and
    # `refused_above` is what makes it checkable afterwards: a reader who cannot
    # reproduce the count can see which CONFIG was refused and why.
    if probe_detail is not None:
        detail.update(probe_detail)
    # The DUT's tick rate is what makes the ticks in `segments` interpretable,
    # so it travels with every result.
    detail["dut"] = info
    # Which channel, if any, carries the STOP marker. The evaluator needs it to
    # measure the stop instant rather than infer it.
    detail["marker_channel"] = marker
    # Queue entries shorter than MIN_CMD_TICKS, named as such.
    #
    # RMT emits them; `i2s_direct` silently discards them and produces nothing
    # at all -- measured, not guessed: a 195 us entry yields zero pulses and a
    # 200 us entry yields all of them, and 200 us is exactly MIN_CMD_TICKS
    # (3200 ticks at 16 MHz). Three 40 us entries totalling 240 us also yield
    # nothing, so the limit is per entry rather than on the program.
    #
    # Recorded because a scenario measuring *zero* steps otherwise reads as a
    # dead pin or a driver that emits nothing, and neither is true: the pin
    # carries every longer move perfectly. Without this note the finding had to
    # be rediscovered from a capture by hand.
    detail["entries_below_min_cmd_ticks"] = sub_min_entries(
        segments if not programs else None, info, programs)
    return ("passed" if passed else "failed"), detail


def ensure_mux(ser, wire, args):
    """Bring the multiplexer up, in THIS session, if this CONFIG needs it.

    It has to be in the same session as the CONFIG, and that is not tidiness:
    the intent is that opening the serial port resets the ESP32, so every command
    batch in this harness starts from a fresh board. `initI2sMux()` cannot run
    twice and does not survive a reset, so a mux brought up in one session is
    gone by the next -- and the CONFIG that follows is refused with `ERR connect
    step 0`, which names no cause and looks exactly like a broken driver.

    Decided from the CONFIG line rather than from a flag, so the two cannot
    disagree: a run that names i2s_mux has the mux, and a run that does not is
    left alone.

    **A refused IMUX is not by itself an error; a mux that is not up is.**
    Opening the port is *meant* to reset the board, and the DTR/RTS toggle does
    not always fire -- which is the same reason a flash occasionally comes up in
    the wrong boot mode. When it does not, the mux from the previous point is
    still running, the second `initI2sMux()` is correctly refused, and the run
    aborted on a board whose multiplexer was working the whole time: a
    three-point sweep died at n=3 with "IMUX was refused" and the first two
    points had passed. So a refusal is resolved by asking the firmware, which
    reports `mux_init=` in its DRIVERS reply: 1 means the multiplexer is up and
    the run carries on, 0 means it is not and the refusal was real.
    """
    if "i2s_mux" not in (wire or ""):
        return False
    if not getattr(args, "imux", False):
        return False
    if send_imux(ser):
        return True
    _present, mux_init = read_drivers(ser)
    if mux_init:
        # initI2sMux() cannot run twice and did not need to: it is already up.
        print("    IMUX refused but the mux is already up (mux_init=1): the "
              "port open did not reset the board; continuing")
        return True
    raise RuntimeError(
        "IMUX was refused for a CONFIG that names i2s_mux, and the firmware "
        "reports mux_init=0: the three bus channels must be free, and "
        "initI2sMux() has to succeed before any mux stepper connects")


def unsupported_scenario_drivers(scenario, board_drivers, native_driver):
    """Drivers a named scenario asks for that this build does not have.

    A scenario may name a driver only some architectures provide -- SR_17 asks
    for `rmt`+`mcpwm_pcnt`, SR_18..20 for `mcpwm_pcnt`, SR_23 for `i2s_direct`,
    none of which exist on a Pico. Asking a board for a capability it was never
    supposed to have must read as **skipped**, not failed: a FAIL is a
    statement about the hardware, and "this ARM core has no RMT peripheral" is
    not a defect in it.

    Returns [] when every named driver is present, and also when the board's
    driver list was not read (`board_drivers` is None for a bare
    `run_tests.py` invocation) -- there the `ERR CONFIG no such driver`
    refusal in `run_scenario()` is the fallback signal.
    """
    if not board_drivers:
        return []
    drivers = config_drivers(SCENARIOS[scenario][0], native_driver)
    return [d for d in drivers if not board_drivers.get(d)]


def run_scenario(tag_key, test_id, args):
    """Program a scenario, capture it, and evaluate the waveform."""
    _config, builder, mask, _desc = SCENARIOS[test_id]

    # A driver this build does not have is skipped before a capture is even
    # opened: there is no measurement to make, and the board's refusal would
    # otherwise be recorded as a failure of the board.
    unsupported = unsupported_scenario_drivers(
        test_id, getattr(args, "board_drivers", None), args.dut_driver)
    if unsupported:
        return "skipped", {
            "reason": "driver(s) this build does not have: "
                      + ", ".join(unsupported),
            "scenario_drivers": config_drivers(SCENARIOS[test_id][0],
                                               args.dut_driver),
            "board_drivers": args.board_drivers,
        }

    # A probed scenario's CONFIG is the probe's to send -- it is what finds the
    # count -- so it is handed no wire at all. Every other scenario's is built
    # from its own pin mode, which is `dir` for all of them but one.
    probe = probe_for(test_id, args.dut_driver)
    status, detail = measure(tag_key, test_id.lower(),
                             None if probe else scenario_wire(test_id,
                                                             args.dut_driver),
                             mask,
                             builder, test_id, args,
                             per_stepper_builder(test_id),
                             scenario=test_id, probe=probe)
    # A named scenario the board refuses is a wiring fault, not a finding, so it
    # is reported as a failure even though the shared path calls it `refused`
    # for the modes -- with one exception: a CONFIG naming a driver this build
    # has no driver for is the same unsupported-capability case as above, which
    # the direct `run_tests.py` path (no board_drivers read ahead of time) can
    # only learn from the refusal itself.
    if status == "refused":
        if "no such driver" in (detail or {}).get("error", ""):
            return "skipped", dict(detail or {},
                                   reason="driver(s) this build does not have")
        return "failed", detail
    return status, detail


def run_modes(tag_key, plans, args, mode):
    """Run every generated point, recording each one as a result.

    A refused point is recorded as `refused` and the plan carries on. For
    `scale` that is the *answer*: the point where the board says no is the
    bound the mode exists to find, and stopping the run there would report one
    fewer stepper than the board supports. For `sync` a refusal means a driver
    this build cannot connect, which is a fact worth recording rather than a
    reason to abandon the other 9 combinations.

    Nothing is re-run that already passed, the same as a named scenario: a mode
    is many runs and re-measuring all of them to look at one is expensive on
    hardware that takes seconds per capture.
    """
    results_dir = Path(args.results_dir)
    # Both directories: run() creates them for the catalogue path, and a mode
    # run writing into a --results-dir the caller just named is the normal case,
    # not a special one. A missing parent here is a FileNotFoundError after the
    # capture has already been spent.
    results_dir.mkdir(parents=True, exist_ok=True)
    Path(args.capture_dir).mkdir(parents=True, exist_ok=True)
    index_file = results_dir / "tag_index.json"
    index = load_index(index_file)
    summary = []

    for plan in plans:
        key = f"{tag_key}_{plan.tag}"
        prev = index.get(key, {}).get("MODE")
        if prev and prev.get("result") == "passed" and not args.force:
            print(f"  {plan.label}: SKIP (already passed)")
            continue

        print(f"  {plan.label}: {plan.wire}")
        # One point must not take the rest of the sweep with it. A `scale` sweep
        # is the measurement of a *range*, so an exception at n=3 leaves n=4..8
        # unmeasured and the report shows one error where it should show a
        # bound; that is how a transient board hiccup here cost a whole sweep
        # (and, through the row's exit code, flagged the row as failed). The
        # point is recorded as `error` with the reason, and the plan continues.
        try:
            status, detail = measure(key, plan.label, plan.wire, plan.mask,
                                     plan.builder, plan.evaluator, args,
                                     plan.per_stepper_builder)
        except Exception as exc:
            status = "error"
            detail = {"error": f"{type(exc).__name__}: {exc}",
                      "wire": plan.wire}
        detail["mode"] = mode
        detail["drivers"] = plan.drivers
        detail["pin_mode"] = plan.pin_mode
        detail["stepper_count"] = plan.count
        detail["goal"] = plan.goal
        # The report groups mode results by arch / sdk / driver list. Until this
        # was recorded here the only place the arch existed was inside the tag
        # key, and the report had to take it apart again -- which means a report
        # table keyed by architecture was reading a *string convention* rather
        # than a recorded fact, and `esp32_idf5_3_0_...` has two underscores
        # where `esp32_arduino_...` has one. A tag key is an index; it is not a
        # schema.
        detail["arch"] = getattr(args, "arch", None)
        detail["framework"] = getattr(args, "framework", None)
        detail["sdk_version"] = getattr(args, "sdk_version", None)
        # What the board said it accepts, recorded with the result. Without it a
        # reader has to attach the hardware to learn that i2s_mux was compiled in
        # but never brought up -- which is the difference between "this driver
        # is broken" and "these three pins are not wired yet".
        detail["board_drivers"] = getattr(args, "board_drivers", None)
        detail["mux_init"] = getattr(args, "mux_init", None)

        result_file = results_dir / f"{key}.json"
        with open(result_file, "w") as f:
            json.dump({"test_id": "MODE", "tag_key": key, "mode": mode,
                       "timestamp": datetime.now().isoformat() + "Z",
                       "result": status, **detail}, f, indent=2)
        record(index, index_file, key, "MODE", status, result_file)
        summary.append({"label": plan.label, "drivers": plan.drivers,
                        "count": plan.count, "result": status,
                        "wire": plan.wire})
        print(f"  {plan.label}: {status.upper()}")

    print(f"Index: {index_file}")
    return summary


def summarize(summary):
    """The plan's outcome as a table: which bound stopped it, per driver.

    The one thing a `scale` run has to say and a list of pass/fail does not is
    *where it stopped*. Recording "8 steppers on RMT: passed" and "4 steppers
    on RMT: refused" is the measurement; the bound that stopped the sweep is
    the answer to the mode's question.
    """
    if not summary:
        return "(nothing ran)"
    out = ["| run | drivers | n | result |", "|---|---|---|---|"]
    for row in summary:
        out.append(f"| {row['label']} | {'+'.join(row['drivers'])} | "
                   f"{row['count']} | {row['result']} |")
    return "\n".join(out)


def load_index(index_file):
    if index_file.exists():
        with open(index_file) as f:
            return json.load(f)
    return {}


def save_index(index_file, index):
    index_file.parent.mkdir(parents=True, exist_ok=True)
    with open(index_file, "w") as f:
        json.dump(index, f, indent=2, sort_keys=True)


def record(index, index_file, tag_key, test_id, result, result_file, extra=None):
    entry = {
        "result": result,
        "file": None if result_file is None else str(result_file),
        "timestamp": datetime.now().isoformat() + "Z",
    }
    if extra:
        entry.update(extra)
    index.setdefault(tag_key, {})[test_id] = entry
    save_index(index_file, index)


def run(tag_key, tests, args):
    results_dir = Path(args.results_dir)
    results_dir.mkdir(parents=True, exist_ok=True)
    Path(args.capture_dir).mkdir(parents=True, exist_ok=True)
    index_file = results_dir / "tag_index.json"
    index = load_index(index_file)

    # SR_00 is the wiring pre-check. Always run it first (one capture per test),
    # so a later test cannot be measured on dead or mis-wired channels.
    #
    # Only when it is absent. `any(t != "SR_00" ...)` was true for the default
    # --tests, which is ALL_TESTS and already starts with SR_00, so SR_00 ran
    # twice on every catalogue run: a second 1 Hz capture and a second verdict.
    # It is not merely wasted -- on 2026-10-05 that second run FAILED, which set
    # sr00_failed and turned all 26 scenarios into SKIP, while the run still
    # exited 0 and the matrix reported the row "ok". A catalogue that measured
    # nothing and a catalogue that passed are then indistinguishable.
    if "SR_00" not in tests:
        tests = ["SR_00"] + tests

    sr00_failed = False
    sr00_reason = "SR_00 failed"
    for test_id in tests:
        prev = index.get(tag_key, {}).get(test_id)
        if prev and prev.get("result") == "passed" and not args.force:
            print(f"{test_id}: SKIP (already passed {prev['timestamp']})")
            continue

        if test_id != "SR_00" and test_id not in SCENARIOS:
            print(f"{test_id}: SKIP (not implemented)")
            record(index, index_file, tag_key, test_id, "skipped", None,
                   {"reason": "not implemented"})
            continue

        if sr00_failed:
            # The reason carries *which* verdict SR_00 recorded, because the two
            # mean different things and the reader needs to tell them apart: a
            # `failed` is a statement about the wiring, an `error` is a
            # statement about the firmware. Collapsing them into "SR_00 failed"
            # is what let a watchdog panic be reported as a dead cable.
            print(f"{test_id}: SKIP ({sr00_reason})")
            record(index, index_file, tag_key, test_id, "skipped", None,
                   {"reason": sr00_reason})
            continue

        print(f"{test_id}: running ...")
        # A BoardError means no measurement was made -- the board did not reset,
        # or the port is gone. It is raised, not recorded: a verdict on the
        # record says something about the driver, and there is nothing here to
        # say. The modes already treat an exception this way (as `error`), and a
        # catalogue run that skipped it would report a scenario that was never
        # attempted as one that was attempted and did not pass.
        if test_id == "SR_00":
            result, extra = run_sr00(tag_key, args)
        else:
            result, extra = run_scenario(tag_key, test_id, args)

        result_file = results_dir / f"{tag_key}_{test_id}.json"
        with open(result_file, "w") as f:
            json.dump({
                "test_id": test_id,
                "tag_key": tag_key,
                "timestamp": datetime.now().isoformat() + "Z",
                "result": result,
                **(extra or {}),
            }, f, indent=2)
        record(index, index_file, tag_key, test_id, result, result_file)
        print(f"{test_id}: {result.upper()}")

        if test_id == "SR_00" and result != "passed":
            sr00_failed = True
            sr00_reason = ("SR_00 errored (firmware fault)"
                           if result == "error" else "SR_00 failed")

    print(f"Index: {index_file}")


def show_list(args):
    index = load_index(Path(args.results_dir) / "tag_index.json")
    if not index:
        print("(no results recorded yet)")
        return
    print(f"{'TEST':7} {'STATUS':10} TAG KEY")
    for tag_key in sorted(index):
        for test_id in sorted(index[tag_key]):
            entry = index[tag_key][test_id]
            print(f"{test_id:7} {entry['result']:10} {tag_key}")


def parse_tests(text):
    if not text:
        return ALL_TESTS
    return [t.strip().upper() for t in text.split(",") if t.strip()]


def main():
    p = argparse.ArgumentParser(
        description="addQueueEntry() characterization orchestrator.")
    p.add_argument("--tag-key", help="tag key {arch}_{driver}_{channel_config}")
    # Recorded into mode results so a report can group by them. The tag key
    # already encodes them, but a key is for indexing, not for parsing back.
    p.add_argument("--arch", default=None, help="recorded in mode results")
    p.add_argument("--framework", default=None,
                   choices=[None, "arduino", "idf"])
    p.add_argument("--sdk-version", default=None,
                   help="e.g. 5.3.0; recorded in mode results")
    p.add_argument("--tests", help="comma list (default: all)")
    p.add_argument("--sample-rate", type=int, default=DEFAULT_RATE,
                   help=f"capture rate in Hz (default: {DEFAULT_RATE})")
    p.add_argument("--sr00-sample-rate", type=int, default=1_000_000,
                   help="SR_00 needs no more than 1 MS/s (default: 1000000)")
    p.add_argument("--seconds", type=float, default=6.0,
                   help="SR_00 capture seconds")
    p.add_argument("--capture-dir", default="capture",
                   help="where .sr/.vcd captures go")
    p.add_argument("--results-dir", default="results")
    p.add_argument("--port", default="/dev/cu.usbserial-0001")
    p.add_argument("--baud", type=int, default=115200)
    p.add_argument("--dut-driver", default=DEFAULT_NATIVE_DRIVER,
                   help="pulse driver for scenarios whose config does not "
                        "name one (timer on AVR, pio on Pico)")
    p.add_argument("--force", action="store_true",
                   help="re-run even if a passed result exists")
    p.add_argument("--list", action="store_true",
                   help="list recorded results and exit")
    args = p.parse_args()

    args.capture_dir = str(anchor_path(args.capture_dir))
    args.results_dir = str(anchor_path(args.results_dir))

    if args.list:
        show_list(args)
        return 0

    if not args.tag_key:
        p.error("--tag-key is required (or use --list)")
    if not all(c.isalnum() or c == "_" for c in args.tag_key):
        p.error("--tag-key must be [a-z0-9_]+")

    tests = parse_tests(args.tests)
    unknown = [t for t in tests if t not in ALL_TESTS]
    if unknown:
        p.error(f"unknown tests: {', '.join(unknown)}")

    run(args.tag_key, tests, args)
    return 0


if __name__ == "__main__":
    sys.exit(main())
# Scenarios that deliberately send the same waveform. SR_25 (`stopMove()`) and
# SR_30 (`forceStopAndNewPosition()`) program the same fill and stop it at the
# same instant; what separates them is the outcome, and it is a real one:
# measured 4080 of 4080 steps after the marker for the first and 0 for the
# second on rmt and mcpwm_pcnt, 67 for i2s_direct.
CONTRASTING_PAIRS = {frozenset(("SR_25", "SR_30"))}
