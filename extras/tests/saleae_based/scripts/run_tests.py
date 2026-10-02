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
    python3 scripts/run_tests.py --mode scale --driver rmt_v2 --pin-mode nodir
    python3 scripts/run_tests.py --mode sync --arch esp32
"""

import argparse
import json
import re
import subprocess
import sys
import time
from datetime import datetime
from pathlib import Path

SCRIPTS = Path(__file__).resolve().parent
sys.path.insert(0, str(SCRIPTS))

import analyze_csv  # noqa: E402
import capture as cap  # noqa: E402
import signal_parser as sp  # noqa: E402

# SR_01..SR_29 are the implemented catalogue plus SR_00; the range is the
# catalogue's numbering, and every id in it that SCENARIOS does not define is
# reported "not implemented" rather than silently missing.
ALL_TESTS = ["SR_00"] + [f"SR_{i:02d}" for i in range(1, 31)]

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
def default_channel_map(count=4, stride=2):
    """{'A': {'step': 'D0', 'dir': 'D1'}, ...} for a count/stride pair."""
    letters = "ABCDEFGH"
    out = {}
    for i in range(count):
        entry = {"step": f"D{i * stride}"}
        if stride > 1:
            entry["dir"] = f"D{i * stride + 1}"
        out[letters[i]] = entry
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
        having three silent steppers. The pin mode is still `dir`: every
        multi-stepper scenario in SCENARIOS is a `dir` one.
        """
        config = SCENARIOS[scenario][0]
        return cls(default_channel_map(CONFIGS[config][0], 2))

    @property
    def count(self):
        return len(self.step)

    @property
    def letters(self):
        """Stepper names, in order: A, B, C ..."""
        return sorted(self.step)

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

# How many steppers each pin mode can carry: 8 channels, 2 per stepper with a
# direction pin and 1 without (white paper 3.3/10.1). The firmware caps the
# count at the smaller of this and the platform's own stepper limit.
CHANNELS = len(STEP_CHANNEL_ORDER)
CHANNELS_PER_STEPPER = {"dir": 2, "nodir": 1}
MAX_STEPPERS_PER_MODE = {
    mode: CHANNELS // per for mode, per in CHANNELS_PER_STEPPER.items()
}

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


# ---------------------------------------------------------------------------
# Serial
# ---------------------------------------------------------------------------


def open_board(port, baud, timeout=6.0):
    """Open the serial port (which resets the board) and wait for READY."""
    import serial
    ser = serial.Serial(port, baud, timeout=0.1)
    deadline = time.time() + timeout
    buf = b""
    while time.time() < deadline:
        data = ser.read(256)
        if data:
            buf += data
            if b"READY" in buf:
                break
    ser.reset_input_buffer()
    return ser


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
MAP_RE = re.compile(r"MAP count=(\d+) mode=(\w+) stride=(\d+) ch=([\d,]*)"
                      r"(?:\s+marker=(\d+))?")
# "OK DRIVERS mux=0 rmt=1 rmt_v2=1 mcpwm_pcnt=1 i2s_direct=1 i2s_mux=1
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


def send_imux(ser, data, bclk, ws):
    """Bring the I2S multiplexer up. Returns True on success.

    `initI2sMux()` has to precede any mux stepper and cannot run twice, so it is
    issued once here rather than retried per scenario. On a board where no
    multiplexer is wired this is simply never called, and `DRIVERS` reports
    i2s_mux present but not up -- which is the distinction that stops a planner
    from offering a driver every CONFIG will refuse.
    """
    text = reply_of(ser, f"IMUX {data} {bclk} {ws}")
    return text.lstrip().startswith("OK IMUX")


def read_map(ser):
    """Read the DUT's channel map: which analyzer channel is which stepper.

    The host must not assume it. `dir` spends two channels per stepper (step,
    dir) and `nodir` one, so 4 steppers is D0,D2,D4,D6 in the first case and
    D0,D1,D2,D3 in the second -- and a host that guessed wrong measures a quiet
    pin and reports zero steps, which reads as a driver that emits nothing.

    Returns (channel_map, pins) where channel_map is the letter -> {step, dir}
    dict the evaluators index and pins is the GPIO behind each channel, for the
    record.
    """
    for _ in range(5):
        text = reply_of(ser, "MAP")
        m = MAP_RE.search(text)
        if m:
            count, mode, stride = int(m.group(1)), m.group(2), int(m.group(3))
            pins = [int(p) for p in m.group(4).split(",") if p]
            chan_map = default_channel_map(count, stride)
            # Cross-check the GPIO list against the capture's wiring: the pin
            # map is the firmware's, but if a channel is not driven at all there
            # is no measurement to make of it.
            for name, entry in chan_map.items():
                if entry["step"] not in STEP_CHANNEL_ORDER[:count * stride]:
                    raise RuntimeError(f"stepper {name} maps to a channel "
                                       f"outside the reported {count * stride}")
            # marker=-1 means no channel is designated as the event marker.
            marker = int(m.group(5)) if m.group(5) else -1
            return chan_map, {"mode": mode, "stride": stride, "pins": pins,
                              "marker": marker}
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


# Scenarios whose subject needs the STOP instant *on the waveform*. Sending STOP
# and then inferring when it landed from "the pulses ceased" cannot work on this
# rig: the capture the host requests is not the capture it gets (24 MHz
# truncates 0.7 s to ~458 ms), so a quiet tail is not evidence of a stop.
SCENARIO_MARKERS = {"SR_25", "SR_30"}


def marker_channel_for(count, stride, channels=8):
    """The highest analyzer channel no stepper owns, or None if there is none.

    The marker has to be readable without ambiguity against a stepper's own
    edges, so it cannot share a channel. At 8 steppers in `nodir` every channel
    is taken and there is no marker to be had -- which is a real limit of this
    approach, and the reason MARK is refused rather than quietly overwriting a
    step pin.
    """
    used = count * stride
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


def load_capture_for_eval(capture_file):
    """Load a recorded .sr as channels + rate, via its change-only VCD."""
    vcd_file = cap.sr_to_vcd(capture_file,
                             Path(str(capture_file).rsplit(".", 1)[0] + ".vcd"))
    if vcd_file is not None:
        return sp.load_vcd(str(vcd_file))
    return sp.load_capture(str(capture_file))


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
    rmt_v2 before this change: 7464 steps after the marker against a bound of
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
    # command, so 16 steps need ticks*16 >= MIN_CMD_TICKS. On rmt_v2 the floor
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
}

# The pulse driver of an architecture that has only one, used to expand the
# "native" spec above. ESP32 has several, so the default here is the one the
# measured baseline was taken on and any other run passes its own.
DEFAULT_NATIVE_DRIVER = "rmt_v2"


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
        """A filename- and tag-key-safe label. '+' and ',' would both leak."""
        return "+".join(self.drivers) + self.pin_mode + f"n{self.count}"


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


def evaluate(evaluator, channels, rate, segments, info, chan_map=None,
             extra=None):
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
    """
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
    m = sp.channel_metrics(pins.step_wave(channels, "A"), rate)
    detail = sp.period_defects(m.inter_step_us, expect_us)
    counts = sp.step_count_defects(m.step_count, segments[0][0])
    # ISR-driven architectures set the step pin from inside a timer interrupt,
    # so the achieved rate is systematically below the commanded one. That is
    # invisible to a step count and to a gross-period check.
    adherence = sp.rate_adherence(m.inter_step_us, expect_us)
    return detail["ok"] and counts["ok"] and adherence["ok"], {
        "ticks": ticks,
        "ticks_per_s": info["ticks_per_s"],
        "period": detail,
        "steps": counts,
        "adherence": adherence,
    }


def eval_step_count(channels, rate, segments, info, pins):
    ticks = segments[0][1]
    expect_us = ticks * 1e6 / info["ticks_per_s"]
    n = sum(steps for steps, _, _ in segments)
    step = pins.step_wave(channels, "A")
    m = sp.channel_metrics(step, rate)
    counts = sp.step_count_defects(m.step_count, n)
    detail = sp.period_defects(m.inter_step_us, expect_us)
    return counts["ok"] and detail["ok"], {
        "ticks": ticks,
        "steps": counts,
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
        m = sp.channel_metrics(channels[ch_name], rate)
        counts = sp.step_count_defects(m.step_count, expected)
        detail = sp.period_defects(m.inter_step_us, expect_us)
        ok = ok and counts["ok"] and detail["ok"]
        mean = (sum(m.inter_step_us) / len(m.inter_step_us)
                if m.inter_step_us else None)
        if mean is not None:
            means.append(mean)
        per_stepper[letter] = {
            "channel": ch_name,
            "steps": counts,
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


def first_step_skew_us(channels, rate, pins):
    """(skew in us, first-step time per stepper) across every mapped stepper.

    The skew is max(first) - min(first): it says how far apart the *extremes*
    are, so one straggler is not hidden by the others being close together.
    """
    firsts = {}
    for letter, ch_name in pins.items():
        if ch_name not in channels:
            continue
        edges = sp.rising_edges(channels[ch_name])
        if edges:
            firsts[letter] = edges[0]
    skew = ((max(firsts.values()) - min(firsts.values())) * 1e6 / rate
            if len(firsts) > 1 else 0.0)
    return skew, firsts


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
    """
    skew_us, firsts = first_step_skew_us(channels, rate, pins)
    ok = True
    per_stepper = {}
    for letter, ch_name in pins.items():
        idx = ord(letter) - ord("A")
        wave = pins.step_wave(channels, letter)
        if wave is None or idx not in programs:
            continue
        segs = programs[idx]
        ticks = segs[0][1]
        expected = sum(n for n, _, _ in segs)
        expect_us = ticks * 1e6 / info["ticks_per_s"]
        m = sp.channel_metrics(wave, rate)
        counts = sp.step_count_defects(m.step_count, expected)
        period = sp.period_defects(m.inter_step_us, expect_us)
        ok = ok and counts["ok"] and period["ok"]
        per_stepper[letter] = {
            "channel": ch_name,
            "ticks": ticks,
            "steps": counts,
            "period": period,
            "mean_period_us": round(sum(m.inter_step_us)
                                    / len(m.inter_step_us), 4)
                              if m.inter_step_us else None,
        }
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
        m = sp.channel_metrics(channels[ch_name], rate)
        counts = sp.step_count_defects(m.step_count, expected)
        detail = sp.period_defects(m.inter_step_us, expect_us)
        ok = ok and counts["ok"] and detail["ok"]
        per_stepper[letter] = {
            # Which channel this stepper was read on, so the record says the map
            # it used instead of leaving the reader to assume one.
            "channel": ch_name,
            "steps": counts,
            "period": detail,
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
    """
    step = pins.step_wave(channels, "A")
    dir_ch = pins.dir_wave(channels, "A")
    rises = sp.rising_edges(step)
    expected_steps = sum(steps for steps, _, _ in segments)
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

    return counts["ok"] and per_phase_ok and dir_ok, {
        "steps": counts,
        "phases": len(phases),
        "phases_expected": want_phases,
        "steps_per_phase": phases,
        "expected_steps": [n for n, _, _ in segments],
        "dir_edges": len(edges),
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
    m = sp.channel_metrics(pins.step_wave(channels, "A"), rate)
    expected = sum(steps for steps, _, _ in segments)
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
        "pause_found": gap_ok,
        "expected_gap_us": round(pause_us, 4) if pause_us else None,
    }


def eval_pulse_width(channels, rate, segments, info, pins):
    """The primary characterization output: high time and duty at one speed."""
    ticks = segments[0][1]
    expect_us = ticks * 1e6 / info["ticks_per_s"]
    m = sp.channel_metrics(pins.step_wave(channels, "A"), rate)
    counts = sp.step_count_defects(m.step_count, segments[0][0])
    detail = sp.period_defects(m.inter_step_us, expect_us)
    adherence = sp.rate_adherence(m.inter_step_us, expect_us)
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
        m = sp.channel_metrics(channels[ch_name], rate)
        counts = sp.step_count_defects(m.step_count, expected)
        period = sp.period_defects(m.inter_step_us, expect_us)
        ok = ok and counts["ok"] and period["ok"]
        detail[letter] = {
            "ticks": ticks,
            "steps": counts,
            "period": period,
            "mean_period_us": round(sum(m.inter_step_us)
                                    / len(m.inter_step_us), 4)
                              if m.inter_step_us else None,
        }

    skew = None
    a_rises = sp.rising_edges(pins.step_wave(channels, "A"))
    b_rises = sp.rising_edges(pins.step_wave(channels, "B"))
    if a_rises and b_rises:
        skew = round(abs(a_rises[0] - b_rises[0]) * 1e6 / rate, 4)
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
    and the residue it left (7655 steps on i2s_direct, 7608 on rmt_v2, against a
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
    ch = channels["D0"]
    m = sp.channel_metrics(ch, rate)
    t = segments[0][1]
    period = sp.period_defects(m.inter_step_us, t * 1e6 / info["ticks_per_s"])
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
    ch = channels["D0"]
    m = sp.channel_metrics(ch, rate)
    t = segments[0][1]
    period = sp.period_defects(m.inter_step_us, t * 1e6 / info["ticks_per_s"])
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
    m = sp.channel_metrics(pins.step_wave(channels, "A"), rate)
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
    expected_steps = sum(steps for steps, _, _ in segments)
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
    delays = sp.dir_to_first_step_us(dir_ch, step, rate)
    counts = sp.step_count_defects(len(sp.rising_edges(step)), expected_steps)
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
    for name, ch in pins.items():
        if ch not in channels:
            continue
        edges = sp.rising_edges(channels[ch])
        counts[name] = len(edges)
        if edges:
            firsts[name] = edges[0]
    skew_us = 0.0
    if len(firsts) > 1:
        skew_us = (max(firsts.values()) - min(firsts.values())) * 1e6 / rate
    expected = segments[0][0]
    period_us = segments[0][1] * 1e6 / info["ticks_per_s"]
    defects = {k: sp.step_count_defects(v, expected)
               for k, v in counts.items()}
    return all(d["ok"] for d in defects.values()), {
        "steps_per_stepper": defects,
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
# of the stop. Measured on rmt_v2: 7464 steps after the marker against a bound of
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
}


# ---------------------------------------------------------------------------
# Runner
# ---------------------------------------------------------------------------


def run_sr00(tag_key, args):
    """Capture the SR_00 pin pattern. Not a queue test; it gates the rest."""
    capture_file = Path(args.capture_dir) / f"sr00_{tag_key}.sr"
    rate = args.sr00_sample_rate
    ser = open_board(args.port, args.baud)
    try:
        proc = start_capture(capture_file, args.seconds, rate)
        time.sleep(0.3)
        send_line(ser, "SR00")
        proc.wait()
        replies = drain(ser, 0.3)
    finally:
        send_line(ser, "STOP")
        ser.close()

    channels, sample_rate = load_capture_for_eval(capture_file)
    passed, channel_results = analyze_csv.evaluate_sr00(channels, sample_rate)
    return ("passed" if passed else "failed"), {
        "sample_rate_hz": sample_rate,
        "channels": channel_results,
        "reply": replies.strip(),
    }


def measure(tag_key, name, wire, mask, builder, evaluator, args,
            per_stepper_for=None, scenario=None):
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

    `wire` is the whole CONFIG line. `evaluator` is (channels, rate, segments,
    info) -> (passed, detail), passed straight to evaluate() so the global pin
    invariants are checked for generated runs too.

    Returns (status, detail). `status` is `refused` when the board would not
    accept the configuration at all -- which is a *result* for `scale`, whose
    question is where the limit is, and a wiring problem for a named scenario.
    """
    ser = open_board(args.port, args.baud)
    try:
        text = reply_of(ser, wire)
        if "OK CONFIG" not in text:
            return "refused", {"error": text.strip(), "wire": wire}
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
        # whose move was shorter than that -- i2s_direct and rmt_v2 at a floor of
        # 80 ticks both run 20000 steps in 0.1 s -- STOP arrived after the move
        # had finished and the marker edge fell past the end of the delivered
        # capture. The run then reported a complete 20000-step move and "no
        # marker edge", which reads as "STOP was never processed".
        #
        # MARK is configuration, like CONFIG, so it belongs in the setup phase.
        marker = None
        if scenario in SCENARIO_MARKERS:
            want = marker_channel_for(len(chan_map), pin_map.get("stride", 2))
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
                return "failed", {"error": "QSEG rejected",
                                  "segments": segments}
        elif not program(ser, segments, check):
            return "failed", {"error": "QSEG rejected", "segments": segments}

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
                return "error", {"error": "QFILL did not reach the queue",
                                 "requested_entries": QUEUE_FILL_ENTRIES,
                                 "reply": drain(ser, 0.2).strip()}
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
        if "ERR QE" in replies and scenario not in REJECTION_SCENARIOS:
            detail = {"error": "queue rejected the command",
                      "firmware_reply": replies.strip(),
                      "segments": segments,
                      "entries_below_min_cmd_ticks": sub_min_entries(
                          segments if not programs else None, info, programs)}
            print(f"    {replies.strip().splitlines()[0]} -- a setup failure, "
                  f"not a measurement")
            return "error", detail
        send_line(ser, "POS")
        replies += drain(ser, 0.2)
    finally:
        send_line(ser, "QCLR")
        ser.close()

    channels, sample_rate = load_capture_for_eval(capture_file)
    # The marker channel is an optional 6th argument, so it goes through `extra`
    # rather than being appended for every evaluator. `extra` is the *value*,
    # forwarded positionally: eval_sync's 6th parameter is the programs dict and
    # gets it the same way.
    passed, detail = evaluate(evaluator, channels, sample_rate, segments, info,
                              chan_map, extra=marker)
    detail.update({
        "capture": str(capture_file),
        "channel_map": chan_map,
        "pin_map": pin_map,
        "sample_rate_hz": sample_rate,
        "capture_seconds_requested": round(seconds, 3),
        "segments": segments,
        "per_stepper_segments": programs,
        "reply": replies.strip(),
    })
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


def run_scenario(tag_key, test_id, args):
    """Program a scenario, capture it, and evaluate the waveform."""
    config, builder, mask, _desc = SCENARIOS[test_id]
    status, detail = measure(tag_key, test_id.lower(),
                             config_wire(config, args.dut_driver), mask,
                             builder, test_id, args,
                             per_stepper_builder(test_id),
                             scenario=test_id)
    # A named scenario that the board refuses is a wiring fault, not a finding,
    # so it is reported as a failure even though the shared path calls it
    # `refused` for the modes.
    return ("failed" if status == "refused" else status), detail


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
        status, detail = measure(key, plan.label, plan.wire, plan.mask,
                                 plan.builder, plan.evaluator, args,
                                 plan.per_stepper_builder)
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
    if any(t != "SR_00" for t in tests):
        tests = ["SR_00"] + tests

    sr00_failed = False
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
            print(f"{test_id}: SKIP (SR_00 failed)")
            record(index, index_file, tag_key, test_id, "skipped", None,
                   {"reason": "SR_00 failed"})
            continue

        print(f"{test_id}: running ...")
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
# second on rmt_v2 and mcpwm_pcnt, 67 for i2s_direct.
CONTRASTING_PAIRS = {frozenset(("SR_25", "SR_30"))}
