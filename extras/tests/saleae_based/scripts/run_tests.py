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

Usage:
    python3 scripts/run_tests.py --list
    python3 scripts/run_tests.py --tag-key esp32_idf5_3_0_mcpwm_pcnt_2ch
    python3 scripts/run_tests.py --tag-key ... --tests SR_01,SR_05
    python3 scripts/run_tests.py --tag-key ... --force
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

ALL_TESTS = ["SR_00"] + [f"SR_{i:02d}" for i in range(1, 27)]

# Step channels for steppers A..D (white paper §3.3).
STEP_CHANNELS = {"A": "D0", "B": "D2", "C": "D4", "D": "D6"}
DIR_CHANNELS = {"A": "D1", "B": "D3", "C": "D5", "D": "D7"}

# Default capture rate. 4 MS/s is the practical minimum to resolve a pulse a
# few us wide at 16 MHz (see README).
DEFAULT_RATE = 4_000_000


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


QINFO_RE = re.compile(r"tps=(\d+) mincmd=(\d+) qlen=(\d+) maxspeed=(\d+)")


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
            return {
                "ticks_per_s": int(m.group(1)),
                "min_cmd_ticks": int(m.group(2)),
                "queue_len": int(m.group(3)),
                "max_speed_ticks": int(m.group(4)),
            }
        time.sleep(0.1)
    raise RuntimeError(f"no QINFO reply, got: {text!r}")


def program(ser, segments):
    """Send QCLR followed by one QSEG per segment. Returns False on error."""
    text = reply_of(ser, "QCLR")
    if "OK QCLR" not in text:
        return False
    for steps, ticks, count_up in segments:
        text = reply_of(ser, f"QSEG {steps} {ticks} {1 if count_up else 0}")
        if "OK QSEG" not in text:
            print(f"    QSEG {steps} {ticks} rejected: {text.strip()}")
            return False
    return True


# ---------------------------------------------------------------------------
# Capture
# ---------------------------------------------------------------------------


def start_capture(output, seconds, rate):
    devices, driver = cap.detect_analyzer("auto")
    channels = ",".join(sorted(set(list(STEP_CHANNELS.values()) +
                                   list(DIR_CHANNELS.values()))))
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
    return max(wanted, need, 160)


def sc_period_exact(info):
    # Comfortably fast, but well inside the 16-bit range.
    return seg_period(8, legal_ticks(info, 8, info["max_speed_ticks"]))


def sc_steps_per_command(info):
    return seg_period(255, legal_ticks(info, 255, info["max_speed_ticks"]))


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


def sc_pulse_high_time(info):
    # 16 steps is enough to measure a stable high time and still short.
    return seg_period(16, max(info["max_speed_ticks"], 160))


def sc_trailing_wait(info):
    t = legal_ticks(info, 2, info["max_speed_ticks"])
    return [(2, t, True), (2, t, True)]


def sc_long_run(info):
    return seg_period(2000, max(info["max_speed_ticks"], 160))


def sc_queue_full(info):
    return seg_period(4000, legal_ticks(info, 4000, info["max_speed_ticks"]))


def sc_pause(info):
    t = max(info["max_speed_ticks"], 160)
    pause = min(65535, t * 20)
    return [(5, t, True), (0, pause, True), (5, t, True)]


def sc_dir_change(info):
    t = max(info["max_speed_ticks"], 160)
    return [(20, t, True), (20, t, False)]


def sc_sync_start(info):
    t = legal_ticks(info, 2000, info["max_speed_ticks"])
    return seg_period(2000, t)


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
    "SR_14": ("2ch", sc_sync_start, 3, "2 steppers, synchronized start"),
    "SR_27": ("1ch", sc_single_step, 1, "single step in one command"),
}


# ---------------------------------------------------------------------------
# Evaluation
# ---------------------------------------------------------------------------


def check_pin_invariants(channels, rate):
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
    anywhere in the program, and a per-scenario check would only catch it in the
    one scenario that happens to change direction.
    """
    per_stepper = {}
    total = 0
    for name, step_ch in STEP_CHANNELS.items():
        dir_ch = DIR_CHANNELS.get(name)
        if step_ch not in channels or dir_ch not in channels:
            continue
        conflicts = sp.dir_changes_during_step_high(
            channels[dir_ch], channels[step_ch], rate)
        if conflicts:
            per_stepper[name] = conflicts
        total += len(conflicts)
    return {
        "n_dir_while_step_high": total,
        "dir_while_step_high": per_stepper,
        "ok": total == 0,
    }


def evaluate(test_id, channels, rate, segments, info):
    """Run a scenario's evaluator, then the global invariants.

    Every result carries the invariant block, and a violation fails the test
    regardless of what the scenario's own checks concluded.
    """
    ok, detail = EVALUATORS[test_id](channels, rate, segments, info)
    inv = check_pin_invariants(channels, rate)
    detail = dict(detail)
    detail["invariants"] = inv
    return ok and inv["ok"], detail


def eval_period_exact(channels, rate, segments, info):
    """Inter-step period must equal the commanded ticks, in microseconds."""
    ticks = segments[0][1]
    expect_us = ticks * 1e6 / info["ticks_per_s"]
    m = sp.channel_metrics(channels[STEP_CHANNELS["A"]], rate)
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


def eval_step_count(channels, rate, segments, info):
    ticks = segments[0][1]
    expect_us = ticks * 1e6 / info["ticks_per_s"]
    n = sum(steps for steps, _, _ in segments)
    step = channels[STEP_CHANNELS["A"]]
    m = sp.channel_metrics(step, rate)
    counts = sp.step_count_defects(m.step_count, n)
    detail = sp.period_defects(m.inter_step_us, expect_us)
    return counts["ok"] and detail["ok"], {
        "ticks": ticks,
        "steps": counts,
        "period": detail,
    }


def eval_pulse_width(channels, rate, segments, info):
    """The primary characterization output: high time and duty at one speed."""
    ticks = segments[0][1]
    expect_us = ticks * 1e6 / info["ticks_per_s"]
    m = sp.channel_metrics(channels[STEP_CHANNELS["A"]], rate)
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


def eval_pause(channels, rate, segments, info):
    """A pause (steps=0) must produce exactly its tick count of silence.

    A pause far shorter than commanded means pulses arrived during it, which is
    a defect rather than a measurement.
    """
    ticks = segments[0][1]
    pause_ticks = segments[1][1]
    pause_us = pause_ticks * 1e6 / info["ticks_per_s"]
    m = sp.channel_metrics(channels[STEP_CHANNELS["A"]], rate)
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


def eval_dir_change(channels, rate, segments, info):
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
    step = channels[STEP_CHANNELS["A"]]
    dir_ch = channels[DIR_CHANNELS["A"]]
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


def eval_sync_start(channels, rate, segments, info):
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
    for name, ch in STEP_CHANNELS.items():
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
    "SR_14": eval_sync_start,
    "SR_27": eval_period_exact,
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


def run_scenario(tag_key, test_id, args):
    """Program a scenario, capture it, and evaluate the waveform."""
    config, builder, mask, _desc = SCENARIOS[test_id]
    ser = open_board(args.port, args.baud)
    try:
        text = reply_of(ser, f"CONFIG {config}")
        if "OK CONFIG" not in text:
            return "failed", {"error": text.strip()}
        info = read_qinfo(ser)

        segments = builder(info)
        if not program(ser, segments):
            return "failed", {"error": "QSEG rejected", "segments": segments}

        seconds = scenario_seconds(segments, info["ticks_per_s"]) + 0.5
        rate = args.sample_rate
        capture_file = Path(args.capture_dir) / f"{test_id.lower()}_{tag_key}.sr"

        proc = start_capture(capture_file, seconds, rate)
        time.sleep(0.3)  # let sigrok-cli start sampling
        send_line(ser, f"QRUN {mask}")
        proc.wait()
        replies = drain(ser, 0.4)
        send_line(ser, "POS")
        replies += drain(ser, 0.2)
    finally:
        send_line(ser, "QCLR")
        ser.close()

    channels, sample_rate = load_capture_for_eval(capture_file)
    passed, detail = evaluate(test_id, channels, sample_rate, segments, info)
    detail.update({
        "capture": str(capture_file),
        "sample_rate_hz": sample_rate,
        "capture_seconds_requested": round(seconds, 3),
        "segments": segments,
        "reply": replies.strip(),
    })
    # The DUT's tick rate is what makes the ticks in `segments` interpretable,
    # so it travels with every result.
    detail["dut"] = info
    return ("passed" if passed else "failed"), detail


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