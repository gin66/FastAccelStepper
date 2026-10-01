#!/usr/bin/env python3
"""
run_tests.py — SR_xx test orchestrator.

Runs the implemented Saleae tests for one hardware tag key, records each result
as a JSON file, and updates a tag index so already-passed tests are skipped on
the next run (idempotent, hardware-friendly: a full matrix can be resumed).

Tag key (white paper §2.3.3) = {arch}_{driver}_{channel_config}, e.g.
`esp32_rmt_v2_8ch_step_only`. It is the index key: a result belongs to the
combination of hardware/driver/config it was measured on.

Usage:
    python3 scripts/run_tests.py --list
    python3 scripts/run_tests.py --tag-key esp32_rmt_v2_8ch_step_only
    python3 scripts/run_tests.py --tag-key esp32_rmt_v2_8ch_step_only --tests SR_01
    python3 scripts/run_tests.py --tag-key ... --force

Behaviour:
    * tests already recorded `passed` for the tag key are skipped (unless
      --force); use --force to re-measure.
    * one capture per test.
    * SR_00 is the standard pre-check and always runs first; if it fails,
      later tests are recorded `skipped`.
    * results live in results/<tag_key>_<test>.json and results/tag_index.json.
"""

import argparse
import json
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

# Implemented tests, in run order. SR_00 must pass before the others run.
IMPLEMENTED = ["SR_00", "SR_01"]
ALL_TESTS = [f"SR_{i:02d}" for i in range(0, 41)]

# Pin roles (white paper §3.3): CH0 = step, CH1 = dir for stepper A.
STEP_CH = "D0"
DIR_CH = "D1"
CAPTURE_CHANNELS = "D0,D1,D2,D3,D4,D5,D6,D7"


def open_board(port, baud, timeout=4.0):
    """Open the serial port (resets the board) and wait for READY."""
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


def send_and_wait(ser, cmd, expect, timeout=5.0):
    """Send a line and read until `expect` appears. Returns the reply text."""
    ser.write((cmd + "\n").encode())
    deadline = time.time() + timeout
    buf = b""
    while time.time() < deadline:
        data = ser.read(256)
        if data:
            buf += data
            if expect.encode() in buf:
                return buf.decode(errors="replace")
    return buf.decode(errors="replace")


def start_capture(args, output, seconds):
    devices, driver = cap.detect_analyzer("auto")
    cmd = cap.build_command(driver, args.sample_rate, CAPTURE_CHANNELS,
                            int(seconds * 1000), output, None)
    return subprocess.Popen(cmd, stdout=subprocess.DEVNULL,
                            stderr=subprocess.DEVNULL)


def serial_capture(args, capture_file, seconds, pre_commands, commands):
    """Capture around a serial-triggered test.

    Order (the capture must be running before the test starts and must
    outlive it so the full waveform is recorded):
        1. open the port (resets the board) and run any pre_commands
        2. start the capture
        3. send the test command(s)
        4. wait for the capture to complete
        5. drain the serial replies
    """
    ser = open_board(args.port, args.baud)
    replies = ""
    try:
        for c in pre_commands:
            replies += send_and_wait(ser, c, "OK", timeout=2.0)

        proc = start_capture(args, str(capture_file), seconds)
        time.sleep(0.3)  # let sigrok-cli start sampling

        for c in commands:
            ser.write((c + "\n").encode())
            time.sleep(0.05)

        proc.wait()  # capture owns the timing; wait for all of it

        time.sleep(0.2)
        while True:
            data = ser.read(4096)
            if not data:
                break
            replies += data.decode(errors="replace")
    finally:
        ser.close()
    return replies


def run_sr00(tag_key, args):
    """Capture the SR_00 pin pattern (started by the host, not on boot)."""
    capture_file = Path("/tmp") / f"sr00_{tag_key}.csv"
    serial_capture(args, capture_file, args.seconds, [], ["SR00"])

    channels, sample_rate = sp.load_csv(str(capture_file))
    passed, channel_results = analyze_csv.evaluate_sr00(channels, sample_rate)
    return ("passed" if passed else "failed"), {
        "sample_rate_hz": sample_rate,
        "channels": channel_results,
    }


def run_sr01(tag_key, args):
    """Capture a constant-speed move and count the step pulses."""
    capture_file = Path("/tmp") / f"sr01_{tag_key}.csv"
    # Capture must cover the whole move: steps * us/step plus ramp margin.
    move_s = args.steps * args.speed_us / 1_000_000.0
    seconds = max(args.seconds, move_s + 1.0)
    reply = serial_capture(args, capture_file, seconds, ["STOP"],
                           [f"SR01 {args.steps} {args.speed_us}"])

    channels, sample_rate = sp.load_csv(str(capture_file))
    step_count = len(sp.rising_edges(channels.get(STEP_CH, [])))
    dir_edges = len(sp.detect_edges(channels.get(DIR_CH, [])))
    passed = (step_count == args.steps)
    return ("passed" if passed else "failed"), {
        "steps_expected": args.steps,
        "steps_measured": step_count,
        "speed_us": args.speed_us,
        "dir_edges": dir_edges,
        "reply": reply.strip(),
    }


RUNNERS = {"SR_00": run_sr00, "SR_01": run_sr01}


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
    index_file = results_dir / "tag_index.json"
    index = load_index(index_file)

    # SR_00 is the standard pre-check that all I/Os work. Always run it first
    # (one capture per test), so a later test cannot be measured on dead or
    # mis-wired channels.
    if "SR_00" not in tests and any(t != "SR_00" for t in tests):
        tests = ["SR_00"] + tests

    sr00_failed = False
    for test_id in tests:
        prev = index.get(tag_key, {}).get(test_id)
        if prev and prev.get("result") == "passed" and not args.force:
            print(f"{test_id}: SKIP (already passed {prev['timestamp']})")
            continue

        if test_id not in IMPLEMENTED:
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
        result, extra = RUNNERS[test_id](tag_key, args)
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
    p = argparse.ArgumentParser(description="Saleae SR_xx test orchestrator.")
    p.add_argument("--tag-key", help="tag key {arch}_{driver}_{channel_config}")
    p.add_argument("--tests", help="comma list (default: all)")
    p.add_argument("--sample-rate", type=int, default=1_000_000)
    p.add_argument("--seconds", type=float, default=5.0,
                   help="capture seconds (SR_00)")
    p.add_argument("--capture", default="capture.csv",
                   help="SR_00 capture file (default: capture.csv)")
    p.add_argument("--results-dir", default="results")
    p.add_argument("--port", default="/dev/cu.usbserial-0001")
    p.add_argument("--baud", type=int, default=115200)
    p.add_argument("--steps", type=int, default=400, help="SR_01 steps")
    p.add_argument("--speed-us", type=int, default=400, help="SR_01 us/step")
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
