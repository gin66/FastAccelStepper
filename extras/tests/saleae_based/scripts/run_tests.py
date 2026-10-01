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
    python3 scripts/run_tests.py --tag-key esp32_rmt_v2_8ch_step_only --force

Behaviour:
    * tests already recorded `passed` for the tag key are skipped (unless
      --force); use --force to re-measure.
    * SR_00 gates the rest: if it fails, later tests are recorded `skipped`.
    * results live in results/<tag_key>_<test>.json and results/tag_index.json.
"""

import argparse
import json
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
IMPLEMENTED = ["SR_00"]
ALL_TESTS = [f"SR_{i:02d}" for i in range(0, 41)]


class Options:
    def __init__(self, sample_rate, seconds, channels, output, strict):
        self.driver = "auto"
        self.channels = channels
        self.sample_rate = sample_rate
        self.seconds = seconds
        self.output = output
        self.trigger = None
        self.strict = strict


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


def run_sr00(tag_key, args):
    """Capture the pins and evaluate SR_00. Returns (result, results_dict)."""
    capture_file = Path(args.capture)
    opts = Options(args.sample_rate, args.seconds, "0,1,2,3,4,5,6,7",
                   str(capture_file), args.force)
    rc = cap.run_capture(opts)
    if rc != 0:
        return "error", None

    channels, sample_rate = sp.load_csv(str(capture_file))
    passed, channel_results = analyze_csv.evaluate_sr00(channels, sample_rate)
    return ("passed" if passed else "failed"), {
        "sample_rate_hz": sample_rate,
        "channels": channel_results,
    }


RUNNERS = {"SR_00": run_sr00}


def run(tag_key, tests, args):
    results_dir = Path(args.results_dir)
    index_file = results_dir / "tag_index.json"
    index = load_index(index_file)

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
    print(f"{'TEST':7} {'STATUS':10} TAG KEY")
    for tag_key in sorted(index) or ["<none recorded>"]:
        for test_id in ALL_TESTS:
            entry = index.get(tag_key, {}).get(test_id)
            if entry:
                print(f"{test_id:7} {entry['result']:10} {tag_key}")
    if not index:
        print("(no results recorded yet)")


def parse_tests(text):
    if not text:
        return ALL_TESTS
    return [t.strip().upper() for t in text.split(",") if t.strip()]


def main():
    p = argparse.ArgumentParser(description="Saleae SR_xx test orchestrator.")
    p.add_argument("--tag-key", help="tag key {arch}_{driver}_{channel_config}")
    p.add_argument("--tests", help="comma list (default: all)")
    p.add_argument("--sample-rate", type=int, default=1_000_000)
    p.add_argument("--seconds", type=float, default=5.0)
    p.add_argument("--capture", default="capture.csv",
                   help="capture file (default: capture.csv)")
    p.add_argument("--results-dir", default="results")
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
