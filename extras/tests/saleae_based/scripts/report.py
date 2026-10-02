#!/usr/bin/env python3
"""Summarise Saleae runs as markdown and CSV.

Reads the per-scenario VCDs a hardware run leaves behind, evaluates each with the
same evaluator the harness uses, and prints the outcome. Two formats, because
they answer different questions: markdown for reading a run, CSV for comparing
architectures across sessions.

Every number here comes from the evaluator in run_tests.py, never from a
recomputation in this file. A report that measured things its own way would be
able to disagree with the tests, and then nobody would know which was right.

    python3 scripts/report.py /tmp/cap/hw            # markdown
    python3 scripts/report.py /tmp/cap/hw --csv      # csv
    python3 scripts/report.py --scenarios           # what is wired, no runs
"""
import argparse
import csv
import io
import sys
from pathlib import Path

_HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(_HERE))
# vcd_fixtures is the single source of the DUT's QINFO values, so the report
# derives its expectations from the same place the tests do rather than from a
# second set of constants.
sys.path.insert(0, str(_HERE / "tests"))

import run_tests as rt          # noqa: E402
import signal_parser as sp       # noqa: E402
import vcd_fixtures as vf        # noqa: E402

def scenario_ids():
    """Every wired scenario id, in SR order."""
    return sorted(rt.SCENARIOS, key=lambda sid: int(sid.split("_")[1]))


def evaluate_file(path, scenario, info, chan_map=None):
    """Evaluate one capture. Returns (ok, detail) or raises on unreadable input.

    `chan_map` is the run's own channel map. A capture recorded from a `nodir`
    run has to be judged with the `nodir` map or stepper B is read on `D2`
    instead of `D1` -- a quiet pin, reported as a driver that emits nothing.
    Callers that have the result record pass the map it carries.
    """
    channels, rate = load_once(path)
    segments = rt.SCENARIOS[scenario][1](info)
    pins = rt.Pins(chan_map) if chan_map is not None \
        else rt.Pins.for_scenario(scenario)
    return rt.evaluate(scenario, channels, rate, segments, info, pins.map)


_CACHE = {}


def load_once(path):
    """Load a VCD, caching by path.

    A 2.2 s capture at 24 MS/s is ~53M samples per channel and the parser is
    pure Python, so a second load of the same file costs seconds per scenario.
    The report loads each capture once and reuses it for both the evaluation and
    the capture length.
    """
    if path not in _CACHE:
        _CACHE[path] = sp.load_vcd(path)
    return _CACHE[path]


def steps_of(detail):
    """The step count a result reports, whichever key it used.

    Every evaluator spells its step count differently, and a scenario that
    measures "steps that would have been" against a rejected command reports 0
    measured -- which is the point of that test, not a missing value.
    """
    for key in ("steps", "per_stepper", "steps_per_phase"):
        if key in detail:
            value = detail[key]
            if isinstance(value, dict) and "steps_measured" in value:
                return value["steps_measured"]
            if isinstance(value, list):
                return sum(value)
    if "steps_measured" in detail:
        return detail["steps_measured"]
    if "steps_before_stop" in detail:
        return detail["steps_before_stop"]
    if "n_steps_measured" in detail:
        return detail["n_steps_measured"]
    return None


def expected_of(detail):
    for key in ("steps", "per_stepper"):
        if key in detail and isinstance(detail[key], dict) and \
                "steps_expected" in detail[key]:
            return detail[key]["steps_expected"]
    return detail.get("requested_steps")


def verdict_of(ok, detail):
    """PASS, FAIL, or a third state for results measured but not gated on.

    A scenario can succeed while reporting a number nobody likes -- the
    cross-driver start skew is the reason this exists. Printing FAIL there would
    be a lie, and printing PASS would bury the number.
    """
    if not ok:
        return "FAIL"
    skew = detail.get("first_step_skew_us")
    if skew:
        return "PASS(reported)"
    return "PASS"


def row_for(scenario, path, info, chan_map=None):
    try:
        ok, detail = evaluate_file(path, scenario, info, chan_map)
    except Exception as exc:                      # noqa: BLE001
        return {"scenario": scenario, "verdict": "ERROR",
                "steps_hw": "", "steps_exp": "", "note": str(exc)[:60],
                "capture_s": ""}
    channels, rate = load_once(path)
    first = next(iter(channels))
    return {
        "scenario": scenario,
        "verdict": verdict_of(ok, detail),
        "steps_hw": steps_of(detail),
        "steps_exp": expected_of(detail),
        "capture_s": round(len(channels[first]) / rate, 3),
        "note": note_of(detail),
    }


def note_of(detail):
    """The one line a reader needs, per scenario."""
    if "first_step_skew_us" in detail:
        return f"skew {detail['first_step_skew_us']} us (reported)"
    if "pause_us" in detail:
        gap = detail.get("measured_gaps_us") or []
        got = gap[0] if gap else None
        return f"pause {detail['pause_us']} us, gap {got} vs " \
               f"{detail['expected_gap_us']} us"
    if "steps_that_would_have_been" in detail:
        return f"{detail['steps_measured']} pulses for a rejected command " \
               f"({detail['steps_that_would_have_been']} requested)"
    if "unterminated_pulses" in detail:
        return f"stopped at {detail['steps_before_stop']} of " \
               f"{detail['requested_steps']}, " \
               f"partial pulses {detail['unterminated_pulses']}"
    if "per_stepper" in detail:
        means = {k: v.get("mean_period_us")
                 for k, v in detail["per_stepper"].items()}
        return "; ".join(f"{k} {v} us" for k, v in sorted(means.items()))
    period = detail.get("period")
    if isinstance(period, dict) and period.get("expected_period_us"):
        return f"period {period['expected_period_us']} us, " \
               f"{period.get('n_long', 0)} long / " \
               f"{period.get('n_short', 0)} short gaps"
    return ""


def collect(run_dir, info):
    rows = []
    for scenario in scenario_ids():
        path = run_dir / f"{scenario}.vcd"
        if path.exists():
            print(f"  evaluating {scenario}...", file=sys.stderr)
            rows.append(row_for(scenario, path, info))
    return rows


def as_markdown(rows, run_dir):
    out = [f"# Saleae run: {run_dir}", ""]
    if not rows:
        out.append("No captures found.")
        return "\n".join(out)
    out.append(f"{'scenario':9} {'verdict':15} {'steps':>13}  capture  note")
    out.append("-" * 78)
    for r in rows:
        steps = f"{r['steps_hw']}/{r['steps_exp']}" if r["steps_exp"] else \
                str(r["steps_hw"])
        out.append(f"{r['scenario']:9} {r['verdict']:15} {steps:>13}  "
                   f"{r['capture_s']:>5}s  {r['note']}")
    failed = [r for r in rows if r["verdict"] in ("FAIL", "ERROR")]
    out.append("")
    out.append(f"{len(rows)} scenarios, {len(failed)} failing.")
    return "\n".join(out)


def as_csv(rows):
    buf = io.StringIO()
    fields = ["scenario", "verdict", "steps_hw", "steps_exp", "capture_s", "note"]
    writer = csv.DictWriter(buf, fieldnames=fields)
    writer.writeheader()
    writer.writerows(rows)
    return buf.getvalue().rstrip()


def as_scenarios():
    """The wired catalogue, for when there are no runs to report on."""
    out = ["| scenario | config | mask | what it pins |", "|---|---|---|---|"]
    for scenario in scenario_ids():
        cfg, builder, mask, desc = rt.SCENARIOS[scenario]
        extra = ""
        if scenario in rt.STOP_AFTER:
            extra = f" (host issues STOP at {rt.STOP_AFTER[scenario]} s)"
        out.append(f"| {scenario} | {cfg} | {mask} | {desc}{extra} |")
    return "\n".join(out)


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("run_dir", nargs="?", default=None,
                    help="directory holding <scenario>.vcd captures")
    ap.add_argument("--csv", action="store_true", help="csv instead of markdown")
    ap.add_argument("--scenarios", action="store_true",
                    help="list the wired scenarios and exit")
    args = ap.parse_args()

    if args.scenarios:
        print(as_scenarios())
        return 0

    if not args.run_dir:
        ap.error("give a run directory, or use --scenarios")

    run_dir = Path(args.run_dir)
    info = vf.Dut().info()
    rows = collect(run_dir, info)
    print(as_csv(rows) if args.csv else as_markdown(rows, run_dir))
    return 1 if any(r["verdict"] in ("FAIL", "ERROR") for r in rows) else 0


if __name__ == "__main__":
    sys.exit(main())