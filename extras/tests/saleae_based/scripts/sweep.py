#!/usr/bin/env python3
"""Sweep a scenario across a parameter and tabulate the results.

The point of a sweep is coverage a single run cannot give: one capture at
ticks=640 says nothing about ticks=3200 or steps=1, and the interesting
boundaries are precisely the ones a hand-picked scenario would skip.

Every sweep point is legal before anything is run. That check is the reason
this script builds its points through the same `legal_ticks` helper the
scenarios use rather than picking values by hand -- a sweep that included an
illegal point would spend most of its runtime measuring the firmware's refusal
to accept it, and would report that as a defect in the step output.

    python3 scripts/sweep.py --list                # what would be run
    python3 scripts/sweep.py SR_02                 # run it against hardware
    python3 scripts/sweep.py SR_05 --csv

Requires the board and a Saleae connected; see report.py for reading results.
"""
import argparse
import csv
import io
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
sys.path.insert(0, str(Path(__file__).resolve().parent / "tests"))

import report                    # noqa: E402
import run_tests as rt           # noqa: E402
import signal_parser as sp       # noqa: E402
import vcd_fixtures as vf        # noqa: E402

# steps values for SR_02. The ends matter more than the middle: 1 is the only
# command that produces no inter-step period at all and takes the other branch
# in the ISR, 2 crosses the ticks*steps boundary, and 255 is uint8_t's maximum
# and the count where the queue has to refill mid-command.
SR_02_STEPS = [1, 2, 3, 4, 8, 16, 32, 63, 64, 100, 127, 128, 200, 250,
               251, 252, 253, 254, 255]

# ticks values for SR_05, spanning the legal range rather than sampling it
# evenly: the 16-bit boundary, the speed floor, the fastest legal period, and
# the values either side of each.
SR_05_TICKS = [3200, 4000, 8000, 16000, 32768, 65535]


def points(scenario, info):
    """[(label, segments)] for one sweep. Empty for an unknown scenario."""
    if scenario == "SR_02":
        return [(f"steps={n}",
                 [(n, rt.legal_ticks(info, n, info["max_speed_ticks"]), True)])
                for n in SR_02_STEPS]
    if scenario == "SR_05":
        # One step, so ticks and ticks*steps coincide and the pulse high time is
        # the only thing varying. Two steps would confound the two.
        return [(f"ticks={t}", [(1, rt.legal_ticks(info, 1, t), True)])
                for t in SR_05_TICKS]
    return []


def legality(scenario, segments, info):
    """Why a point is unusable, or None if it is fine.

    Mirrors the firmware's admission test. Catching an illegal point here is
    the whole reason the sweep plans itself: `steps=1` at ticks below the floor
    is refused with ErrorTicksTooLow, and measuring that refusal once per sweep
    would tell us nothing new 19 times over.
    """
    for steps, ticks, _ in segments:
        rate = ticks * (steps if steps > 1 else 1)
        if rate < info["min_cmd_ticks"]:
            return (f"{steps} steps at {ticks} ticks is {rate} ticks of motion, "
                    f"below the floor of {info['min_cmd_ticks']}")
        if ticks > 65535:
            return f"ticks={ticks} does not fit the 16-bit field"
    return None


def plan(scenario, info):
    """[(label, segments, problem_or_None)] for every point."""
    return [(label, segs, legality(scenario, segs, info))
            for label, segs in points(scenario, info)]


def as_markdown(scenario, rows):
    out = [f"# Sweep: {scenario}", "",
           "| point | verdict | steps hw/exp | period us | note |",
           "|---|---|---|---|---|"]
    for label, verdict, hw, exp, period, note in rows:
        steps = f"{hw}/{exp}" if exp else str(hw)
        out.append(f"| {label} | {verdict} | {steps} | {period} | {note} |")
    bad = [r for r in rows if r[1] in ("FAIL", "ERROR")]
    out.append("")
    out.append(f"{len(rows)} points, {len(bad)} failing.")
    return "\n".join(out)


def as_csv(rows):
    buf = io.StringIO()
    writer = csv.writer(buf)
    writer.writerow(["point", "verdict", "steps_hw", "steps_exp", "period_us",
                     "note"])
    writer.writerows(rows)
    return buf.getvalue().rstrip()


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("scenario", nargs="?", help="SR_02 or SR_05")
    ap.add_argument("--list", action="store_true",
                    help="print the sweep plan and any illegal points, run nothing")
    ap.add_argument("--run-dir", default=None,
                    help="directory holding <scenario>_<label>.vcd captures")
    ap.add_argument("--csv", action="store_true")
    args = ap.parse_args()

    if not args.scenario:
        ap.error("give a scenario, or use --list")
    info = vf.Dut().info()

    if args.list:
        print(as_plan(args.scenario, info))
        return 0

    if not args.run_dir:
        ap.error("give --run-dir with the captures to tabulate")

    rows = []
    for label, segs, problem in plan(args.scenario, info):
        if problem:
            rows.append((label, "ILLEGAL", "", "", "", problem))
            continue
        path = Path(args.run_dir) / f"{args.scenario}_{label.replace('=', '_')}.vcd"
        if not path.exists():
            rows.append((label, "MISSING", "", "", "", "no capture"))
            continue
        rows.append(evaluate_point(args.scenario, label, segs, path, info))

    print(as_csv(rows) if args.csv else as_markdown(args.scenario, rows))
    return 1 if any(r[1] in ("FAIL", "ERROR", "ILLEGAL", "MISSING")
                    for r in rows) else 0


def evaluate_point(scenario, label, segments, path, info):
    """Evaluate one sweep capture with the scenario's own evaluator."""
    channels, rate = sp.load_vcd(path)
    ok, detail = rt.evaluate(scenario, channels, rate, segments, info)
    period = detail.get("period")
    mean = ""
    if isinstance(period, dict) and period.get("expected_period_us"):
        mean = period["expected_period_us"]
    return (label, report.verdict_of(ok, detail), report.steps_of(detail),
            report.expected_of(detail), mean, report.note_of(detail))


def as_plan(scenario, info):
    rows = plan(scenario, info)
    if not rows:
        return f"{scenario}: no sweep defined"
    out = [f"# Sweep plan: {scenario}", "",
           "| point | segments | legal |", "|---|---|---|"]
    for label, segs, problem in rows:
        out.append(f"| {label} | {segs} | {'yes' if problem is None else problem} |")
    bad = [r for r in rows if r[2]]
    out.append("")
    out.append(f"{len(rows)} points, {len(bad)} illegal.")
    return "\n".join(out)


if __name__ == "__main__":
    sys.exit(main())