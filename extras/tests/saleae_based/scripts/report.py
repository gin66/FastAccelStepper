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
    python3 scripts/report.py /tmp/r3/sync --results-dir /tmp/r3/sync
    python3 scripts/report.py --scenarios           # what is wired, no runs

A catalogue run leaves VCDs; a mode run (`--mode scale|sync`) leaves result JSON
and no captures. Both are reported: the markdown gains a parallel-count table and
a sync-permutation table, keyed by target and driver list, whenever the results
directory holds mode results.
"""
import argparse
import csv
import io
import json
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
    if "dir_per_phase" in detail:
        # SR_11/SR_12: say which way each phase actually went, not just how many
        # steps it took. A count that adds up while the direction is wrong is the
        # failure POS cannot see, and "FAIL" alone does not name it.
        got = detail.get("dir_per_phase") or []
        want = detail.get("expected_dir_per_phase") or []
        per = detail.get("steps_per_phase") or []
        return "dir " + "".join(str(d) for d in got) + \
            " vs " + "".join(str(d) for d in want) + \
            (f", steps {per}" if per else "")
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


def as_markdown(rows, run_dir, results_dir=None):
    out = [f"# Saleae run: {run_dir}", ""]
    if not rows:
        out.append("No captures found.")
        tables = as_mode_tables(results_dir)
        if tables:
            out += ["", tables]
            return "\n".join(out)
        out.append("")
        out.append("No mode results found either.")
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
    tables = as_mode_tables(results_dir)
    if tables:
        out += ["", tables]
    return "\n".join(out)


def as_csv(rows):
    buf = io.StringIO()
    fields = ["scenario", "verdict", "steps_hw", "steps_exp", "capture_s", "note"]
    writer = csv.DictWriter(buf, fieldnames=fields)
    writer.writeheader()
    writer.writerows(rows)
    return buf.getvalue().rstrip()


def target_of(record):
    """The arch / sdk a mode result was recorded on.

    Read from the record, not recovered from the tag key. The key encodes the
    same three things, but as a naming convention: `esp32_arduino_...` splits
    on two underscores and `esp32_idf5_3_0_...` on one, and a table of
    architecture results that depends on getting that string surgery right is a
    table that will quietly group two targets under one heading.
    """
    arch = record.get("arch") or "?"
    framework = record.get("framework") or "?"
    sdk = record.get("sdk_version")
    return f"{arch} / {framework}" + (f" / sdk {sdk}" if sdk and sdk != "?" else "")


def collect_modes(results_dir):
    """Every recorded mode result in a directory, one row per file.

    A mode run records JSON and no capture, so this is the whole of what there
    is to report on: there is no VCD to evaluate, which is why the mode tables
    read numbers rather than re-deriving them.
    """
    if not results_dir or not results_dir.is_dir():
        return []
    records = {}
    for path in sorted(results_dir.glob("*.json")):
        if path.name == "tag_index.json":
            continue
        try:
            record = json.loads(path.read_text())
        except (ValueError, OSError):
            continue
        # Only the catalogue filter lives here. Filtering on `mode` as well made
        # the unclassified table unreachable -- a record whose mode key was gone
        # or misspelt was discarded before anything could name it.
        if record.get("test_id") != "MODE":
            continue
        # Keyed by file name, not by the record's own tag_key. The filename is
        # the tag key, so the two agree in practice -- but keying on the field
        # meant a directory holding two records with a colliding key silently
        # reported one, and a table that drops a row without saying so is worse
        # than no table.
        records[path.stem] = record
    return [records[k] for k in sorted(records)]


def group_modes(records):
    """Mode records as (target, rows), ordered so a report is reproducible."""
    groups = {}
    for record in records:
        groups.setdefault(target_of(record), []).append(record)
    return [(t, groups[t]) for t in sorted(groups)]


def driver_list_of(record):
    return "+".join(record.get("drivers", [])) or "?"


def scale_rows(records):
    """One row per parallel-count point: every stepper's period and step count.

    The per-stepper cell is the measurement. A count that passes while a later
    stepper shows a short count or a wrong period is the failure this table
    exists to show, and collapsing it to "8/8 passed" would hide it.
    """
    rows = []
    for record in records:
        per_stepper = record.get("per_stepper") or {}
        cells = []
        for letter in sorted(per_stepper):
            e = per_stepper[letter]
            steps = (e.get("steps") or {}).get("steps_measured")
            period = e.get("mean_period_us")
            cell = f"{letter} {period}us" if period is not None else f"{letter} ?"
            if steps is not None:
                cell += f"x{steps}/{e['steps'].get('steps_expected')}"
            faults = []
            if (e.get("steps") or {}).get("extra_steps"):
                faults.append(f"+{e['steps']['extra_steps']}")
            if not ((e.get("steps") or {}).get("ok", True)
                    and (e.get("period") or {}).get("ok", True)):
                faults.append("DEVIATED")
            cells.append(cell + (" !" + ",".join(faults) if faults else ""))
        rows.append({
            "drivers": driver_list_of(record),
            "n": record.get("stepper_count", "?"),
            "pins": record.get("pin_mode", "?"),
            "steppers": " ".join(cells) or "-",
            "spread": record.get("period_spread_us"),
            "result": record.get("result", "?"),
            "error": record.get("error", ""),
        })
    return sorted(rows, key=lambda r: (r["drivers"], str(r["n"])))


def adherence_of(record):
    """Per-stepper 'kept its own period and count', or why it could not be said.

    The count is spelled out next to the period, and a deviation is named,
    because a bare `DEVIATED` cannot tell two very different failures apart --
    and in this table they sit next to each other. `mcpwm_pcnt+mcpwm_pcnt`
    posts the *smallest* first-step skew of any combination in the table
    (6.25 us) while being the one driver that does not stop. Its period is
    flawless: it emitted 10 883 steps at exactly the commanded 19.9991 us and
    never stopped. Reported as "skew 6.25 us" that row is the winner; reported
    as "skew 6.25 us, B emitted 10 883 steps where 64 were commanded" it is the
    bug the mode exists to find. A skew number alone would have reported the
    defect as the best result in the run.
    """
    per_stepper = record.get("per_stepper") or {}
    if not per_stepper:
        # The skew column already carries the reason; repeating it here would
        # make a refused row twice as wide as a measured one for no gain.
        return "-"
    cells = []
    for letter in sorted(per_stepper):
        e = per_stepper[letter]
        steps = e.get("steps") or {}
        period = e.get("period") or {}
        cell = f"{letter} {e.get('mean_period_us')}us"
        measured = steps.get("steps_measured")
        if measured is not None:
            cell += f" x{measured}/{steps.get('steps_expected')}"
        faults = []
        if steps.get("extra_steps"):
            faults.append(f"+{steps['extra_steps']} extra steps")
        if steps.get("missing_steps"):
            faults.append(f"-{steps['missing_steps']} missing")
        if not period.get("ok", True):
            faults.append(f"{period.get('n_long', 0)} long/"
                          f"{period.get('n_short', 0)} short periods")
        cells.append(cell + (" (" + ", ".join(faults) + ")" if faults else ""))
    return ", ".join(cells)


def sync_unmeasured(record):
    """Why a driver list has no skew to print. Never an empty cell.

    A blank skew column reads as "nothing to report" and is indistinguishable
    from a bug in the report. For a refused driver list the reason *is* the
    measurement, and it belongs in the table rather than in a log the reader
    has to go and find.
    """
    if record.get("result") == "refused":
        return f"not measured: refused ({record.get('error', 'no reason given')})"
    if record.get("result") != "passed":
        return f"not measured: {record.get('result')}"
    return "not measured: no first-step skew in the record"


def sync_rows(records):
    rows = []
    for record in records:
        skew = record.get("first_step_skew_us")
        rows.append({
            "drivers": driver_list_of(record),
            "n": record.get("stepper_count", "?"),
            "pins": record.get("pin_mode", "?"),
            "skew_us": f"{skew} us" if skew is not None
                       else sync_unmeasured(record),
            "periods": record.get("skew_periods"),
            "adherence": adherence_of(record),
            "result": record.get("result", "?"),
        })
    return sorted(rows, key=lambda r: (r["drivers"], str(r["n"])))


def as_scale_table(rows):
    if not rows:
        return None
    out = ["### Parallel stepper count (`scale`)", "",
           "Each row is one point of the count sweep: what every stepper "
           "measured, not just whether the run passed.",
           "",
           "| driver list | n | pins | steppers: period x steps | spread us |"
           " result |", "|---|---|---|---|---|---|"]
    for r in rows:
        spread = f"{r['spread']}" if r["spread"] is not None else "-"
        note = r["result"]
        if r["error"]:
            note += f" ({r['error'][:40]})"
        out.append(f"| {r['drivers']} | {r['n']} | {r['pins']} | "
                   f"{r['steppers']} | {spread} | {note} |")
    return "\n".join(out)


def as_sync_table(rows):
    if not rows:
        return None
    out = ["### Synced start (`sync`)", "",
           "Skew in microseconds *and* in step periods: the second is the "
           "readable one, since a period is what the machine was told to "
           "produce. A driver list that could not be measured says so in the "
           "skew column rather than printing an empty table.",
           "",
           "| driver list | n | pins | first-step skew | in step periods |"
           " per-stepper adherence | result |",
           "|---|---|---|---|---|---|---|"]
    for r in rows:
        periods = f"{r['periods']}" if r["periods"] is not None else "-"
        out.append(f"| {r['drivers']} | {r['n']} | {r['pins']} | "
                   f"{r['skew_us']} | {periods} | {r['adherence']} | "
                   f"{r['result']} |")
    return "\n".join(out)


def as_capability_table(records):
    """What each measured target's board said it accepts.

    A run's capability answer is a fact about the *build*, and it is the answer
    to "what else can this target do" -- so it belongs next to the results
    rather than in a log. It also settles an ambiguity the results alone cannot:
    `i2s_mux=1 mux_init=0` means the driver is compiled in but its three pins
    were never assigned, which is a different thing from a driver that is
    broken, and a reader shown only "i2s_mux refused" cannot tell them apart.
    """
    seen = {}
    for record in records:
        board = record.get("board_drivers")
        if not board:
            continue
        seen[target_of(record)] = (board, record.get("mux_init"))
    if not seen:
        return None
    out = ["### Driver capability", "",
           "Read from the board by the `DRIVERS` command, not from a host "
           "table. A host table could only be a copy of the library's declared "
           "`QUEUES_*` constants, and those count allocations rather than "
           "working steppers: `QUEUES_MCPWM_PCNT` is 6, the board does allocate "
           "six, and only one of them runs.",
           "",
           "| target | drivers this build accepts | i2s multiplexer |",
           "|---|---|---|"]
    for target, (board, mux_init) in sorted(seen.items()):
        accepts = ", ".join(sorted(d for d, ok in board.items() if ok)) or "-"
        rejects = ", ".join(sorted(d for d, ok in board.items() if not ok))
        if mux_init:
            mux = "up"
        elif board.get("i2s_mux"):
            # Present in the build, so a CONFIG naming it is accepted and then
            # fails to connect. That reads as a contradiction unless the
            # missing step is named: three pins have not been assigned.
            mux = "compiled in, not brought up (no pins assigned)"
        elif "i2s_mux" in board:
            mux = "compiled out by this build"
        else:
            # Not reported at all, which on a timer or pio build is the answer.
            mux = "not reported (no I2S on this target)"
        out.append(f"| {target} | {accepts} | {mux} |")
    return "\n".join(out)


def as_mode_tables(results_dir):
    """Both tables, grouped by target. Sections with nothing are left out."""
    records = collect_modes(results_dir)
    if not records:
        return None
    out = ["## Mode results", ""]
    any_table = False
    for target, group in group_modes(records):
        scale = as_scale_table(scale_rows([r for r in group
                                           if r.get("mode") == "scale"]))
        sync = as_sync_table(sync_rows([r for r in group
                                        if r.get("mode") == "sync"]))
        # A MODE record whose mode key is missing or misspelt belongs to neither
        # table, and filtering on it alone means the run disappears from the
        # report with nothing said. That is the worst outcome available: the
        # reader concludes it was never measured, which is a different claim.
        unclassified = [r for r in group
                        if r.get("mode") not in ("scale", "sync")]
        if not (scale or sync or unclassified):
            continue
        out += [f"### Target: {target}", ""]
        if scale:
            out += [scale, ""]
        if sync:
            out += [sync, ""]
        if unclassified:
            out += [as_unclassified_table(unclassified), ""]
        capability = as_capability_table(group)
        if capability:
            out += [capability, ""]
        any_table = True
    return "\n".join(out).rstrip() if any_table else None


def as_unclassified_table(records):
    """Mode records that fit neither table, named rather than dropped."""
    out = ["### Mode records with no recognised mode", "",
           "Each was recorded as a mode run but carries no mode this report "
           "knows how to tabulate. Listed so they cannot be mistaken for runs "
           "that were never made.",
           "",
           "| record | drivers | n | result |", "|---|---|---|---|"]
    for record in sorted(records, key=lambda r: r.get("tag_key", "")):
        out.append(f"| {record.get('tag_key', '?')} | "
                   f"{driver_list_of(record)} | "
                   f"{record.get('stepper_count', '?')} | "
                   f"{record.get('result', '?')} |")
    return "\n".join(out)


def as_scenarios():
    """The wired catalogue, for when there are no runs to report on."""
    out = ["| scenario | config | mask | what it pins |", "|---|---|---|---|"]
    for scenario in scenario_ids():
        cfg, builder, mask, desc = rt.SCENARIOS[scenario]
        extra = ""
        if scenario in rt.STOP_AFTER:
            extra = (" (host issues "
                     f"{rt.SCENARIO_STOP.get(scenario, 'STOP')} a quarter of "
                     "the way into the fill)")
        out.append(f"| {scenario} | {cfg} | {mask} | {desc}{extra} |")
    return "\n".join(out)


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("run_dir", nargs="?", default=None,
                    help="directory holding <scenario>.vcd captures")
    ap.add_argument("--csv", action="store_true", help="csv instead of markdown")
    ap.add_argument("--scenarios", action="store_true",
                    help="list the wired scenarios and exit")
    ap.add_argument("--results-dir", default=None,
                    help="directory holding mode result JSON. Defaults to "
                         "run_dir. A mode run (--mode scale|sync) records "
                         "JSON there and no VCDs, so this is how the "
                         "parallel-count and sync tables are built")
    args = ap.parse_args()

    if args.scenarios:
        print(as_scenarios())
        return 0

    if not args.run_dir:
        ap.error("give a run directory, or use --scenarios")

    run_dir = Path(args.run_dir)
    results_dir = Path(args.results_dir) if args.results_dir else run_dir
    info = vf.Dut().info()
    rows = collect(run_dir, info)
    print(as_csv(rows) if args.csv else as_markdown(rows, run_dir, results_dir))
    return 1 if any(r["verdict"] in ("FAIL", "ERROR") for r in rows) else 0


if __name__ == "__main__":
    sys.exit(main())