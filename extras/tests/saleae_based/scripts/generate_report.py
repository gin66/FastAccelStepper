#!/usr/bin/env python3
"""Generate the human-readable markdown report from recorded run results.

Reads the JSON records `run_hardware.py --results` writes and produces the
artefacts the white paper's section 8 asks for: a dashboard, one detail page per
test, a spec-compliance table, a regression view against a baseline, per-tag
summaries, and a CSV export.

Every number here comes out of a result file. This program formats; it never
re-parses a capture, so it cannot disagree with the run that produced the
measurements.

    python3 scripts/generate_report.py --results /tmp/cap/hw/results
    python3 scripts/generate_report.py --results ... --out docs/reports
    python3 scripts/generate_report.py --results ... --baseline /tmp/base
"""
import argparse
import csv
import io
import json
import sys
from collections import defaultdict
from datetime import datetime, timezone
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
sys.path.insert(0, str(Path(__file__).resolve().parent / "tests"))

import signal_parser as sp       # noqa: E402
import vcd_fixtures as vf        # noqa: E402

# Metrics the paper's CSV names, mapped onto what a result record carries. A
# column is left out rather than filled with a placeholder: a blank cell reads as
# "not measured", which is true, where a zero would read as "measured zero".
CSV_COLUMNS = ["timestamp", "test_id", "arch", "driver", "channel_config",
               "stepper", "ticks", "ticks_per_s", "step_count", "expected_count",
               "period_us", "period_min_us", "period_max_us", "period_spread_us",
               "avg_high_us", "high_min_us", "high_max_us", "avg_low_us",
               "duty_percent", "pass"]


def load_results(results_dir):
    """Every result record, sorted by test id then timestamp."""
    out = []
    for path in sorted(Path(results_dir).glob("*.json")):
        try:
            out.append(json.loads(path.read_text()))
        except ValueError:
            # A half-written file from an interrupted run must not take the
            # whole report down; one missing test is better than no report.
            print(f"warning: skipping unreadable {path.name}", file=sys.stderr)
    return sorted(out, key=lambda r: (r["test_id"], r.get("timestamp", "")))


def latest_per_test(results):
    """The newest record for each test, so a re-run supersedes its predecessor."""
    best = {}
    for r in results:
        key = r["test_id"]
        if key not in best or r.get("timestamp", "") >= best[key].get(
                "timestamp", ""):
            best[key] = r
    return [best[k] for k in sorted(best,
                                    key=lambda t: int(t.split("_")[1]))]


def expected_steps(result):
    """Steps the command program asked for, summed over its segments."""
    return sum(n for n, _t, _u in result.get("segments", []))


def fmt(value, digits=4):
    if value is None:
        return "—"
    if isinstance(value, float):
        return f"{value:.{digits}f}".rstrip("0").rstrip(".") or "0"
    return str(value)


def verdict(result):
    return "PASS" if result.get("pass") else "FAIL"


def measured_note(result):
    """The one-line finding for a result, chosen by what makes it interesting."""
    detail = result.get("evaluator_detail", {})
    if "first_step_skew_us" in detail:
        return f"skew {fmt(detail['first_step_skew_us'])} us"
    if "unterminated_pulses" in detail:
        return (f"stopped at {detail['steps_before_stop']} of "
                f"{detail['requested_steps']}, "
                f"partial pulses {detail['unterminated_pulses']}")
    if "pause_us" in detail:
        # The measured list holds every gap wider than a pulse, which still
        # includes the ordinary inter-step periods. Taking the first would
        # report a 40 us gap for an 800 us pause, so take the widest: for a
        # pause, the pause is the widest thing in the list by construction.
        gaps = detail.get("measured_gaps_us") or []
        widest = max(gaps) if gaps else None
        return (f"pause {fmt(detail['pause_us'])} us → gap {fmt(widest)} us "
                f"(expected {fmt(detail.get('expected_gap_us'))})")
    m = result.get("measurements", {})
    a = m.get("A", {}).get("inter_step_us", {})
    if a.get("min") is not None:
        return f"period {fmt(a['min'])}–{fmt(a['max'])} us"
    return ""


# ---------------------------------------------------------------------------
# Tables
# ---------------------------------------------------------------------------

def results_table(results):
    """The main table: one row per test, measurements inline."""
    head = ("| test | verdict | steps | period us (min–max) | pulse high us "
            "(min–max) | tag | note |")
    out = [head, "|---|---|---|---|---|---|---|"]
    for r in results:
        m = r.get("measurements", {})
        a = m.get("A", {})
        per = a.get("inter_step_us", {})
        high = a.get("pulse_high_us", {})
        steps = f"{a.get('step_count', '—')}/{expected_steps(r)}" \
            if expected_steps(r) else str(a.get("step_count", "—"))
        period = (f"{fmt(per.get('min'))}–{fmt(per.get('max'))}"
                  if per.get("min") is not None else "—")
        high_s = (f"{fmt(high.get('min'))}–{fmt(high.get('max'))}"
                  if high.get("min") is not None else "—")
        out.append(f"| {r['test_id']} | {verdict(r)} | {steps} | {period} | "
                   f"{high_s} | {r.get('tag', '—')} | {measured_note(r)} |")
    return "\n".join(out)


def stepper_table(result):
    """Per-stepper measurements, stepper by stepper."""
    out = ["| stepper | steps | period us (min–max, spread) | pulse high us "
           "(min–max, spread) | duty % |", "|---|---|---|---|---|"]
    for letter, m in sorted(result.get("measurements", {}).items()):
        per = m.get("inter_step_us", {})
        high = m.get("pulse_high_us", {})
        out.append(
            f"| {letter} | {m.get('step_count', '—')} | "
            f"{fmt(per.get('min'))}–{fmt(per.get('max'))}, "
            f"{fmt(per.get('spread'))} | "
            f"{fmt(high.get('min'))}–{fmt(high.get('max'))}, "
            f"{fmt(high.get('spread'))} | "
            f"{fmt(m.get('duty_cycle_percent'), 2)} |")
    return "\n".join(out)


def spec_table(results):
    """Measured against the design spec each result carries."""
    out = ["| test | metric | expected | measured | verdict |", "|---|---|---|---|---|"]
    for r in results:
        ticks = None
        segs = r.get("segments", [])
        if segs:
            ticks = segs[0][1]
        tps = r.get("dut", {}).get("ticks_per_s")
        for letter, m in sorted(r.get("measurements", {}).items()):
            per = m.get("inter_step_us", {})
            if ticks and tps and per.get("min") is not None:
                want = ticks * 1e6 / tps
                spread = per.get("spread") or 0.0
                # Tolerance is the sample period: a measurement cannot be more
                # wrong than the resolution it was taken at.
                tol = max(1e6 / r.get("sample_rate_hz", 1e9), want * 0.02)
                ok = abs(per["mean"] - want) <= tol
                out.append(f"| {r['test_id']} | {letter} inter-step period | "
                           f"{fmt(want, 2)} us ({ticks} ticks) | "
                           f"{fmt(per['mean'], 2)} us | "
                           f"{'✓' if ok else '✗'} |")
            expected = expected_steps(r)
            if expected:
                got = m.get("step_count")
                out.append(f"| {r['test_id']} | {letter} step count | "
                           f"{expected} | {fmt(got)} | "
                           f"{'✓' if got == expected else '✗'} |")
    return "\n".join(out)


def csv_export(results):
    """The paper's schema. Pulse-width columns are statistics, not a verdict."""
    buf = io.StringIO()
    # LF, not the csv module's default CRLF: the paper asks for git-diffable
    # output, and CRLF makes every regenerated CSV show as a whole-file change.
    writer = csv.writer(buf, lineterminator="\n")
    writer.writerow(CSV_COLUMNS)
    for r in results:
        tps = r.get("dut", {}).get("ticks_per_s")
        segs = r.get("segments", [])
        ticks = segs[0][1] if segs else ""
        exp = expected_steps(r)
        for letter, m in sorted(r.get("measurements", {}).items()):
            per = m.get("inter_step_us", {})
            high = m.get("pulse_high_us", {})
            low = m.get("pulse_low_us", {})
            writer.writerow([
                r.get("timestamp", ""), r["test_id"], r.get("arch", ""),
                r.get("driver", ""), r.get("channel_config", ""), letter,
                ticks, tps, m.get("step_count", ""), exp or "",
                per.get("mean", ""), per.get("min", ""), per.get("max", ""),
                per.get("spread", ""), high.get("mean", ""),
                high.get("min", ""), high.get("max", ""), low.get("mean", ""),
                m.get("duty_cycle_percent", ""),
                "true" if r.get("pass") else "false",
            ])
    return buf.getvalue().rstrip()


def index_report(results, baseline):
    """The dashboard: counts, pass rate per tag, then the full results table."""
    total = len(results)
    passed = sum(1 for r in results if r.get("pass"))
    by_tag = defaultdict(lambda: [0, 0])
    for r in results:
        by_tag[r.get("tag", "—")][0] += 1
        by_tag[r.get("tag", "—")][1] += 1 if r.get("pass") else 0

    lines = ["# Saleae characterization report", ""]
    lines.append(f"- **Tests:** {total} run, {passed} passed, "
                 f"{total - passed} failed")
    if total:
        lines.append(f"- **Pass rate:** {100.0 * passed / total:.1f}%")
    latest = max((r.get("timestamp", "") for r in results), default="—")
    lines.append(f"- **Latest result:** {latest}")
    lines.append("")
    lines.append("## Pass rate by tag")
    lines.append("")
    lines.append("| tag | passed | run | rate |")
    lines.append("|---|---|---|---|")
    for tag, (ran, ok) in sorted(by_tag.items()):
        lines.append(f"| {tag} | {ok} | {ran} | "
                     f"{100.0 * ok / ran:.1f}% |")
    lines.append("")
    lines.append("## Results")
    lines.append("")
    lines.append(results_table(results))
    lines.append("")

    if len(by_tag) > 1:
        lines.append("## Cross-configuration comparison")
        lines.append("")
        lines.append("More than one configuration is present, so the same test "
                     "may have been measured under more than one. Rows only "
                     "appear where a test was actually run in both.")
        lines.append("")
        lines.append(cross_tag_table(results))
        lines.append("")

    lines.append("## Reading the numbers")
    lines.append("")
    lines.append(
        "Periods and pulse widths are reported as a **distribution** "
        "(min–max, spread, median), not a single average. A driver that holds a "
        "fixed pulse width shows min == max and that is itself the finding; one "
        "short pulse in ten thousand moves only the minimum. Pulse width is "
        "recorded rather than judged: the driver sets it, so its value is a "
        "property of the silicon and becomes the baseline a regression is "
        "measured against. Step counts and periods are what the library "
        "promises, and those are asserted.")
    lines.append("")
    if baseline:
        lines.append("A baseline was supplied; see `regression.md`.")
    return "\n".join(lines)


def cross_tag_table(results):
    """Same test, different configuration, side by side."""
    by_test = defaultdict(list)
    for r in results:
        by_test[r["test_id"]].append(r)
    out = ["| test | configuration | period us (min–max) | pulse high us |",
           "|---|---|---|---|"]
    shared = [t for t, rs in by_test.items() if len(rs) > 1]
    if not shared:
        return ("_No test was run under more than one configuration in this "
                "results set, so there is nothing to compare side by side. "
                "Point `--baseline` at a results directory from another board "
                "or driver to get a comparison._")
    for test, rs in sorted(by_test.items(),
                           key=lambda kv: int(kv[0].split("_")[1])):
        if len(rs) < 2:
            continue
        for r in rs:
            a = r.get("measurements", {}).get("A", {})
            per = a.get("inter_step_us", {})
            high = a.get("pulse_high_us", {})
            out.append(
                f"| {test} | {r.get('tag', '—')} | "
                f"{fmt(per.get('min'))}–{fmt(per.get('max'))} | "
                f"{fmt(high.get('mean'))} |")
    return "\n".join(out)


def test_report(result):
    """One detail page per test, in the shape the paper's section 8.3 shows."""
    dut = result.get("dut", {})
    segs = result.get("segments", [])
    program = " | ".join(f"QSEG {n} {t} {int(bool(u))}" for n, t, u in segs)
    # A scenario with per-stepper speeds sent one QSEG per stepper, and printing
    # only the first would misrepresent what the board was actually told to do.
    per = result.get("per_stepper")
    if per:
        program = " ; ".join(
            f"stepper {idx}: " + " | ".join(
                f"QSEG {idx} {n} {t} {int(bool(u))}" for n, t, u in prog)
            for idx, prog in sorted(per.items()))
    lines = [f"# {result['test_id']} — {result.get('goal', '')}", ""]
    lines.append(f"**Test ID:** {result['test_id']}")
    lines.append(f"**Goal:** {result.get('goal', '—')}")
    lines.append(f"**Program:** `{program}`")
    lines.append(f"**DUT:** {dut.get('ticks_per_s', '—')} ticks/s, "
                 f"`MIN_CMD_TICKS` {dut.get('min_cmd_ticks', '—')}, "
                 f"`QUEUE_LEN` {dut.get('queue_len', '—')}, "
                 f"fastest legal {dut.get('max_speed_ticks', '—')} ticks")
    lines.append(f"**Configuration:** {result.get('channel_config', '—')} "
                 f"on {result.get('driver', '—')} "
                 f"({result.get('arch', '—')})")
    lines.append(f"**Tag:** `{result.get('tag', '—')}`")
    lines.append(f"**Captured at:** {result.get('sample_rate_hz', '—')} Hz")
    lines.append(f"**Result:** **{verdict(result)}**")
    lines.append("")
    lines.append("## Measured")
    lines.append("")
    lines.append(stepper_table(result))
    lines.append("")
    lines.append("## Detail")
    lines.append("")
    detail = result.get("evaluator_detail", {})
    if detail:
        lines.append("```json")
        lines.append(json.dumps(detail, indent=2, sort_keys=True))
        lines.append("```")
    else:
        lines.append("_No evaluator detail recorded._")
    lines.append("")
    lines.append("## Capture")
    lines.append("")
    lines.append(f"`{result.get('capture', '—')}`")
    lines.append("")
    return "\n".join(lines)


def regression_report(results, baseline):
    """Compare against a baseline: what changed, and did any verdict flip."""
    base = {r["test_id"]: r for r in latest_per_test(baseline)}
    lines = ["# Regression against baseline", ""]
    if not baseline:
        lines.append("_No baseline supplied, so nothing to compare._")
        return "\n".join(lines)
    lines.append(f"Baseline: {len(base)} results.")
    lines.append("")
    lines.append("| test | verdict now | verdict then | period now | "
                 "period then | change |")
    lines.append("|---|---|---|---|---|---|")
    for r in results:
        b = base.get(r["test_id"])
        now_a = r.get("measurements", {}).get("A", {}).get("inter_step_us", {})
        if b is None:
            lines.append(f"| {r['test_id']} | {verdict(r)} | _new_ | "
                         f"{fmt(now_a.get('mean'))} | — | — |")
            continue
        then_a = b.get("measurements", {}).get("A", {}).get("inter_step_us", {})
        now, then = now_a.get("mean"), then_a.get("mean")
        if now is None or then is None:
            change = "—"
        else:
            delta = now - then
            # A period is only "changed" by more than the measurement can
            # resolve; below that it is the same number seen twice.
            tol = 1e6 / r.get("sample_rate_hz", 1e9)
            change = f"{delta:+.4f} us" + (" ⚠" if abs(delta) > tol else "")
        lines.append(f"| {r['test_id']} | {verdict(r)} | {verdict(b)} | "
                     f"{fmt(now)} | {fmt(then)} | {change} |")
    lines.append("")
    flipped = [r["test_id"] for r in results
               if r["test_id"] in base
               and bool(r.get("pass")) != bool(base[r["test_id"]].get("pass"))]
    lines.append("## Verdict changes")
    lines.append("")
    if flipped:
        for t in flipped:
            was = "pass" if base[t].get("pass") else "fail"
            now = "pass" if dict((x["test_id"], x) for x in results)[t].get(
                "pass") else "fail"
            lines.append(f"- **{t}**: {was} → {now}")
    else:
        lines.append("None. Every test present in both runs kept its verdict.")
    return "\n".join(lines)


def tag_summary(results, tag):
    """A summary page per tag, so per-configuration runs can be read alone."""
    rs = [r for r in results if r.get("tag") == tag]
    lines = [f"# {tag}", ""]
    arch = rs[0].get("arch") if rs else "—"
    driver = rs[0].get("driver") if rs else "—"
    lines.append(f"{len(rs)} tests, "
                 f"{sum(1 for r in rs if r.get('pass'))} passed.")
    lines.append("")
    lines.append(f"- **Architecture:** {arch}")
    lines.append(f"- **Driver:** {driver}")
    lines.append("")
    lines.append(results_table(rs))
    lines.append("")
    return "\n".join(lines)


def write(path, text):
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(text.rstrip() + "\n")
    return path


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--results", required=True,
                    help="directory of JSON result files")
    ap.add_argument("--out", default="reports",
                    help="output directory for the markdown and CSV")
    ap.add_argument("--baseline", default=None,
                    help="results directory to compare against")
    ap.add_argument("--quiet", action="store_true")
    args = ap.parse_args()

    results = latest_per_test(load_results(args.results))
    if not results:
        print(f"no results found in {args.results}", file=sys.stderr)
        return 1
    baseline = (latest_per_test(load_results(args.baseline))
                if args.baseline else [])

    out = Path(args.out)
    written = [write(out / "index.md", index_report(results, baseline))]

    for r in results:
        written.append(write(out / f"test_{r['test_id']}.md", test_report(r)))

    written.append(write(out / "spec_compliance.md",
                         "# Spec compliance\n\n"
                         "Measured against what the library promises: the "
                         "commanded period and the step count. Pulse width is "
                         "not listed as a compliance item because the driver "
                         "sets it and the library does not promise a value.\n\n"
                         + spec_table(results) + "\n"))

    written.append(write(out / "regression.md",
                         regression_report(results, baseline)))

    csv_path = out / "all_results.csv"
    csv_path.parent.mkdir(parents=True, exist_ok=True)
    csv_path.write_text(csv_export(results) + "\n")
    written.append(csv_path)

    tags = sorted({r.get("tag", "—") for r in results})
    for tag in tags:
        written.append(write(out / "tag_summary" / f"{tag}.md",
                             tag_summary(results, tag)))

    if not args.quiet:
        for path in written:
            print(f"wrote {path}")
    failed = [r["test_id"] for r in results if not r.get("pass")]
    if failed and not args.quiet:
        print(f"\nfailing: {', '.join(failed)}", file=sys.stderr)
    return 1 if failed else 0


if __name__ == "__main__":
    sys.exit(main())