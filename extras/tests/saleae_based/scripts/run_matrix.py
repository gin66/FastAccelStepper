#!/usr/bin/env python3
"""
run_matrix.py — the platform-release matrix, one flash per firmware.

Every matrix row is a *firmware*, and the firmware is a compile-time choice, so
it is built and flashed exactly once. Everything else the matrix measures is a
runtime CONFIG -- which driver, how many steppers, which pin mode -- and is run
against that one flash. Flashing per configuration would spend a minute of
board time per row to learn nothing: the same binary would answer the same
question again.

The rows and the runs per row are DATA, and they live in harness.py
(RELEASE_MATRIX, release_runs()). This file does the I/O -- flash once, ask the
board which drivers it accepts, call harness.py for each run, and write the
report -- and it holds no matrix of its own, so the matrix can be read in one
place and the report cannot disagree with it.

    # everything, on the default analyzer and serial port
    python3 scripts/run_matrix.py

    # one row, to see a flash work end to end
    python3 scripts/run_matrix.py --targets arduino-6.13.0

    # the plan and nothing else: no build, no flash, no capture
    python3 scripts/run_matrix.py --plan

Results and captures land in the harness's own directories
(extras/tests/saleae_based/results, .../capture) whatever the working directory
is; the report lands in extras/tests/saleae_based/reports/.
"""

import argparse
import json
import subprocess
import sys
import time
from datetime import datetime
from pathlib import Path

SCRIPTS = Path(__file__).resolve().parent
HARNESS = SCRIPTS.parent
ROOT = HARNESS.parents[2]
sys.path.insert(0, str(SCRIPTS))

import harness  # noqa: E402
import run_tests  # noqa: E402

LOG_DIR = Path("/tmp/saleae_matrix")
# Index of what was run against what: which tag key belongs to which matrix
# row and which run produced it. Written next to the results so the report can
# be rebuilt without the board, and kept out of git with the rest of the output.
INDEX = "matrix_index.json"


def log_path(target_id, label):
    safe = label.replace(":", "_").replace("/", "_")
    return LOG_DIR / target_id / f"{safe}.log"


def run_logged(argv, log, cwd=None, check=False):
    """Run a command, teeing its output to `log`. Returns (rc, seconds)."""
    log.parent.mkdir(parents=True, exist_ok=True)
    started = time.monotonic()
    with open(log, "w") as fh:
        fh.write(f"$ {' '.join(argv)}\n\n")
        fh.flush()
        # Not captured: a build that prints a page of warnings is a build whose
        # log has to be readable, and the matrix runs unattended.
        rc = subprocess.call(argv, cwd=cwd or ROOT, stdout=fh,
                             stderr=subprocess.STDOUT)
    return rc, time.monotonic() - started


def flash(target, port):
    """Build and flash one matrix row. Once. Returns (ok, seconds, log)."""
    _tag, proj, env, _rate = target.derive(["--driver", "rmt", "--count", "1"])
    log = log_path(target.id, "flash")
    argv = ["pio", "run", "-d", proj, "-e", env, "-t", "upload",
            "--upload-port", port]
    print(f"  flash {env} ({target.framework} {target.version}, "
          f"ESP-IDF {target.esp_idf}) ... ", end="", flush=True)
    rc, secs = run_logged(argv, log)
    print("ok" if rc == 0 else f"FAILED (rc={rc}, see {log})")
    return rc == 0, secs, str(log)


def board_drivers(port, baud):
    """What the running firmware accepts, asked of the firmware.

    Not the host's DRIVERS table: that is a belief about the SDK, and the whole
    point of a per-SDK matrix is that the belief is what is being checked. A
    board that cannot be asked is an error, never a fallback to the table.
    """
    ser = run_tests.open_board(port, baud)
    try:
        present, _mux_init = run_tests.read_drivers(ser)
    finally:
        ser.close()
    return [d for d, ok in present.items() if ok]


def run_target(target, port, baud, force, capture_dir, results_dir):
    """Flash `target` once, then make every run of harness.release_runs()."""
    print(f"\n=== {target.id}  (ESP-IDF {target.esp_idf}"
          f"{', ' + target.note if target.note else ''})")
    row = {"id": target.id, "framework": target.framework,
           "version": target.version, "esp_idf": target.esp_idf,
           "note": target.note, "env": target.env, "flash_ok": False,
           "drivers": [], "runs": []}

    ok, secs, log = flash(target, port)
    row["flash_seconds"] = round(secs, 1)
    row["flash_log"] = log
    if not ok:
        print("  build or flash failed; no run is possible on this row")
        row["runs"].append({"label": "flash", "rc": 1,
                            "detail": "build or flash failed"})
        return row
    row["flash_ok"] = True

    drivers = board_drivers(port, baud)
    row["drivers"] = drivers
    absent = [d for d in harness.DRIVERS["esp"] if d not in drivers]
    print(f"  board accepts: {', '.join(drivers)}")
    if absent:
        print(f"  not built in:  {', '.join(absent)}")

    for run in harness.release_runs(target, drivers):
        argv = [sys.executable, str(SCRIPTS / "harness.py")] + run.argv + [
            "--no-build", "--port", port, "--baud", str(baud),
            "--capture-dir", str(capture_dir), "--results-dir", str(results_dir)]
        if force:
            argv.append("--force")
        log = log_path(target.id, run.label)
        print(f"  {run.label:16} ... ", end="", flush=True)
        rc, secs = run_logged(argv, log)
        tag, _proj, _env, _rate = target.derive(run.argv[4:])
        print("ok" if rc == 0 else f"FAILED (rc={rc})")
        row["runs"].append({"label": run.label, "driver": run.driver,
                            "rc": rc, "seconds": round(secs, 1),
                            "tag_key": tag, "log": str(log)})
    return row


def load_results(results_dir):
    """Every result record, plus the index of what produced it."""
    out = []
    for path in sorted(Path(results_dir).glob("*.json")):
        if path.name == INDEX or path.name == "tag_index.json":
            continue
        try:
            out.append(json.loads(path.read_text()))
        except ValueError:
            print(f"warning: skipping unreadable {path.name}", file=sys.stderr)
    return out


def statuses(results, tag_keys):
    """{test_id: result} for the tags of one matrix row."""
    wanted = set(tag_keys)
    got = {}
    for r in results:
        if r.get("tag_key") in wanted:
            got[r["test_id"]] = r
    return got


def note_of(r):
    """One line explaining a non-pass, from whatever the record carries."""
    if r is None:
        return ""
    if r.get("result") != "failed":
        return ""
    for key in ("error", "firmware_reply", "reply"):
        if r.get(key):
            return str(r[key]).strip().splitlines()[0][:110]
    for key in ("period", "steps", "adherence", "invariants", "per_stepper"):
        v = r.get(key)
        if isinstance(v, dict):
            bad = [k for k, x in v.items() if x is False]
            if bad:
                return "failed: " + ", ".join(bad)
    return "failed"


def capture_link(r):
    p = r.get("capture")
    if not p:
        return ""
    vcd = Path(str(p) + ".vcd")
    if vcd.exists():
        try:
            return f"[vcd]({vcd.relative_to(HARNESS)})"
        except ValueError:
            return f"[vcd]({vcd})"
    return ""


def cell(r):
    """One matrix cell: the verdict, and a capture link when there is one."""
    if r is None:
        return "–"
    res = r.get("result", "?")
    mark = {"passed": "pass", "failed": "**FAIL**", "refused": "refused",
            "error": "**error**", "skipped": "skip"}.get(res, res)
    link = capture_link(r)
    return f"{mark} {link}".strip()


def report(rows, results, args):
    """The matrix report: what was run, what passed, and what did not."""
    by_row = {r["id"]: r for r in rows}
    results_by_tag = {}
    for res in results:
        results_by_tag.setdefault(res.get("tag_key"), []).append(res)

    ids = [r["id"] for r in rows]
    tests = sorted({res["test_id"] for res in results
                    if res.get("test_id", "").startswith("SR_")})

    # tag_key -> row, from what the runner recorded (never parsed back out of
    # a tag string: the tag is for indexing, not for reading).
    tags_of = {}
    for row in rows:
        for run in row["runs"]:
            if run.get("tag_key"):
                tags_of[run["tag_key"]] = (row["id"], run["label"])

    L = []
    now = datetime.now().strftime("%Y-%m-%d %H:%M")
    L.append("# Saleae harness — ESP32 platform-release matrix")
    L.append("")
    L.append(f"- **Generated:** {now}")
    L.append(f"- **Board:** ESP32-DevKitC, Saleae Logic 8ch (`fx2lafw:conn=8.88`), "
             f"serial `{args.port}`")
    L.append(f"- **Firmware rows:** {len(rows)} — one build+flash each")
    L.append(f"- **Matrix definition:** `scripts/harness.py` "
             f"(`RELEASE_MATRIX`, `release_runs()`)")
    L.append(f"- **Raw results:** `results/` (git-ignored), captures in "
             f"`capture/`; per-run logs under `{LOG_DIR}`")
    L.append("")
    L.append("Every row is a firmware flashed **once**; all driver and "
             "combination runs below were measured against that one flash. "
             "Drivers come from asking the board what it accepts, so a column "
             "that is absent is a driver this SDK has no queues for.")
    L.append("")

    # --- targets -----------------------------------------------------------
    L.append("## Firmware matrix")
    L.append("")
    L.append("| framework | version | ESP-IDF | PlatformIO env | drivers the "
             "board accepts | catalogue | scale sweeps | sync |")
    L.append("|---|---|---|---|---|---|---|---|")
    for row in rows:
        cat = [run["label"] for run in row["runs"] if run["label"] == "catalogue"]
        scales = [run["label"] for run in row["runs"]
                  if run["label"].startswith("scale")]
        sync = [run["label"] for run in row["runs"] if run["label"] == "sync"]
        L.append(
            f"| {row['framework']} | {row['version']} | {row['esp_idf']} | "
            f"`{row['env']}` | {', '.join(row['drivers']) or '–'} | "
            f"{cat[0] if cat else '–'} | {len(scales)} | "
            f"{sync[0] if sync else '–'} |")
    L.append("")
    for row in rows:
        if not row["flash_ok"]:
            L.append(f"> **{row['id']}**: build or flash failed "
                     f"({row.get('flash_log')}); nothing was measured.")
            L.append("")

    # --- catalogue ---------------------------------------------------------
    L.append("## Scenario catalogue (SR_00 … SR_30)")
    L.append("")
    L.append("Unparameterized: every scenario runs its own fixed program and is "
             "judged by its own evaluator.")
    L.append("")
    head = "| test | " + " | ".join(ids) + " |"
    L.append(head)
    L.append("|---" * (len(ids) + 1) + "|")
    for test in tests:
        row_cells = []
        for rid in ids:
            row = by_row.get(rid)
            if row is None or not row["flash_ok"]:
                row_cells.append("–")
                continue
            tags = [run["tag_key"] for run in row["runs"]
                    if run["label"] == "catalogue" and run.get("tag_key")]
            got = statuses(results, tags)
            row_cells.append(cell(got.get(test)))
        L.append(f"| {test} | " + " | ".join(row_cells) + " |")
    L.append("")
    L.append("`pass` / `FAIL` / `refused` / `error` / `skip` are the recorded "
             "verdicts; a capture link opens the VCD the verdict came from. "
             "`skip` is either *not implemented* (SR_22, SR_24, SR_28, SR_29) or "
             "*SR_00 failed*, which is the harness refusing to measure on dead "
             "channels.")
    L.append("")

    # --- non-passes --------------------------------------------------------
    L.append("## Everything that did not pass")
    L.append("")
    bad = []
    for res in results:
        if res.get("result") in ("passed", "skipped"):
            continue
        where = tags_of.get(res.get("tag_key"))
        bad.append((where[0] if where else res.get("tag_key", "?"),
                    res.get("test_id", "?"), res.get("result", "?"),
                    note_of(res)))
    if bad:
        L.append("| matrix row | test | verdict | note |")
        L.append("|---|---|---|---|")
        for rid, test, res, note in bad:
            L.append(f"| {rid} | {test} | {res} | {note} |")
    else:
        L.append("_Nothing: every measurement that ran passed._")
    L.append("")

    # --- scale sweeps ------------------------------------------------------
    L.append("## Driver scale sweeps (how many steppers in parallel)")
    L.append("")
    L.append("`nodir`, one shared program, each stepper's own step count and "
             "period asserted. The board decides where the sweep stops: a "
             "CONFIG refusal is the measured bound.")
    L.append("")
    for rid in ids:
        row = by_row.get(rid)
        if row is None or not row["flash_ok"]:
            continue
        for run in row["runs"]:
            if not run["label"].startswith("scale"):
                continue
            recs = sorted(results_by_tag.get(run.get("tag_key"), []),
                          key=lambda r: r.get("stepper_count", 0))
            if not recs:
                continue
            L.append(f"**{rid} / {run['label']}**")
            L.append("")
            L.append("| n | verdict | steps each | period us | note |")
            L.append("|---|---|---|---|---|")
            for rec in recs:
                per = rec.get("per_stepper") or {}
                first = next(iter(per.values()), {})
                steps = (first.get("steps") or {}).get("steps_measured")
                exp = (first.get("steps") or {}).get("steps_expected")
                adh = rec.get("adherence") or first.get("adherence") or {}
                mean = adh.get("mean_period_us")
                L.append(
                    f"| {rec.get('stepper_count')} | "
                    f"{cell(rec)} | "
                    f"{steps if steps is not None else '–'}"
                    f"{f'/{exp}' if exp else ''} | "
                    f"{f'{mean:.2f}' if isinstance(mean, (int, float)) else '–'} "
                    f"| {note_of(rec)} |")
            L.append("")

    # --- sync --------------------------------------------------------------
    L.append("## Driver combinations (synchronized start)")
    L.append("")
    L.append("Every driver-list combination this board could connect, two "
             "steppers each, each at its own period.")
    L.append("")
    for rid in ids:
        row = by_row.get(rid)
        if row is None or not row["flash_ok"]:
            continue
        for run in row["runs"]:
            if run["label"] != "sync":
                continue
            recs = sorted(results_by_tag.get(run.get("tag_key"), []),
                          key=lambda r: "+".join(r.get("drivers") or []))
            if not recs:
                continue
            L.append(f"**{rid} / sync**")
            L.append("")
            L.append("| drivers | verdict | first-step skew us | note |")
            L.append("|---|---|---|---|")
            for rec in recs:
                per = rec.get("per_stepper") or {}
                skew = rec.get("first_step_skew_us")
                if skew is None:
                    skews = [v.get("first_step_us") for v in per.values()
                             if isinstance(v, dict)]
                    skews = [s for s in skews if s is not None]
                drivers = "+".join(rec.get("drivers") or [])
                L.append(f"| {drivers} | {cell(rec)} | "
                         f"{skew if skew is not None else '–'} | "
                         f"{note_of(rec)} |")
            L.append("")

    L.append("## Reading this")
    L.append("")
    L.append("- A `refused` is a measurement: the firmware refused the CONFIG, "
             "which is how a driver reaches its own queue count.")
    L.append("- `error` is the host or the capture failing, not a step that came "
             "out wrong; the run log says which.")
    L.append("- Periods come from the analyzer, so they carry the sample period "
             "as their resolution; the tolerance each evaluator uses is in its "
             "own result JSON.")
    L.append("- `frameworks/versions` are PlatformIO platform versions; the "
             "ESP-IDF column is what runs underneath. Every Arduino row is "
             "IDF 4.4.7 because Arduino core is built on IDF 4.4.7 on every "
             "espressif32 release (see `extras/doc/platformio-espressif-versions.md`).")
    L.append("")
    return "\n".join(L)


def main():
    p = argparse.ArgumentParser(description=__doc__.splitlines()[1])
    p.add_argument("--port", default="/dev/cu.usbserial-0001")
    p.add_argument("--baud", type=int, default=115200)
    p.add_argument("--targets", help="comma list of matrix row ids "
                                     "(default: all of RELEASE_MATRIX)")
    p.add_argument("--capture-dir", default=str(HARNESS / "capture"))
    p.add_argument("--results-dir", default=str(HARNESS / "results"))
    p.add_argument("--reports-dir", default=str(HARNESS / "reports"))
    p.add_argument("--report", default="esp32_platform_matrix.md")
    p.add_argument("--force", action="store_true",
                   help="re-measure even where a passed result exists")
    p.add_argument("--plan", action="store_true",
                   help="print the plan and exit: no build, no flash, no capture")
    args = p.parse_args()

    targets = harness.release_targets(
        [t.strip() for t in args.targets.split(",")] if args.targets else None)

    print(f"Saleae platform-release matrix: {len(targets)} firmware row(s)")
    for t in targets:
        _tag, _proj, env, _rate = t.derive(["--driver", "rmt", "--count", "1"])
        print(f"  {t.id:18} env {env:22} ESP-IDF {t.esp_idf:6} "
              f"{t.note}")

    if args.plan:
        print()
        for t in targets:
            print(f"{t.id}:")
            for run in harness.release_runs(t, harness.DRIVERS["esp"]):
                print(f"  {run.label:16} {' '.join(run.argv)}")
        return 0

    if not Path("/dev").exists():
        raise SystemExit("no /dev; this is a hardware run")

    # The generated PlatformIO projects (pio_dirs/saleae, pio_espidf/saleae)
    # hold the versioned envs, and they are symlink farms the build script
    # recreates. Once per matrix, not once per row.
    print("\nlinking the PlatformIO projects ...")
    run_logged(["bash", "extras/scripts/build-pio-dirs.sh"], LOG_DIR / "pio_dirs.log")

    rows = []
    for t in targets:
        try:
            rows.append(run_target(t, args.port, args.baud, args.force,
                                   args.capture_dir, args.results_dir))
        except Exception as exc:  # a board that stops answering is a row, not
            # a lost matrix: record it and go on to the next firmware
            print(f"  {t.id}: aborted: {exc}")
            rows.append({"id": t.id, "framework": t.framework,
                         "version": t.version, "esp_idf": t.esp_idf,
                         "note": t.note, "env": "?", "flash_ok": False,
                         "drivers": [], "flash_seconds": 0,
                         "flash_log": "",
                         "runs": [{"label": "matrix", "rc": 1,
                                   "detail": f"aborted: {exc}"}]})
        finally:
            # After every row, so an interrupted matrix can be resumed from the
            # result index rather than from nothing.
            Path(args.results_dir).mkdir(parents=True, exist_ok=True)
            (Path(args.results_dir) / INDEX).write_text(
                json.dumps({"generated": datetime.now().isoformat(),
                            "port": args.port, "rows": rows}, indent=2))

    results = load_results(args.results_dir)
    reports = Path(args.reports_dir)
    reports.mkdir(parents=True, exist_ok=True)
    out = reports / args.report
    out.write_text(report(rows, results, args))

    print(f"\nreport: {out}")
    for row in rows:
        ran = [r for r in row["runs"] if r["label"] != "flash"]
        bad = [r for r in ran if r.get("rc")]
        print(f"  {row['id']:18} flash={'ok' if row['flash_ok'] else 'FAILED':6} "
              f"runs={len(ran):2} failed={len(bad)}")
    return 0


if __name__ == "__main__":
    sys.exit(main())