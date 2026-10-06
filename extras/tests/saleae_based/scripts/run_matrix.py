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

    # the report again, from results already on disk: no build, no flash, no
    # capture, no serial port -- for when the board is busy or absent
    python3 scripts/run_matrix.py --report-only

Results and captures land in the harness's own directories
(extras/tests/saleae_based/results, .../capture) whatever the working directory
is; the report lands in extras/tests/saleae_based/reports/.
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
HARNESS = SCRIPTS.parent
ROOT = HARNESS.parents[2]
sys.path.insert(0, str(SCRIPTS))

import capture  # noqa: E402
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


def run_logged(argv, log, cwd=None, check=False, append=False):
    """Run a command, teeing its output to `log`. Returns (rc, seconds).

    `append` adds to an existing log instead of replacing it, so a flash that
    is retried leaves both attempts in one file -- a retried upload's log is
    only readable if the failure that caused the retry is in it.
    """
    log.parent.mkdir(parents=True, exist_ok=True)
    started = time.monotonic()
    with open(log, "a" if append else "w") as fh:
        if append:
            fh.write(f"\n$ {' '.join(argv)}   (retry)\n\n")
        else:
            fh.write(f"$ {' '.join(argv)}\n\n")
        fh.flush()
        # Not captured: a build that prints a page of warnings is a build whose
        # log has to be readable, and the matrix runs unattended.
        rc = subprocess.call(argv, cwd=cwd or ROOT, stdout=fh,
                             stderr=subprocess.STDOUT)
    return rc, time.monotonic() - started


def analyzer_ident():
    """The connected logic analyzer as sigrok names it, or None.

    Recorded per row rather than printed once, because it is not a constant:
    this harness has been run against a Saleae Logic 8 and against a **clone**,
    and the two enumerate with different `conn=` firmware strings and different
    channel counts. A report that names one analyzer in its header while a
    column of its numbers came off the other is a report that cannot be read.

    Read-only (`sigrok-cli --scan`); it touches no board and no serial port.
    """
    try:
        _devices, driver = capture.detect_analyzer()
    except (SystemExit, OSError):
        return None
    return driver or None


def flash(target, port, attempts=3):
    """Build and flash one matrix row. Returns (ok, seconds, log).

    Retried, because an upload that reaches esptool and then fails with "Wrong
    boot mode detected" is the board, not the build: the image is already
    compiled and linked at that point, and the DevKitC's DTR/RTS auto-reset
    occasionally does not fire, so the chip is handed to esptool running rather
    than in download mode. It took the arduino-4.4.0 row out of a full matrix
    here, and a matrix row that is lost to a reset costs a whole row of
    measurements -- so it is worth the retries. The retry is bounded and
    reported: a row that fails every attempt is recorded as a failed flash, not
    retried forever.

    Three attempts rather than two because the reset fault is not reliably fixed
    by one retry. Each failed attempt costs ~10 s of esptool plus the settle
    below, against a whole row of measurements, so the third attempt is cheap
    next to losing the row.

    The retries are insurance, not the fix. Measured on this board: 1 upload in
    8 succeeded while the fault was active, and 5 in 5 once the board had been
    power-cycled -- and `lsof` held no reader of the port throughout, so nothing
    was competing for it. The chip's USB-UART bridge latches into a state where
    DTR/RTS no longer drives the reset, and only removing power clears it. The
    board log at the time carried an `esp_core_dump_flash` dump mid-write, which
    is consistent with the bridge being busy rather than idle. None of that is
    reachable from here, so a row that still fails every attempt says so and the
    operator power-cycles; see the report note.
    """
    _tag, proj, env, _rate = target.derive(["--driver", "rmt", "--count", "1"])
    log = log_path(target.id, "flash")
    argv = ["pio", "run", "-d", proj, "-e", env, "-t", "upload",
            "--upload-port", port]
    print(f"  flash {env} ({target.framework} {target.version}, "
          f"ESP-IDF {target.esp_idf}) ... ", end="", flush=True)
    total = 0.0
    for attempt in range(1, attempts + 1):
        rc, secs = run_logged(argv, log, append=attempt > 1)
        total += secs
        if rc == 0:
            print("ok" if attempt == 1 else f"ok (on attempt {attempt})")
            return True, total, str(log)
        # The build is already done, so a retry only re-links if it must; give
        # the board a moment to settle rather than re-entering esptool into the
        # same state that just failed. The wait is flat, because a longer one
        # was measured not to help: the fault is not a chip that has not woken
        # up yet, it is the DTR/RTS auto-reset not putting the chip in download
        # mode at all, and waiting cannot make a line toggle that did not.
        time.sleep(5)
    print(f"FAILED (rc={rc} after {attempts} attempts, see {log})")
    return False, total, str(log)


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
           "drivers": [], "runs": [],
           # Which analyzer this row's numbers came off, read once per row
           # rather than once per matrix: a clone and a real Saleae Logic
           # enumerate differently and the report has to say which.
           "analyzer": analyzer_ident(),
           # Seconds, and a space rather than a T: the backfill below writes a
           # space, and one column is not worth a second date format.
           "measured_at": datetime.now().isoformat(timespec="seconds")
           .replace("T", " ")}

    ok, secs, log = flash(target, port)
    row["flash_seconds"] = round(secs, 1)
    row["flash_log"] = log
    if not ok:
        print("  build or flash failed; no run is possible on this row\n"
              "    if the log says 'Wrong boot mode detected', the board's\n"
              "    USB-UART bridge stopped resetting into download mode:\n"
              "    power-cycle it and re-run this row. That is measured, not\n"
              "    guessed -- see flash()'s docstring.")
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


def mode_tag(base_tag, result):
    """True when `result` is one of the points of the mode run `base_tag`.

    A mode run writes ONE RECORD PER POINT, and each point's tag is the run's
    tag with the point's own suffix on it -- `{tag_key}_{plan.tag}` in
    run_tests.run_modes(). So the scale and sync tables cannot look the records
    up by equality with the tag the runner recorded for the run: a tag is the
    run's, and every point under it is a different key. Matching the prefix is
    what joins the two. (The first version compared them for equality, which
    found nothing, and printed two empty tables under headings promising the
    per-n results -- a report that silently drops the measurements it was
    written to show is worse than no report.)
    """
    tag = result.get("tag_key") or ""
    return bool(base_tag) and tag.startswith(base_tag + "_")


def statuses(results, tag_keys):
    """{test_id: result} for the tags of one matrix row."""
    wanted = set(tag_keys)
    got = {}
    for r in results:
        if r.get("tag_key") in wanted:
            got[r["test_id"]] = r
    return got


# ESP-IDF boot and log noise that precedes a panic in a captured serial reply.
# A firmware that crashes answers CONFIG with a hundred lines of bootloader
# traceback, and a report that quotes its first line ("D (420) rmt: new simple
# encoder @0x3ffb8a7c") describes the debug print before the crash rather than
# the crash.
ESP_LOG_NOISE = re.compile(
    r"^(?:[IWED]\s*\(\d+\)|ets |rst:|configsip:|clk_drv:|mode:|load:|entry |"
    r"ELF file SHA256|Rebooting|Warning:|abort\(\)|assert failed|"
    r"Project version|Compile time|boot:|Partition Table|esp_image:|"
    r"\*\*\* |[A-Za-z ]+: 0x[0-9a-f]+$)")


def one_line(text, limit=110):
    """The most informative single line of a serial reply, log noise removed.

    A crash is searched for FIRST, wherever it sits in the reply. A firmware
    that panics mid-CONFIG leaves the host holding a fragment of the half-sent
    line, then the ESP-IDF log lines, then the panic — and the panic is the only
    part that says anything. Taking the first surviving line instead reported
    the fragment ("0", "DONE 0", a replacement character) as the note for four
    measurements whose real content was the `Guru Meditation` two lines down,
    which split one 24-measurement finding into four groups of 19, 4 and 1.
    """
    lines = [ln.strip() for ln in str(text).strip().splitlines()]
    for line in lines:
        if any(marker in line for marker in FIRMWARE_CRASH):
            return line[:limit]
    for line in lines:
        if not line or ESP_LOG_NOISE.match(line) or junk_line(line):
            continue
        return line[:limit]
    return lines[0][:limit] if lines else ""


def junk_line(line):
    """A line that is a fragment of a half-sent reply rather than an answer.

    Short, or carrying a byte the serial read cut in half. Never rendered as a
    note on its own -- see garbled().
    """
    return len(line) < 5 or "�" in line


# What a non-pass *is*. Four kinds, and the distinction is the difference
# between a report of defects and a log of the harness talking to the board.
#
# It matters because "everything that did not pass" was one table holding all
# four, and two of them are not failures of anything:
#
#   BOUND      the board declined a CONFIG at a limit. `scale` exists to find
#              it, so it is the answer, and it belongs in the sweep table where
#              it reads "MCPWM/PCNT reaches 6" rather than in a defect list.
#   CAPABILITY this build has no queues for the named driver. The firmware is
#              answering a question correctly and the scenario was never
#              applicable -- SR_23 on every row without I2S.
#   INCOMPLETE the measurement could not be made: a channel the stepper needs
#              was not captured. A hole in the harness, not a wrong step.
#   DEFECT     anything else. A panic, a stack overflow, a step count or a
#              period that is wrong. This is the only class that is a finding.
BOUND = "bound"
CAPABILITY = "capability"
INCOMPLETE = "incomplete"
DEFECT = "defect"

# The "matrix row" cell for a non-pass no row in THIS run claims. The results
# directory is cumulative, so a result from a row that did not flash this time
# lands here rather than being pinned to a row that shares its tag prefix.
UNATTRIBUTED = "(not measured this run)"

# The firmware's own words for the two non-defect refusals. Matched on the raw
# reply, not on the note, so classification does not depend on the note
# rendering.
NO_SUCH_DRIVER = re.compile(r"no such driver")
CONNECT_REFUSED = re.compile(r"ERR connect step \d+")


def classify(res):
    """Which of the four kinds of non-pass this result is.

    Deliberately not derived from the verdict alone. `failed` and `refused` are
    both "not passed", and they say opposite things: a `failed` step count is a
    defect, while a `failed` scenario that only ever got as far as
    `ERR CONFIG no such driver` is a build without that driver.
    """
    if res.get("incomplete_capture"):
        return INCOMPLETE
    text = " ".join(str(res.get(k) or "") for k in
                    ("error", "firmware_reply", "reply"))
    if NO_SUCH_DRIVER.search(text):
        return CAPABILITY
    if CONNECT_REFUSED.search(text):
        return BOUND
    if res.get("result") == "refused":
        # A refusal is the board declining, whatever its wording. Not a defect.
        return BOUND
    return DEFECT


def what_of(r):
    """What this result is *about*, for a table that lists non-passes.

    A catalogue result is its scenario id. A mode result is one point of a sweep
    or one driver combination, and `MODE` names neither: eight identical rows
    saying `MODE` is the difference between a report that says "this driver is
    refused past 6 steppers" and one that says something happened eight times.
    """
    test = r.get("test_id", "?")
    if test != "MODE":
        return test
    mode = r.get("mode", "")
    drivers = "+".join(r.get("drivers") or []) or "?"
    if mode == "sync":
        return f"sync {drivers}"
    if mode == "scale":
        return f"scale {drivers} n={r.get('stepper_count')}"
    return f"{mode} {drivers}".strip()


# The reply tokens the firmware actually emits (saleae_app.cpp: OK <CMD>, ERR
# <what>, POS <n>, DONE <n>). A captured reply that starts with none of these,
# and names no crash, is a fragment of something else.
FIRMWARE_REPLY = re.compile(r"^(?:OK|ERR|POS|DONE|MAP)\b")
FIRMWARE_CRASH = ("Guru Meditation", "stack overflow", "panic'ed",
                  "assert failed", "abort()")


def garbled(text):
    """True when a captured serial reply is not usable as a note.

    A firmware that panics mid-CONFIG leaves the host holding whatever bytes had
    already arrived -- "E 0", "0", a replacement character where a byte was cut
    in half. Those are real, and hiding them would be dishonest, but quoting a
    fragment as though it were the board's answer is worse than saying the
    answer was cut off, so they are named as what they are.

    Asked as "is this the firmware's answer at all", not as "does it look odd":
    the earlier version tried to spot the odd shapes and swallowed every
    `ERR connect step 6 n=6 drv=...` in the report, which are the refusals that
    are the whole measurement.
    """
    t = text.strip()
    if not t:
        return True
    if FIRMWARE_REPLY.match(t):
        return False
    if any(marker in t for marker in FIRMWARE_CRASH):
        return False
    return junk_line(t)


def note_of(r):
    """One line explaining a non-pass, from whatever the record carries.

    A `refused` gets a line too, not only a `failed`: the refusal text is the
    most informative thing a refused sweep point has ("ERR connect step 6
    n=6" names the bound the whole mode exists to find), and leaving the cell
    blank in the table of everything that did not pass is what made that table
    useless.
    """
    if r is None:
        return ""
    verdict = r.get("result")
    if verdict in ("passed", "skipped"):
        return ""
    # `ok` is a clean success and `POS` is the board's own tally, which agrees
    # with the capture or it would not be this verdict. Neither is a reason
    # anything failed, so they are not notes -- see the comment inside.
    def reports_problem(v):
        return bool(v) and not re.match(r"\s*(OK|POS)\b", str(v))

    for key in ("incomplete_capture", "error", "firmware_reply", "reply"):
        # Only a reply that *reports a problem* is a note. A run that failed its
        # measurement still carries the board's confirmation of the move it did
        # make -- "OK QRUN / POS 64 64" -- and printing that as the reason puts
        # the word "OK" in the one column a reader looks at first. Four `defect`
        # rows read "OK QRUN" before this check. `classify` reads the same
        # strings and does want the successful ones; that is a different
        # question and does not go through here.
        if reports_problem(r.get(key)):
            # An incomplete capture is its own verdict, and it is checked before
            # the generic scan below: the record also carries a `reply` that
            # says "OK QRUN / POS 64 64", which is the board confirming a move
            # it did make -- so the note read "OK QRUN" for a run that could not
            # be judged because a channel the stepper needs was never captured.
            if key == "incomplete_capture":
                inc = r[key]
                missing = inc.get("missing_channels") or []
                steppers = inc.get("missing_steppers") or []
                return ("incomplete capture: " +
                        (", ".join(str(m) for m in missing) or "channels") +
                        (f" missing (stepper "
                         f"{', '.join(str(s) for s in steppers)})"
                         if steppers else ""))
            return one_line(r[key]) if not garbled(one_line(r[key])) \
                else "(serial output cut off mid-reply -- the board stopped " \
                     "answering)"
    # The measured numbers, which is where a `sync`/`scale` failure actually
    # lives. `per_stepper` is one level deeper -- {"A": {"steps": {...}}} -- so a
    # flat "which key is False" scan never sees it and a run whose only failing
    # sub-measurement is per-stepper fell through to "failed" with no reason.
    for key in ("period", "steps", "adherence", "invariants"):
        v = r.get(key)
        if isinstance(v, dict):
            bad = [k for k, x in v.items() if x is False]
            if bad:
                return "failed: " + ", ".join(bad)
    per = r.get("per_stepper")
    if isinstance(per, dict):
        parts = []
        for letter, sub in per.items():
            if not isinstance(sub, dict):
                continue
            bad = [k for k, x in sub.items()
                   if isinstance(x, dict) and x.get("ok") is False]
            if not bad:
                continue
            # The measurement, not only the verdict: "period off-grid" says what
            # to go and look at, "failed: period" does not.
            how = []
            for k in bad:
                m = sub[k]
                extra = m.get("extra_steps")
                missing = m.get("missing_steps")
                got = m.get("steps_measured")
                want = m.get("steps_expected")
                if k == "steps" and (extra or missing):
                    bits = []
                    if missing:
                        bits.append(f"{missing} missing")
                    if extra:
                        bits.append(f"{extra} extra")
                    how.append(f"{k} {got}/{want} ({', '.join(bits)})")
                elif k == "period" and m.get("n_off_grid"):
                    how.append(f"period {m['n_off_grid']} off-grid")
                else:
                    how.append(k)
            parts.append(f"{letter} " + ", ".join(how))
        if parts:
            return "failed: " + "; ".join(parts)
    return verdict or "?"


def cell(r):
    """One matrix cell: the verdict.

    **No link to the capture, on purpose.** This file is in git and the
    captures are not: a mux `dir` capture is a 100 MB VCD and `capture/` is
    ignored by design, so a `[vcd](capture/...)` in a committed report is a dead
    link in every checkout that has not just run the matrix. There were 30 of
    them per report. The verdict stands alone, and the header says where the
    waveform is and how to name it.

    The mark follows the *class*, not just the verdict word. A scenario whose
    CONFIG names a driver this build has no queues for is recorded `failed` --
    correctly, because the scenario did not run -- but printing that as a bold
    **FAIL** in a matrix column made four of six rows look like a broken
    library when the only thing wrong is that ESP-IDF 4 defines no I2S queues.
    It is `n/a` here, and it is in the findings table's own legend.
    """
    if r is None:
        return "–"
    kind = classify(r)
    if kind == CAPABILITY:
        return "n/a (no such driver)"
    if kind == INCOMPLETE:
        mark = "**incomplete**"
    else:
        res = r.get("result", "?")
        mark = {"passed": "pass", "failed": "**FAIL**",
                "refused": "refused (bound)", "error": "**error**",
                "skipped": "skip"}.get(res, res)
    return mark


def report(rows, results, args):
    """The matrix report: what was run, what passed, and what did not."""
    by_row = {r["id"]: r for r in rows}

    # RELEASE_MATRIX order, not the order the rows happened to be measured in.
    # A resumed matrix runs its re-flashed row last and carries the rest, so
    # without this a report puts the row just re-measured at the bottom and the
    # matrix stops reading as an ordered comparison of SDKs.
    order = [t.id for t in harness.release_targets()]
    rows = sorted(rows, key=lambda r: order.index(r["id"]) if r.get("id") in order
                  else len(order))

    ids = [r["id"] for r in rows]

    # When was this row measured? A row recorded by a run that stamped it says
    # so; one carried from an older index does not, and its result records do --
    # each measurement carries its own timestamp. Backfilled from those rather
    # than left blank, because a blank in this column reads as "not measured"
    # next to a full table of results for exactly that row.
    for row in rows:
        if row.get("measured_at"):
            continue
        tags = [run["tag_key"] for run in row["runs"] if run.get("tag_key")]
        stamps = [res.get("timestamp") for res in results
                  if res.get("timestamp")
                  and (res.get("tag_key") in tags
                       or any(mode_tag(t, res) for t in tags))]
        if stamps:
            row["measured_at"] = max(stamps).replace("T", " ")[:19]
    tests = sorted({res["test_id"] for res in results
                    if res.get("test_id", "").startswith("SR_")})

    # tag_key -> row, from what the runner recorded (never parsed back out of
    # a tag string: the tag is for indexing, not for reading).
    tags_of = {}
    for row in rows:
        for run in row["runs"]:
            if run.get("tag_key"):
                tags_of[run["tag_key"]] = (row["id"], run["label"])

    def row_of(res):
        """(row id, run label) for one result, or None if this run did not
        produce it.

        A mode point's tag is not the run's tag, so the lookup has to try the
        prefix as well as the exact key -- otherwise every sweep point and every
        driver combination in this report is attributed to a bare tag string,
        which is an index and not a name.

        None is load-bearing and is NOT a licence to fall back to the tag. The
        results directory accumulates: a row that failed to flash this time still
        has its results from the time it worked, and matching those against the
        prefix of some other row's tag put an `idf-6.13.0` finding on a result
        recorded against `idf-7.1.2` -- a stale measurement presented as one
        this matrix took. A result no row claims is reported as unattributed
        (see `unattributed` below) rather than pinned to whichever row happens
        to share its prefix.
        """
        tag = res.get("tag_key")
        if tag in tags_of:
            return tags_of[tag]
        for base, where in tags_of.items():
            if mode_tag(base, res):
                return where
        return None

    L = []
    now = datetime.now().strftime("%Y-%m-%d %H:%M")
    L.append("# Saleae harness — ESP32 platform-release matrix")
    L.append("")
    L.append(f"- **Generated:** {now}"
             + ("  _(rebuilt from the recorded results; nothing was measured in "
                "this invocation — `--report-only`)_"
                if getattr(args, "report_only", False) else ""))
    # The analyzer is named per row from what sigrok reported at the time, not
    # from a constant in this file: a Saleae Logic 8 and a clone enumerate with
    # different `conn=` strings, and a header that names one of them over a table
    # whose other column came off the other is a header that lies.
    analyzers = sorted({r.get("analyzer") for r in rows if r.get("analyzer")})
    board = ("ESP32-DevKitC, serial `" + args.port + "`")
    L.append(f"- **Board:** {board}")
    L.append(f"- **Analyzer:** {', '.join('`' + a + '`' for a in analyzers)}"
             if analyzers else
             "- **Analyzer:** not recorded (this report was rebuilt from an "
             "index written before the analyzer was identified per row)")
    L.append(f"- **Firmware rows:** {len(rows)} — one build+flash each")
    L.append(f"- **Matrix definition:** `scripts/harness.py` "
             f"(`RELEASE_MATRIX`, `release_runs()`)")
    L.append("- **Where the waveforms are:** captures and result records are "
             "**local and git-ignored** — `capture/<tag>.sr` plus the `.vcd` "
             "sigrok derives beside it, and `results/<tag>.json`, per-run logs "
             f"under `{LOG_DIR}` — so nothing in this file links to them and a "
             "fresh checkout has none of them. A measurement is named by its "
             "tag key, which is what those filenames are built from.")
    L.append("")
    L.append("Every row is a firmware flashed **once**; all driver and "
             "combination runs below were measured against that one flash. "
             "Drivers come from asking the board what it accepts, so a column "
             "that is absent is a driver this SDK has no queues for.")
    if getattr(args, "report_only", False):
        L.append("")
        L.append("> **This file was rebuilt from the recorded results, not "
                 "measured.** The verdicts below are re-evaluations of captures "
                 "already on disk, which is what makes it possible to refresh "
                 "the report when the board is busy or absent — and it is why "
                 "the *measured* column, not the *generated* one above, is the "
                 "date to read: it is when each row's firmware was flashed.")
    L.append("")

    # --- targets -----------------------------------------------------------
    L.append("## Firmware matrix")
    L.append("")
    L.append("| framework | version | ESP-IDF | PlatformIO env | drivers the "
             "board accepts | catalogue | scale sweeps | sync | measured |")
    L.append("|---|---|---|---|---|---|---|---|---|")
    for row in rows:
        cat = [run["label"] for run in row["runs"] if run["label"] == "catalogue"]
        scales = [run["label"] for run in row["runs"]
                  if run["label"].startswith("scale")]
        sync = [run["label"] for run in row["runs"] if run["label"] == "sync"]
        L.append(
            f"| {row['framework']} | {row['version']} | {row['esp_idf']} | "
            f"`{row['env']}` | {', '.join(row['drivers']) or '–'} | "
            f"{cat[0] if cat else '–'} | {len(scales)} | "
            f"{sync[0] if sync else '–'} | "
            f"{row.get('measured_at', '–')} |")
    L.append("")
    stamps = {r.get("measured_at") for r in rows if r.get("measured_at")}
    if len(stamps) > 1:
        L.append("> The rows were not measured in one sitting: a row whose "
                 "upload failed is re-run on its own, and the *measured* column "
                 "is when each row's flash happened. Rows that share a timestamp "
                 "were measured against the board back to back.")
        L.append("")
    for row in rows:
        if not row["flash_ok"]:
            L.append(f"> **{row['id']}**: build or flash failed "
                     f"({row.get('flash_log')}); nothing was measured.")
            # Name the recoverable case, because "nothing was measured" reads as
            # a dead row when the board only needs power-cycling. A build error
            # is the opposite -- no amount of power-cycling compiles a missing
            # symbol -- and the log says which it was.
            L.append(">")
            L.append("> *If that log says `Wrong boot mode detected`, the row is "
                     "recoverable without a rebuild: the board's USB-UART "
                     "bridge stopped resetting into download mode and only "
                     "removing its power clears it. Power-cycle and re-run this "
                     "row alone. If it is a compiler error instead, it is a "
                     "source defect and a power-cycle will not touch it.*")
            L.append("")

    # --- catalogue ---------------------------------------------------------
    # From ALL_TESTS rather than from `tests`, which is what this run happened to
    # measure: the heading states the catalogue's numbering, and a run that
    # recorded nothing must still produce a report.
    L.append(f"## Scenario catalogue ({run_tests.ALL_TESTS[0]} … "
             f"{run_tests.ALL_TESTS[-1]})")
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
    L.append("`pass` / `FAIL` / `incomplete` / `refused (bound)` / `n/a` / `skip` "
             "are the recorded verdicts, and there is no link on them: the "
             "waveform a verdict came from is a capture in `capture/`, which is "
             "git-ignored (a mux `dir` capture is a 100 MB VCD), so a link here "
             "would be dead in every checkout that has not just run the matrix. "
             "`skip` is either *not implemented* (SR_22, "
             "SR_24, SR_28, SR_29) or *SR_00 failed*, which is the harness "
             "refusing to measure on dead channels. **`FAIL` and `incomplete` "
             "are the only cells here that are findings** — see Findings.")
    L.append("")
    L.append("`n/a` means this build has no queues for the driver the scenario "
             "CONFIGS (`ERR CONFIG no such driver`), so the scenario was never "
             "applicable to this row and its `failed` verdict is not a defect. "
             "`refused (bound)` is the board declining a CONFIG at a limit, "
             "which is what `scale` and `sync` exist to find.")
    L.append("")

    # --- non-passes --------------------------------------------------------
    L.append("## Findings")
    L.append("")
    L.append("Defects only: a panic, a crash, a step count or a period that is "
             "wrong, and measurements that could not be made at all. Refusals "
             "are **not** listed here — a `scale` sweep is *asked* where a "
             "driver's limit is and a refusal is its answer, so those live in "
             "the sweep tables below where they read as a bound. The one "
             "exception is a scenario that names a driver the build does not "
             "have; that is a capability answer, not a failure, and it is "
             "counted at the bottom of this section rather than tabulated.")
    L.append("")
    bad = []
    counts = {BOUND: 0, CAPABILITY: 0, INCOMPLETE: 0, DEFECT: 0}
    for res in results:
        if res.get("result") in ("passed", "skipped"):
            continue
        kind = classify(res)
        counts[kind] += 1
        if kind == BOUND or kind == CAPABILITY:
            continue
        where = row_of(res)
        # No row claims it, so this matrix did not measure it. Listed on its own
        # rather than folded into a row's findings: attributing it to a row would
        # claim a measurement that row's flash never produced.
        bad.append(((where[0] if where else UNATTRIBUTED),
                    what_of(res), kind, note_of(res)))

    # Grouped before listed, because a driver that fails on *every* scenario of
    # one firmware is one finding and twenty-six table rows, and a reader should
    # not have to spot the pattern by counting.
    groups = {}
    for rid, what, kind, note in bad:
        groups.setdefault((rid, kind, note), []).append(what)
    loud = sorted(((len(v), k, v) for k, v in groups.items()),
                  key=lambda t: (-t[0], t[1][0]))
    if loud:
        L.append("| matrix row | what | measurements | note |")
        L.append("|---|---|---|---|")
        for count, (rid, kind, note), whats in loud:
            L.append(f"| {rid} | {kind} | {count} | {note} |")
        L.append("")
        L.append("Expanded below, one line per measurement.")
        L.append("")
    else:
        L.append("_Nothing: every measurement that ran, passed._")
        L.append("")

    L.append("## Every finding, one line each")
    L.append("")
    if bad:
        L.append("| matrix row | test | class | note |")
        L.append("|---|---|---|---|")
        for rid, test, kind, note in bad:
            L.append(f"| {rid} | {test} | {kind} | {note} |")
    else:
        L.append("_Nothing._")
    L.append("")

    # The two classes that are not findings are counted, not tabulated, so the
    # report cannot be read as having hidden them and cannot be read as having
    # 60 defects either.
    L.append(f"Not findings, and not listed above: "
             f"**{counts[BOUND]}** refusal(s), which are the measured limits "
             f"in the sweep tables below, and **{counts[CAPABILITY]}** "
             f"`no such driver` answer(s), which are scenarios this build has "
             f"no queues for. A full accounting of every result, including "
             f"the ones this file does not tabulate, is in the local "
             f"`results/` directory (git-ignored), indexed by "
             f"`results/tag_index.json`.")
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
            recs = sorted((r for r in results
                           if mode_tag(run.get("tag_key"), r)),
                          key=lambda r: r.get("stepper_count", 0))
            if not recs:
                L.append(f"**{rid} / {run['label']}** — no results recorded "
                         f"under this run's tag; nothing was measured.")
                L.append("")
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
                # A scale point records its mean period per stepper, not an
                # `adherence` block, so the column read as "–" for every n --
                # which is the period the whole sweep asserts on.
                mean = adh.get("mean_period_us", first.get("mean_period_us"))
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
             "steppers each, each at its own period. First-step skew is "
             "**reported, not gated** (eval_sync): how closely two steppers "
             "begin is a property of the pulse driver and of the interrupt "
             "latency at that instant, not a correctness property of the queue — "
             "so read the ratio in step periods, which is the only comparable "
             "form of it. What *is* asserted per stepper is its own commanded "
             "step count and period — counted over the *commanded move*, not "
             "over the capture: a capture starts before the test is triggered "
             "over serial and outlives it, so it holds the host's own round "
             "trip and the board's idle afterwards. A pulse outside the move is "
             "not evidence about a driver; it is still recorded, as "
             "`window.steps_outside` and each pulse's offset, and "
             "`report.py` prints the count beside the step count.")
    L.append("")
    for rid in ids:
        row = by_row.get(rid)
        if row is None or not row["flash_ok"]:
            continue
        for run in row["runs"]:
            if run["label"] != "sync":
                continue
            recs = sorted((r for r in results
                           if mode_tag(run.get("tag_key"), r)),
                          key=lambda r: "+".join(r.get("drivers") or []))
            if not recs:
                L.append(f"**{rid} / sync** — no results recorded under this "
                         f"run's tag; nothing was measured.")
                L.append("")
                continue
            L.append(f"**{rid} / sync**")
            L.append("")
            L.append("| drivers | verdict | first-step skew us | in step periods "
                     "| note |")
            L.append("|---|---|---|---|---|")
            for rec in recs:
                skew = rec.get("first_step_skew_us")
                if skew is None:
                    # Fall back to the per-stepper first-step instants. The
                    # evaluator normally records the skew itself; this is for a
                    # record that has the instants but not their difference.
                    # (The first version computed the list and then dropped it,
                    # which printed the same column as before either way.)
                    times = [v.get("first_step_us") for v in
                             (rec.get("per_stepper") or {}).values()
                             if isinstance(v, dict)]
                    times = [t for t in times if t is not None]
                    if times:
                        skew = round(max(times) - min(times), 2)
                drivers = "+".join(rec.get("drivers") or [])
                periods = rec.get("skew_periods")
                periods_txt = (f"{periods:.2f}"
                               if isinstance(periods, (int, float)) else "–")
                L.append(f"| {drivers} | {cell(rec)} | "
                         f"{skew if skew is not None else '–'} | "
                         f"{periods_txt} | {note_of(rec)} |")
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
    L.append("- A named scenario that CONFIGS a driver the build has no queues "
             "for is recorded **failed** with `ERR CONFIG no such driver`. That "
             "is the firmware reporting a capability, not a measurement going "
             "wrong, and the *drivers the board accepts* column of the firmware "
             "matrix is where it is read as the capability it is. It is not "
             "suppressed here because the scenario did not run.")
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
    p.add_argument("--report-only", action="store_true",
                   help="rebuild the report from the results and the recorded "
                        "index: no build, no flash, no capture, no serial port")
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

    # --- report-only -------------------------------------------------------
    #
    # Rebuild the report from what is already on disk, touching neither the
    # board nor the analyzer. It exists because the report is generated from two
    # things that have very different lifetimes -- a firmware build, and a
    # results directory -- and only the second one has to be current: after a
    # change to the *evaluators*, every recorded result re-evaluates to a new
    # verdict and the report on disk is stale until something re-runs it. Doing
    # that with the board attached costs a flash per row and buys no
    # measurement; doing it with the board *elsewhere* -- mid-test, mid-session
    # -- costs the board its state.
    #
    # So this path reads the recorded index for the rows and the results for the
    # measurements, and writes exactly what a measuring run would have written
    # from the same two. It says so in the header: a report that rebuilt itself
    # from disk must not read like one that measured everything just now, and
    # the *measured* column is what tells the reader which measurements are
    # current.
    if args.report_only:
        index_file = Path(args.results_dir) / INDEX
        if not index_file.exists():
            raise SystemExit(
                f"--report-only needs {index_file}, which a measuring run "
                f"writes. There is nothing recorded to report on.")
        recorded = json.loads(index_file.read_text()).get("rows", [])
        if args.targets:
            want = {t.strip() for t in args.targets.split(",")}
            recorded = [r for r in recorded if r.get("id") in want]
            missing = want - {r.get("id") for r in recorded}
            if missing:
                raise SystemExit(
                    f"--report-only: no recorded row for {', '.join(sorted(missing))}"
                    f"; --targets selects which rows to REPORT, and a row that "
                    f"was never measured is not one of them")
        rows = recorded
        print(f"report-only: {len(rows)} recorded row(s), "
              f"{sum(len(r.get('runs', [])) for r in rows)} recorded run(s); "
              f"no build, no flash, no capture, no serial")
    else:
        rows = None
        if not Path("/dev").exists():
            raise SystemExit("no /dev; this is a hardware run")

        # The generated PlatformIO projects (pio_dirs/saleae, pio_espidf/saleae)
        # hold the versioned envs, and they are symlink farms the build script
        # recreates. Once per matrix, not once per row.
        print("\nlinking the PlatformIO projects ...")
        run_logged(["bash", "extras/scripts/build-pio-dirs.sh"],
                   LOG_DIR / "pio_dirs.log")

    # Rows measured by an EARLIER invocation, kept so a resumed matrix is still
    # a matrix.
    #
    # A row is not retried for free: one row's upload failing (an ESP32 that
    # comes up in the wrong boot mode is a routine outcome, and it took the
    # arduino-4.4.0 row out of the first full run here) used to mean the only
    # way to get that row was to re-run every target, because the row table was
    # built from scratch and written over. Re-running all six re-flashes all six
    # to measure nothing -- the runs themselves skip, but a minute of board time
    # per row buys no measurement. So a narrowed `--targets` merges into the
    # recorded rows instead of replacing them, and each row carries the time it
    # was measured, because a merged report otherwise presents rows measured
    # hours apart as one sitting.
    index_file = Path(args.results_dir) / INDEX
    earlier = []
    if args.targets and index_file.exists():
        try:
            earlier = json.loads(index_file.read_text()).get("rows", [])
        except ValueError:
            print(f"warning: {index_file} is unreadable; writing a fresh index",
                  file=sys.stderr)
    doing = {t.id for t in targets}
    carried = [r for r in earlier if r.get("id") not in doing]
    if carried:
        print(f"carrying {len(carried)} row(s) from an earlier run: "
              f"{', '.join(r['id'] for r in carried)}")

    if not args.report_only:
        rows = list(carried)
        for t in targets:
            try:
                rows.append(run_target(t, args.port, args.baud, args.force,
                                       args.capture_dir, args.results_dir))
            except Exception as exc:  # a board that stops answering is a row,
                # not a lost matrix: record it and go on to the next firmware
                print(f"  {t.id}: aborted: {exc}")
                rows.append({"id": t.id, "framework": t.framework,
                             "version": t.version, "esp_idf": t.esp_idf,
                             "note": t.note, "env": "?", "flash_ok": False,
                             "drivers": [], "flash_seconds": 0,
                             "flash_log": "", "analyzer": None,
                             "runs": [{"label": "matrix", "rc": 1,
                                       "detail": f"aborted: {exc}"}]})
            finally:
                # After every row, so an interrupted matrix can be resumed from
                # the result index rather than from nothing. Skipped entirely by
                # --report-only, which measured nothing and must not restamp the
                # record of what was.
                Path(args.results_dir).mkdir(parents=True, exist_ok=True)
                index_file.write_text(
                    json.dumps({"generated": datetime.now().isoformat(),
                                "port": args.port, "rows": rows}, indent=2))

    # Matrix order is applied in report(), so a row resumed into this list out
    # of order still lands where RELEASE_MATRIX puts it.
    index_file = Path(args.results_dir) / INDEX

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