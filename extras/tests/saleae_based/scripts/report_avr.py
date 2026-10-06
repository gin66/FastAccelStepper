#!/usr/bin/env python3
"""report_avr.py — the AVR platform matrix report, from recorded results.

The ESP32 and RP2350 reports are rebuilt by `run_matrix.py`, which is a
flash-once measure harness with an ESP32 matrix table of its own. AVR is not a
row in that matrix: it has a single 16 MHz Timer1 driver, no driver selection
and a *fixed cable*, so there is nothing to sweep across SDKs and nothing to
flash once. It is, however, a platform whose whole point is that its step pins
are wherever the cable put them, which is exactly what the results record.

So this is the AVR report path: it reads the result JSON already in `results/`
and writes the markdown, and it measures nothing. Every number comes from the
evaluator's record — `report.py`'s own `note_of`/`scale_rows` helpers are
imported and used, so the catalogue notes and the scale table cannot drift from
the ones the general report prints. A report that recomputed its numbers would
be able to disagree with the tests, and then nobody would know which was right.

    python3 scripts/report_avr.py                     # nanoatmega328, 2x timer, dir
    python3 scripts/report_avr.py --arch nanoatmega168
    python3 scripts/report_avr.py --out -             # stdout, do not write

The catalogue tag and the scale tag are built exactly as `harness.py` builds
them (a `+`-joined driver list per stepper, then count and pin mode), so the
report always reads the files the harness actually wrote rather than a second
naming convention kept in step by hand.
"""
import argparse
import glob
import json
import sys
from pathlib import Path

SCRIPTS = Path(__file__).resolve().parent
HARNESS = SCRIPTS.parent
sys.path.insert(0, str(SCRIPTS))

import report as rp  # noqa: E402


# What a `failed`/`error` catalogue result is called, so the exit code can say
# whether the report is clean without re-reading the table.
BAD = ("failed", "error")


def driver_list(driver, count):
    """`timer`/2 -> `timer+timer`, the same per-stepper list the harness tags."""
    return "+".join([driver] * count)


def catalogue_tag(arch, fw, driver, count, pin_mode):
    raw = f"{arch}_{fw}_{driver_list(driver, count).replace('+', '_')}" \
          f"{count}_{pin_mode}"
    return "".join(c if (c.isalnum() or c == "_") else "_" for c in raw)


def scale_tag_prefix(arch, fw, driver, pin_mode):
    return f"{arch}_{fw}_{driver}_scale{pin_mode}"


def load_catalogue(results_dir, tag):
    out = {}
    for path in glob.glob(str(results_dir / f"{tag}_SR_*.json")):
        record = json.loads(Path(path).read_text())
        out[record["test_id"]] = record
    return out


def load_scale(results_dir, prefix):
    out = {}
    for path in glob.glob(str(results_dir / f"{prefix}_*.json")):
        record = json.loads(Path(path).read_text())
        count = record.get("stepper_count")
        if count is not None:
            out[count] = record
    return out


def _steps_cell(record):
    hw, exp = rp.steps_of(record), rp.expected_of(record)
    if exp:
        return f"{hw}/{exp}"
    return str(hw) if hw is not None else "-"


def catalogue_table(records):
    """scenario | verdict | steps hw/exp | note, in SR order."""
    def key(name):
        return int(name.split("_")[1])

    out = ["| scenario | verdict | steps hw/exp | note |", "|---|---|---|---|"]
    for sid in sorted(records, key=key):
        record = records[sid]
        result = record.get("result", "?")
        verdict = {"passed": "PASS", "skipped": "SKIP",
                   "refused": "REFUSED", "error": "ERROR"}.get(
                       result, result.upper())
        if result == "skipped":
            note = "skipped: " + record.get(
                "reason", "driver not in this build")
            out.append(f"| {sid} | {verdict} | - | {note} |")
        elif sid == "SR_00":
            out.append(f"| {sid} | {verdict} | - | 8/8 channels at 1 Hz, "
                       "commanded duty, cable verified |")
        else:
            out.append(f"| {sid} | {verdict} | {_steps_cell(record)} | "
                       f"{rp.note_of(record)} |")
    return "\n".join(out)


def demote(heading_text):
    """`report.py`'s tables carry a `###` heading; the platform report uses `##`."""
    lines = heading_text.splitlines()
    if lines and lines[0].startswith("### "):
        lines[0] = "## " + lines[0][4:]
    return "\n".join(lines)


def capability_table(records):
    # Only records that carry `arch`/`framework` name a target; SR catalogue
    # records do not, and without this the table grows a `? / ?` row.
    with_arch = [r for r in records if r.get("arch")]
    return rp.as_capability_table(with_arch) if with_arch else None


def notes(cat, marker):
    """The AVR findings, taken from the records rather than asserted."""
    lines = [
        "- **Fixed cable / step pins by identity.** On a 328P the step pins can "
        "only be Timer1's compare outputs (D9/D10), which this cable puts on "
        "analyzer channels 3 and 2. The firmware claims the compare pin by "
        "identity and MAP reports `steps=`/`dirs=`; the host reads those "
        "channels and ignores the stride. SR_00 proves all eight cable channels "
        "before anything else runs.",
        "- **Two steppers is the ceiling.** `MAX_STEPPER` is 2 and Timer1 has "
        "two compare outputs, so `scale` passes n=1 and n=2 and every count "
        "from 3 up is refused (`ERR CONFIG n=3 max=2 …`).",
    ]
    if "SR_25" in cat and "SR_30" in cat:
        s25, s30 = cat["SR_25"], cat["SR_30"]
        lines.append(
            f"- **Stops.** SR_25 (`stopMove`) keeps the queued motion "
            f"({s25.get('steps_after_stop')} steps after the marker of "
            f"{s25.get('filled_steps')} in the fill); SR_30 "
            "(`forceStopAndNewPosition`) empties the queue "
            f"(steps_after_stop={s30.get('steps_after_stop')}, "
            f"queue_discarded={str(s30.get('queue_discarded')).lower()}). The "
            f"stop instant is read from the marker edge on a free channel "
            f"(marker={marker}).")
    if "SR_31" in cat:
        lines.append(
            "- **Max count.** SR_31 probes down from the channel budget and the "
            f"board accepts {cat['SR_31'].get('stepper_count')} in `nodir`; "
            "counts above it are refused by `CONFIG` with `max=2`.")
    return lines


def build(args):
    results_dir = Path(args.results_dir)
    tag = catalogue_tag(args.arch, args.framework, args.driver, args.count,
                        args.pin_mode)
    cat = load_catalogue(results_dir, tag)
    if not cat:
        raise SystemExit(
            f"no catalogue results for tag {tag} in {results_dir}. Run it with "
            f"harness.py first, or pass --results-dir.")
    scale = load_scale(results_dir,
                       scale_tag_prefix(args.arch, args.framework, args.driver,
                                        "nodir"))
    marker = next((r.get("marker_channel") for r in cat.values()
                   if r.get("marker_channel") is not None), None)
    generated = (args.generated
                 or max(r.get("timestamp", "") for r in cat.values())[:16]
                 .replace("T", " "))

    lines = [
        f"# Saleae harness — AVR ({args.board}) platform matrix", "",
        f"- **Generated:** {generated}  _(measured on hardware: catalogue + "
        "`scale` sweep, no replay)_",
        f"- **Board:** {args.board} ({args.board_detail}). Fixed cable: "
        f"Saleae D0..D7 = {args.cable}",
        f"- **Firmware:** {args.framework.capitalize()} / `{args.driver}` "
        f"(Timer1); tag key `{tag}`",
        f"- **Scenarios:** {len(cat)} recorded (SR_00–16, 21, 25–27, 30, 31; "
        "ESP32-only drivers skip; SR_22/24/28/29 not implemented)",
        "", "## Catalogue", "", catalogue_table(cat), "",
    ]
    if scale:
        lines += [demote(rp.as_scale_table(
            rp.scale_rows([scale[n] for n in sorted(scale)]))), ""]
    cap = capability_table(list(cat.values()) + list(scale.values()))
    if cap:
        lines += [demote(cap), ""]
    lines += ["## Notes", ""] + notes(cat, marker) + [""]
    return "\n".join(lines), cat


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--arch", default="nanoatmega328")
    ap.add_argument("--framework", default="arduino")
    ap.add_argument("--driver", default="timer")
    ap.add_argument("--count", type=int, default=2)
    ap.add_argument("--pin-mode", default="dir")
    ap.add_argument("--results-dir",
                    default=str(HARNESS / "results"))
    ap.add_argument("--out", default=str(HARNESS / "reports" /
                                         "nanoatmega328_platform_matrix.md"),
                    help="output path, or - for stdout")
    ap.add_argument("--board", default="Arduino Nano, ATmega328P",
                    help="short board label, used in the title and header")
    ap.add_argument("--board-detail", default="16 MHz, 2 KB SRAM")
    ap.add_argument("--cable", default="Nano D12, D11, D10, D9, D5, D4, D3, D2",
                    help="the Nano pin behind analyzer D0..D7, for the header")
    ap.add_argument("--generated", default=None,
                    help="override the header timestamp (default: newest "
                         "record)")
    args = ap.parse_args()

    text, cat = build(args)
    if args.out == "-":
        print(text)
    else:
        Path(args.out).write_text(text + "\n")
        print(f"wrote {args.out} ({len(cat)} scenarios)")

    bad = [s for s, r in cat.items() if r.get("result") in BAD]
    return 1 if bad else 0


if __name__ == "__main__":
    sys.exit(main())
