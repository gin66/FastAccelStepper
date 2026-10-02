#!/usr/bin/env python3
"""Run harness scenarios on a real board and check them against their evaluators.

The hardware cross-check the fixtures stand in for: each scenario goes to the
board, gets captured by the Saleae clone, and is run through the *same*
evaluator the fixtures use. The evaluator is not re-implemented here. A runner
with its own idea of what a correct waveform looks like could disagree with the
tests, and then nothing would say which was right.

One scenario per cold boot. The board is reset over RTS before each run, because
the question every scenario asks is what happens on a fresh start -- a board
carrying state from the previous scenario would answer a different one.

    python3 scripts/run_hardware.py SR_01 SR_25
    python3 scripts/run_hardware.py --list
    python3 scripts/run_hardware.py --all

Requires pyserial, a board on --port, and a Saleae reachable by sigrok-cli.
Nothing is connected to a motor: a stepper driver must not be powered from this
setup.
"""
import argparse
import json
import subprocess
import sys
import time
from pathlib import Path

try:
    import serial
except ImportError:                                      # pragma: no cover
    serial = None

sys.path.insert(0, str(Path(__file__).resolve().parent))
sys.path.insert(0, str(Path(__file__).resolve().parent / "tests"))

import run_tests as rt           # noqa: E402
import signal_parser as sp        # noqa: E402
import vcd_fixtures as vf         # noqa: E402

DEFAULT_PORT = "/dev/cu.usbserial-0001"
DEFAULT_DRIVER = "fx2lafw:conn=8.88"
DEFAULT_OUT = Path("/tmp/cap/hw")
DEFAULT_SECONDS = 4.0
DEFAULT_RATE = 24_000_000

# Seconds to let sigrok arm before the run starts, per scenario.
#
# The clone's hardware buffer holds 64 MSamples -- 2.66 s at 24 MS/s -- so the
# arm delay eats directly into the observable window. SR_25 needs a quiet tail
# after the run ends to prove the stop was clean, and at the default delay the
# buffer filled just as the final step landed.
ARM_DELAY = {"SR_25": 1.0}

# CONFIG takes a name from {1ch, 2ch, 4ch_rmt, 4ch_mcpwm, mixed <drivers>}. The
# scenario table uses short names to say which driver a scenario *requires*, so
# the two are mapped here rather than duplicating firmware vocabulary in the
# scenario table.
WIRE_CONFIG = {
    "1ch": "1ch",
    "2ch": "2ch",
    "mcpwm": "mixed mcpwm",
    "mixed_rmt_mcpwm": "mixed rmt,mcpwm",
    "i2s": "mixed i2s",
}

# Configs that drive a second stepper on D2/D3, so those channels must be
# captured. Sparse selections drop channels on this clone: -C D0,D2 yields
# nothing on D2 while -C D0,D1,D2 works, so multi-stepper captures stay
# contiguous.
MULTI_STEPPER = ("2ch", "mixed_rmt_mcpwm")


class BoardError(RuntimeError):
    """The board refused a command, or could not be reached."""


def send(ser, line, wait=1.0):
    ser.reset_input_buffer()
    ser.write(line.encode() + b"\n")
    time.sleep(wait)
    return ser.read_all().decode(errors="replace").strip()


def cold_boot(port):
    """Reset the board and wait for it to come up.

    RTS toggling is the ESP32's reset line. The boot banner is then drained
    completely: leftover banner bytes get read as the reply to the next command,
    which looks like "ERR unknown" from a command the firmware handled fine.
    """
    ser = serial.Serial(port, 115200, timeout=1.0)
    ser.dtr = False
    ser.rts = True
    time.sleep(0.12)
    ser.rts = False
    time.sleep(2.5)
    ser.reset_input_buffer()
    time.sleep(0.6)
    ser.read_all()
    return ser


def program_seconds(segments, info):
    """How long the command program runs, in seconds.

    A pause contributes its ticks directly; a move contributes steps*ticks.
    Conflating the two reports a pause as zero duration and then sizes the
    capture too short to contain it.
    """
    ticks = sum(t if n == 0 else n * t for n, t, _ in segments)
    return ticks / info["ticks_per_s"]


def wire_plan(scenario):
    """(firmware CONFIG string, channels to capture, QRUN mask) for a scenario."""
    cfg = rt.SCENARIOS[scenario][0]
    if cfg not in WIRE_CONFIG:
        raise BoardError(f"no wiring known for config {cfg!r}")
    two = cfg in MULTI_STEPPER
    return WIRE_CONFIG[cfg], ("D0,D1,D2,D3" if two else "D0,D1"), \
        ("3" if two else "1")


def run_segments(segments, wire_cfg, channels, mask, name, info,
                 port=DEFAULT_PORT, driver=DEFAULT_DRIVER,
                 out_dir=DEFAULT_OUT, seconds=DEFAULT_SECONDS,
                 rate=DEFAULT_RATE, capture_script=None, arm_delay=2.5,
                 stop_after=None, scenario=None):
    """Send an arbitrary segment program to a cold board and capture it.

    The sweep uses this rather than run_scenario: a sweep point is not a named
    scenario, it is one parameter value of one. Returns (vcd_path, note), and
    raises BoardError if the board refuses the program -- a refusal is a wiring
    or firmware problem, not a timing result.
    """
    if serial is None:
        raise BoardError("pyserial is not installed")
    out_dir = Path(out_dir)
    out_dir.mkdir(parents=True, exist_ok=True)

    ser = cold_boot(port)
    try:
        reply = send(ser, f"CONFIG {wire_cfg}")
        if reply.startswith("ERR"):
            raise BoardError(f"CONFIG refused: {reply}")

        # Most scenarios drive every stepper from one shared program, which is
        # the 3-argument QSEG. A scenario that needs per-stepper speeds uses the
        # 4-argument form with a stepper index instead, and sends one list per
        # stepper -- which is the only way to ask for two different periods.
        per_stepper = rt.per_stepper_programs(scenario, info) if scenario \
            else None
        if per_stepper:
            for idx, prog in sorted(per_stepper.items()):
                for steps, ticks, up in prog:
                    r = send(ser, f"QSEG {idx} {steps} {ticks} "
                                  f"{int(bool(up))}")
                    if r.startswith("ERR"):
                        raise BoardError(f"QSEG {idx} {steps} {ticks} "
                                         f"refused: {r}")
        else:
            for steps, ticks, up in segments:
                r = send(ser, f"QSEG {steps} {ticks} {int(bool(up))}")
                if r.startswith("ERR"):
                    raise BoardError(f"QSEG {steps} {ticks} refused: {r}")

        # The capture has to outlast the program, or a long phase reads as
        # "nothing after it" when it was still running.
        window = max(seconds, round(program_seconds(segments, info) * 2.5 + 1.0,
                                    1))
        script = Path(capture_script or
                      (Path(__file__).resolve().parent / "capture.py"))
        cap = subprocess.Popen(
            [sys.executable, str(script), "--driver", driver,
             "--sample-rate", str(rate), "--seconds", str(window),
             "--channels", channels, "--vcd", "--strict",
             "--output", str(out_dir / f"{name}.sr")],
            stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True)
        time.sleep(arm_delay)
        run_reply = send(ser, f"QRUN {mask}",
                         wait=1.5 if stop_after is None else stop_after)
        if stop_after is not None:
            # STOP mid-run: the reply is the point of the test, so wait for it
            # and record the position the board froze at.
            run_reply += " | STOP -> " + send(ser, "STOP", wait=0.6)
        _, cap_err = cap.communicate(timeout=120)
        if cap.returncode:
            raise BoardError(f"capture failed: {cap_err.strip()[:200]}")
        pos = send(ser, "POS")
        return out_dir / f"{name}.vcd", \
            (run_reply if "ERR" in run_reply else pos)
    finally:
        ser.close()


def run_scenario(scenario, port=DEFAULT_PORT, driver=DEFAULT_DRIVER,
                 out_dir=DEFAULT_OUT, seconds=DEFAULT_SECONDS,
                 rate=DEFAULT_RATE, capture_script=None):
    """Run one named scenario on a cold board and capture it."""
    info = vf.Dut().info()
    segments = rt.SCENARIOS[scenario][1](info)
    wire_cfg, channels, mask = wire_plan(scenario)
    vcd, note = run_segments(
        segments, wire_cfg, channels, mask, scenario, info,
        port=port, driver=driver, out_dir=out_dir, seconds=seconds, rate=rate,
        capture_script=capture_script,
        arm_delay=ARM_DELAY.get(scenario, 2.5),
        stop_after=rt.STOP_AFTER.get(scenario), scenario=scenario)
    return vcd, note


def judge(scenario, vcd):
    """Evaluate a capture with the scenario's own evaluator."""
    info = vf.Dut().info()
    segments = rt.SCENARIOS[scenario][1](info)
    channels, rate = sp.load_vcd(vcd)
    ok, detail = rt.evaluate(scenario, channels, rate, segments, info)
    return ok, detail, channels, rate


def summarise(scenario, ok, detail, channels, rate):
    """One result line: the evaluator's own numbers, not a recount."""
    counts = detail.get("steps")
    if isinstance(counts, dict) and "steps_measured" in counts:
        got, exp = counts["steps_measured"], counts["steps_expected"]
    else:
        metrics = sp.channel_metrics(channels["D0"], rate)
        got, exp = metrics.step_count, "?"
    return got, exp, [round(p, 2) for p in metrics_periods(channels, rate)[:6]]


def metrics_periods(channels, rate):
    return sp.channel_metrics(channels["D0"], rate).inter_step_us


def scenario_ids():
    return sorted(rt.SCENARIOS, key=lambda sid: int(sid.split("_")[1]))


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("scenarios", nargs="*", help="scenario ids, e.g. SR_01")
    ap.add_argument("--all", action="store_true", help="run every scenario")
    ap.add_argument("--list", action="store_true", help="list scenarios and exit")
    ap.add_argument("--port", default=DEFAULT_PORT)
    ap.add_argument("--driver", default=DEFAULT_DRIVER)
    ap.add_argument("--out", default=str(DEFAULT_OUT))
    ap.add_argument("--seconds", type=float, default=DEFAULT_SECONDS)
    ap.add_argument("--sample-rate", type=int, default=DEFAULT_RATE)
    ap.add_argument("--quiet", action="store_true",
                    help="print only the summary line")
    args = ap.parse_args()

    if args.list:
        for sid in scenario_ids():
            cfg, _b, mask, desc = rt.SCENARIOS[sid]
            extra = (f" (host issues STOP at {rt.STOP_AFTER[sid]} s)"
                     if sid in rt.STOP_AFTER else "")
            print(f"{sid}  {cfg:16} mask={mask}  {desc}{extra}")
        return 0

    todo = scenario_ids() if args.all else args.scenarios
    if not todo:
        ap.error("give scenario ids, or --all, or --list")

    print(f"{'scenario':9} {'verdict':10} {'steps hw':>9} {'steps exp':>10}"
          f"  periods")
    failed = []
    for scenario in todo:
        try:
            vcd, note = run_scenario(
                scenario, port=args.port, driver=args.driver,
                out_dir=args.out, seconds=args.seconds,
                rate=args.sample_rate)
        except BoardError as exc:
            print(f"{scenario:9} {'ERROR':10} {exc}")
            failed.append(scenario)
            continue
        ok, detail, channels, rate = judge(scenario, vcd)
        counts = detail.get("steps")
        if isinstance(counts, dict) and "steps_measured" in counts:
            got, exp = counts["steps_measured"], counts["steps_expected"]
        else:
            metrics = sp.channel_metrics(channels["D0"], rate)
            got, exp = metrics.step_count, "?"
        periods = metrics_periods(channels, rate)[:6]
        print(f"{scenario:9} {'PASS' if ok else 'FAIL':10} {got:>9} "
              f"{str(exp):>10}  {[round(p, 2) for p in periods]}")
        if not ok:
            failed.append(scenario)
            if not args.quiet:
                print("          ", json.dumps(detail, sort_keys=True)[:400])
        elif not args.quiet:
            print(f"          note: {note}")
    print(f"\n{len(todo) - len(failed)}/{len(todo)} accepted by their "
          f"evaluator on hardware")
    return 1 if failed else 0


if __name__ == "__main__":
    sys.exit(main())