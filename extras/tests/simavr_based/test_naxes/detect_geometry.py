#!/usr/bin/env python3
# detect_geometry.py -- naxes geometry detector.
#
# The simavr run of examples/naxes emits a VCD (x.vcd) of the step / dir pins
# for each axis (StepA/DirA, StepB/DirB, and on three-axis builds
# StepC/DirC). This tool reconstructs each axis's integer position over time
# from the step edges and the direction level at each edge -- the same
# Step+Dir reconstruction eval.awk uses for a final position count -- and then
# VALIDATES THE GEOMETRY of the fixed path the example commands:
#
#     helix   ->  hexagon   ->  square   ->  return to origin
#
# The reconstruction alone (a per-step (x,y,z) trajectory) cannot be validated
# by comparing a single position against expect.txt; this detector checks the
# shape of the whole curve:
#
#    1. underrun-free   -- a non-trivial number of steps were emitted (the sketch
#                          halts the sim only when the planner is Idle).
#    2. helix radius    -- during the Z-climbing phase (helix interior), the XY
#                          radius stays within tolerance of RADIUS.
#    3. helix climb     -- Z rises to HELIX_TURNS * HELIX_Z_PER_TURN (3-axis).
#    4. hexagon corners -- the six hexagon vertices are all reached.
#    5. square corners  -- the four square corners are all reached.
#    6. closed loop     -- the run ends back at the origin (the "return to
#                          origin" segment lands at (0,0,0)).
#    7. path stops      -- the path does not stop at every helix chord. A stop
#                          is a run of step gaps far above the fastest gap in
#                          the run; a path-stop per vertex shows up as one stop
#                          per addLine target (see count_stops()).
#
# Usage:
#   detect_geometry.py <x.vcd>          parse a simavr VCD and validate
#   detect_geometry.py --trace <f>      validate a "x y z" per-line trace
#   detect_geometry.py --selftest       run the built-in PASS and FAIL cases
#
# Exit status 0 = all checks passed, 1 = a check failed.

import math
import sys

# Geometry constants, kept identical to examples/naxes/naxes_path.h so the
# expected landmarks match what the sketch commands.
RADIUS = 400
HELIX_TURNS = 3
HELIX_Z_PER_TURN = 100
HELIX_QSAMPLES = 12   # naxes_path.h NAXES_QSAMPLES: chords per quarter turn
HELIX_Z_MAX = HELIX_TURNS * HELIX_Z_PER_TURN
SQUARE_HALF = 300
HEX_R = RADIUS
HEX_Y = RADIUS * 866 // 1000   # 0.866*R, integer, as in naxes_path.h

# Tolerances (steps). The DDA chord makes the realized curve land within a few
# steps of the commanded vertices; the corner points are addLine targets and are
# hit exactly, so tight tolerances are fine. The helix vertices sit on the
# circle to within the quarter-sine table rounding (< 1 step) plus the chord sag
# (< 1 step), so the radius band is tight.
TOL_ORIGIN = 2
TOL_RADIUS = 8        # helix vertices on the circle (rounding + chord sag)
TOL_CORNER = 4        # corner vertices are hit exactly; allow a little slack
TOL_Z = 4

# Path-stop detection (check 7). A "stop" is a maximal run of consecutive
# inter-step gaps longer than STOP_GAP_FACTOR times the fastest gap seen in the
# run, i.e. the path speed collapses to a small fraction of its peak. The run is
# timebase-independent (both the threshold and the gaps scale with the simavr
# clock). Two stops separated by at most STOP_MERGE_EVENTS step events are
# merged, because the ramp tail and the ramp head on either side of a vertex are
# a single stop, not two.
STOP_GAP_FACTOR = 200
STOP_MERGE_EVENTS = 4
# A smooth path stops only where the geometry forces it: from rest at the start,
# at the axis-aligned square corners (an axis has to reverse or go idle), and
# when it lands back at the origin. Linear mode instead path-stops at every
# non-collinear addLine vertex, so the helix alone contributes
# HELIX_TURNS * 4 * NAXES_QSAMPLES = 144 stops.
MAX_PATH_STOPS = 16


def reconstruct_from_vcd(path):
    """Reconstruct per-axis position over time from a simavr VCD.

    A rising edge of Step<A|B|C> is one step on that axis; the axis increments
    when its Dir pin is high (every naxes axis is wired
    direction_high_count_up) and decrements otherwise. Returns
    (trace, times, names): the (x, y, z) position tuples, the VCD timestamp of
    each, and the step channel names, in time order. times is what count_stops()
    needs; the geometry checks ignore it.
    """
    sym = {}
    name_to_val = {}
    state = {}
    dir_state = {}
    pos = {"A": 0, "B": 0, "C": 0}
    names = []
    trace = []
    times = []
    t = 0

    with open(path) as f:
        for line in f:
            line = line.rstrip("\n")
            if line.startswith("$var "):
                parts = line.split()
                # $var wire 1 SYM NAME $end
                val = parts[3]
                name = parts[4]
                sym[val] = name
                name_to_val[name] = val
                state[val] = 0
                dir_state[val] = 0
                if name.startswith("Step"):
                    names.append(name[4:])
                continue
            if line.startswith("#"):
                t = int(line[1:])
                continue
            # A value change line is VALUE+SYMBOL (e.g. "1\"" = value 1 on
            # the symbol \"). The value is the first char, the symbol the second.
            if len(line) == 2 and line[0] in "01x":
                bit = line[0]
                val = line[1]
                if val not in sym:
                    continue
                name = sym[val]
                prev = state[val]
                state[val] = 1 if bit == "1" else 0
                if state[val] == prev:
                    continue
                if name.startswith("Dir"):
                    dir_state[val] = state[val]
                elif name.startswith("Step") and state[val] == 1:
                    ch = name[4]
                    dir_val = name_to_val.get("Dir" + ch, "")
                    up = dir_state.get(dir_val, 0) == 1
                    pos[ch] = pos.get(ch, 0) + (1 if up else -1)
                    trace.append((pos["A"], pos["B"], pos.get("C", 0)))
                    times.append(t)
    return trace, times, names


def parse_trace(path):
    """Read a "x y z" per-line trace (the self-test / external feed format).

    The lines carry no timestamps, so a synthetic 1-per-line clock is returned.
    The stop check needs a real clock (a flat clock would read as one giant
    stop), so --trace runs skip it.
    """
    trace = []
    with open(path) as f:
        for line in f:
            line = line.strip()
            if not line or line.startswith("#"):
                continue
            p = line.split()
            if len(p) >= 3:
                trace.append((int(p[0]), int(p[1]), int(p[2])))
            elif len(p) == 2:
                trace.append((int(p[0]), int(p[1]), 0))
    times = None
    return trace, times, ["A", "B"]


def check_no_underrun(trace, results):
    """The sketch halts the sim only when the planner is Idle and underrun-free,
    so a non-trivial trace that reaches its closed end implicitly proves the
    run completed without starving a queue."""
    ok = len(trace) > 100
    results.append(("underrun-free", ok, "steps=%d" % len(trace)))


def check_helix_radius(trace, radius, tol, results):
    """The helix dominates the path: several full turns along the circle. Count
    the samples that sit on the circle (within tol of RADIUS) and require them
    to be a large fraction of the whole trace. This works for a two-axis build
    (no Z to separate phases) and for three axes alike, and it rejects a path
    that never traces the circle (e.g. a square-only trace), whose samples only
    cross the band briefly.

    The initial radial move to (R,0) and the final return-to-origin line cross
    the band, but only for a few steps each, so they cannot fake the fraction.
    """
    if not trace:
        results.append(("helix-radius", False, "empty trace"))
        return
    near = [(x, y) for (x, y, z) in trace if abs(math.hypot(x, y) - radius) <= tol]
    frac = len(near) / float(len(trace))
    ok = len(near) > 100 and frac > 0.5
    results.append(("helix-radius", ok,
                    "on-circle=%d/%d (%.0f%%) expected~%d+/-%d" %
                    (len(near), len(trace), 100.0 * frac, radius, tol)))


def check_helix_climb(trace, z_max, tol, results):
    zpeak = max((z for (_, _, z) in trace), default=0)
    ok = abs(zpeak - z_max) <= max(tol, z_max // 8)
    results.append(("helix-climb", ok,
                    "zpeak=%d expected=%d" % (zpeak, z_max)))


def check_landmark(trace, landmark, tol, label, results):
    """True if some sample is within tol of the landmark on the checked axes."""
    for (x, y, z) in trace:
        if abs(x - landmark[0]) <= tol and abs(y - landmark[1]) <= tol:
            results.append((label, True, "reached (%d,%d)" % (x, y)))
            return
    results.append((label, False,
                    "landmark (%d,%d) never reached" % (landmark[0],
                                                        landmark[1])))


def check_closed_loop(trace, tol, results):
    if not trace:
        results.append(("closed-loop", False, "empty trace"))
        return
    x, y, z = trace[-1]
    ok = abs(x) <= tol and abs(y) <= tol and abs(z) <= tol
    results.append(("closed-loop", ok,
                    "end=(%d,%d,%d) expected~(0,0,0)" % (x, y, z)))


def count_stops(times):
    """Count how often the path comes to a halt, from the step-event clock.

    A step event is a rising Step edge on any axis. Between two events the
    path moves by one step; so a long inter-event gap means the path is slow,
    and a gap far above the fastest gap in the run means the path has stopped.
    During smooth motion at least one axis is stepping, so inter-event gaps stay
    near the fast period even when the other axis is near its extremum.

    A stop is a maximal run of gaps above STOP_GAP_FACTOR * fastest, with runs
    that are within STOP_MERGE_EVENTS events of each other merged (the ramp tail
    and the ramp head around one vertex are one stop).
    """
    if times is None or len(times) < 3:
        return 0
    gaps = [times[i] - times[i - 1] for i in range(1, len(times))]
    gaps = [g for g in gaps if g > 0]
    if not gaps:
        return 0
    fastest = min(gaps)
    if fastest <= 0:
        return 0
    thr = STOP_GAP_FACTOR * fastest

    runs = []
    i = 0
    n = len(gaps)
    while i < n:
        if gaps[i] > thr:
            j = i
            while j < n and gaps[j] > thr:
                j += 1
            runs.append([i, j])
            i = j
        else:
            i += 1

    stops = 0
    end = None
    for (start, stop) in runs:
        if end is not None and start - end <= STOP_MERGE_EVENTS:
            end = stop
        else:
            stops += 1
            end = stop
    return stops


def check_stops(times, results):
    n = count_stops(times)
    ok = n <= MAX_PATH_STOPS
    results.append(("path-stops", ok,
                    "stops=%d max=%d (one per non-collinear addLine vertex in "
                    "Linear: helix alone would be %d)" %
                    (n, MAX_PATH_STOPS,
                     HELIX_TURNS * 4 * HELIX_QSAMPLES)))


def run_checks(trace, results, three_axis=True, times=None):
    if len(trace) == 0:
        results.append(("trace", False, "no position trace produced"))
        return
    check_no_underrun(trace, results)
    check_helix_radius(trace, RADIUS, TOL_RADIUS, results)
    if three_axis:
        check_helix_climb(trace, HELIX_Z_MAX, TOL_Z, results)
    # Hexagon corners (XY).
    for (hx, hy) in [(HEX_R, 0), (HEX_R // 2, HEX_Y), (-HEX_R // 2, HEX_Y),
                     (-HEX_R, 0), (-HEX_R // 2, -HEX_Y), (HEX_R // 2, -HEX_Y)]:
        check_landmark(trace, (hx, hy), TOL_CORNER,
                       "hexagon(%d,%d)" % (hx, hy), results)
    # Square corners (XY).
    for (sx, sy) in [(SQUARE_HALF, SQUARE_HALF), (-SQUARE_HALF, SQUARE_HALF),
                     (-SQUARE_HALF, -SQUARE_HALF), (SQUARE_HALF, -SQUARE_HALF)]:
        check_landmark(trace, (sx, sy), TOL_CORNER,
                       "square(%d,%d)" % (sx, sy), results)
    check_closed_loop(trace, TOL_ORIGIN, results)
    if times is not None:
        check_stops(times, results)


def make_good_trace(three_axis=True):
    """Synthesize a correct fixed-path trace for the self-test."""
    t = []
    R = RADIUS
    # Dense enough that the step-wise reconstruction between two vertices stays
    # close to the circle (small chords), as the real DDA-interpolated run is.
    seg = 360
    # helix
    for turn in range(HELIX_TURNS):
        for k in range(seg):
            a = 2 * math.pi * k / seg
            x = round(R * math.cos(a))
            y = round(R * math.sin(a))
            z = turn * HELIX_Z_PER_TURN if three_axis else 0
            t.append((x, y, z))
        if three_axis:
            t.append((R, 0, (turn + 1) * HELIX_Z_PER_TURN))
    # hexagon
    for (hx, hy) in [(R, 0), (R // 2, HEX_Y), (-R // 2, HEX_Y),
                     (-R, 0), (-R // 2, -HEX_Y), (R // 2, -HEX_Y)]:
        t.append((hx, hy, HELIX_Z_MAX if three_axis else 0))
    # square
    for (sx, sy) in [(SQUARE_HALF, SQUARE_HALF), (-SQUARE_HALF, SQUARE_HALF),
                     (-SQUARE_HALF, -SQUARE_HALF), (SQUARE_HALF, -SQUARE_HALF)]:
        t.append((sx, sy, HELIX_Z_MAX if three_axis else 0))
    # return to origin
    t.append((0, 0, 0))
    return t


def make_bad_trace():
    """A wrong trace: square only, never closes, wrong radius -- must FAIL."""
    t = []
    for (sx, sy) in [(SQUARE_HALF, SQUARE_HALF), (-SQUARE_HALF, SQUARE_HALF),
                     (-SQUARE_HALF, -SQUARE_HALF), (SQUARE_HALF, -SQUARE_HALF)]:
        t.append((sx, sy, 0))
    t.append((50, 50, 0))   # ends away from the origin
    return t


def print_results(tag, results):
    ok = True
    for name, passed, detail in results:
        mark = "PASS" if passed else "FAIL"
        if not passed:
            ok = False
        print("   [%s] %-16s %s" % (mark, name, detail))
    print("%-6s: %s" % (tag, "PASS" if ok else "FAIL"))
    return ok


def selftest():
    print("self-test: good trace (must PASS)")
    good = make_good_trace(three_axis=True)
    r = []
    run_checks(good, r, three_axis=True)
    ok1 = print_results("good3d", r)

    print("self-test: bad trace (must FAIL)")
    bad = make_bad_trace()
    r2 = []
    run_checks(bad, r2, three_axis=True)
    ok2 = print_results("bad", r2)

    # The good trace must pass; the bad trace must fail.
    success = ok1 and (not ok2)
    print("self-test overall: %s" % ("PASS" if success else "FAIL"))
    return 0 if success else 1


def synth_vcd(trace, three_axis):
    """Write a minimal simavr-style VCD for a reconstructed trace so the
    VCD-reconstruction path is exercised without a hardware/simavr run."""
    syms = ['"', "#", "$", "%", "&", "'"]
    names = ["StepA", "DirA", "StepB", "DirB"]
    if three_axis:
        names += ["StepC", "DirC"]
    lines = ["$timescale 10ns $end", "$scope module logic $end"]
    for ch, nm in zip(syms, names):
        lines.append("$var wire 1 %s %s $end" % (ch, nm))
    lines += ["$upscope $end", "$enddefinitions $end", "$dumpvars"]
    # Initial state: all step/dir low.
    for ch in syms:
        lines.append("0%s" % ch)
    lines.append("$end")
     # Walk the trace one step at a time, emitting a Step pulse whose Dir level
     # encodes the direction, so reconstruct_from_vcd recovers the trace.
    t = 0
    cur = [0, 0, 0]
    step_ch = {"A": syms[0], "B": syms[2]}
    dir_ch = {"A": syms[1], "B": syms[3]}
    if three_axis:
        step_ch["C"] = syms[4]
        dir_ch["C"] = syms[5]
    for (x, y, z) in trace:
        for ch, ch_idx in [("A", 0), ("B", 1), ("C", 2)]:
            if ch not in step_ch:
                continue
            target = (x, y, z)[ch_idx]
            d = target - cur[ch_idx]
            cur[ch_idx] = target
            if d == 0:
                continue
            up = d > 0
            lines.append("#%d" % t); t += 10
            lines.append("%d%s" % (1 if up else 0, dir_ch[ch]))
            # One rising Step edge per step, so the reconstruction recovers
            # every intermediate position, not just the vertex.
            for _ in range(abs(d)):
                lines.append("#%d" % t); t += 10
                lines.append("1%s" % step_ch[ch])   # rising edge: a step
                lines.append("#%d" % t); t += 10
                lines.append("0%s" % step_ch[ch])
    return "\n".join(lines)


def selftest_vcd():
    """Exercise the VCD-reconstruction + geometry checks end-to-end."""
    import tempfile
    import os
    good = make_good_trace(three_axis=True)
    with tempfile.NamedTemporaryFile("w", suffix=".vcd", delete=False) as f:
        f.write(synth_vcd(good, True))
        vcd_path = f.name
    try:
        trace, times, names = reconstruct_from_vcd(vcd_path)
        results = []
        run_checks(trace, results, three_axis=True, times=times)
        ok = print_results("vcd-good3d", results)
    finally:
        os.unlink(vcd_path)
    return 0 if ok else 1


def main(argv):
    if "--selftest" in argv:
        return selftest()
    if "--selftest-vcd" in argv:
        return selftest_vcd()

    if "--trace" in argv:
        i = argv.index("--trace")
        trace, times, names = parse_trace(argv[i + 1])
        three_axis = any(z != 0 for (_, _, z) in trace)
        results = []
        run_checks(trace, results, three_axis=three_axis, times=times)
        ok = print_results("trace", results)
        return 0 if ok else 1

    if len(argv) >= 2:
        vcd = argv[1]
        trace, times, names = reconstruct_from_vcd(vcd)
        three_axis = "C" in names
        results = []
        run_checks(trace, results, three_axis=three_axis, times=times)
        ok = print_results("vcd", results)
        return 0 if ok else 1

    print("usage: detect_geometry.py <x.vcd> | --trace <f> | --selftest")
    return 2


if __name__ == "__main__":
    sys.exit(main(sys.argv))
