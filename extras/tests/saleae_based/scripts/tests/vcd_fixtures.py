#!/usr/bin/env python3
"""
vcd_fixtures.py — golden waveforms for analyzer negative testing.

Why this file exists
--------------------
The evaluators in `run_tests.py` were written by an LLM. An LLM-written
analyzer has a specific failure mode: it passes, because "no problems found" is
what a harness that never rejects anything looks like. Every rule the analyzer
enforces therefore needs a waveform that must make it fail. Without such a
fixture, "PASS" is not evidence.

The good fixtures are the *measured* waveforms we expect. The bad fixtures are
the same waveforms with one specific, realistic driver fault injected. Each
bad fixture names the fault it injects and the key that must appear in the
evaluator's result when it correctly rejects the capture.

Anti-drift
----------
Good fixtures are rendered from the real scenario builders in `run_tests.py`
(`sc_period_exact` and friends) and evaluated with the real evaluators. If a
scenario changes, the fixtures no longer match and the tests fail -- which is
the point. The fixtures are therefore specification, not recorded samples.

Only the *corruptions* are hand-written.

Regenerate with:
    python3 scripts/tests/make_fixtures.py
"""

import sys
from dataclasses import dataclass, field
from pathlib import Path
from typing import Dict, List, Optional, Tuple

SCRIPTS = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(SCRIPTS))

import run_tests as rt  # noqa: E402

FIXTURE_DIR = Path(__file__).resolve().parent / "fixtures"

# The capture rate the VCD timestamps are expressed in. Matches
# run_tests.DEFAULT_RATE.
CAPTURE_HZ = 4_000_000

Event = Tuple[int, int]  # (tick, level)


# ---------------------------------------------------------------------------
# The DUT under test, as QINFO would report it
# ---------------------------------------------------------------------------


@dataclass
class Dut:
    """QINFO values for the simulated DUT behind every fixture.

    16 MHz is only a fixture convenience. The evaluators must derive every
    expectation from this, which is why they are the only source of time
    constants -- if an evaluator ever hardcodes 16 MHz, good_ticks_max below
    fails.
    """

    ticks_per_s: int = 16_000_000
    min_cmd_ticks: int = 3200
    queue_len: int = 32
    # 640 ticks = 40 us, comfortably above any realistic speed floor.
    max_speed_ticks: int = 640

    def info(self) -> Dict[str, int]:
        """The dict shape `read_qinfo()` returns, so evaluators run unchanged."""
        return {
            "ticks_per_s": self.ticks_per_s,
            "min_cmd_ticks": self.min_cmd_ticks,
            "queue_len": self.queue_len,
            "max_speed_ticks": self.max_speed_ticks,
        }


# Every run starts after a lead-in of low samples. A VCD holds one value per
# channel per instant, so a pulse rising exactly at t=0 has no preceding low
# sample and would be invisible to the analyzer. A real capture triggered to
# start exactly on the first step has the same blind spot.
LEAD = 40  # 10 us

PULSE_HIGH = 16  # 1 us at 16 MHz

# A direction change costs the driver a DIR drain before the next step
# (MIN_CMD_TICKS on ESP32, 2 us elsewhere). Shown as a gap in the waveform.
DIR_DRAIN = 3200  # 200 us


# ---------------------------------------------------------------------------
# Rendering a segment list into a waveform
# ---------------------------------------------------------------------------


def render(segments, high: int = PULSE_HIGH, lead: int = LEAD,
           drain: int = DIR_DRAIN):
    """Render `segments` -- the exact tuples `sc_*()` builders return -- into
    step and dir events.

    A segment of (n, ticks, count_up) with n > 0 emits n pulses `ticks` apart
    and then holds the line for one more period, matching the queue's trailing
    wait. n == 0 is a pause: the line simply stays low for `ticks`. A change of
    `count_up` costs a DIR drain.
    """
    step: List[Event] = []
    dirs: List[Event] = []
    t = lead
    # A VCD channel starts at 0, so the initial direction has to be written out
    # or a forward phase would look like a permanently low dir line.
    dir_value = 1 if segments and segments[0][2] else 0
    dirs.append((0, dir_value))
    # The first entry needs no drain: the direction is set before the run
    # starts, so only an actual change costs one.
    for n, ticks, up in segments:
        value = 1 if up else 0
        if value != dir_value:
            dirs.append((t, value))
            dir_value = value
            t += drain
        if n:
            for _ in range(n):
                step.append((t, 1))
                step.append((t + high, 0))
                t += ticks
            t += ticks  # trailing wait
        else:
            t += ticks
    return step, dirs


def sag(rendered, sag_ticks: int):
    """Delay every step by `sag_ticks` in total, one step at a time.

    The ISR-overrun signature. No step is lost and no single step is grossly
    wrong, so a step count and a "were any periods way off" check both pass
    while the achieved rate is quietly low. Only `rate_adherence` catches it.
    """
    step, dirs = rendered
    out = []
    extra = 0
    rise = 0
    for tick, level in step:
        if level == 1:
            # Fixed ISR cost per step: step i lands i*sag later, so every
            # period is uniformly `period + sag` and only the rate drops.
            rise = tick + extra
            extra += sag_ticks
            out.append((rise, 1))
        else:
            out.append((rise + PULSE_HIGH, 0))
    return out, dirs


# ---------------------------------------------------------------------------
# Fixture description
# ---------------------------------------------------------------------------


@dataclass
class Fixture:
    name: str
    scenario: str                # the run_tests.py scenario it stands in for
    why: str                     # what the analyzer must conclude
    step: List[Event]
    expect_pass: bool
    dirs: Optional[List[Event]] = None
    dut: Dut = field(default_factory=Dut)
    fault: Optional[str] = None  # for bad fixtures: what was injected
    expect_detail: Optional[str] = None  # key that must appear when rejected
    # For a value that is measured but deliberately not gated on: the test
    # asserts the reported number matches this (within `measurement_tol_us`).
    # Lets a fixture prove a metric is actually computed even though a poor
    # value is acceptable.
    expect_measurement: Optional[float] = None
    measurement_key: str = "first_step_skew_us"
    # Boolean flags the result must carry. For states that are *correct*
    # behaviour but easy to lose -- e.g. a delay too small to resolve, which
    # must be flagged rather than reported as 0 us.
    expect_flags: Dict[str, bool] = field(default_factory=dict)
    # Additional channels, for multi-stepper scenarios. A and B live on D0/D2.
    extra: Dict[str, List[Event]] = field(default_factory=dict)

    @property
    def segments(self):
        """The segment list this fixture depicts, from the real builder."""
        return SCENARIO_BUILDERS[self.scenario](self.dut.info())

    def info(self) -> Dict[str, int]:
        return self.dut.info()

    @property
    def path(self) -> Path:
        return FIXTURE_DIR / f"{self.name}.vcd"

    def channels(self) -> Dict[str, List[Event]]:
        ev = {"D0": self.step}
        if self.dirs is not None:
            ev["D1"] = self.dirs
        ev.update(self.extra)
        return ev

    def write(self) -> Path:
        return write_vcd(self)


SCENARIO_BUILDERS = {
    "SR_01": rt.sc_period_exact,
    "SR_02": rt.sc_steps_per_command,
    "SR_03": rt.sc_ticks_min,
    "SR_04": rt.sc_ticks_max,
    "SR_05": rt.sc_pulse_high_time,
    "SR_06": rt.sc_trailing_wait,
    "SR_07": rt.sc_long_run,
    "SR_08": rt.sc_queue_full,
    "SR_09": rt.sc_pause,
    "SR_10": rt.sc_dir_change,
    "SR_14": rt.sc_sync_start,
    "SR_27": rt.sc_single_step,
}


# ---------------------------------------------------------------------------
# VCD writing
# ---------------------------------------------------------------------------


def _ids(names: List[str]) -> Dict[str, str]:
    out = {}
    for i, n in enumerate(names):
        out[n] = chr(33 + i) if i < 94 else "\\" + str(i)
    return out


def write_vcd(fx: Fixture) -> Path:
    events = fx.channels()
    names = sorted(events.keys())
    ids = _ids(names)

    # The comment states the ACQUISITION rate, i.e. what the timestamps are
    # expressed in -- not the DUT's tick rate. `load_vcd` reads it back so the
    # analyzer never has to assume a capture rate.
    if CAPTURE_HZ >= 1e6:
        rate_txt = f"{CAPTURE_HZ / 1e6:g} MHz"
    elif CAPTURE_HZ >= 1e3:
        rate_txt = f"{CAPTURE_HZ / 1e3:g} kHz"
    else:
        rate_txt = f"{CAPTURE_HZ:g} Hz"

    lines = [
        "$date generated by make_fixtures.py $end",
        "$version libsigrok 0.5.2 $end",
        "$comment",
        f"  Acquisition with {len(names)}/{len(names)} channels at {rate_txt}",
        "$end",
        "$timescale 250 ns $end",
        "$scope module libsigrok $end",
    ]
    for n in names:
        lines.append(f"$var wire 1 {ids[n]} {n} $end")
    lines += ["$upscope $end", "$enddefinitions $end"]

    changes = {n: [] for n in names}
    for ch, evs in events.items():
        for tick, value in evs:
            changes[ch].append((tick, value))
    for ch in changes:
        changes[ch].sort()

    # A tick is 1/ticks_per_s seconds and a sample is 1/CAPTURE_HZ seconds, so
    # a tick is CAPTURE_HZ/ticks_per_s samples wide -- 0.25 samples at 16 MHz
    # and 4 MS/s. Never assume a tick is a nanosecond; the DUT declares it.
    # Real captures round to the nearest sample, so a period that is not a
    # whole number of samples (the 16-bit maximum) quantises rather than
    # failing.
    samples_per_tick = CAPTURE_HZ / fx.dut.ticks_per_s

    def sample_of(tick: int) -> int:
        return int(round(tick * samples_per_tick))

    by_sample: Dict[int, List[str]] = {}
    for ch, evs in events.items():
        for tick, value in evs:
            by_sample.setdefault(sample_of(tick), []).append(
                f"{value}{ids[ch]}")

    # Several channels can change on the same sample; sigrok writes them on one
    # line, and so does this.
    for s in sorted(by_sample):
        lines.append(f"#{s} " + " ".join(by_sample[s]))

    FIXTURE_DIR.mkdir(parents=True, exist_ok=True)
    path = FIXTURE_DIR / f"{fx.name}.vcd"
    path.write_text("\n".join(lines) + "\n")
    return path


# ---------------------------------------------------------------------------
# The fixtures
# ---------------------------------------------------------------------------

_DUT = Dut()


def _good(scenario: str, name: str, why: str, **kw) -> Fixture:
    """Render the live scenario and expect it to be accepted.

    `dir_delay_us` overrides the DIR drain so a fixture can pin a specific
    dir->step delay; the default matches the ESP32 MCPWM worst case.
    """
    drain = kw.pop("dir_delay_us", DIR_DRAIN)
    step, dirs = render(SCENARIO_BUILDERS[scenario](_DUT.info()), drain=drain)
    return Fixture(name=name, scenario=scenario, why=why, step=step,
                   dirs=dirs or None, expect_pass=True, **kw)


def _bad(scenario: str, name: str, why: str, mutate, fault: str,
         expect_detail: str) -> Fixture:
    step, dirs = render(SCENARIO_BUILDERS[scenario](_DUT.info()))
    out = mutate(step, dirs)
    if isinstance(out, tuple):          # a mutation may touch the dir line too
        step, dirs = out
    else:
        step = out
    return Fixture(name=name, scenario=scenario, why=why, step=step,
                   dirs=dirs or None, expect_pass=False, fault=fault,
                   expect_detail=expect_detail)


# --- Corruptions -----------------------------------------------------------


def _rises(step):
    return [t for t, v in step if v == 1]


def _period(step):
    r = _rises(step)
    return r[1] - r[0]


def _merge_one_pulse(step, _dirs=None):
    """Two commanded steps arrive as one: step 4's rise comes early.

    The gap between steps 3 and 4 collapses to half a period while every other
    period stays correct, so a plain step count still sees all 8 steps.
    """
    p = _period(step)
    out = list(step)
    out[2 * 3] = (_rises(step)[2] - p // 2, 1)
    return sorted(out)


def _drop_one_step(step, _dirs=None):
    """One step never arrives, leaving a single gap of two periods."""
    return list(step[:10]) + list(step[12:])


def _add_extra_pulse(step, _dirs=None):
    """A spurious narrow pulse lands between two commanded steps."""
    p = _period(step)
    between = _rises(step)[1] + p // 4
    return sorted(list(step) + [(between, 1), (between + 4, 0)])


# Sized deliberately. `period_defects` allows 5 % of the commanded period
# (2 us at 640 ticks) and `rate_adherence` allows 2 % (0.8 us). A sag between
# the two -- 24 ticks = 1.5 us = 3.75 % -- slips past the gross period check
# and past the step count, and is caught only by the rate measurement. A larger
# sag would be caught anyway, which would make the fixture useless as evidence
# that rate adherence is actually measured.
SAG_TICKS = 24


def _sag_all_steps(step, _dirs=None):
    """Every step ~4 % late, so the rate is wrong but nothing else is."""
    return sag((step, None), sag_ticks=SAG_TICKS)[0]


def _shorten_pause(step, _dirs=None):
    """The pause is cut short: steps keep arriving while it should be silent."""
    step, _ = render(SCENARIO_BUILDERS["SR_09"](_DUT.info()))
    r = _rises(step)
    pause = r[5] - r[4]
    shift = pause - 800  # leave an 800-tick gap instead
    return [(t, v) for t, v in step if t < r[5]] + \
           [(t - shift, v) for t, v in step if t >= r[5]]


def _drop_step_in_second_command(step, _dirs=None):
    """A step is swallowed in the *second* command of a two-command scenario.

    Placed deliberately after the command boundary, because the trailing wait
    makes that gap two periods wide and the per-period check now ignores it.
    Without this fixture, dropping the boundary gap from the period check would
    have quietly blinded it to a fault on the far side.
    """
    r = _rises(step)
    victim = r[3]
    ticks = _period(step)
    return [ev for ev in step if not (victim <= ev[0] < victim + ticks)]


def _short_reverse_phase(step, _dirs=None):
    """The reverse phase loses its last three steps."""
    r = _rises(step)
    return [ev for ev in step if ev[0] < r[-4]]


def _dir_glitch_in_step_high(step, dirs):
    """Toggle the direction pin in the middle of a step pulse.

    A driver latches the direction on the STEP edge, so a DIR transition inside
    the high window can make it decode the new direction for that step. The
    step timing and the step count are both perfectly correct here -- only the
    pin protocol is broken, which is why this is checked as a global invariant
    rather than inside any one scenario.
    """
    rise = _rises(step)[2]
    return list(step), sorted(list(dirs) + [(rise + 4, 0), (rise + 12, 1)])


def _no_steps(step, _dirs=None):
    """The commanded step never appears."""
    return []


FIXTURES: List[Fixture] = [
    _good("SR_01", "good_period_8_steps",
          "8 steps, period exactly as commanded"),
    _good("SR_02", "good_steps_255",
          "255 steps in one command, the uint8_t maximum"),
    # The speed floor, which is the *largest* legal ticks value. SR_01 already
    # covers 640 ticks, so pinning the floor here is what makes the two ends of
    # the range distinguishable -- and `sc_ticks_min` used to send 640 ticks,
    # identical to SR_01, so the floor was never actually exercised.
    _good("SR_03", "good_ticks_min",
          "8 steps at min_cmd_ticks, the slowest legal speed (200 us)"),
    _good("SR_04", "good_ticks_max",
          "4 steps at ticks=65535, the 16-bit maximum"),
    _good("SR_05", "good_rate_adherence",
          "16 steps, achieved rate matches the commanded rate"),
    _good("SR_09", "good_pause",
          "5 steps, a pause, 5 steps"),
    # SR_06 exists to pin the trailing wait, and it is the regression fixture
    # for a real evaluator bug: the period check used to compare the gap
    # *between* two commands against a single commanded period, so this
    # scenario failed on its own correct waveform. The gap there is two periods
    # wide by design.
    _good("SR_06", "good_trailing_wait",
          "two 2-step commands; the inter-command gap is the trailing wait"),
    _good("SR_27", "good_single_step",
          "one step, one command: no inter-step period to measure"),
    # The two ends of the per-platform range in white paper 1.3, each pinned to
    # a known value so the reported number cannot drift.
    _good("SR_10", "dir_delay_200us",
          "ESP32 MCPWM: dir->step delay of 3200 ticks (200 us)",
          expect_measurement=200.0,
          measurement_key="dir_to_first_step_min_us"),
    _good("SR_10", "good_dir_change",
          "AVR: dir->step delay of 640 ticks (40 us)",
          dir_delay_us=640,
          expect_measurement=40.0,
          measurement_key="dir_to_first_step_min_us",
          expect_flags={"below_capture_resolution": False}),
    # The Pico case: DIR and STEP land inside a single 250 ns sample, so no
    # separation is measurable. Must still pass, flagged as unresolvable.
    _good("SR_10", "dir_delay_below_resolution",
          "Pico: dir and step land in one sample, not resolvable at 4 MS/s",
          dir_delay_us=1,
          expect_measurement=0.0,
          measurement_key="dir_to_first_step_min_us",
          expect_flags={"below_capture_resolution": True}),

    _bad("SR_01", "bad_merged_pulse",
         "two steps arrive as one; the period halves",
         _merge_one_pulse, "merged step pulses", "n_short"),
    _bad("SR_02", "bad_dropped_step",
         "a step never arrives; one gap is two periods",
         _drop_one_step, "swallowed step", "missing_steps"),
    _bad("SR_01", "bad_extra_pulse",
         "a spurious pulse between two commanded steps",
         _add_extra_pulse, "spurious step", "extra_steps"),
    _bad("SR_05", "bad_rate_sag",
         "every step ~4% late: ISR overhead, step count still correct",
         _sag_all_steps, "systematic rate sag", "n_out_of_tolerance"),
    _bad("SR_09", "bad_short_pause",
         "the pause is cut short and steps arrive during it",
         _shorten_pause, "pause too short", "pause_found"),
    _bad("SR_01", "bad_dir_during_step_high",
         "the dir pin toggles inside a step pulse",
         _dir_glitch_in_step_high, "DIR changed while STEP was high",
         "n_dir_while_step_high"),
    _bad("SR_27", "bad_single_step",
         "the single commanded step never arrives",
         _no_steps, "swallowed single step", "missing_steps"),
    _bad("SR_06", "bad_dropped_step_second_command",
         "a step is swallowed in the second command, past the boundary",
         _drop_step_in_second_command, "swallowed step", "missing_steps"),
    _bad("SR_10", "bad_dir_change",
         "the reverse phase loses three steps",
         _short_reverse_phase, "swallowed reverse steps", "missing_steps"),
]

def _drop_from_second_stepper(step, _dirs=None):
    """Stepper B starts perfectly aligned but loses one step in the middle.

    The counterpart to `bad_sync_skew`: this one keeps the alignment and breaks
    only the count, so each half of the rule can be failed on its own.
    """
    return list(step[:20]) + list(step[22:])


# Two-channel fixture: stepper B's first step is three periods late. Both
# steppers get identical, correct step trains -- the only difference is when
# they start, so this isolates the alignment rule from the step-count rule.
# (render() returns (step, dirs); the steppers have no dir line here.)
_skew_a, _skew_dirs = render(SCENARIO_BUILDERS["SR_14"](_DUT.info()))
_skew_b = [(t + 3 * _DUT.max_speed_ticks, v) for t, v in _skew_a]
# Three periods of skew is an acceptable outcome on a driver that cannot do
# better, so this fixture must PASS. What it proves is that the skew is
# measured and reported accurately, which is the part that can silently rot.
FIXTURES.append(Fixture(
    name="skew_three_periods", scenario="SR_14",
    why="stepper B starts three periods after stepper A; reported, not failed",
    step=_skew_a, expect_pass=True,
    expect_measurement=3 * 40.0,
    measurement_key="first_step_skew_us",
    extra={"D2": _skew_b}))

_align_a, _align_dirs = render(SCENARIO_BUILDERS["SR_14"](_DUT.info()))
FIXTURES.append(Fixture(
    name="bad_sync_missing_step", scenario="SR_14",
    why="stepper B is perfectly aligned but loses one step",
    step=_align_a, expect_pass=False,
    fault="stepper B swallowed a step",
    expect_detail="steps_per_stepper",
    extra={"D2": _drop_from_second_stepper(_align_a)}))



def by_name(name: str) -> Fixture:
    for fx in FIXTURES:
        if fx.name == name:
            return fx
    raise KeyError(name)


def write_all() -> List[Path]:
    return [fx.write() for fx in FIXTURES]


if __name__ == "__main__":
    for p in write_all():
        print(p)
