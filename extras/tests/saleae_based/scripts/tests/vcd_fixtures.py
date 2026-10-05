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
    # The fixture prints one floor per stepper, comma-separated, plus the max --
    # the same reply shape the firmware sends and read_qinfo() parses. A fixture
    # with a different shape would let a QINFO parsing bug pass every test here.
    max_speed_per_stepper: Tuple[int, ...] = (max_speed_ticks,)

    # What QFILL reported reaching. Present because a hardware run's info dict
    # carries it (measure() records it before the evaluator is called) and an
    # evaluator that read the key unconditionally would then raise on every
    # real capture. The queue here is 32 deep, so a 16-entry request is met in
    # full; a board where it is not would report the smaller number, and the
    # bound follows the report rather than the request.
    queue_filled_entries: int = rt.QUEUE_FILL_ENTRIES

    def info(self, steppers: int = 1) -> Dict[str, int]:
        """The dict shape `read_qinfo()` returns, so evaluators run unchanged."""
        floors = [self.max_speed_ticks] * steppers
        return {
            "ticks_per_s": self.ticks_per_s,
            "min_cmd_ticks": self.min_cmd_ticks,
            "queue_len": self.queue_len,
            "max_speed_first_ticks": floors[0],
            "max_speed_per_stepper": floors,
            # The largest floor, which is what every builder means by "the
            # fastest period this program may use".
            "max_speed_ticks": max(floors),
            "queue_filled_entries": self.queue_filled_entries,
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

    A segment of (n, ticks, count_up) with n > 0 emits n pulses `ticks` apart.
    n == 0 is a pause: the line simply stays low for `ticks`. A change of
    `count_up` costs a DIR drain.

    There is no extra period after a command. Measured on ESP32/RMT at 24 MS/s
    with two 2-step commands at ticks=3200, the gap from the last step of one
    command to the first step of the next was 199.833 us -- identical to the
    intra-command period, ratio 1.000. An earlier version of this function
    appended one extra `ticks` after every command ("trailing wait"), which
    described a 2x gap the hardware never produces.
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
    # How many steppers this fixture's configuration connects. Only needed to
    # work out which channel a marker would occupy -- 1 for every scenario in
    # the catalogue today, but the marker channel moves if that ever changes,
    # and a test that hardcoded D7 would quietly score the wrong channel.
    steppers: int = 1
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

    @property
    def stride(self) -> int:
        """2 when the scenario drives a dir pin alongside each step pin."""
        return 2 if self.dirs is not None else 1

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
    "SR_11": rt.sc_dir_change_both_ways,
    "SR_12": rt.sc_multi_step_direction,
    "SR_13": rt.sc_ticks_error_rejected,
    "SR_18": rt.sc_mcpwm_overrun_after_255,
    "SR_19": rt.sc_mcpwm_overrun_boundary,
    "SR_20": rt.sc_pause_after_full_command,
    "SR_14": rt.sc_sync_start,
    "SR_16": rt.sc_multi_stepper_timing,
    "SR_17": rt.sc_sync_cross_driver,
    "SR_15": rt.sc_sync_independent_speeds,
    "SR_21": rt.sc_rmt_buffer_split,
    "SR_23": rt.sc_i2s_timing,
    "SR_25": rt.sc_emergency_stop,
    "SR_26": rt.sc_pause_ticks_max,
    "SR_27": rt.sc_single_step,
    "SR_30": rt.sc_emergency_stop,
    "SR_31": rt.sc_max_stepper_count,
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


def _drop_steps_from_reverse_phase(step, _dirs=None):
    """A phase loses steps, so the per-phase counts no longer match.

    SR_12's real claim is that each phase contributes its own step count, not
    merely that the total is right. Dropping three from the middle phase keeps
    the total correct for a naive counter while breaking the per-phase rule.
    """
    r = _rises(step)
    mid = len(r) // 2
    ticks = _period(step)
    victim = r[mid]
    return [ev for ev in step
            if not (victim <= ev[0] < victim + 3 * ticks)]


def _render_rejected_steps():
    """The 8 steps a refused SR_13 command must not emit.

    The fixture for "the rejection is reported but the steps happen anyway",
    which is the defect SR_13 targets.
    """
    steps = [(i * 640, 1) for i in range(8)]
    out = []
    for t, _ in steps:
        out += [(t, 1), (t + 16, 0)]
    return out


def _drop_trailing_step(step, _dirs=None):
    """The last commanded step never arrives.

    The PCNT high-limit case: after a full 255-step run, a single-step command
    has to re-arm the limit from the live counter. If it does not, that step is
    lost. This is the one fixture whose whole purpose is to prove the rule can
    fail on its own target defect.
    """
    return list(step[:-2])


def _duplicate_trailing_step(step, _dirs=None):
    """One pulse too many after a pause, i.e. a limit left over from before."""
    r = _rises(step)
    extra = r[-1] + _period(step) // 2
    return sorted(list(step) + [(extra, 1), (extra + 4, 0)])


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
    # The MCPWM/PCNT overrun cases. The good fixture is what the hardware
    # produces (verified on the ESP32 at 24 MS/s: 255 steps at 40 us, one
    # 439.67 us gap, then exactly one more step -- 256 total). The bad fixture
    # drops that final single step, which is precisely the defect SR_18 exists
    # to detect, so the rule is proven able to fail on its own target.
    _good("SR_18", "good_overrun_after_255",
          "255 steps, a 440 us gap, then exactly one more"),
    _good("SR_19", "good_overrun_boundary",
          "200 steps then a single step at the boundary"),
    _good("SR_20", "good_pause_after_full_command",
          "255, a pause, then 255 more"),

    _bad("SR_18", "bad_lost_trailing_step",
         "the single step after a 255-step run is swallowed",
         _drop_trailing_step, "swallowed trailing step", "missing_steps"),
    _bad("SR_20", "bad_extra_step_after_pause",
         "a duplicate pulse follows the pause",
         _duplicate_trailing_step, "duplicate step after pause", "extra_steps"),

    _good("SR_11", "good_dir_change_both_ways",
          "20 reverse then 20 forward; the drain is symmetric"),
    _good("SR_12", "good_multi_step_direction",
          "forward, reverse, forward: two direction changes"),

    _bad("SR_12", "bad_phase_step_count",
         "a phase loses three steps; the per-phase counts stop matching",
         _drop_steps_from_reverse_phase, "swallowed phase steps",
         "steps_per_phase"),
]

# The dir pin pinned to the wrong level: every commanded step still arrives, so
# a step-count-only check passes, and what fails is the dir tracking the phases.
# Built directly rather than through _bad() because the mutation needs to know
# the scenario's final direction to pick a level that is actually wrong --
# SR_11 and SR_12 both finish forward, so "stuck at the starting level" would
# be correct half the time.
_dir_steps, _ = render(SCENARIO_BUILDERS["SR_11"](_DUT.info()))
FIXTURES.append(Fixture(
    name="bad_dir_never_follows", scenario="SR_11",
    why="the dir pin never follows either commanded direction",
    step=_dir_steps, dirs=[(0, 0)], expect_pass=False,
    fault="dir pin never followed the command",
    expect_detail="expected_final_dir"))

# SR_16/17 are two-channel: stepper A on D0/D1, stepper B on D2/D3.
_s16_a, _ = render(SCENARIO_BUILDERS["SR_16"](_DUT.info()))
FIXTURES.append(Fixture(
    name="good_two_steppers_even", scenario="SR_16",
    why="two steppers, both at the commanded period",
    step=_s16_a, dirs=[(0, 1)], expect_pass=True,
    extra={"D2": list(_s16_a), "D3": [(0, 1)]}))

_s17_a, _ = render(SCENARIO_BUILDERS["SR_17"](_DUT.info()))
# The measured cross-driver offset on the ESP32: RMT arms first, MCPWM+PCNT
# follows 29.583 us later at ticks=640 (40 us), i.e. about three quarters of a
# period. Reported, not gated on -- white paper 1.3 -- but pinned here so the
# number cannot silently decay to zero, which is what would hide the fact that
# these two drivers are not aligned at all.
FIXTURES.append(Fixture(
    name="cross_driver_skew_30us", scenario="SR_17",
    why="RMT and MCPWM+PCNT start ~30 us apart; reported, not failed",
    step=_s17_a, dirs=[(0, 1)], expect_pass=True,
    expect_measurement=29.583,
    measurement_key="first_step_skew_us",
    extra={"D2": [(t + 473, v) for t, v in _s17_a], "D3": [(0, 1)]}))

# The negative fixture for eval_multi_stepper_periods: stepper B's period is
# stretched, which a check on stepper A alone would miss. That is the whole
# point of SR_16 -- the second stepper is the subject.
_s16_b = _s16_a[:20] + [(t + 160, v) for t, v in _s16_a[20:]]
FIXTURES.append(Fixture(
    name="bad_second_stepper_slow", scenario="SR_16",
    why="stepper B runs at a longer period than commanded",
    step=_s16_a, dirs=[(0, 1)], expect_pass=False,
    fault="second stepper period wrong",
    expect_detail="per_stepper",
    extra={"D2": _s16_b, "D3": [(0, 1)]}))

# SR_15: stepper A on D0/D1 at its own ticks, stepper B on D2/D3 at twice them.
# The evaluator checks each against *its* expected period, so a capture where
# both steppers ran at the same speed must fail -- that is the defect SR_15
# exists to catch, and it cannot be expressed with the shared program.
_per15 = rt.per_stepper_programs("SR_15", _DUT.info())
_a15 = render(_per15[0])[0]
_b15 = render(_per15[1])[0]
# The same cross-driver offset SR_17 measured, in ticks: ticks, not samples.
_s15_skew = round(29.583 * _DUT.info()["ticks_per_s"] / 1e6)
FIXTURES.append(Fixture(
    name="good_independent_speeds", scenario="SR_15",
    why="both steppers start together, then each keeps its own period",
    step=_a15, dirs=[(0, 1)], expect_pass=True,
    expect_measurement=round(_s15_skew * 1e6 / _DUT.info()["ticks_per_s"], 4),
    measurement_key="first_step_skew_us",
    extra={"D2": [(t + _s15_skew, v) for t, v in _b15], "D3": [(0, 1)]}))

# B collapsed onto A's period: the arm aligned and both steppers stepped, which
# is exactly the wrong answer and the only way this scenario can fail.
FIXTURES.append(Fixture(
    name="bad_independent_speeds_collapsed", scenario="SR_15",
    why="both steppers ran at A's period instead of their own",
    step=_a15, dirs=[(0, 1)], expect_pass=False, expect_detail="B",
    fault="stepper B's period is A's, not its own",
    extra={"D2": _a15, "D3": [(0, 1)]}))

# B missing half its steps: independent period, wrong count.
FIXTURES.append(Fixture(
    name="bad_independent_speeds_lost_steps", scenario="SR_15",
    why="stepper B dropped steps while keeping its own period",
    step=_a15, dirs=[(0, 1)], expect_pass=False, expect_detail="B",
    fault="stepper B lost steps",
    extra={"D2": _b15[:-2 * 40], "D3": [(0, 1)]}))

# SR_21/23/26 need no bespoke construction: the renderer already produces a
# correct waveform from the builder's own segments, which is the point -- these
# three assert nothing the renderer cannot express.
FIXTURES.append(Fixture(
    name="good_rmt_no_split_gap", scenario="SR_21",
    why="200 RMT steps with no inter-step gap at a buffer split",
    step=render(SCENARIO_BUILDERS["SR_21"](_DUT.info()))[0], dirs=[(0, 1)],
    expect_pass=True))
FIXTURES.append(Fixture(
    name="good_i2s_period", scenario="SR_23",
    why="I2S step output at the commanded period",
    step=render(SCENARIO_BUILDERS["SR_23"](_DUT.info()))[0], dirs=[(0, 1)],
    expect_pass=True))
FIXTURES.append(Fixture(
    name="good_pause_16bit", scenario="SR_26",
    why="a pause of exactly 65535 ticks between two steps",
    step=render(SCENARIO_BUILDERS["SR_26"](_DUT.info()))[0], dirs=[(0, 1)],
    expect_pass=True))

# SR_25 and SR_30 send the *same* waveform and assert opposite outcomes, because
# that is the property being pinned: `stopMove()` must not truncate queued
# motion, `forceStopAndNewPosition()` must discard it. One scenario with a flag
# would assert half a contract and call it the whole thing.
#
# There is no scenario for `forceStop()` and there will not be one. Its only
# effect on a queue this harness fills itself is `ignore_commands = true`, which
# refuses *later* addQueueEntry() calls -- and the feeder is stopped after the
# start, so there are none and the call cannot fail. Measured: identical
# waveforms, 4080 of 4080 steps, on rmt, i2s_direct and mcpwm_pcnt alike.
# `stopMove()` is weaker still, being a flag the ramp generator consults for its
# next command while this harness drives addQueueEntry() directly and never runs
# one, so SR_25 measures the drain rather than the API -- it is kept because it
# is the baseline SR_30 is read against.
#
# Both carry a marker channel (D7) that steps high at the instant the stop was
# processed, so the boundary is on the waveform. Without it the only available
# inference was "the pulses stopped", and the capture this rig delivers is not the
# capture it requests (24 MHz truncates), so that inference could not tell a stop
# from the recording ending -- which is how SR_25 came to report a complete
# move as "STOP was never processed".
_stop_ticks = rt.legal_ticks(_DUT.info(), rt.QUEUE_FILL_STEPS,
                             _DUT.info()["max_speed_ticks"])


def run_tests_stop_quarter_of_fill():
    """The marker depth the real harness would use on this DUT.

    Asked of `stop_after_for()` rather than written down, so a fixture cannot
    depict a stop the runner would never issue -- the failure mode the SR_25/SR_30
    pair had when the delay was a fixed 1 ms and i2s_direct's marker landed
    before its first step.
    """
    info = _DUT.info()
    segs = rt.sc_emergency_stop(info)
    return int(round(rt.stop_after_for("SR_30", segs, info)
                     * info["ticks_per_s"] / _stop_ticks))


# The whole run. QRUN on a QFILLed queue adds nothing after the start, so the
# waveform is the fill and only the fill, however long the program behind it is.
_drained = render([(rt.QUEUE_FILL_STEPS, _stop_ticks, True)])[0]
# Where the marker sits: a quarter into the fill, which is what stop_after_for()
# issues on a DUT of this shape (640 ticks = 40 us, so a quarter of 40.8 ms).
# Fractional rather than a round step count because the delay is now derived from
# the DUT's tick rate and period -- 1020 steps here, and it has to track the
# waveform or the fixture would depict a stop the harness never issues.
_marker_in_steps = run_tests_stop_quarter_of_fill()
_early = _drained[0][0] + _marker_in_steps * _stop_ticks

# SR_25, stopMove(): the marker fires 1 ms in and the whole filled queue still
# runs. Passing here means the stop did *not* cut anything queued.
FIXTURES.append(Fixture(
    name="good_stop_move_does_not_truncate", scenario="SR_25",
    why="stopMove() is a flag for the ramp's next command: the fill drains",
    step=_drained, dirs=[(0, 1)], extra={"D7": [(_early, 1)]},
    expect_pass=True,
    expect_flags={"stop_measured": True, "not_truncated": True}))

# The violation: a stopMove that did truncate would be the library breaking its
# own contract. With the feeder stopped after the start this is a *short* run --
# the queue lost most of its contents -- rather than a run shorter than the
# program, which the feeder could have produced by itself.
_truncated = render([(255, _stop_ticks, True)])[0]
FIXTURES.append(Fixture(
    name="bad_stop_move_truncated", scenario="SR_25",
    why="the filled queue was cut short, which stopMove() must never do",
    step=_truncated, dirs=[(0, 1)], extra={"D7": [(_early, 1)]},
    expect_pass=False, expect_detail="not_truncated",
    fault="stopMove truncated a queued move, against its documented contract"))

# SR_30, forceStopAndNewPosition(): the queue is emptied, so the filled queue
# does not drain. This is what SR_25's good waveform looks like, which is the
# point: the two APIs are told apart by exactly this.
_aborted = render([(_marker_in_steps + 1, _stop_ticks, True)])[0]
_aborted_marker = _aborted[0][0] + _marker_in_steps * _stop_ticks
FIXTURES.append(Fixture(
    name="good_force_stop_and_new_pos_discards", scenario="SR_30",
    why="the queue is emptied, so only the steps already out came out",
    step=_aborted, dirs=[(0, 1)], extra={"D7": [(_aborted_marker, 1)]},
    expect_pass=True,
    expect_flags={"stop_measured": True, "queue_discarded": True,
                  "stop_interrupted_the_run": True}))

# The violation: the fill drained, so forceStopAndNewPosition() did not empty
# anything. Sending the stop a driver ignores, or wiring it to the wrong API,
# looks exactly like this -- and it is what forceStop() does, which is why that
# one has no scenario.
FIXTURES.append(Fixture(
    name="bad_abort_queue_still_drained", scenario="SR_30",
    why="the filled queue drained behind the abort, which empties nothing",
    step=_drained, dirs=[(0, 1)], extra={"D7": [(_early, 1)]},
    expect_pass=False, expect_detail="queue_discarded",
    fault="the queue drained, so the stop was forceStop() rather than "
          "forceStopAndNewPosition()"))

# A driver that keeps stepping commands it can no longer be told about: more
# than two entries past the marker. The gate catches it, and the count is the
# number a driver with a hardware transmit buffer would be characterised by.
FIXTURES.append(Fixture(
    name="bad_abort_queue_leaked_a_tail", scenario="SR_30",
    why="three entries of steps came out after the queue was emptied",
    # Marker plus the tail bound plus one more entry, so the leak is over the
    # bound rather than at it -- the depth is measured from the marker, which
    # moves with the harness's own delay. Two events per step, so the cut is in
    # event indices and lands on a falling edge.
    step=_drained[:2 * (_marker_in_steps + (rt.ABORT_TAIL_ENTRIES + 1) * 255)],
    dirs=[(0, 1)],
    extra={"D7": [(_early, 1)]},
    expect_pass=False, expect_detail="steps_after_stop",
    fault="the driver emitted queued commands after the queue was emptied"))

# SR_13 is the inverse of every other fixture: the command is refused, so the
# correct waveform is a pin that never moves. There is no `render()` output to
# start from -- the whole point is that nothing is emitted.
FIXTURES.append(Fixture(
    name="good_rejected_emits_nothing", scenario="SR_13",
    why="a command below MIN_CMD_TICKS is refused and emits no pulse",
    step=[], dirs=[(0, 1)], expect_pass=True))

# The matching bad fixture is the bug SR_13 exists to catch: the rejection is
# reported but the steps still happen.
FIXTURES.append(Fixture(
    name="bad_rejected_still_steps", scenario="SR_13",
    why="a refused command still emits its 8 steps",
    step=_render_rejected_steps(), dirs=[(0, 1)], expect_pass=False,
    fault="rejected command still stepped",
    expect_detail="steps_measured"))


# SR_31: the maximum stepper count, `nodir`, eight channels and eight steppers.
#
# The widest map the pin mode allows, which is what `Pins.for_scenario("SR_31")`
# builds, so every analyzer channel carries a stepper here. That is the whole
# point of the fixture: a max-count test evaluated against a 2-stepper map would
# report on steppers A and B and call the run done, and the six channels it never
# looked at are the six that would have shown the defect.
_s31, _ = render(SCENARIO_BUILDERS["SR_31"](_DUT.info()))
FIXTURES.append(Fixture(
    name="good_max_count_all_steppers", scenario="SR_31",
    why="every one of the eight steppers gets all 64 steps at the period",
    step=_s31, expect_pass=True, steppers=8,
    extra={f"D{i}": list(_s31) for i in range(1, 8)}))

# The defect AGENTS.md records for MCPWM/PCNT above n=2, which is exactly the
# shape this scenario exists to reach: stepper H emits its own share correctly
# *and then keeps going*, so a check that only asks "did the first steps arrive"
# passes. eval_scale counts every stepper's total, so the free-run is the defect
# -- 22 143 edges where 64 were commanded is the measured signature.
_s31_ticks = _period(_s31)
_h_free_run = list(_s31) + [(t + rt.SCALE_STEPS * _s31_ticks, v)
                            for t, v in _s31]
FIXTURES.append(Fixture(
    name="bad_max_count_free_running_stepper", scenario="SR_31",
    why="the last stepper never stops; its share is right and its total is not",
    step=_s31, expect_pass=False, steppers=8,
    fault="stepper H free-runs past the end of the program",
    expect_detail="per_stepper",
    extra={**{f"D{i}": list(_s31) for i in range(1, 7)},
           "D7": _h_free_run}))

# And the other half: one stepper silently short. A count check that summed the
# channels instead of judging each would see 8 x 64 - 2 and pass it.
FIXTURES.append(Fixture(
    name="bad_max_count_one_stepper_short", scenario="SR_31",
    why="stepper E swallows two steps while every other stepper is exact",
    step=_s31, expect_pass=False, steppers=8,
    fault="stepper E swallowed two steps",
    expect_detail="per_stepper",
    extra={**{f"D{i}": list(_s31) for i in (1, 2, 3, 5, 6, 7)},
           "D4": list(_s31[:20]) + list(_s31[24:])}))


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
