#!/usr/bin/env python3
"""
test_analyzer_fixtures.py — does the analyzer actually fail when it should?

The evaluators in `run_tests.py` decide whether a capture is acceptable. They
were written by an LLM, and the characteristic LLM failure here is a harness
that can only say yes: it finds a period, compares it loosely, and returns
pass. This suite is the counterweight.

It drives the *real* evaluators over the *real* golden waveforms in
`vcd_fixtures.py`, in both directions:

  * every good fixture must be accepted, and
  * every bad fixture must be rejected, with the specific defect named.

plus two anti-rot guards, because a test suite that silently stops covering
things is worse than no suite:

  * every rule the analyzer enforces has at least one fixture that must fail
    it, so no rule can be added without a negative test, and
  * every fixture is reachable from a scenario, so no fixture is left unrun.

Run with:
    python3 -m unittest discover -s scripts/tests -v
"""

import json
import sys
import unittest
from pathlib import Path

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE.parent))

import run_tests as rt  # noqa: E402
import signal_parser as sp  # noqa: E402
import vcd_fixtures as vf  # noqa: E402


def evaluate(fx: vf.Fixture):
    """Run the real evaluator for a fixture's scenario over its waveform.

    Goes through the same path as a hardware run: the VCD is parsed from disk
    with `load_vcd`, and the evaluator registered in `EVALUATORS` is called
    with the segment list from the real scenario builder. Nothing is stubbed,
    so a fixture that passes here really does mean the analyzer accepts it.
    """
    if fx.scenario not in rt.EVALUATORS:
        raise unittest.SkipTest(f"{fx.scenario} has no evaluator yet")
    channels, rate = sp.load_vcd(fx.path)
    # `evaluate`, not the raw evaluator: that applies the global pin invariants
    # too, so a fixture that violates one of them fails here as well.
    return rt.evaluate(fx.scenario, channels, rate, fx.segments, fx.info(),
                      rt.Pins.for_scenario(fx.scenario).map)


def flatten(detail) -> str:
    """Flatten a result dict so a test can assert a key is present anywhere.

    The named defect may sit at any depth -- `period.n_short` for a merged
    pulse, `adherence.n_out_of_tolerance` for rate sag -- and the test should
    not have to know which branch produced it.
    """
    return json.dumps(detail, sort_keys=True)


class TestGoodFixtures(unittest.TestCase):
    """A correct waveform must be accepted. These guard against over-strict
    evaluators, which are as broken as under-strict ones."""

    def test_good_fixtures_pass(self):
        good = [fx for fx in vf.FIXTURES if fx.expect_pass]
        self.assertTrue(good, "no good fixtures defined")
        for fx in good:
            with self.subTest(fixture=fx.name, scenario=fx.scenario):
                self.assertTrue(fx.path.exists(),
                                f"{fx.path.name} missing; run make_fixtures.py")
                ok, detail = evaluate(fx)
                self.assertTrue(
                    ok, f"{fx.name} ({fx.why}) was rejected: "
                        f"{json.dumps(detail, sort_keys=True)}")


class TestLongRuns(unittest.TestCase):
    """SR_07 (2000 steps) and SR_08 (4000 steps) at full size.

    These two are not committed as fixtures: a 4000-step change-only VCD is
    ~300 KB of committed bytes that buys one assertion -- the analyzer counts
    every step of a long run. Generating them keeps the same coverage against
    the same real evaluators, at the real step counts, for no repo weight.

    The point is scale. A waveform this long is where an off-by-one in a
    window, a cap on how many periods are examined, or an assumption that all
    segments share one period shows up -- none of which a 16-step fixture
    would reach.
    """

    def _render(self, scenario, mutate=None):
        import tempfile

        info = vf.Dut().info()
        segs = vf.SCENARIO_BUILDERS[scenario](info)
        step, dirs = vf.render(segs)
        if mutate:
            step = mutate(step)
        fx = vf.Fixture(name=f"tmp_{scenario}", scenario=scenario, why="",
                        step=step, dirs=dirs, expect_pass=True)
        # Write to a scratch dir: these are too large to keep in the repo, and
        # the evaluator still reads them back from disk through load_vcd, so
        # the path is exercised exactly as it is for a real capture.
        with tempfile.TemporaryDirectory() as tmp:
            vf.FIXTURE_DIR = Path(tmp)
            try:
                fx.write()
                return evaluate(fx)
            finally:
                vf.FIXTURE_DIR = HERE / "fixtures"

    def test_long_run_counts_every_step(self):
        for scenario, expect in (("SR_07", 2000), ("SR_08", 4000)):
            with self.subTest(scenario=scenario):
                ok, detail = self._render(scenario)
                self.assertTrue(ok, f"{scenario} rejected a correct long run: "
                                    f"{json.dumps(detail, sort_keys=True)}")
                self.assertEqual(detail["steps"]["steps_measured"], expect)
                self.assertEqual(detail["steps"]["steps_expected"], expect)

    def test_long_run_periods_are_all_checked(self):
        """Every intra-command period must be examined, not a window of them.

        Guards against a cap in the period check: with 1999 periods in SR_07,
        truncating to the first few would pass a run whose tail is wrong.
        """
        for scenario, expect_periods in (("SR_07", 1999), ("SR_08", 3999)):
            with self.subTest(scenario=scenario):
                _ok, detail = self._render(scenario)
                self.assertEqual(detail["period"]["periods_measured"],
                                 expect_periods)

    def test_a_dropped_step_at_the_end_of_a_long_run_is_caught(self):
        """The last step, not the first.

        A defect at the tail of a long run is the one a bounded window misses,
        and the tail is where a queue that ran dry underflows.
        """
        def drop_last(step):
            rises = [t for t, v in step if v == 1]
            victim = rises[-1]
            ticks = rises[1] - rises[0]
            return [ev for ev in step
                    if not (victim <= ev[0] < victim + ticks)]

        ok, detail = self._render("SR_07", drop_last)
        self.assertFalse(ok, "a dropped final step in a long run was accepted")
        self.assertEqual(detail["steps"]["missing_steps"], 1)


class TestBadFixtures(unittest.TestCase):
    """The point of the whole exercise. A bad waveform must be rejected, and
    the reason must be visible in the result rather than swallowed."""

    def test_bad_fixtures_fail(self):
        bad = [fx for fx in vf.FIXTURES if not fx.expect_pass]
        self.assertTrue(bad, "no bad fixtures defined")
        for fx in bad:
            with self.subTest(fixture=fx.name, fault=fx.fault):
                self.assertTrue(fx.path.exists(),
                                f"{fx.path.name} missing; run make_fixtures.py")
                ok, detail = evaluate(fx)
                self.assertFalse(
                    ok, f"{fx.name} ({fx.fault}) was ACCEPTED but must be "
                        f"rejected: {json.dumps(detail, sort_keys=True)}")

    def test_rejection_names_the_defect(self):
        """A rejection that gives no reason is not much use to whoever reads
        the report, so the specific defect has to appear in the result."""
        for fx in [f for f in vf.FIXTURES if not f.expect_pass]:
            with self.subTest(fixture=fx.name, expect=fx.expect_detail):
                self.assertIsNotNone(fx.expect_detail,
                                     "bad fixture must name a defect to look for")
                ok, detail = evaluate(fx)
                self.assertFalse(ok)
                self.assertIn(
                    fx.expect_detail, flatten(detail),
                    f"{fx.name} rejected without reporting {fx.expect_detail!r}: "
                    f"{json.dumps(detail, sort_keys=True)}")


class TestGlobalInvariants(unittest.TestCase):
    """Rules that apply to every capture, not just one scenario.

    The one invariant so far: the direction pin must never change while the step
    pin is high. A driver latches direction on the STEP edge, so a DIR
    transition inside the pulse window can make it decode the new direction for
    that step.

    It belongs here rather than in a scenario because a DIR-during-high would
    be a bug anywhere in the program -- a per-scenario check would only catch it
    in whichever scenario happens to change direction.
    """

    def test_every_result_reports_the_invariants(self):
        for fx in vf.FIXTURES:
            with self.subTest(fixture=fx.name):
                _ok, detail = evaluate(fx)
                self.assertIn("invariants", detail,
                              f"{fx.name}: result carries no invariant block")
                self.assertIn("ok", detail["invariants"])

    def test_only_the_planted_fixture_violates_them(self):
        for fx in vf.FIXTURES:
            if fx.name == "bad_dir_during_step_high":
                continue
            with self.subTest(fixture=fx.name):
                _ok, detail = evaluate(fx)
                self.assertTrue(
                    detail["invariants"]["ok"],
                    f"{fx.name} violates a pin invariant: "
                    f"{json.dumps(detail['invariants'])}")

    def test_the_invariant_fixture_fails_only_on_the_invariant(self):
        """The point of a global rule.

        This capture has a correct step count, a correct inter-step period and
        correct rate adherence. Only the pin protocol is broken. If the
        scenario's own checks were the whole verdict, it would pass.
        """
        fx = vf.by_name("bad_dir_during_step_high")
        ok, detail = evaluate(fx)
        self.assertFalse(ok)
        self.assertFalse(detail["invariants"]["ok"])
        self.assertGreater(detail["invariants"]["n_dir_while_step_high"], 0)
        # The scenario's own measurements are all clean.
        for key in ("period", "steps", "adherence"):
            self.assertTrue(detail[key]["ok"],
                            f"the invariant fixture should be clean on {key}, so "
                            f"the invariant is what rejects it")

    def test_boundary_edges_are_not_violations(self):
        """A DIR change on the rise or fall sample is where it belongs."""
        step = [0, 0, 1, 1, 1, 1, 0, 0, 0, 0]   # high from 2 to 6
        at_rise = [0, 0, 1, 1, 1, 1, 1, 0, 0, 0]  # rises at 2, falls after 6
        self.assertEqual(
            sp.dir_changes_during_step_high(at_rise, step, 1_000_000), [])
        inside = [0, 0, 0, 0, 1, 0, 0, 0, 0, 0]   # changes at 4 and 5
        self.assertTrue(
            sp.dir_changes_during_step_high(inside, step, 1_000_000))


class TestMeasurements(unittest.TestCase):
    """Some values are measured but deliberately not gated on.

    Synchronized-start skew is the clear case: how closely the steppers begin
    depends on the pulse driver and on what the processor is doing at that
    instant, so a few microseconds of offset is a platform characteristic, not a
    bug. Failing a capture for it would report a characteristic as a defect.

    The risk is the opposite one -- a metric that is reported but never actually
    computed, so it reads 0.0 forever. These fixtures pin the number down: the
    waveform has a known skew, and the reported value has to match it.
    """

    def test_reported_measurements_are_accurate(self):
        measured = [fx for fx in vf.FIXTURES if fx.expect_measurement is not None]
        self.assertTrue(measured, "no measurement fixtures defined")
        for fx in measured:
            with self.subTest(fixture=fx.name):
                ok, detail = evaluate(fx)
                # A poor value is acceptable...
                self.assertTrue(ok, f"{fx.name} should pass: "
                                    f"{json.dumps(detail, sort_keys=True)}")
                # ...but it has to be the right poor value, not a default.
                self.assertIn(fx.measurement_key, detail)
                got = detail[fx.measurement_key]
                self.assertAlmostEqual(
                    got, fx.expect_measurement, delta=1.0,
                    msg=f"{fx.name}: {fx.measurement_key}={got}, expected "
                        f"{fx.expect_measurement}")

    def test_expected_flags_are_set(self):
        """States that are correct but easy to lose.

        The clearest case: a dir->step delay smaller than one capture sample.
        The capture is fine, but the number must be reported as unresolvable
        rather than as 0 us -- otherwise a Pico run reads as "no delay at all"
        and nobody notices the measurement was never taken.
        """
        flagged = [fx for fx in vf.FIXTURES if fx.expect_flags]
        self.assertTrue(flagged, "no flag fixtures defined")
        for fx in flagged:
            with self.subTest(fixture=fx.name):
                ok, detail = evaluate(fx)
                self.assertTrue(ok)
                for key, want in fx.expect_flags.items():
                    self.assertIn(key, detail,
                                  f"{fx.name}: result has no {key!r}")
                    self.assertEqual(
                        detail[key], want,
                        f"{fx.name}: {key}={detail[key]!r}, expected {want!r}")


class TestAntiRot(unittest.TestCase):
    """Guards so the suite cannot quietly stop covering the analyzer."""

    def test_every_rule_has_a_failing_fixture(self):
        """Each rule an evaluator enforces needs a waveform that must fail it.

        Without this, a new rule can be added -- or an existing one loosened --
        with no test that it can still reject anything. Pass for one rule does
        not prove another works, so each is checked on its own.

        Keyed on the evaluator function rather than the scenario id: SR_02,
        SR_03, SR_04 and SR_06 all run `eval_step_count`, and one waveform that
        defeats it demonstrates the rule for all of them.
        """
        for name, evaluator in sorted(rt.EVALUATORS.items()):
            negatives = [fx for fx in vf.FIXTURES
                         if rt.EVALUATORS[fx.scenario] is evaluator
                         and not fx.expect_pass]
            with self.subTest(rule=evaluator.__name__, first_used_by=name):
                self.assertTrue(
                    negatives,
                    f"{evaluator.__name__} can reject nothing: add a bad "
                    f"fixture for a scenario that uses it")

    def test_every_fixture_is_reachable_from_a_scenario(self):
        """A fixture no scenario uses is a fixture no run ever checks."""
        for fx in vf.FIXTURES:
            with self.subTest(fixture=fx.name):
                self.assertIn(fx.scenario, rt.EVALUATORS,
                              f"{fx.name} targets {fx.scenario}, which has no "
                              f"evaluator in run_tests.EVALUATORS")
                self.assertIn(fx.scenario, rt.SCENARIOS,
                              f"{fx.name} targets {fx.scenario}, which is not a "
                              f"runnable scenario in run_tests.SCENARIOS")

    def test_every_wired_scenario_has_coverage(self):
        """A scenario with no fixture is a scenario nothing has ever checked.

        The rule-level guard above keys on the *evaluator function*, because
        SR_02, SR_03, SR_04 and SR_06 all run `eval_step_count` and one bad
        waveform demonstrates it for all of them. That is the right granularity
        for proving a rule can fail, and the wrong one for proving a scenario
        is tested: it happily allows a newly wired scenario to have no fixture
        at all.

        SR_03 is the case that motivated this. It had no fixture, and its
        builder turned out to send the same ticks as SR_01, so the "speed floor"
        test was not testing the floor.

        Coverage counts either a committed fixture or one of the generated
        long-run scenarios, so the scale tests above are honoured here.
        """
        generated = {"SR_07", "SR_08"}  # see TestLongRuns
        for scenario in sorted(rt.EVALUATORS):
            has = [fx.name for fx in vf.FIXTURES if fx.scenario == scenario]
            with self.subTest(scenario=scenario):
                self.assertTrue(
                    has or scenario in generated,
                    f"{scenario} is wired in run_tests.EVALUATORS but no fixture "
                    f"and no generated test covers it")

    def test_a_scenario_is_not_a_duplicate_of_another(self):
        """Two scenarios sending identical segment lists test one thing.

        Catches a scenario whose builder was copied and left pointing at the
        wrong constant, which makes the suite look broader than it is: SR_03
        and SR_01 both sent `(8, 640, True)`, so the pair bought one test, not
        two.

        Compared within a config only. SR_14 is `2ch` and legitimately sends
        the same segment list as SR_07's `1ch` -- same steps, but a different
        question (when the second stepper starts, not whether 2000 steps
        arrive), reached through a different evaluator.
        """
        seen = {}
        for scenario, (cfg, builder, _mask, _name) in rt.SCENARIOS.items():
            segs = tuple(builder(vf.Dut().info()))
            if not segs:
                continue
            for (other, other_segs) in seen.get(cfg, []):
                with self.subTest(scenario=scenario, same_as=other, config=cfg):
                    self.assertNotEqual(
                        segs, other_segs,
                        f"{scenario} and {other} are both {cfg} and send the "
                        f"identical segment list {segs}, so they exercise the "
                        f"same waveform")
            seen.setdefault(cfg, []).append((scenario, segs))

    def test_fixtures_match_their_scenario(self):
        """A fixture must depict the segment list its scenario actually sends.

        This is what stops the golden waveforms from drifting away from the
        code they are meant to describe.
        """
        for fx in vf.FIXTURES:
            with self.subTest(fixture=fx.name):
                cfg, builder, _mask, _name = rt.SCENARIOS[fx.scenario]
                self.assertEqual(
                    list(fx.segments), list(builder(fx.info())),
                    f"{fx.name} no longer matches {fx.scenario}")


class TestRateAdherence(unittest.TestCase):
    """Rate adherence is a separate measurement from "are the periods right".

    A driver whose ISR sets the step pin emits every step slightly late: no step
    is lost, no single period is grossly wrong, and the step count is perfect --
    while the achieved rate is quietly below the commanded one. These check the
    metric sees that, and that it is derived from the DUT's own tick rate.
    """

    def test_sag_is_detected_though_count_is_right(self):
        fx = vf.by_name("bad_rate_sag")
        ok, detail = evaluate(fx)
        self.assertFalse(ok)
        counts = detail["steps"]
        # The whole point: the count is fine, only the rate is wrong.
        self.assertEqual(counts["extra_steps"], 0)
        self.assertEqual(counts["missing_steps"], 0)
        self.assertGreater(detail["adherence"]["n_out_of_tolerance"], 0)
        self.assertGreater(detail["adherence"]["sag_pct"], 0)

    def test_clean_run_has_no_sag(self):
        fx = vf.by_name("good_rate_adherence")
        ok, detail = evaluate(fx)
        self.assertTrue(ok, json.dumps(detail, sort_keys=True))
        self.assertEqual(detail["adherence"]["n_out_of_tolerance"], 0)
        self.assertEqual(detail["adherence"]["sag_pct"], 0.0)

    def test_commanded_rate_comes_from_the_dut_not_a_constant(self):
        """`sag_pct` is relative, so a hardcoded 16 MHz would still give a
        plausible-looking number here. The absolute rate is the check."""
        periods = [40.0] * 8
        for tps in (16_000_000, 20_000_000, 8_000_000):
            with self.subTest(ticks_per_s=tps):
                r = sp.rate_adherence(periods, 1e6 * 640 / tps)
                self.assertAlmostEqual(r["commanded_rate_hz"],
                                       tps / 640, delta=0.01)


if __name__ == "__main__":
    unittest.main(verbosity=2)
