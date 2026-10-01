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
    return rt.evaluate(fx.scenario, channels, rate, fx.segments, fx.info())


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
