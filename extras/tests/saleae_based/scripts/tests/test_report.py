"""Tests for the reporting layer.

The report is the part of the harness a person actually reads, and it is also
the part most able to quietly disagree with the tests: given the same capture it
could measure things its own way and print a number the evaluators never agreed
to. So these tests check two separate things -- that it renders, and that what it
prints comes from run_tests.evaluate rather than from a second implementation.

The runs here are built from fixtures rather than from hardware captures on
purpose. A hardware VCD is ~53M samples of pure-Python waveform and would make
this suite take minutes; the fixtures are the same shape in miniature.
"""
import csv
import io
import shutil
import subprocess
import sys
import tempfile
import unittest
from contextlib import redirect_stdout
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
sys.path.insert(0, str(Path(__file__).resolve().parent))

import report                     # noqa: E402
import run_tests as rt            # noqa: E402
import vcd_fixtures as vf         # noqa: E402

SCRIPTS = Path(__file__).resolve().parents[1]
FIXTURE_DIR = Path(__file__).resolve().parent / "fixtures"


def build_run_dir(scenario, fixture_name=None):
    """A directory holding <scenario>.vcd, taken from a fixture.

    Uses the first fixture for that scenario that is meant to pass, so the
    report has something valid to say.
    """
    tmp = Path(tempfile.mkdtemp())
    name = fixture_name
    if name is None:
        for fx in vf.FIXTURES:
            if fx.scenario == scenario and fx.expect_pass:
                name = fx.name
                break
    if name is None:
        shutil.rmtree(tmp)
        raise unittest.SkipTest(f"no passing fixture for {scenario}")
    shutil.copy(FIXTURE_DIR / f"{name}.vcd", tmp / f"{scenario}.vcd")
    return tmp


def run_report(*args):
    """Invoke report.py as a subprocess, the way a person would."""
    return subprocess.run(
        [sys.executable, str(SCRIPTS / "report.py"), *args],
        capture_output=True, text=True)


class TestCatalogue(unittest.TestCase):
    def test_catalogue_lists_every_wired_scenario(self):
        out = run_report("--scenarios").stdout
        for scenario in rt.SCENARIOS:
            self.assertIn(scenario, out,
                          f"{scenario} is wired but absent from the catalogue")

    def test_catalogue_names_the_config_and_mask(self):
        out = run_report("--scenarios").stdout
        # A row that lost its config would make the catalogue useless for
        # reconstructing a run, since the config decides which driver is used.
        for scenario, (cfg, _, mask, desc) in rt.SCENARIOS.items():
            row = [ln for ln in out.splitlines() if ln.startswith(f"| {scenario} ")]
            self.assertEqual(len(row), 1, f"{scenario} missing from catalogue")
            self.assertIn(cfg, row[0])
            self.assertIn(str(mask), row[0])
            self.assertIn(desc, row[0])

    def test_stop_scenarios_are_marked_as_needing_the_host(self):
        out = run_report("--scenarios").stdout
        for scenario in rt.STOP_AFTER:
            self.assertIn("STOP", out)
            self.assertIn(scenario, out)


class TestRendering(unittest.TestCase):
    def setUp(self):
        self.tmp = build_run_dir("SR_01")

    def tearDown(self):
        shutil.rmtree(self.tmp, ignore_errors=True)

    def test_markdown_names_the_scenario_and_a_verdict(self):
        result = run_report(str(self.tmp))
        self.assertIn("SR_01", result.stdout)
        self.assertIn("PASS", result.stdout)
        self.assertIn("scenario", result.stdout)

    def test_csv_has_a_stable_header(self):
        result = run_report(str(self.tmp), "--csv")
        rows = list(csv.DictReader(io.StringIO(result.stdout)))
        self.assertEqual(
            list(rows[0].keys()),
            ["scenario", "verdict", "steps_hw", "steps_exp", "capture_s", "note"])

    def test_passing_run_exits_zero(self):
        self.assertEqual(run_report(str(self.tmp)).returncode, 0)

    def test_empty_directory_is_reported_not_crashed(self):
        empty = Path(tempfile.mkdtemp())
        try:
            result = run_report(str(empty))
            self.assertIn("No captures", result.stdout)
        finally:
            shutil.rmtree(empty, ignore_errors=True)

    def test_capture_is_loaded_once_per_scenario(self):
        """A second load of a 53M-sample VCD costs seconds; the cache matters."""
        report._CACHE.clear()
        calls = []
        real = report.sp.load_vcd

        def counting(path):
            calls.append(Path(path).name)
            return real(path)

        report.sp.load_vcd = counting
        try:
            report.collect(self.tmp, vf.Dut().info())
        finally:
            report.sp.load_vcd = real
        self.assertEqual(len(calls), len(set(calls)),
                         f"a capture was parsed more than once: {calls}")


class TestUsesTheEvaluators(unittest.TestCase):
    """The report must not measure anything itself."""

    def setUp(self):
        self.tmp = build_run_dir("SR_01")

    def tearDown(self):
        shutil.rmtree(self.tmp, ignore_errors=True)

    def test_report_delegates_to_run_tests_evaluate(self):
        seen = []
        real = rt.evaluate

        def spy(scenario, channels, rate, segments, info):
            seen.append(scenario)
            return real(scenario, channels, rate, segments, info)

        rt.evaluate = spy
        report.rt.evaluate = spy
        try:
            buf = io.StringIO()
            with redirect_stdout(buf):
                report.collect(self.tmp, vf.Dut().info())
        finally:
            rt.evaluate = real
            report.rt.evaluate = real
        self.assertIn("SR_01", seen)

    def test_steps_column_comes_from_the_evaluator_detail(self):
        rows = report.collect(self.tmp, vf.Dut().info())
        channels, rate = report.load_once(self.tmp / "SR_01.vcd")
        info = vf.Dut().info()
        ok, detail = rt.evaluate(
            "SR_01", channels, rate, rt.SCENARIOS["SR_01"][1](info), info)
        # Reading the evaluator's own count back out is the point: if the report
        # invented a number instead, this would not match it.
        self.assertTrue(ok)
        self.assertEqual(rows[0]["steps_hw"], report.steps_of(detail))
        self.assertEqual(rows[0]["steps_exp"], report.expected_of(detail))

    def test_reported_skew_is_labelled_as_measured_not_gated(self):
        """A skew nobody asserts must not be dressed up as a pass or a fail."""
        self.assertEqual(report.verdict_of(True, {"first_step_skew_us": 29.583}),
                         "PASS(reported)")
        self.assertEqual(report.verdict_of(False, {"first_step_skew_us": 29.583}),
                         "FAIL")
        self.assertEqual(report.verdict_of(True, {}), "PASS")


if __name__ == "__main__":
    unittest.main()