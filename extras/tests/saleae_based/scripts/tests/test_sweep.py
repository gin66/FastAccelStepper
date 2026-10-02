"""Tests for the sweep planner.

The sweep's value is that every point is legal before any hardware runs. That
is worth asserting directly: a sweep that included a refused command would
spend its runtime measuring ErrorTicksTooLow and call the result a step-timing
defect. The rest of the tests check that the boundaries a hand-picked scenario
would skip are actually present in the plan.
"""
import shutil
import subprocess
import sys
import tempfile
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
sys.path.insert(0, str(Path(__file__).resolve().parent))

import sweep                  # noqa: E402
import run_tests as rt        # noqa: E402
import vcd_fixtures as vf     # noqa: E402

SCRIPTS = Path(__file__).resolve().parents[1]


class TestLegality(unittest.TestCase):
    def setUp(self):
        self.info = vf.Dut().info()

    def test_every_planned_point_is_legal(self):
        for scenario in ("SR_02", "SR_05"):
            for label, segments, problem in sweep.plan(scenario, self.info):
                self.assertIsNone(
                    problem,
                    f"{scenario} {label} would be refused: {problem}")

    def test_points_below_the_floor_are_caught(self):
        # The exact case SR_13 exists for: one step whose ticks alone sit under
        # the floor. If this were ever planned, every run would measure a
        # refusal instead of a step period.
        problem = sweep.legality("SR_02", [(1, 640, True)], self.info)
        self.assertIsNotNone(problem)
        self.assertIn("below the floor", problem)

    def test_ticks_beyond_16_bits_are_caught(self):
        problem = sweep.legality("SR_02", [(1, 65536, True)], self.info)
        self.assertIsNotNone(problem)
        self.assertIn("16-bit", problem)

    def test_ticks_times_steps_counts_for_more_than_one_step(self):
        # 2 steps at 1600 ticks is 3200, exactly at the floor, and legal.
        self.assertIsNone(
            sweep.legality("SR_02", [(2, 1600, True)], self.info))
        # The same ticks with one step is not, which is why steps=1 needs 3200.
        self.assertIsNone(
            sweep.legality("SR_02", [(1, 3200, True)], self.info))

    def test_a_pause_is_not_mistaken_for_a_move(self):
        # A pause carries ticks directly rather than ticks*steps, so the floor
        # check must not multiply it by zero into a "legal" verdict for the
        # wrong reason.
        self.assertIsNone(
            sweep.legality("SR_26", [(0, 65535, True)], self.info))


class TestCoverage(unittest.TestCase):
    """A sweep that misses the boundaries is just a slower single run."""

    def setUp(self):
        self.info = vf.Dut().info()

    def test_sr02_covers_the_boundary_steps(self):
        steps = sweep.SR_02_STEPS
        self.assertEqual(steps[0], 1, "steps=1 takes a different ISR branch")
        self.assertEqual(steps[1], 2, "steps=2 crosses ticks*steps")
        self.assertEqual(steps[-1], 255, "255 is uint8_t's maximum")
        self.assertEqual(len(steps), len(set(steps)), "duplicate sweep point")

    def test_sr02_covers_the_isisr_branch_and_the_queue_refill(self):
        steps = set(sweep.SR_02_STEPS)
        for boundary in (1, 2, 255):
            self.assertIn(boundary, steps)

    def test_sr05_spans_the_legal_ticks_range(self):
        ticks = sweep.SR_05_TICKS
        self.assertEqual(min(ticks), self.info["min_cmd_ticks"])
        self.assertEqual(max(ticks), 65535, "the 16-bit boundary")
        self.assertEqual(len(ticks), len(set(ticks)))

    def test_sr05_uses_one_step_so_ticks_and_rate_coincide(self):
        for _label, segments, _p in sweep.plan("SR_05", self.info):
            self.assertEqual(segments[0][0], 1,
                             "more than one step confounds ticks with rate")

    def test_unknown_scenario_yields_no_points(self):
        self.assertEqual(sweep.plan("SR_99", self.info), [])


class TestOutput(unittest.TestCase):
    def test_list_mode_names_every_point(self):
        result = subprocess.run(
            [sys.executable, str(SCRIPTS / "sweep.py"), "SR_02", "--list"],
            capture_output=True, text=True)
        self.assertEqual(result.returncode, 0, result.stderr)
        for label, _segs, _p in sweep.plan("SR_02", vf.Dut().info()):
            self.assertIn(label, result.stdout)

    def test_list_mode_reports_no_illegal_points(self):
        result = subprocess.run(
            [sys.executable, str(SCRIPTS / "sweep.py"), "SR_05", "--list"],
            capture_output=True, text=True)
        self.assertIn("0 illegal", result.stdout)

    def test_missing_captures_are_reported_not_crashed(self):
        empty = Path(tempfile.mkdtemp())
        try:
            result = subprocess.run(
                [sys.executable, str(SCRIPTS / "sweep.py"), "SR_05",
                 "--run-dir", str(empty)],
                capture_output=True, text=True)
            self.assertIn("MISSING", result.stdout)
            self.assertEqual(result.returncode, 1)
        finally:
            shutil.rmtree(empty, ignore_errors=True)

    def test_run_dir_is_required_for_tabulation(self):
        result = subprocess.run(
            [sys.executable, str(SCRIPTS / "sweep.py"), "SR_02"],
            capture_output=True, text=True)
        self.assertNotEqual(result.returncode, 0)
        self.assertIn("--run-dir", result.stderr)


if __name__ == "__main__":
    unittest.main()