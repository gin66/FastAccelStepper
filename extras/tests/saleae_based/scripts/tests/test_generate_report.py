"""Tests for the report generator and the statistics behind it.

Two separate claims are checked here. That the numbers in a report are the
numbers the run measured -- so a generator that recomputed or invented a figure
would fail. And that the statistics describe a distribution rather than
collapsing it, because that distinction is the reason the report carries min,
max and spread at all.

Results are synthesised here rather than captured: a 24 MS/s hardware VCD is
~96M samples and would make this suite take minutes.
"""
import csv
import io
import json
import shutil
import subprocess
import sys
import tempfile
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
sys.path.insert(0, str(Path(__file__).resolve().parent))

import generate_report as gr      # noqa: E402
import signal_parser as sp        # noqa: E402

SCRIPTS = Path(__file__).resolve().parents[1]


def result(test_id="SR_01", tag="esp32_auto_1ch", passed=True,
           mean=40.0, lo=39.9583, hi=40.0, high_mean=15.625,
           arch="esp32", steps=8):
    return {
        "timestamp": "2026-10-02T09:46:15Z",
        "test_id": test_id,
        "goal": "inter-step period equals ticks",
        "arch": arch,
        "driver": "auto",
        "channel_config": "1ch",
        "tag": tag,
        "dut": {"ticks_per_s": 16_000_000, "min_cmd_ticks": 3200,
                "queue_len": 32, "max_speed_ticks": 640},
        "segments": [[8, 640, True]],
        "per_stepper": None,
        "sample_rate_hz": 24_000_000,
        "capture": "/tmp/x/SR_01.vcd",
        "pass": passed,
        "evaluator_detail": {"period": {"expected_period_us": 40.0, "ok": True}},
        "measurements": {
            "A": {
                "edge_count": steps * 2,
                "step_count": steps,
                "pulse_high_us": {"n": steps, "min": 15.5417, "max": high_mean,
                                  "mean": high_mean, "median": high_mean,
                                  "spread": 0.0833, "stdev": 0.02},
                "pulse_low_us": {"n": steps, "min": 24.33, "max": 24.46,
                                "mean": 24.35, "median": 24.35,
                                "spread": 0.13, "stdev": 0.04},
                "inter_step_us": {"n": steps - 1, "min": lo, "max": hi,
                                  "mean": mean, "median": mean,
                                  "spread": round(hi - lo, 4), "stdev": 0.03},
                "duty_cycle_percent": 39.06,
                "max_pulse_width_us": 15.625,
            }
        },
    }


def write_results(directory, results):
    directory.mkdir(parents=True, exist_ok=True)
    for r in results:
        (directory / f"{r['test_id']}.json").write_text(
            json.dumps(r, indent=2, sort_keys=True))
    return directory


def run_generator(results_dir, out_dir, *extra):
    return subprocess.run(
        [sys.executable, str(SCRIPTS / "generate_report.py"),
         "--results", str(results_dir), "--out", str(out_dir),
         "--quiet", *extra],
        capture_output=True, text=True)


class TestStatistics(unittest.TestCase):
    def test_empty_distribution_reports_nothing_rather_than_zero(self):
        d = sp.describe([])
        self.assertEqual(d["n"], 0)
        # Zero would read as "measured zero"; None reads as "not present".
        self.assertIsNone(d["min"])
        self.assertIsNone(d["mean"])

    def test_constant_distribution_has_no_spread(self):
        d = sp.describe([15.625] * 8)
        self.assertEqual(d["min"], d["max"])
        self.assertEqual(d["spread"], 0.0)
        self.assertEqual(d["stdev"], 0.0)

    def test_spread_captures_what_the_mean_hides(self):
        # Nine ordinary periods and one short one: the mean barely moves, the
        # minimum is the whole finding. This is why both are reported.
        values = [40.0] * 9 + [39.0]
        d = sp.describe(values)
        self.assertAlmostEqual(d["min"], 39.0)
        self.assertAlmostEqual(d["max"], 40.0)
        self.assertAlmostEqual(d["spread"], 1.0)
        self.assertAlmostEqual(d["mean"], 39.9, places=6)

    def test_median_resists_a_tail_the_mean_does_not(self):
        d = sp.describe([40.0] * 9 + [100.0])
        self.assertAlmostEqual(d["median"], 40.0)
        self.assertGreater(d["mean"], d["median"])

    def test_even_length_median_is_the_middle_pair(self):
        self.assertAlmostEqual(sp.describe([1.0, 3.0])["median"], 2.0)

    def test_single_value_has_no_spread(self):
        d = sp.describe([7.5])
        self.assertEqual((d["min"], d["max"], d["spread"]), (7.5, 7.5, 0.0))

    def test_stepper_metrics_expose_distributions_not_just_averages(self):
        # A 1-high, N-low square-ish wave: two edges per period.
        samples = [0]
        for _ in range(3):
            samples.extend([1] * 4 + [0] * 20)
        m = sp.stepper_metrics(samples, 1_000_000)
        self.assertEqual(m["step_count"], 3)
        self.assertIn("min", m["pulse_high_us"])
        self.assertIn("spread", m["inter_step_us"])
        self.assertGreater(m["duty_cycle_percent"], 0)


class TestReportContent(unittest.TestCase):
    def setUp(self):
        self.tmp = Path(tempfile.mkdtemp())
        self.results = write_results(self.tmp / "results",
                                     [result("SR_01"), result("SR_25")])
        self.out = self.tmp / "reports"
        self.run = run_generator(self.results, self.out)
        self.index = (self.out / "index.md").read_text()

    def tearDown(self):
        shutil.rmtree(self.tmp, ignore_errors=True)

    def test_every_expected_artefact_is_written(self):
        for name in ("index.md", "spec_compliance.md", "regression.md",
                     "all_results.csv"):
            self.assertTrue((self.out / name).exists(), f"{name} not written")
        self.assertTrue((self.out / "test_SR_01.md").exists())

    def test_index_reports_counts_and_pass_rate(self):
        self.assertIn("**Tests:** 2 run, 2 passed", self.index)
        self.assertIn("**Pass rate:** 100.0%", self.index)

    def test_results_table_carries_the_measurement_range(self):
        self.assertIn("period us (min–max)", self.index)
        self.assertIn("39.9583–40", self.index)

    def test_a_failing_test_is_visible_in_the_index_and_the_exit_code(self):
        write_results(self.results, [result("SR_01"),
                                     result("SR_02", passed=False)])
        run = run_generator(self.results, self.out)
        self.assertEqual(run.returncode, 1)
        self.assertIn("FAIL", (self.out / "index.md").read_text())

    def test_report_explains_that_pulse_width_is_recorded_not_asserted(self):
        # The distinction matters: the driver sets the width, so judging it
        # against an invented threshold would be meaningless.
        self.assertIn("recorded rather than judged", self.index)

    def test_per_test_page_names_the_program_and_measurements(self):
        page = (self.out / "test_SR_01.md").read_text()
        self.assertIn("QSEG 8 640 1", page)
        self.assertIn("## Measured", page)
        self.assertIn("MIN_CMD_TICKS", page)

    def test_tag_summary_is_written_per_tag(self):
        tags = list((self.out / "tag_summary").glob("*.md"))
        self.assertTrue(tags)
        self.assertIn("esp32_auto_1ch", tags[0].name)

    def test_csv_has_the_documented_schema(self):
        rows = list(csv.DictReader(io.StringIO(
            (self.out / "all_results.csv").read_text())))
        self.assertEqual(list(rows[0].keys()), gr.CSV_COLUMNS)
        self.assertEqual(rows[0]["test_id"], "SR_01")
        self.assertEqual(rows[0]["stepper"], "A")
        self.assertEqual(rows[0]["step_count"], "8")

    def test_csv_carries_statistics_not_a_single_period(self):
        rows = list(csv.DictReader(io.StringIO(
            (self.out / "all_results.csv").read_text())))
        row = next(r for r in rows if r["test_id"] == "SR_01")
        self.assertEqual(row["period_min_us"], "39.9583")
        self.assertEqual(row["period_max_us"], "40.0")
        self.assertEqual(row["high_min_us"], "15.5417")

    def test_csv_gives_one_row_per_stepper(self):
        two = result("SR_15")
        two["measurements"]["B"] = dict(two["measurements"]["A"],
                                         step_count=200)
        write_results(self.results, [two])
        run_generator(self.results, self.out)
        rows = [r for r in csv.DictReader(io.StringIO(
            (self.out / "all_results.csv").read_text()))
            if r["test_id"] == "SR_15"]
        self.assertEqual(sorted(r["stepper"] for r in rows), ["A", "B"])

    def test_pause_note_reports_the_widest_gap_not_the_first(self):
        # The measured list also contains ordinary inter-step periods, so
        # taking the first would report a 40 us gap for an 800 us pause.
        r = result("SR_09")
        r["evaluator_detail"] = {"pause_us": 800.0, "expected_gap_us": 840.0,
                                  "measured_gaps_us": [39.95, 39.96, 839.29]}
        write_results(self.results, [r])
        note = gr.measured_note(r)
        self.assertIn("839.29", note)
        self.assertNotIn("39.95", note)

    def test_single_step_result_leaves_the_period_blank_not_zero(self):
        r = result("SR_27")
        r["measurements"]["A"]["inter_step_us"] = sp.describe([])
        write_results(self.results, [r])
        run_generator(self.results, self.out)
        rows = [r for r in csv.DictReader(io.StringIO(
            (self.out / "all_results.csv").read_text()))
            if r["test_id"] == "SR_27"]
        # A single-step command has no inter-step period. Blank, not zero:
        # zero would read as "measured zero" and be indistinguishable from a
        # real measurement.
        self.assertEqual(rows[0]["period_us"], "")
        self.assertIn("—", (self.out / "index.md").read_text())

    def test_unreadable_result_does_not_take_the_report_down(self):
        (self.results / "SR_99.json").write_text("{not json")
        run = run_generator(self.results, self.out)
        self.assertEqual(run.returncode, 0)
        self.assertIn("skipping unreadable", run.stderr)


class TestRegression(unittest.TestCase):
    def setUp(self):
        self.tmp = Path(tempfile.mkdtemp())
        self.base = write_results(self.tmp / "base", [result("SR_01")])
        self.out = self.tmp / "reports"

    def tearDown(self):
        shutil.rmtree(self.tmp, ignore_errors=True)

    def test_identical_baseline_reports_no_change(self):
        cur = write_results(self.tmp / "cur", [result("SR_01")])
        run_generator(cur, self.out, "--baseline", str(self.base))
        page = (self.out / "regression.md").read_text()
        self.assertIn("Every test present in both runs kept its verdict", page)

    def test_a_changed_period_is_flagged_beyond_resolution(self):
        cur = write_results(self.tmp / "cur",
                            [result("SR_01", mean=40.5, hi=40.5)])
        run_generator(cur, self.out, "--baseline", str(self.base))
        self.assertIn("⚠", (self.out / "regression.md").read_text())

    def test_a_difference_below_resolution_is_not_flagged(self):
        # 0.001 us is well under one sample at 24 MS/s, so calling it a change
        # would report noise as a regression.
        cur = write_results(self.tmp / "cur",
                            [result("SR_01", mean=40.001, hi=40.0)])
        run_generator(cur, self.out, "--baseline", str(self.base))
        self.assertNotIn("⚠", (self.out / "regression.md").read_text())

    def test_a_flipped_verdict_is_called_out(self):
        cur = write_results(self.tmp / "cur", [result("SR_01", passed=False)])
        run_generator(cur, self.out, "--baseline", str(self.base))
        page = (self.out / "regression.md").read_text()
        self.assertIn("pass → fail", page)

    def test_a_new_test_is_marked_rather_than_compared(self):
        cur = write_results(self.tmp / "cur",
                            [result("SR_01"), result("SR_02")])
        run_generator(cur, self.out, "--baseline", str(self.base))
        self.assertIn("_new_", (self.out / "regression.md").read_text())


class TestCrossConfiguration(unittest.TestCase):
    def test_no_shared_tests_says_so_instead_of_printing_an_empty_table(self):
        tmp = Path(tempfile.mkdtemp())
        try:
            results = write_results(tmp / "results", [
                result("SR_01", tag="esp32_auto_1ch"),
                result("SR_02", tag="esp32_mcpwm_pcnt_mcpwm"),
            ])
            run_generator(results, tmp / "reports")
            page = (tmp / "reports" / "index.md").read_text()
            self.assertIn("nothing to compare side by side", page)
        finally:
            shutil.rmtree(tmp, ignore_errors=True)

    def test_same_test_under_two_tags_is_tabulated(self):
        tmp = Path(tempfile.mkdtemp())
        try:
            results = write_results(tmp / "results", [
                result("SR_01", tag="esp32_auto_1ch", mean=40.0),
                result("SR_01", tag="esp32_mcpwm_pcnt_mcpwm", mean=41.0),
            ])
            run_generator(results, tmp / "reports")
            page = (tmp / "reports" / "index.md").read_text()
            self.assertIn("esp32_mcpwm_pcnt_mcpwm", page)
            self.assertIn("41", page)
        finally:
            shutil.rmtree(tmp, ignore_errors=True)

    def test_newest_result_per_test_wins(self):
        old = result("SR_01", passed=False)
        old["timestamp"] = "2026-01-01T00:00:00Z"
        new = result("SR_01", passed=True)
        new["timestamp"] = "2026-10-02T00:00:00Z"
        latest = gr.latest_per_test([old, new])
        self.assertEqual(len(latest), 1)
        self.assertTrue(latest[0]["pass"])


if __name__ == "__main__":
    unittest.main()