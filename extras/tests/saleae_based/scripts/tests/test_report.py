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
import json
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
import run_matrix                 # noqa: E402
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

        def spy(scenario, channels, rate, segments, info, chan_map=None,
                extra=None):
            seen.append(scenario)
            return real(scenario, channels, rate, segments, info, chan_map,
                        extra)

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
            "SR_01", channels, rate, rt.SCENARIOS["SR_01"][1](info), info,
            rt.Pins.for_scenario("SR_01").map)
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


def mode_record(**over):
    """A mode result as run_modes() writes one, minimal but shaped like real."""
    record = {
        "test_id": "MODE", "tag_key": "t", "mode": "sync", "result": "passed",
        "arch": "esp32", "framework": "arduino", "sdk_version": "latest",
        "drivers": ["rmt", "rmt"], "pin_mode": "dir",
        "stepper_count": 2, "first_step_skew_us": 37.5, "skew_periods": 3.75,
        "per_stepper": {
            "A": {"channel": "D0", "ticks": 160, "mean_period_us": 10.0,
                  "steps": {"steps_expected": 64, "steps_measured": 64,
                            "missing_steps": 0, "extra_steps": 0, "ok": True},
                  "period": {"expected_period_us": 10.0, "n_long": 0,
                             "n_short": 0, "ok": True}},
            "B": {"channel": "D2", "ticks": 320, "mean_period_us": 20.0,
                  "steps": {"steps_expected": 64, "steps_measured": 64,
                            "missing_steps": 0, "extra_steps": 0, "ok": True},
                  "period": {"expected_period_us": 20.0, "n_long": 0,
                             "n_short": 0, "ok": True}},
        },
    }
    record.update(over)
    return record


class TestModeTables(unittest.TestCase):
    """R5: the parallel-count and sync-permutation tables.

    Both are read out of mode result JSON, because a mode run records a result
    and no capture -- there is no VCD for the report to evaluate, which is
    exactly why these tables exist rather than being another evaluator.
    """

    def setUp(self):
        self.tmp = Path(tempfile.mkdtemp())
        self.addCleanup(shutil.rmtree, self.tmp)

    def write(self, records):
        for i, record in enumerate(records):
            (self.tmp / f"run{i}.json").write_text(json.dumps(record))
        return self.tmp

    def test_both_tables_appear_for_a_run_that_has_both_modes(self):
        scale = mode_record(mode="scale", drivers=["rmt"],
                            stepper_count=1, first_step_skew_us=None,
                            skew_periods=None, period_spread_us=0.0,
                            per_stepper={"A": {
                                "mean_period_us": 10.0,
                                "steps": {"steps_expected": 64,
                                          "steps_measured": 64, "ok": True},
                                "period": {"ok": True}}})
        text = report.as_mode_tables(self.write([scale, mode_record()]))
        self.assertIn("Parallel stepper count (`scale`)", text)
        self.assertIn("Synced start (`sync`)", text)
        self.assertIn("rmt", text)
        self.assertIn("37.5 us", text)

    def test_a_driver_list_with_no_measured_skew_says_so(self):
        """The todo's own requirement: no empty table, and no empty cell.

        A blank skew column is indistinguishable from a report bug, so the row
        states what happened instead. For a refused driver list the refusal *is*
        the measurement.
        """
        refused = mode_record(result="refused", drivers=["i2s_mux", "i2s_mux"],
                              per_stepper={}, first_step_skew_us=None,
                              skew_periods=None,
                              error="ERR connect step 0 n=0 drv=i2s_mux")
        text = report.as_mode_tables(self.write([refused]))
        row = [ln for ln in text.splitlines() if ln.startswith("| i2s_mux")][0]
        self.assertIn("not measured", row)
        self.assertIn("ERR connect step 0", row)
        self.assertNotIn("|  |", row)
        # The adherence column must stay silent rather than claim the steppers
        # were fine. A cell reading "ok" beside a refused row is a report that
        # reports a driver as healthy because it was never asked.
        adherence = row.split("|")[-3]
        self.assertEqual(adherence.strip(), "-", adherence)
        self.assertNotIn("ok", adherence)

    def test_a_passed_run_whose_record_lacks_skew_does_not_claim_zero(self):
        silent = mode_record(first_step_skew_us=None, skew_periods=None,
                             per_stepper={})
        text = report.as_mode_tables(self.write([silent]))
        row = [ln for ln in text.splitlines() if ln.startswith("| rmt")][0]
        self.assertIn("no first-step skew", row)
        self.assertNotIn("None", row)

    def test_the_skew_table_shows_step_periods_as_well_as_microseconds(self):
        """Microseconds alone do not say whether a skew is small.

        37.5 us is three quarters of one step period or three quarters of a
        hundred, and only one of those is a defect a reader can act on.
        """
        text = report.as_mode_tables(self.write([mode_record()]))
        self.assertIn("37.5 us", text)
        self.assertIn("3.75", text)

    def test_the_runaway_is_visible_even_though_its_skew_is_the_smallest(self):
        """The measured case, and the reason adherence is its own column.

        `mcpwm_pcnt+mcpwm_pcnt` posts the smallest first-step skew of any
        combination in the sync run -- 6.25 us -- while being the driver that
        never stops. Its *period* is flawless: 10 883 steps at exactly the
        commanded 19.9991 us. Reported as a skew alone the defect would be the
        best row in the table, so the count has to sit beside the period.
        """
        runaway = mode_record(
            drivers=["mcpwm_pcnt", "mcpwm_pcnt"],
            first_step_skew_us=6.25, skew_periods=0.625,
            per_stepper={
                "A": mode_record()["per_stepper"]["A"],
                "B": {"channel": "D2", "ticks": 320,
                      "mean_period_us": 19.9991,
                      "steps": {"steps_expected": 64,
                                "steps_measured": 10883, "missing_steps": 0,
                                "extra_steps": 10819, "ok": False},
                      "period": {"expected_period_us": 20.0, "n_long": 0,
                                 "n_short": 0, "ok": True}},
            })
        text = report.as_mode_tables(self.write([runaway]))
        row = [ln for ln in text.splitlines()
               if ln.startswith("| mcpwm_pcnt+mcpwm_pcnt")][0]
        self.assertIn("6.25 us", row)
        self.assertIn("x10883/64", row)
        self.assertIn("+10819", row)

    def test_the_target_comes_from_the_record_not_the_tag_key(self):
        """Grouping by architecture must not mean parsing one.

        The key encodes the target, but as a convention: `esp32_arduino_...`
        splits on two underscores and `esp32_idf5_3_0_...` on one. Two records
        whose keys disagree with their own fields must group by the fields.
        """
        a = mode_record(tag_key="esp32_arduino_rmt_syncdir_n2")
        b = mode_record(tag_key="esp32_idf5_3_0_rmt_syncdir_n2",
                        framework="idf", sdk_version="5.3.0")
        c = mode_record(tag_key="nanoatmega328_arduino_timer_syncdir_n2",
                        arch="nanoatmega328", framework="arduino")
        text = report.as_mode_tables(self.write([a, b, c]))
        self.assertIn("esp32 / arduino", text)
        self.assertIn("esp32 / idf / sdk 5.3.0", text)
        self.assertIn("nanoatmega328 / arduino", text)
        self.assertIn("Target: ", text)

    def test_a_missing_target_is_shown_not_guessed(self):
        old = mode_record()
        for key in ("arch", "framework", "sdk_version"):
            old.pop(key)
        text = report.as_mode_tables(self.write([old]))
        self.assertIn("Target: ? / ?", text)

    def test_each_stepper_keeps_its_own_period_in_the_scale_table(self):
        """Not just the count: a shared-program run must still show per-stepper."""
        scale = mode_record(mode="scale", drivers=["rmt"] * 2,
                            stepper_count=2, first_step_skew_us=None,
                            skew_periods=None, period_spread_us=0.004,
                            per_stepper=mode_record()["per_stepper"])
        text = report.as_mode_tables(self.write([scale]))
        row = [ln for ln in text.splitlines()
               if ln.startswith("| rmt+rmt ")][0]
        self.assertIn("A 10.0usx64/64", row)
        self.assertIn("B 20.0usx64/64", row)

    def test_a_scale_point_that_deviated_is_marked_in_the_table(self):
        scale = mode_record(mode="scale", drivers=["rmt"] * 3,
                            stepper_count=3, first_step_skew_us=None,
                            skew_periods=None,
                            per_stepper={
                                "A": mode_record()["per_stepper"]["A"],
                                "B": mode_record()["per_stepper"]["B"],
                                "C": {"mean_period_us": 10.0,
                                      "steps": {"steps_expected": 64,
                                                "steps_measured": 64,
                                                "extra_steps": 12000,
                                                "ok": False},
                                      "period": {"ok": True}}})
        text = report.as_mode_tables(self.write([scale]))
        row = [ln for ln in text.splitlines()
               if ln.startswith("| rmt+rmt+rmt")][0]
        self.assertIn("DEVIATED", row)
        self.assertIn("+12000", row)

    def test_two_records_sharing_a_tag_key_are_both_reported(self):
        """A table that drops a row without saying so is worse than no table.

        The filename is the tag key, so in practice they agree -- but keying the
        dedup by the record's own field meant a collision silently reported one
        run instead of two.
        """
        scale = mode_record(mode="scale", drivers=["rmt"],
                            stepper_count=1, first_step_skew_us=None,
                            skew_periods=None)
        text = report.as_mode_tables(self.write([scale, mode_record()]))
        self.assertIn("Parallel stepper count (`scale`)", text)
        self.assertIn("Synced start (`sync`)", text)

    def test_a_lost_step_is_named_not_just_flagged(self):
        """`DEVIATED` alone does not say which way it went.

        A swallowed step and an extra one are opposite failures that look
        identical in a flag, and they call for opposite responses -- a driver
        that stops early and one that never stops are not the same bug.
        """
        short = mode_record(per_stepper={
            "A": mode_record()["per_stepper"]["A"],
            "B": {"channel": "D2", "ticks": 320, "mean_period_us": 20.0,
                  "steps": {"steps_expected": 64, "steps_measured": 51,
                            "missing_steps": 13, "extra_steps": 0, "ok": False},
                  "period": {"expected_period_us": 20.0, "n_long": 0,
                             "n_short": 0, "ok": True}}})
        text = report.as_mode_tables(self.write([short]))
        row = [ln for ln in text.splitlines() if ln.startswith("| rmt+")][0]
        self.assertIn("x51/64", row)
        self.assertIn("-13 missing", row)
        self.assertNotIn("+", row.split("|")[-3])

    def test_a_wrong_period_is_named_not_just_flagged(self):
        """A stepper that keeps its count but drifts is the subtler failure.

        Nothing about the step count says the rate was wrong, so without the
        period named in the same cell a table of counts would call this row
        healthy.
        """
        drifted = mode_record(per_stepper={
            "A": mode_record()["per_stepper"]["A"],
            "B": {"channel": "D2", "ticks": 320, "mean_period_us": 21.4,
                  "steps": {"steps_expected": 64, "steps_measured": 64,
                            "missing_steps": 0, "extra_steps": 0, "ok": True},
                  "period": {"expected_period_us": 20.0, "n_long": 7,
                             "n_short": 2, "ok": False}}})
        text = report.as_mode_tables(self.write([drifted]))
        row = [ln for ln in text.splitlines() if ln.startswith("| rmt+")][0]
        self.assertIn("7 long/2 short periods", row)
        self.assertIn("21.4us", row)
        self.assertIn("x64/64", row)

    def test_a_mode_record_with_an_unknown_mode_is_named_not_dropped(self):
        """It went missing before.

        A MODE record belongs to neither table once its `mode` key is gone or
        misspelt, and filtering on that key alone dropped it from the report
        with nothing said -- so the reader would conclude the run was never
        made, which is a different claim from "made and unreported".
        """
        lost = mode_record(mode=None)
        text = report.as_mode_tables(self.write([lost]))
        self.assertIn("no recognised mode", text)
        self.assertIn(lost["tag_key"], text)

    def test_the_capability_table_comes_from_the_board_not_a_host_table(self):
        old = mode_record(board_drivers={"rmt": True, "rmt": True,
                                         "mcpwm_pcnt": True,
                                         "i2s_direct": True,
                                         "i2s_mux": True}, mux_init=False)
        text = report.as_mode_tables(self.write([old]))
        self.assertIn("Driver capability", text)
        for name in ("rmt", "mcpwm_pcnt", "i2s_direct", "i2s_mux"):
            self.assertIn(name, text)

    def test_a_mux_compiled_in_but_not_up_is_distinguishable_from_a_broken_one(self):
        # "i2s_mux=1 mux_init=0" is three unassigned pins. "i2s_mux absent" is a
        # build that never had it. A reader shown only a refusal cannot tell
        # them apart, and would call a wiring gap a driver defect.
        not_up = mode_record(board_drivers={"rmt": True, "i2s_mux": True},
                             mux_init=False)
        up = mode_record(board_drivers={"rmt": True, "i2s_mux": True},
                         mux_init=True)
        absent = mode_record(board_drivers={"rmt": True,
                                            "i2s_mux": False}, mux_init=False)
        unreported = mode_record(board_drivers={"timer": True}, mux_init=False)
        for record, expected in ((not_up, "compiled in, not brought up"),
                                 (up, "| up |"),
                                 (absent, "compiled out by this build"),
                                 (unreported, "not reported")):
            text = report.as_mode_tables(self.write([record]))
            row = [ln for ln in text.splitlines()
                   if ln.startswith("| esp32 / arduino")][0]
            self.assertIn(expected, row, record["mux_init"])
        # The three must read as three different situations.
        cells = set()
        for record in (not_up, up, absent, unreported):
            cells.add([ln for ln in report.as_mode_tables(self.write([record]))
                       .splitlines() if ln.startswith("| esp32 / arduino")][0])
        self.assertEqual(len(cells), 4, cells)

    def test_no_capability_section_when_the_board_was_never_asked(self):
        old = mode_record()
        old.pop("board_drivers", None)
        text = report.as_mode_tables(self.write([old]))
        self.assertNotIn("Driver capability", text)

    def test_a_results_dir_with_no_mode_records_yields_no_section(self):
        (self.tmp / "SR_01.json").write_text(json.dumps({"test_id": "SR_01"}))
        self.assertIsNone(report.as_mode_tables(self.tmp))
        self.assertIsNone(report.as_mode_tables(self.tmp / "nope"))

    def test_catalogue_and_mode_results_combine_in_one_report(self):
        """A person should not have to ask twice to see a whole session."""
        run_dir = build_run_dir("SR_01")
        self.addCleanup(shutil.rmtree, run_dir)
        results = self.write([mode_record()])
        text = report.as_markdown(report.collect(run_dir, vf.Dut().info()),
                                  run_dir, results)
        self.assertIn("SR_01", text)
        self.assertIn("Synced start (`sync`)", text)

    def test_a_run_with_only_mode_records_still_reports(self):
        text = report.as_markdown([], self.tmp, self.write([mode_record()]))
        self.assertIn("Synced start (`sync`)", text)
        self.assertIn("No captures found.", text)

    def test_the_tables_read_the_recorded_drivers(self):
        """A driver-list column that said 'rmt' for an rmt+mcpwm run would
        make the table's whole reason for existing -- comparing combinations --
        meaningless."""
        text = report.as_mode_tables(self.write([mode_record()]))
        self.assertIn("rmt+rmt", text)



class TestMatrixFindingClassification(unittest.TestCase):
    """A refusal is not a defect, and a capability answer is not a failure.

    This is the distinction the platform-matrix report is built on, and it is
    the difference between a report of defects and a log of the harness talking
    to the board: the first release-matrix run produced a 26-row "everything
    that did not pass" table in which two rows were real library bugs and the
    rest were `scale` doing its job and a firmware correctly saying it has no
    I2S queues.

    Nothing else in the suite touches run_matrix.py, which is how the same run
    also shipped two *empty* tables (mode points key as `{run_tag}_{point}`, and
    the report compared for equality) without a single test noticing.
    """

    def test_a_connect_refusal_is_the_measured_bound(self):
        # `scale` finding where MCPWM/PCNT stops. This is the answer the mode
        # exists to give, not something that went wrong.
        self.assertEqual(run_matrix.classify({
            "test_id": "MODE", "mode": "scale", "result": "refused",
            "error": "ERR connect step 6 n=6 drv=mcpwm_pcnt nodir=1",
        }), run_matrix.BOUND)

    def test_any_refusal_is_a_bound_not_a_defect(self):
        # Whatever the wording, the board declined. `scale` and `sync` record
        # refusals on purpose, so a refusal never reaches the findings table.
        self.assertEqual(run_matrix.classify({
            "test_id": "MODE", "mode": "sync", "result": "refused",
            "error": "ERR CONFIG mode dir|nodir",
        }), run_matrix.BOUND)

    def test_a_missing_driver_is_a_capability_answer(self):
        # ESP-IDF 4 and every Arduino build define no I2S queues, so SR_23 gets
        # here. It is recorded `failed` -- the scenario did not run -- and it is
        # not a defect of anything.
        self.assertEqual(run_matrix.classify({
            "test_id": "SR_23", "result": "failed",
            "error": "ERR CONFIG no such driver",
        }), run_matrix.CAPABILITY)

    def test_a_capability_answer_renders_as_not_applicable(self):
        # Not a bold FAIL in a matrix column: four of six rows looked like a
        # broken library when the only thing wrong was the IDF version.
        self.assertEqual(run_matrix.cell({
            "test_id": "SR_23", "result": "failed",
            "error": "ERR CONFIG no such driver",
        }), "n/a (no such driver)")

    def test_a_panic_is_a_defect(self):
        self.assertEqual(run_matrix.classify({
            "test_id": "SR_01", "result": "failed",
            "error": "Guru Meditation Error: Core  0 panic'ed (LoadProhibited)",
        }), run_matrix.DEFECT)

    def test_a_wrong_step_count_is_a_defect(self):
        # No error text at all -- the waveform disagreed with the command, which
        # is the defect this harness exists to find.
        self.assertEqual(run_matrix.classify({
            "test_id": "SR_02", "result": "failed",
            "steps": {"steps_measured": 254, "steps_expected": 255},
        }), run_matrix.DEFECT)

    def test_a_missing_channel_is_an_incomplete_measurement(self):
        # Not a wrong step count either: there was nothing to count.
        self.assertEqual(run_matrix.classify({
            "test_id": "MODE", "mode": "sync", "result": "failed",
            "incomplete_capture": {"missing_channels": ["S2"],
                                   "missing_steppers": ["B"]},
        }), run_matrix.INCOMPLETE)

    def test_a_note_is_not_taken_from_a_reply_the_board_confirmed(self):
        # An incomplete capture also carries `reply: "OK QRUN / POS 64 64"` --
        # the board confirming the move it did make. Reading `reply` first gave
        # a failed run the note "OK QRUN".
        rec = {"test_id": "MODE", "result": "failed",
               "reply": "OK QRUN\nPOS 64 64",
               "incomplete_capture": {"missing_channels": ["S2"],
                                      "missing_steppers": ["B"]}}
        self.assertIn("S2", run_matrix.note_of(rec))
        self.assertNotIn("OK QRUN", run_matrix.note_of(rec))


if __name__ == "__main__":
    unittest.main()