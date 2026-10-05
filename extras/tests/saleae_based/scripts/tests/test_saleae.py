#!/usr/bin/env python3
"""
Unit tests for the Saleae harness signal parser and SR_00 evaluation.

Run from extras/tests/saleae_based:
    python3 -m unittest discover -s scripts/tests -v
"""

import argparse
import itertools
import json
import os
import re
import shutil
import subprocess
import sys
import tempfile
import unittest
from unittest import mock
from pathlib import Path

SCRIPTS = Path(__file__).resolve().parents[1]
COMMON = SCRIPTS.parents[0] / "common"
# The library, for the checks that are about src/ rather than about this harness
# -- the mux direction/enable pin write is library code, and a defect there is
# found from here but fixed and guarded there.
LIB = SCRIPTS.parents[3] / "src"
sys.path.insert(0, str(SCRIPTS))

import analyze_csv  # noqa: E402
import harness  # noqa: E402
import run_hardware  # noqa: E402
import run_tests  # noqa: E402
import signal_parser as sp  # noqa: E402
import vcd_fixtures as vf  # noqa: E402

# The synthetic I2S mux bus, shared with the decoder's own tests rather than
# written out twice: a bus renderer that disagrees with itself between two test
# modules is a bus renderer that can be wrong.
from test_i2s_mux_decoder import Bus as MuxBus  # noqa: E402
from test_i2s_mux_decoder import write_bus_vcd as write_mux_bus_vcd  # noqa: E402

# The QINFO values the scenarios are planned against. Nothing here measures
# them; run_hardware only needs them to size the capture window.
_FAKE_DUT = vf.Dut()


def _two_or_three(capture):
    """load_capture_for_eval's two return shapes, from one (channels, rate).

    With `with_vcd` it also hands back the VCD path, which is what the mux decoder
    reads. None is the honest value here: a fixture has no .sr behind it, and
    handing the decoder a path that is not there would fail in the decoder rather
    than in the test that forgot to mock it.
    """
    def load(_file, with_vcd=False):
        return (capture[0], capture[1], None) if with_vcd else capture
    return load


def square(period_samples, high_samples, n_samples):
    """1/0 waveform: high for the first high_samples of each period."""
    return [1 if (i % period_samples) < high_samples else 0
            for i in range(n_samples)]


class TestSignalParser(unittest.TestCase):
    def test_square_wave_metrics(self):
        # 1 MHz, period 1000 samples (=1 ms -> 1 kHz), 10 % duty.
        samples = square(period_samples=1000, high_samples=100, n_samples=5000)
        m = sp.channel_metrics(samples, 1_000_000)
        self.assertAlmostEqual(m.frequency_hz, 1000.0, delta=1.0)
        self.assertAlmostEqual(m.duty_cycle_percent, 10.0, delta=0.5)
        # Starts HIGH, so the first pulse has no rising edge in-capture:
        # 4 rising edges over 5000 samples.
        self.assertEqual(m.step_count, 4)
        self.assertAlmostEqual(m.avg_high_us, 100.0, delta=0.5)
        self.assertTrue(
            sp.period_defects(m.inter_step_us, 1000.0)["ok"])

    def test_period_defects_flags_merged_and_dropped(self):
        # Two steps collapsed into one 500 us period, then a 2000 us gap
        # because one step never arrived.
        d = sp.period_defects([1000.0, 500.0, 1000.0, 2000.0], 1000.0)
        self.assertEqual(d["n_short"], 1)
        self.assertEqual(d["n_long"], 1)
        self.assertFalse(d["ok"])

    def test_step_count_defects(self):
        self.assertTrue(sp.step_count_defects(10, 10)["ok"])
        extra = sp.step_count_defects(11, 10)
        self.assertEqual(extra["extra_steps"], 1)
        self.assertFalse(extra["ok"])
        missing = sp.step_count_defects(9, 10)
        self.assertEqual(missing["missing_steps"], 1)
        self.assertFalse(missing["ok"])

    def test_pulse_widths_are_full_intervals(self):
        samples = square(1000, 250, 3000)
        m = sp.channel_metrics(samples, 1_000_000)
        for w in m.high_widths_us:
            self.assertAlmostEqual(w, 250.0, delta=0.5)
        for w in m.low_widths_us:
            self.assertAlmostEqual(w, 750.0, delta=0.5)

    def test_detect_edges(self):
        self.assertEqual(sp.detect_edges([0, 0, 1, 1, 0, 1]),
                         [(2, 1), (4, 0), (5, 1)])

    def test_dir_to_first_step(self):
        dir_samples = [0] * 1000 + [1] * 100
        step_samples = [0] * 1050 + [1] * 10 + [0] * 40
        delays = sp.dir_to_first_step_us(dir_samples, step_samples, 1_000_000)
        self.assertEqual(len(delays), 1)
        self.assertAlmostEqual(delays[0], 50.0, delta=0.5)

    def test_cross_channel_skew(self):
        a = [0] * 10 + [1] * 10 + [0] * 100
        b = [0] * 260 + [1] * 10 + [0] * 100
        skew = sp.cross_channel_skew_us({"A": a, "B": b}, 1_000_000)
        self.assertAlmostEqual(skew, 250.0, delta=0.5)

    def test_parse_rate(self):
        self.assertEqual(sp.parse_rate("1 MHz"), 1_000_000)
        self.assertEqual(sp.parse_rate(" 4MHz"), 4_000_000)
        self.assertEqual(sp.parse_rate("20 kHz"), 20_000)
        self.assertEqual(sp.parse_rate("1000000 Hz"), 1_000_000)

    def test_load_csv_roundtrip(self):
        with tempfile.TemporaryDirectory() as tmp:
            path = os.path.join(tmp, "capture.csv")
            with open(path, "w") as f:
                f.write("; CSV generated by libsigrok\n")
                f.write("; Samplerate: 1 MHz\n")
                f.write("logic,logic\n")
                f.write("0,1\n1,0\n0,1\n")
            channels, rate = sp.load_csv(path)
            self.assertEqual(rate, 1_000_000)
            self.assertEqual(channels["D0"], [0, 1, 0])
            self.assertEqual(channels["D1"], [1, 0, 1])

    def test_parse_timescale(self):
        self.assertAlmostEqual(sp.parse_timescale("1 us"), 1000.0)
        self.assertAlmostEqual(sp.parse_timescale("100 ps"), 0.1)
        self.assertAlmostEqual(sp.parse_timescale("1 ns"), 1.0)

    def test_load_vcd_expands_changes(self):
        # 1 MHz -> sigrok writes a 1 us timescale; changes only, no repeats.
        with tempfile.TemporaryDirectory() as tmp:
            path = os.path.join(tmp, "capture.vcd")
            with open(path, "w") as f:
                f.write("$timescale 1 us $end\n")
                f.write("$scope module libsigrok $end\n")
                f.write("$var wire 1 ! D0 $end\n")
                f.write("$var wire 1 \" D1 $end\n")
                f.write("$upscope $end\n$enddefinitions $end\n")
                f.write("#0 1! 0\"\n")
                f.write("#2 0!\n")
                f.write("#5 1!\n")
            channels, rate = sp.load_vcd(path)
            self.assertEqual(rate, 1_000_000)
            self.assertEqual(list(channels["D0"]), [1, 1, 0, 0, 0, 1])
            self.assertEqual(list(channels["D1"]), [0, 0, 0, 0, 0, 0])

    def test_vcd_is_padded_to_the_declared_capture_length(self):
        """A flat tail must survive the trip through VCD.

        A VCD records value changes only, so a channel that stops changing
        early ends the file short of the real capture. Without the sidecar the
        two cases -- a line that went quiet, and a recording that ran out --
        are indistinguishable, which is what made every SR_25 capture look
        like it stopped on its final pulse.
        """
        with tempfile.TemporaryDirectory() as tmp:
            vcd = Path(tmp) / "c.vcd"
            # Shaped like sigrok's own output, including the $comment that
            # carries the acquisition rate, so the real parse path is exercised.
            vcd.write_text(
                "$comment\n  Acquisition with 2/8 channels at 1 MHz\n"
                "$end\n"
                "$timescale 1 ns $end\n"
                "$scope logic $end\n"
                "$var wire 1 ! D0 $end\n"
                "$upscope $end\n"
                "$enddefinitions $end\n"
                "#0 1!\n")
            bare, _ = sp.load_vcd(str(vcd))
            self.assertEqual(len(bare["D0"]), 1, "no sidecar means no padding")

            (Path(tmp) / "c.meta").write_text(json.dumps(
                {"samples": 500, "sample_rate": 1_000_000}))
            padded, rate = sp.load_vcd(str(vcd))
            self.assertEqual(rate, 1_000_000)
            self.assertEqual(len(padded["D0"]), 500)
            self.assertEqual(set(padded["D0"]), {1}, "the flat level is kept")

    def test_unreadable_sidecar_is_ignored(self):
        """A corrupt sidecar must not make a capture unloadable."""
        import tempfile
        with tempfile.TemporaryDirectory() as tmp:
            vcd = Path(tmp) / "c.vcd"
            vcd.write_text("$timescale 1 ns $end\n"
                          "$scope logic $end\n"
                          "$var wire 1 ! D0 $end\n"
                          "$upscope $end\n"
                          "$enddefinitions $end\n"
                          "#0 1!\n")
            (Path(tmp) / "c.meta").write_text("{not json")
            channels, _ = sp.load_vcd(str(vcd))
            self.assertEqual(list(channels["D0"]), [1])

    def test_sr_to_vcd_matches_sr(self):
        """sigrok-cli .sr -> VCD must reproduce the samples exactly."""
        sr_path = Path(__file__).resolve().parents[2] / "capture.sr"
        if not sr_path.exists() or not shutil.which("sigrok-cli"):
            self.skipTest("no capture.sr or sigrok-cli")
        with tempfile.TemporaryDirectory() as tmp:
            vcd = os.path.join(tmp, "capture.vcd")
            result = subprocess.run(
                ["sigrok-cli", "-I", "srzip", "-i", str(sr_path),
                 "-O", "vcd", "-o", vcd],
                capture_output=True, text=True, timeout=60)
            self.assertEqual(result.returncode, 0, result.stderr)
            sr_channels, sr_rate = sp.load_sr(str(sr_path))
            vcd_channels, vcd_rate = sp.load_vcd(vcd)
            self.assertEqual(vcd_rate, sr_rate)
            # The VCD holds changes only, so it can end before the last
            # constant stretch of the capture.
            n = min(len(samples) for samples in vcd_channels.values())
            for name, samples in sr_channels.items():
                # bytearray, not list: a 24 MS/s capture is 96M samples and a
                # list of ints would cost ~770 MB per channel.
                self.assertEqual(bytes(samples[:n]), bytes(vcd_channels[name]),
                                 name)


class TestSR00(unittest.TestCase):
    DUTY_MS = [50, 100, 150, 200, 250, 300, 350, 400]

    def _channels(self, period_samples=1000, n_periods=3, invert=None):
        channels = {}
        for i, high in enumerate(self.DUTY_MS):
            name = f"D{i}"
            samples = square(period_samples, high, period_samples * n_periods)
            if invert == name:
                samples = [0 if s else 1 for s in samples]
            channels[name] = samples
        return channels

    def test_all_channels_pass(self):
        channels = self._channels()
        passed, results = analyze_csv.evaluate_sr00(channels, 1000)
        self.assertTrue(passed)
        for i, high in enumerate(self.DUTY_MS):
            self.assertAlmostEqual(results[f"D{i}"]["duty_cycle_percent"],
                                   high / 10.0, delta=0.5)
            self.assertTrue(results[f"D{i}"]["passed"])

    def test_inverted_channel_detected(self):
        channels = self._channels(invert="D0")  # 5 % -> 95 %
        passed, results = analyze_csv.evaluate_sr00(channels, 1000)
        self.assertFalse(passed)
        self.assertTrue(results["D0"]["inverted"])
        self.assertFalse(results["D0"]["passed"])

    def test_wrong_frequency_fails(self):
        # 500 samples/period at 1000 Hz -> 2 Hz
        channels = self._channels(period_samples=500)
        passed, _ = analyze_csv.evaluate_sr00(channels, 1000)
        self.assertFalse(passed)


# Every CONFIG the harness can emit, as (logical config, native driver) pairs.
# The firmware's grammar is not exercised by any unit test -- it needs a board --
# so what is pinned here is that the host never produces a line the firmware
# would refuse, and never names no driver at all.
CONFIG_CASES = [
    (cfg, native)
    for cfg in run_tests.CONFIGS
    for native in ("rmt", "timer", "pio", "mcpwm_pcnt")
]

# The names the firmware's parse_driver() knows about. A driver it does not know
# is refused, so a typo here would surface as a refused CONFIG and an ERR that
# reads like a hardware fault.
FIRMWARE_DRIVERS = {
    "rmt", "rmt", "mcpwm", "mcpwm_pcnt", "i2s", "i2s_direct", "i2s_mux",
    "timer", "pio",
}


class TestAvrRamBudget(unittest.TestCase):
    """No string literal may reach SRAM on AVR.

    The AVR linker script copies `.rodata` into RAM so it can initialise it at
    reset, so every `const` object and every string literal in the firmware is a
    permanent SRAM allocation. Measured on the saleae_avr build before this was
    fixed: .data 1136 B of which only 96 B were variables -- the remaining
    1040 B, 51 % of a 328P's 2048 bytes, was the string pool, and the build had
    80 bytes free.

    The library is already PROGMEM-clean through `FAS_PSTR`
    (src/fas_arch/result_codes.h). This harness is a serial protocol and used
    not to be: two earlier revisions were pushed past 90 % of SRAM by adding a
    few words to an error message. So the rule is checked here rather than
    trusted -- a host-side test cannot see it, and a 328P build only fails once
    the part is full.

    Nothing here builds anything. It is a source-level check, because the thing
    it guards is a source-level habit; the measurement it quotes comes from
    `avr-size -A .pio/build/saleae_avr/firmware.elf`, where `.data` is the whole
    story (a 328P's stack lives in `.data` too).
    """

    # Wrappers that put a literal in flash, and the compile-time-only contexts
    # where a literal costs nothing because it is never emitted.
    _FLASH_WRAPPERS = ("SAL_PSTR(",)
    _COMPILE_TIME = ("static_assert(", "_Static_assert(", "assert(", "#error",
                     "#pragma")

    def _sources(self):
        for path in sorted(COMMON.glob("*.[ch]")) + sorted(COMMON.glob("*.cpp")):
            yield path, path.read_text()

    def _bare_literals(self, path, text):
        """(line_no, line_text) for every literal that would land in RAM.

        A hand-rolled scan rather than a regex, because the cases that matter
        are exactly the ones a regex gets wrong: a literal split across lines by
        clang-format, parentheses inside the text, and two adjacent literals
        concatenated as one argument.
        """
        # saleae_str.h defines SAL_PSTR in terms of PSTR, which declares
        # `static const char[] PROGMEM = (s)`. Checking it would report the
        # definition of the rule as a violation of it.
        if path.name == "saleae_str.h":
            return []

        out = []            # scanned text, literals replaced by a space, so
        #                     the "what precedes this" checks below see only
        #                     real code
        offenders = []
        i, n = 0, len(text)
        depth = 0           # parenthesis nesting of the scanned text
        pstr_until = None   # depth at which the innermost SAL_PSTR( closes

        def since(sym):
            """`out` from just after its last `sym` -- a statement or a line."""
            joined = "".join(out)
            k = joined.rfind(sym)
            return joined[k + 1:] if k >= 0 else joined

        while i < n:
            c = text[i]
            if text.startswith("//", i):
                j = text.find("\n", i)
                i = n if j < 0 else j
                out.append(" ")
                continue
            if text.startswith("/*", i):
                j = text.find("*/", i)
                i = n if j < 0 else j + 2
                out.append(" ")
                continue
            if c != '"':
                out.append(c)
                i += 1
                if c == "(":
                    depth += 1
                    # `SAL_PSTR(` opening: every literal until the matching
                    # `)` belongs to it, which is what makes a format string
                    # split across lines by clang-format -- and two adjacent
                    # literals concatenated as one argument -- come out clean.
                    if "".join(out[:-1]).rstrip().endswith("SAL_PSTR"):
                        pstr_until = depth
                elif c == ")":
                    depth -= 1
                    if pstr_until is not None and depth < pstr_until:
                        pstr_until = None
                continue

            # A literal. Consume it whole, honouring backslash escapes.
            j = i + 1
            while j < n and text[j] != '"':
                j += 2 if text[j] == "\\" else 1
            j = min(j + 1, n)
            line_ctx = since("\n")
            stmt_ctx = since(";")
            exempt = (
                pstr_until is not None          # a SAL_PSTR(...) argument
                or line_ctx.rstrip().endswith("extern")  # extern "C"
                or re.search(r"#\s*include", line_ctx)
                or any(t in stmt_ctx for t in self._COMPILE_TIME))
            if exempt:
                out.append(" ")
            else:
                line_no = text.count("\n", 0, i) + 1
                line = text.splitlines()[line_no - 1].strip()
                offenders.append(f"{path.name}:{line_no}: {line[:70]}")
                out.append(" ")
            i = j
        return offenders

    def test_no_bare_string_literal_reaches_ram(self):
        offenders = []
        for path, text in self._sources():
            offenders += self._bare_literals(path, text)
        self.assertEqual(
            offenders, [],
            "string literals outside SAL_PSTR() become SRAM on AVR -- wrap "
            "them in SAL_PSTR() and use reply_p()/sal_snprintf()/sal_strcmp():\n"
            "  " + "\n  ".join(offenders))

    def test_flash_strings_are_never_printed_through_the_ram_path(self):
        # `reply()` dereferences its argument as RAM. A flash literal reaching
        # it returns whatever SRAM happens to hold at that address, which reads
        # as correct on ESP32 and prints garbage on a 328P -- the worst kind of
        # platform-dependent bug, because the host sees a plausible line.
        source = (COMMON / "saleae_app.cpp").read_text()
        self.assertIn(
            "static void reply(const char* text) "
            "{ saleae_hal_serial_write(text); }", source,
            "reply() must go through the RAM HAL -- it is what a formatted "
            "buffer is sent with")
        self.assertIn(
            "static void reply_p(const char* text) "
            "{ saleae_hal_serial_write_p(text); }", source,
            "reply_p() must go through the flash HAL")
        # Every literal reply goes through the flash path.
        self.assertNotRegex(source, r'reply\(\s*"')
        # ...and the HAL really can write a flash string on AVR. If the
        # __FlashStringHelper cast were dropped, Print would pick the
        # `const char*` overload and read flash as RAM.
        hal = (COMMON / "saleae_hal_arduino.cpp").read_text()
        self.assertIn("__FlashStringHelper", hal)

    def test_const_tables_are_progmem(self):
        # `const` is not free on AVR either: a lookup table is .rodata, and
        # .rodata is copied into SRAM. kChanPin (8 B), saleae_pins (22 B) and
        # saleae_high_ms (22 B) were 52 bytes of SRAM for data nothing writes.
        for name in ("kChanPin", "saleae_pins", "saleae_high_ms"):
            decls = [(p, line) for p, t in self._sources()
                     for line in t.splitlines()
                     if re.search(r"^static const .*\b%s\[" % name, line)]
            self.assertTrue(decls, f"{name} not found -- did it move or rename?")
            for path, line in decls:
                self.assertIn(
                    "SAL_PROGMEM", line,
                    f"{name} is const, so it is .rodata, and .rodata is RAM on "
                    f"AVR: {line.strip()}")

    def test_saleae_str_wraps_the_libc_calls_that_read_ram(self):
        # The header's value is the mapping: on AVR each of these reads its
        # format/operand out of flash, off AVR each is the plain libc call. If a
        # new call site appears that needs one of these and is not routed
        # through saleae_str.h, this is where the omission is visible.
        header = (COMMON / "saleae_str.h").read_text()
        for call, avr_form in (("sal_strcmp", "strcmp_P"),
                               ("sal_sscanf", "sscanf_P"),
                               ("sal_snprintf", "snprintf_P")):
            self.assertRegex(header, r"#define %s %s" % (call, avr_form),
                             f"{call} has no AVR form in saleae_str.h")
        for macro in ("SAL_PROGMEM", "SAL_PSTR", "sal_pgm_read_byte"):
            self.assertIn("#define %s" % macro, header,
                          f"saleae_str.h is missing {macro}")
        # The one helper that is a function rather than a macro, because it has
        # a loop in it: the flash -> RAM copy that every `%s` argument needs.
        self.assertIn("sal_to_ram(char* dst, const char* src, size_t cap)",
                      header)
        self.assertRegex(header, r"sal_pgm_read_byte\(src \+ i\)",
                         "sal_to_ram must read its source through "
                         "pgm_read_byte, or it copies garbage on AVR")


class TestConfigGrammar(unittest.TestCase):
    def test_wire_line_names_every_driver(self):
        for cfg, native in CONFIG_CASES:
            wire = run_tests.config_wire(cfg, native)
            tokens = wire.split()
            self.assertEqual(tokens[0], "CONFIG", wire)
            # count, then one driver per stepper, then the pin mode.
            self.assertTrue(tokens[1].isdigit(), wire)
            drivers = tokens[2].split(",")
            self.assertEqual(len(drivers), int(tokens[1]), wire)
            self.assertIn(tokens[3], ("dir", "nodir"), wire)
            self.assertTrue(all(drivers), f"empty driver name in {wire!r}")

    def test_no_driver_is_ever_auto(self):
        for cfg, native in CONFIG_CASES:
            for driver in run_tests.config_drivers(cfg, native):
                self.assertIn(driver, FIRMWARE_DRIVERS,
                              f"{cfg} sends {driver!r}, which CONFIG refuses")

    def test_driver_names_resolve_per_architecture(self):
        # One driver per stepper is the invariant; the count comes from the
        # config, not from the length of a driver list the caller padded.
        self.assertEqual(run_tests.config_drivers("1ch", "timer"), ["timer"])
        self.assertEqual(run_tests.config_drivers("2ch", "pio"), ["pio", "pio"])
        self.assertEqual(run_tests.config_drivers("mixed_rmt_mcpwm", "rmt"),
                         ["rmt", "mcpwm_pcnt"])

    def test_scenarios_only_use_known_configs(self):
        for sid, (cfg, *_rest) in run_tests.SCENARIOS.items():
            self.assertIn(cfg, run_tests.CONFIGS,
                          f"{sid} names config {cfg!r}, which has no CONFIG")

    def test_firmware_has_no_implicit_driver_choice(self):
        # The firmware is not unit-tested, so this is the only place the rule
        # can be checked without a board: SA_AUTO and DRIVER_DONT_CARE would
        # both reintroduce a driver nobody named.
        source = (COMMON / "saleae_app.cpp").read_text()
        code = "\n".join(
            line for line in source.splitlines()
            if not line.lstrip().startswith("//"))
        self.assertNotIn("SA_AUTO", code)
        self.assertNotIn("DRIVER_DONT_CARE", code)

    def test_firmware_refuses_the_superseded_presets(self):
        # `CONFIG 1ch` is the old vocabulary. It must not reach a connected
        # stepper, and the count token is validated whole so atol cannot read
        # the leading digit and quietly configure one stepper.
        source = (COMMON / "saleae_app.cpp").read_text()
        self.assertIn('*end != \'\\0\'', source)
        for name in ("1ch", "2ch", "4ch_rmt", "4ch_mcpwm"):
            self.assertNotIn(f'strcmp(name, "{name}")', source)

    def test_a_filled_queue_is_started_not_replayed(self):
        """QFILL queues the program; QRUN must not queue it a second time.

        Both commands arm the same cursors, so QRUN re-arming from scratch would
        push the program into a queue that already holds it and every one of
        those steps would come out twice -- a run with twice the steps asked
        for, and no error anywhere to say so. The firmware is not unit-tested,
        so this checks the source.

        The other half is that a filled cursor must not be topped up before the
        start: qe_pump() prefills and then feeds, and both loops have to skip a
        cursor that is waiting for its QRUN, or the depth QFILL reported is
        stale before the first step.
        """
        source = (COMMON / "saleae_app.cpp").read_text()
        self.assertIn("!fill_only && c->fill_only", source,
                      "QRUN re-arms a QFILLed cursor instead of starting it")
        pump = source[source.index("static void qe_pump"):]
        pump = pump[:pump.index("\nstatic ")]
        self.assertEqual(pump.count("c->fill_only"), 2,
                         "qe_pump must skip a fill_only cursor in both loops: "
                         "the prefill and the top-up")
        # QFILL reports the depth it reached, never the one it was asked for:
        # QUEUE_LEN is 16 on AVR and 32 on ESP32, and QE_ROOM_RESERVE holds
        # entries back, so a request for 16 cannot be met everywhere.
        self.assertIn('"OK QFILL q=%u\\n"', source)
        self.assertIn("slots[i].stepper->queueEntries()", source)
        self.assertIn('!sal_strcmp(cmd, SAL_PSTR("QFILL"))', source)

    @classmethod
    def _firmware_stepper_bound(cls):
        """SALEAE_STEPPER_BOUND: the widest stepper count this ladder is written
        against (8 without I2S, 32 with it).

        Read from the firmware rather than restated here, because the buffer
        ladder is keyed on it: a check that assumed 8 while the firmware builds
        32 would resolve the wrong rung and pass on a buffer that truncates a
        32-stepper CONFIG.
        """
        source = (COMMON / "saleae_app.cpp").read_text()
        m = re.search(r"#define SALEAE_STEPPER_BOUND (\d+)", source)
        if not m:
            raise AssertionError("SALEAE_STEPPER_BOUND is not defined")
        return int(m.group(1))

    @classmethod
    def _resolve_size(cls, expr, max_steppers=8):
        """A buffer size from its declaration: a literal or CONSTANT + n."""
        expr = expr.strip()
        if expr.isdigit():
            return int(expr)
        m = re.fullmatch(r"(\w+)\s*\+\s*(\d+)", expr)
        if m:
            return (cls._firmware_constant(m.group(1), max_steppers)
                    + int(m.group(2)))
        raise AssertionError(f"cannot resolve buffer size {expr!r}")

    @classmethod
    def _firmware_constant(cls, name, max_steppers=8):
        """Resolve a #define from saleae_app.cpp for a given stepper count.

        The buffer sizes are a #if ladder on SALEAE_MAX_STEPPERS rather than one
        expression, because the value is also an sscanf field width and a format
        string cannot hold `12 * SALEAE_MAX_STEPPERS`. So it is resolved here the
        same way the preprocessor would, for the worst case (8 steppers), which
        is the one that has to fit.
        """
        source = (COMMON / "saleae_app.cpp").read_text()
        # Rungs of a #if ladder on SALEAE_MAX_STEPPERS, including the trailing
        # `#else`. The `#else` rung is the unconditional top of the ladder and
        # has no condition of its own -- leaving it out made this resolver answer
        # with the last #elif's value (72 where 96 was meant), which is exactly
        # the sort of off-by-one-rung slip the test this feeds exists to catch.
        conditional = re.compile(
            r"#(?:if|elif) SALEAE_MAX_STEPPERS <= (\d+)\n"
            r"#define %s (\d+)\n" % name)
        # `#else` is allowed a comment block before its #define, because the top
        # rung of a ladder is exactly where the reasoning for it lives -- and the
        # ARG2_MAX one documents the 32-stepper case this test now resolves. A
        # regex that demanded the #define on the next line raised "no #else rung"
        # for the one rung that carries the number under test, which read as a
        # missing rung rather than a regex that cannot see past a comment.
        unconditional = re.compile(
            r"#else\n(?://[^\n]*\n)*#define %s (\d+)\n" % name)
        rungs = conditional.findall(source)
        if rungs:
            for cap, value in rungs:
                if max_steppers <= int(cap):
                    return int(value)
            top = unconditional.search(source)
            if not top:
                raise AssertionError(f"the {name} ladder has no #else rung")
            return int(top.group(1))

        # Not a ladder: a plain expression, which can still name another
        # constant. SALEAE_LINE_MAX is SALEAE_ARG2_MAX + slack, and ARG2_MAX is
        # the ladder -- so resolve the name it references and add.
        expr = re.search(r"#define %s \((\w+) \+ (\d+)\)" % name, source)
        if expr:
            return (cls._firmware_constant(expr.group(1), max_steppers)
                    + int(expr.group(2)))
        raise AssertionError(
            f"cannot resolve {name} in the firmware: no ladder, and not "
            f"'OTHER + n'")

    def test_qinfo_reply_fits_its_buffer_with_every_stepper(self):
        # QINFO grows by ~13 bytes per stepper (" maxspeedN=65535") on top of a
        # fixed prefix. It was printed into SALEAE_SHORT_REPLY_MAX, which is
        # sized for a reply with no per-stepper fields, so an 8-stepper board
        # truncated its own reply mid-number -- and the host then reported "no
        # QINFO reply" with no hint that the firmware had run out of buffer.
        source = (COMMON / "saleae_app.cpp").read_text()
        m = re.search(r"#define QINFO_REPLY_FOR\(n\) \((\d+) \+ (\d+) \* \(n\)"
                      r"(?: \+ (\d+))?\)", source)
        self.assertIsNotNone(m, "QINFO_REPLY_FOR is not defined by term, so the "
                               "buffer size cannot be checked against the "
                               "stepper count it has to cover")
        base, per = int(m.group(1)), int(m.group(2))
        slack = int(m.group(3) or 0)
        # Each rung must cover the stepper count that rung admits. A 328P has
        # SALEAE_MAX_STEPPERS 2, so sizing its QINFO for 32 would cost RAM the
        # tightest target does not have.
        #
        # Two bounds now: SALEAE_STEPPER_BOUND is what the ladder is written
        # against (8 without I2S, 32 with it), and the mux 32 is the case that
        # actually broke a reply -- QINFO on a 32-stepper i2s_mux board is 32
        # " maxspeedN=65535" fields, which is 500-odd bytes into a buffer sized
        # for eight.
        max_steppers = re.search(r"#define SALEAE_STEPPER_BOUND (\d+)", source)
        platform_cap = int(max_steppers.group(1))
        worst = len("QINFO tps=16000000 mincmd=65535 qlen=32 maxall=65535 ")
        for count in (2, 4, 8, platform_cap):
            worst_fields = base
            for i in range(count):
                worst_fields += len(f" maxspeed{i}=65535")
            worst_fields += len("\n") + 1        # the trailing NUL
            self.assertLessEqual(worst_fields, base + per * count + slack,
                                 f"a {count}-stepper QINFO needs "
                                 f"{worst_fields} bytes")

    def test_qinfo_reply_leads_with_the_field_the_host_plans_against(self):
        # maxall must come before the per-stepper fields, because a truncated
        # reply loses its tail: a leading field survives, a trailing one does
        # not, and the one that matters is the one the host cannot reconstruct.
        source = (COMMON / "saleae_app.cpp").read_text()
        body = source.split("static void handle_qinfo")[1].split("\n}\n")[0]
        self.assertLess(body.index("maxall="), body.index("maxspeed%u="),
                        "maxall is printed after the per-stepper fields, so a "
                        "buffer overrun truncates the value the host plans "
                        "against while leaving the ones it does not need")

    def _qinfo_body(self):
        return (COMMON / "saleae_app.cpp").read_text().split(
            "static void handle_qinfo")[1].split("\n}\n")[0]

    def _render_qinfo(self, floors):
        """Build the reply the firmware's own format strings produce.

        Rendered from the *source's* format strings rather than from a
        hand-written example, because the bug being guarded against was a
        disagreement between the firmware's grammar and the host's regex -- and
        a hand-written example would agree with whichever side the test author
        was looking at.
        """
        body = self._qinfo_body()
        head = re.search(r'"(QINFO tps=%\w+ mincmd=%\w+ qlen=%\w+ [^"]*)"',
                         body)
        self.assertIsNotNone(head, "handle_qinfo has no leading format string")
        # The floor is appended by a *separate* snprintf, so the header carries
        # only tps/mincmd/qlen and the name of the field that follows.
        self.assertRegex(head.group(1), r"[a-z]+=$",
                         "the QINFO header does not end in a field name, so "
                         "the floor value that follows has no field to belong "
                         "to and cannot be parsed")
        per = re.search(r'"( maxspeed%u=%lu)"', body)
        self.assertIsNotNone(per, "no per-stepper field in the QINFO reply")
        text = head.group(1) % (16_000_000, 3200, 32)
        m = re.search(r'"([^"]*%lu)"', body[body.index(head.group(1)) + 40:])
        self.assertIsNotNone(m, "the floor value is never formatted")
        text += m.group(1) % max(floors)
        for i, f in enumerate(floors):
            text += per.group(1) % (i, f)
        return text + "\n"

    def test_qinfo_names_every_stepper_separately(self):
        # The bug: one unseparated number per stepper, so a 3-stepper board sent
        # `maxspeed=808080` and the host read that as the single value 808080.
        # Every QSEG built from it then exceeded the 16-bit ticks field and was
        # refused, and the firmware's complaint was about the tick range rather
        # than about what was wrong.
        for floors in ([80], [80, 80], [80, 640, 320], [80] * 8):
            text = self._render_qinfo(floors)
            m = run_tests.QINFO_RE.search(text)
            self.assertIsNotNone(m,
                                 f"a {len(floors)}-stepper QINFO does not "
                                 f"parse: {text!r}")
            per = [int(v) for v in re.findall(r"maxspeed\d+=(\d+)",
                                              m.group(0))]
            self.assertEqual(per, floors,
                             f"floors came back wrong for {text!r}")

    def test_qinfo_reply_carries_the_floor_the_host_plans_against(self):
        # maxall is the field a shared program depends on. If it is dropped from
        # the reply the host cannot reconstruct it -- and the run then plans
        # against whichever field happens to survive.
        text = self._render_qinfo([80, 640])
        m = run_tests.QINFO_RE.search(text)
        self.assertIsNotNone(m, text)
        self.assertIn("maxall=", m.group(0))
        self.assertEqual(int(m.group(4)), 640,
                         "maxall must be the largest floor, not the first")

    def test_qinfo_uses_its_own_reply_buffer(self):
        # QINFO grew by ~13 bytes per stepper, into a buffer sized for a reply
        # with no per-stepper fields. An 8-stepper board truncated its own reply
        # mid-number, and the host reported "no QINFO reply" with no hint that
        # the firmware had run out of buffer.
        self.assertIn("char buf[SALEAE_QINFO_REPLY_MAX]", self._qinfo_body())

    def test_qinfo_stops_before_it_overruns_its_buffer(self):
        # The per-stepper fields are appended into a bounded buffer, so the
        # bounds have to be checked per field. snprintf's return value is how
        # much it *would* have written, which is the only way to notice.
        body = self._qinfo_body()
        # sal_snprintf, not snprintf: on AVR the format is a flash literal and
        # only snprintf_P can read it (saleae_str.h). Its return value is the
        # would-be length, exactly as snprintf's. Whitespace-tolerant, because
        # clang-format reflows the call across lines as the line grows.
        self.assertRegex(body, r"int n\s*=\s*sal_snprintf",
                         "the QINFO append does not check what snprintf wanted "
                         "to write, so it cannot detect an overrun")
        self.assertIn("sizeof(buf)", body)

    def test_host_reads_the_largest_floor_not_the_first(self):
        # A shared program is walked by every stepper, so the fastest period
        # legal for all of them is the largest floor -- not stepper A's own.
        # Reading the first field would plan too fast whenever a later stepper
        # is slower (MCPWM/PCNT next to RMT is exactly that case).
        text = ("QINFO tps=16000000 mincmd=3200 qlen=32 maxall=640 "
                "maxspeed0=80 maxspeed1=640")
        m = run_tests.QINFO_RE.search(text)
        self.assertIsNotNone(m, text)
        per = [int(v) for v in re.findall(r"maxspeed\d+=(\d+)", m.group(0))]
        self.assertEqual(per, [80, 640])
        self.assertEqual(int(m.group(4)), 640)
        self.assertEqual(max(per), 640)

    def test_qinfo_with_one_stepper_still_parses(self):
        # The single-stepper reply is the one the existing 25 scenarios ran on,
        # so it must keep parsing. A grammar change that only works for N > 1
        # would pass every plan test and break every recorded scenario.
        for text in ("QINFO tps=16000000 mincmd=3200 qlen=32 maxall=640 "
                     "maxspeed0=640",
                     "QINFO tps=16000000 mincmd=3200 qlen=32 maxall=640"):
            self.assertIsNotNone(run_tests.QINFO_RE.search(text), text)

    def test_legal_ticks_never_exceeds_the_16_bit_field(self):
        # Two floors concatenated by a sloppy QINFO parse once reached
        # legal_ticks() as 808080. The firmware's refusal was then accurate --
        # out of ticks range -- and about a number nothing had asked for. The
        # clamp turns that class of mistake into a slow but legal run.
        info = {"min_cmd_ticks": 3200, "max_speed_ticks": 808080}
        self.assertLessEqual(run_tests.legal_ticks(info, 1, 65535), 65535)
        self.assertLessEqual(run_tests.legal_ticks(info, 8, 808080), 65535)

    def test_harness_supplies_everything_the_runner_reads(self):
        # harness.py builds the args namespace run_tests.py then reads. Two were
        # missing -- capture_dir and sr00_sample_rate -- so *every* non-dry-run
        # invocation died on its first capture with an AttributeError,
        # including the --flash example in AGENTS.md. Nothing exercised the
        # path, because a test that only parses arguments never gets that far.
        args = harness.parse_args(["--arch", "esp32", "--driver", "rmt",
                                   "--tests", "SR_01"])
        for name in ("capture_dir", "sr00_sample_rate", "results_dir",
                     "sample_rate", "seconds", "port", "baud", "force",
                     "capture"):
            self.assertTrue(hasattr(args, name),
                            f"harness args lack {name!r}, which run_tests "
                            f"reads on every run")

    def test_the_catalogue_path_reaches_the_runner(self):
        # The end of the same regression: not just that the attributes exist,
        # but that a plain catalogue run gets past the first capture. The board
        # and the analyzer are faked, so this asserts only that the runner is
        # handed everything it needs before any hardware is touched.
        args = harness.parse_args(["--arch", "esp32", "--driver", "rmt",
                                   "--tests", "SR_01", "--results-dir",
                                   "/tmp/does-not-exist-yet",
                                   "--capture-dir", "/tmp/r3-none"])
        args.dut_driver = args.driver
        # A temp results dir, so the assertion that run() creates it does not
        # depend on -- and then leave behind -- a fixed path.
        with tempfile.TemporaryDirectory() as tmp:
            args.results_dir = str(Path(tmp) / "results")
            args.capture_dir = str(Path(tmp) / "capture")
            seen = {}

            def fake_open(port, baud, timeout=6.0):
                seen["opened"] = True
                return mock.Mock()

            # SR_00 is the gate, and it needs an analyzer too; this test is
            # about reaching the board at all, so let SR_00 pass on the fixture
            # and check SR_01 gets its turn.
            def fake_sr00(tag_key, a):
                seen["sr00"] = True
                return "passed", {"channels": {}}

            def fake_load(*a, **k):
                return (({"D0": [0] * 400}, 1_000_000, None)
                        if k.get("with_vcd") else
                        ({"D0": [0] * 400}, 1_000_000))

            with mock.patch.object(run_tests, "open_board", fake_open), \
                    mock.patch.object(run_tests, "run_sr00", fake_sr00), \
                    mock.patch.object(run_tests, "load_capture_for_eval",
                                      fake_load), \
                    mock.patch.object(run_tests, "read_map",
                                      lambda ser:
                                          (run_tests.default_channel_map(),
                                           {})), \
                    mock.patch.object(run_tests, "read_qinfo",
                                      lambda ser: dict(vf.Dut().info())), \
                    mock.patch.object(run_tests, "program",
                                      lambda *a: True), \
                    mock.patch.object(run_tests, "start_capture",
                                      lambda *a, **k: mock.Mock(
                                          wait=lambda: None)), \
                    mock.patch.object(run_tests, "send_line",
                                      lambda *a, **k: ""), \
                    mock.patch.object(run_tests, "reply_of",
                                      lambda *a: "OK QCLR"), \
                    mock.patch.object(run_tests, "drain",
                                      lambda *a, **k: ""):
                try:
                    run_tests.run("r3_probe", ["SR_01"], args)
                except Exception as exc:                  # noqa: BLE001
                    self.fail(f"catalogue run raised before any hardware: "
                              f"{exc!r}")
            self.assertTrue(seen.get("opened"),
                            "the runner never reached open_board")
            self.assertTrue(seen.get("sr00"), "SR_00 never ran")
            # run() creates the results directory; a caller naming one must not
            # get a FileNotFoundError after a capture has already been spent.
            self.assertTrue(Path(args.results_dir).is_dir(), args.results_dir)

    def test_config_line_fits_the_firmware_line_buffer(self):
        # The firmware reads a line into a fixed buffer and copies each argument
        # into a fixed buffer. A CONFIG the host generates that does not fit is
        # truncated on the way in and refused with a misleading "no such driver"
        # on the half-cut last name -- so the budget is checked here, where it
        # can be changed when the stepper count rises.
        #
        # The count is the widest the protocol can produce, NOT the widest the
        # analyzer channels allow. Those differ by 4x for the mux: a multiplexed
        # stepper spends a bit of the 32-bit word and no channel, so `nodir`
        # reaches 32 and `dir` 16 (run_tests.MUX_SLOT_COUNT) against the 8 and 4
        # that CHANNELS bounds. Deriving the count from MAX_STEPPERS_PER_MODE --
        # which is what this test used to do -- checked an 8-stepper line against
        # the 8-stepper rung of the buffer ladder and could not see the 32-stepper
        # rung at all, so a buffer that only truncated at 32 passed. The parser
        # refusal this item tracks (extras/todo/182) lived in exactly that gap.
        source = (COMMON / "saleae_app.cpp").read_text()
        # The ladder rung that has to hold the widest legal line, resolved the
        # way the preprocessor resolves it for that stepper count.
        cap = self._firmware_stepper_bound()
        line_max = self._firmware_constant("SALEAE_LINE_MAX", max_steppers=cap)
        # Every buffer a command-line argument is copied into, with its size.
        buffers = {name: self._resolve_size(size) for name, size in
                   re.findall(r"SAL_REPLY_BUF char (arg\d|cmd)\[([^\]]+)\]", source)}
        self.assertEqual(sorted(buffers), ["arg1", "arg2", "arg3", "arg4",
                                           "cmd"],
                         f"expected cmd and arg1..arg4, found {sorted(buffers)}")
        # Each field's width is `sizeof(its own buffer) - 1`, so a width and its
        # buffer cannot drift apart. That is stronger than the check this test
        # used to make, which compared each buffer against a literal width
        # spelled out in an sscanf format string -- two things to keep in sync,
        # with a silent truncation (or an overflow) as the failure mode of
        # getting it wrong.
        fields = re.findall(r"\{(cmd|arg\d), sizeof\(\1\) - 1\}", source)
        self.assertEqual(sorted(fields), sorted(buffers),
                         "every argument buffer needs a sal_field entry of the "
                         f"form {{name, sizeof(name) - 1}}; found {fields}")
        # Every buffer, resolved at the widest stepper count rather than the
        # default: the argument widths come from the same ladder as the line
        # buffer, so a rung that fits the line can still be too narrow for the
        # driver list, and only the top rung is exercised by a 32-stepper CONFIG.
        buffers = {name: self._resolve_size(size, max_steppers=cap)
                   for name, size in
                   re.findall(r"SAL_REPLY_BUF char (arg\d|cmd)\[([^\]]+)\]",
                              source)}
        widest = max(buffers.values()) - 1

        # `dir` is the longer line at a given count (six more characters), and
        # the mux doubles the count again. Both bounds are checked, so a fix
        # that sizes for `nodir` alone still fails here.
        counts = {
            "channel-bound": max(max(run_tests.CONFIGS[c][0]
                                     for c in run_tests.CONFIGS),
                                 max(run_tests.MAX_STEPPERS_PER_MODE.values())),
            "mux-nodir": run_tests.MUX_SLOT_COUNT,
            "mux-dir": run_tests.MUX_SLOT_COUNT // 2,
        }
        for label, count in counts.items():
            for mode in ("dir", "nodir"):
                for driver in FIRMWARE_DRIVERS:
                    line = (f"CONFIG {count} "
                            f"{','.join([driver] * count)} {mode}")
                    self.assertLessEqual(
                        len(line), line_max - 1,
                        f"{label} {mode}: {line!r} ({len(line)} chars) does not "
                        f"fit SALEAE_LINE_MAX ({line_max})")
                    self.assertLessEqual(
                        count * len(driver) + count - 1, widest,
                        f"{label} {mode}: a {count}x {driver!r} driver list is "
                        f"truncated by the argument width ({widest})")


class TestStackBudget(unittest.TestCase):
    """No reply or argument buffer may go back on the stack.

    This is the guard for extras/doc/implemented/idf55_main_task_stack_overflow.md. Every
    CONFIG constructs its drivers from the FreeRTOS `main` task, whose stack is
    `CONFIG_ESP_MAIN_TASK_STACK_SIZE` -- 3584 B on ESP32. A local array reserves
    its slot for the whole function, so `handle_config`'s reply buffer was held
    while it called `rmt_new_tx_channel`, and on ESP-IDF 5.5.3 the overflow ran
    off the top of the stack into the DRAM tlsf pool and surfaced much later as a
    corrupted free list inside `tlsf_malloc`.

    Measured peak stack for `CONFIG 1 rmt dir` on IDF 5.5.3, before and after:

        before   4272 B of 3584 B   -- over budget, panics
        after    2336 B of 3584 B   -- 1248 B spare

    The two changes were moving these buffers to `static` and dropping libc
    `sscanf`/`snprintf` (1496 B and 384 B per call) for the small formatter in
    saleae_str.h. The buffer change is the larger half: `handle_config` alone
    reserved SALEAE_CFG_REPLY_MAX = 1152 B on a 32-stepper ESP32 build.

    Source-level, like TestAvrRamBudget, because that is the habit being guarded
    and a host test cannot see it. `static` is not a pessimisation on AVR either:
    a 328P's stack lives in `.data` too, so the bytes are the same bytes.
    """

    # Anything at or above this is a buffer whose size is a tuning decision
    # rather than a token. The largest legitimate stack buffer left in the
    # firmware is far below it.
    LIMIT = 64

    def test_no_large_buffer_is_a_stack_local(self):
        offenders = []
        for path in sorted(COMMON.glob("*.[ch]")) + sorted(COMMON.glob("*.cpp")):
            for line_no, line in enumerate(path.read_text().splitlines(), 1):
                stripped = line.strip()
                # SAL_REPLY_BUF expands to `static` off AVR and to nothing on
                # AVR, and the AVR choice is deliberate -- see its definition in
                # saleae_app.cpp. Neither form is a bare stack local off AVR.
                if stripped.startswith("static ") or stripped.startswith(
                        "SAL_REPLY_BUF "):
                    continue
                m = re.match(r"(?:const\s+)?char\s+(\w+)\[([^\]]*)\]", stripped)
                if not m:
                    continue
                name, size = m.group(1), m.group(2)
                if not size.strip().isdigit():
                    continue  # sized from a constant; checked by its own test
                if int(size) >= self.LIMIT:
                    offenders.append(f"{path.name}:{line_no} char {name}[{size}]")
        self.assertEqual(offenders, [],
                         "these buffers are stack locals; move them to static "
                         "or the main task overflows into the heap:\n  "
                         + "\n  ".join(offenders))

    def test_reply_buffer_placement_is_explicit_and_platform_aware(self):
        # The rule has two halves that look contradictory -- `static` off AVR,
        # automatic on it -- so assert both are still spelled out. If the macro
        # is collapsed to one form, the stack budget regresses on one platform
        # or the other.
        source = (COMMON / "saleae_app.cpp").read_text()
        avr = re.search(r"#if defined\(__AVR__\)\n#define SAL_REPLY_BUF\n"
                        r"#else\n#define SAL_REPLY_BUF static\n#endif", source)
        self.assertIsNotNone(
            avr, "SAL_REPLY_BUF must be empty on AVR and static elsewhere")

    def test_the_measured_budget_is_recorded_next_to_the_fix(self):
        # The number above is a measurement, not a derivation, so it belongs in
        # the repository or it rots silently. It is recorded in the implemented
        # doc that closed todo 015/016, which is where the analysis lives now.
        doc = "doc/implemented/idf55_main_task_stack_overflow.md"
        text = (SCRIPTS.parents[2] / doc).read_text()
        self.assertIn("2336", text,
                      f"{doc} no longer records the post-fix peak stack "
                      "usage, so the budget claim cannot be checked")


class TestSaleaeFmt(unittest.TestCase):
    """The tiny formatter must cover every format string, and only those.

    `sal_snprintf` is this harness's own implementation of five conversions
    (saleae_str.h), not libc: libc cost 384 B of stack per call against a
    3584 B task that also has to reach the driver constructors. It returns -1
    for anything else rather than printing something plausible, which is safe
    but silent -- so the set of conversions actually used is checked here.
    """

    SUPPORTED = {"s", "u", "d", "lu", "ld"}

    def test_no_format_string_uses_an_unsupported_conversion(self):
        seen = set()
        for path in sorted(COMMON.glob("*.cpp")):
            for fmt in re.findall(r'SAL_PSTR\("([^"]*)"\)', path.read_text()):
                for m in re.finditer(r"%(-?\d*)(l?)([a-zA-Z%])", fmt):
                    width, length, conv = m.groups()
                    if conv == "%":
                        continue
                    spec = length + conv
                    seen.add(spec)
                    self.assertIn(spec, self.SUPPORTED,
                                  f"{path.name}: %{'%'}{spec} in {fmt!r} is "
                                  f"not implemented by sal_snprintf, which "
                                  f"returns -1 and prints nothing")
        # Not vacuous: if the scan ever stops finding conversions this test
        # would pass for the wrong reason.
        self.assertTrue(seen, "no format strings were scanned at all")

    def test_widths_and_zero_padding_are_absent(self):
        # `len += sal_snprintf(buf + len, sizeof(buf) - len, ...)` accumulates,
        # so a width or a zero pad would silently misalign every multi-part
        # reply. Neither is implemented; make sure nobody starts using one.
        for path in sorted(COMMON.glob("*.cpp")):
            for fmt in re.findall(r'SAL_PSTR\("([^"]*)"\)', path.read_text()):
                for m in re.finditer(r"%(.)", fmt):
                    self.assertNotIn(m.group(1), "0123456789.-+ #",
                                     f"{path.name}: flags/width in {fmt!r} are "
                                     "not implemented by sal_snprintf")


class TestChannelMap(unittest.TestCase):
    """Which analyzer channel carries which stepper.

    This is the thing a hardcoded map gets wrong, and it gets it wrong
    *quietly*: with the map A=D0, B=D2, C=D4, D=D6, a `nodir` run's stepper B is
    really on D1, so the evaluator reads a quiet pin, counts 0 steps, and calls
    it a driver that emits nothing. So the map is derived from the count and
    stride the firmware reports, and that derivation is checked here against both
    shapes -- including the one the old constant was wrong about.
    """

    def test_dir_mode_interleaves_step_and_dir(self):
        m = run_tests.default_channel_map(4, 2)
        self.assertEqual(m, {
            "A": {"step": "D0", "dir": "D1"},
            "B": {"step": "D2", "dir": "D3"},
            "C": {"step": "D4", "dir": "D5"},
            "D": {"step": "D6", "dir": "D7"},
        })
        # The old hardcoded map, which must survive this shape unchanged so no
        # recorded result is re-interpreted.
        self.assertEqual(run_tests.step_channels(m), {
            "A": "D0", "B": "D2", "C": "D4", "D": "D6"})

    def test_nodir_mode_is_one_channel_per_stepper(self):
        m = run_tests.default_channel_map(8, 1)
        self.assertEqual(sorted(m), list("ABCDEFGH"))
        self.assertEqual([e["step"] for e in m.values()],
                         [f"D{i}" for i in range(8)])
        # No dir channel to confuse a step channel with.
        self.assertEqual(run_tests.dir_channels(m), {})

    def test_nodir_puts_stepper_b_on_d1_not_d2(self):
        # The exact case the hardcoded map got wrong.
        self.assertEqual(run_tests.default_channel_map(2, 1)["B"]["step"], "D1")
        self.assertNotEqual(run_tests.default_channel_map(2, 1)["B"]["step"],
                            run_tests.default_channel_map(2, 2)["B"]["step"])

    def test_channel_budget_bounds_each_mode(self):
        self.assertEqual(run_tests.MAX_STEPPERS_PER_MODE,
                         {"dir": 4, "nodir": 8})
        self.assertEqual(len(run_tests.STEP_CHANNEL_ORDER), 8)
        for mode, cap in run_tests.MAX_STEPPERS_PER_MODE.items():
            stride = run_tests.CHANNELS_PER_STEPPER[mode]
            m = run_tests.default_channel_map(cap, stride)
            self.assertEqual(cap * stride, run_tests.CHANNELS, mode)
            self.assertEqual(len(m), cap, mode)

    def test_firmware_stride_matches_the_host_budget(self):
        # The host's stride table and the firmware's must agree, or a MAP reply
        # produces a map the host cannot build.
        source = (COMMON / "saleae_app.cpp").read_text()
        self.assertIn("#define SALEAE_STRIDE_DIR 2", source)
        self.assertIn("#define SALEAE_STRIDE_NODIR 1", source)
        self.assertIn("#define SALEAE_CHANNELS 8", source)
        self.assertEqual(
            run_tests.CHANNELS,
            int(re.search(r"#define SALEAE_CHANNELS (\d+)", source).group(1)))

    def test_map_reply_is_parsed_into_the_right_shape(self):
        """MAP count=8 mode=nodir stride=1 ch=... -> A..H on D0..D7."""
        line = "MAP count=8 mode=nodir stride=1 ch=2,0,4,16,17,5,18,19"
        m = run_tests.MAP_RE.search(line)
        self.assertIsNotNone(m, line)
        self.assertEqual(int(m.group(1)), 8)
        self.assertEqual(m.group(2), "nodir")
        self.assertEqual(int(m.group(3)), 1)
        self.assertEqual([int(p) for p in m.group(4).split(",")],
                         [2, 0, 4, 16, 17, 5, 18, 19])
        # Every field after `ch=` is optional, so a board with no mux (AVR, Pico)
        # answers without them and a board with one answers with all of them.
        self.assertIsNone(m.group(5))
        self.assertIsNone(m.group(6))
        self.assertIsNone(m.group(7))
        self.assertIsNone(m.group(8))

    def test_harness_refuses_a_count_the_channels_cannot_carry(self):
        args = harness.parse_args(["--arch", "esp32", "--count", "5",
                                   "--pin-mode", "dir"])
        with self.assertRaises(SystemExit) as cm:
            harness.derive(args)
        self.assertIn("4", str(cm.exception))

    def test_harness_accepts_eight_steppers_in_nodir(self):
        args = harness.parse_args(["--arch", "esp32", "--count", "8",
                                   "--pin-mode", "nodir",
                                   "--drivers", ",".join(["rmt"] * 8)])
        tag, _proj, _env, _rate = harness.derive(args)
        self.assertIn("nodir", tag)

    def test_firmware_refuses_counts_the_channels_cannot_carry(self):
        source = (COMMON / "saleae_app.cpp").read_text()
        # Two budgets, and the refusal names both because "too many steppers"
        # cannot say which one bit: the slot array (SALEAE_MAX_STEPPERS) is a
        # different fact from the analyzer's channel budget, and a driver that
        # binds first (MCPWM/PCNT has 6 queues on IDF 5) is a third.
        self.assertIn("SALEAE_MAX_STEPPERS", source)
        self.assertIn("want_phy > chan_cap", source)
        self.assertIn("ERR CONFIG n=%ld max=%u slots=%u chans=%u/%u", source)
        # The channel budget is the count the mux does NOT spend: a
        # multiplexed stepper is a bit of the 32-bit word, not a wire.
        self.assertIn("SALEAE_CHANNELS - SALEAE_BUS_COUNT", source)
        self.assertIn("want_mux", source)

    def test_nodir_forces_count_up_because_there_is_no_dir_pin(self):
        # count_up=false with no dir pin set is refused by the queue with
        # ErrorNoDirPinToToggle, so a nodir run has to drive it true. If that
        # line goes, a `nodir` scenario with dir=0 emits nothing at all.
        source = (COMMON / "saleae_app.cpp").read_text()
        self.assertIn("chan_stride == SALEAE_STRIDE_NODIR) ? true : seg->count_up",
                      source)

    def test_nodir_connects_no_direction_pin(self):
        # A step-only stepper gets no dir pin at all, not a repeated one, so
        # setDirectionPin() is not called. Calling it anyway would put a pin into
        # the library's dir state that the capture never drives, and any
        # direction-observing evaluator would then have a second pin to read.
        source = (COMMON / "saleae_app.cpp").read_text()
        # CHAN_PIN() rather than kChanPin[] directly: the table is PROGMEM on
        # AVR (saleae_str.h), so a plain subscript would read flash as RAM.
        #
        # chan_used, not idx * stride: a multiplexed stepper spends no channel
        # at all, so the physical channel cursor only moves for the steppers
        # that own a wire. Indexing by the stepper number is what made every
        # i2s_mux CONFIG claim a channel it does not have.
        self.assertIn("nodir ? 0 : CHAN_PIN(chan_used + 1)", source)
        # ...and the call is guarded, not unconditional.
        self.assertRegex(source, r"if \(!nodir\) \{\s*\n\s*s->setDirectionPin")

    def test_firmware_accepts_both_pin_modes(self):
        # parse_pin_mode has to recognise both names; a build where `nodir` is
        # unreachable would refuse the 8-stepper case with "mode dir|nodir" while
        # every host-side test still passed.
        source = (COMMON / "saleae_app.cpp").read_text()
        for mode in ("dir", "nodir"):
            # Anchored to the `if`: `if (false && !sal_strcmp(...))` still
            # contains the comparison, so a plain substring check passes on a
            # build where the mode is unreachable -- which is exactly the
            # mutation that has to be caught here.
            #
            # sal_strcmp/SAL_PSTR, not strcmp/"": on AVR the mode names are
            # flash literals, because a string literal is an SRAM allocation
            # there. A stray plain literal is the regression this whole
            # conversion exists to prevent, so the test asserts the form.
            self.assertRegex(
                source,
                r'if \(!sal_strcmp\(mode_text, SAL_PSTR\("%s"\)\)\) \{'
                % mode)
        self.assertIn("*stride = SALEAE_STRIDE_DIR", source)
        self.assertIn("*stride = SALEAE_STRIDE_NODIR", source)


class TestBoardCommands(unittest.TestCase):
    """What the runner actually puts on the wire.

    Nothing else here talks to a board, and the one thing that goes wrong
    silently is a line that is *nearly* right: `CONFIG CONFIG 1 rmt dir` was
    refused by the firmware on all 25 scenarios at once and read as a wiring
    fault rather than as the doubled keyword it was. A fake board that records
    what it was asked is the only place that shows up without one.
    """

    def _sent_to_board(self, scenario, dut_driver="rmt"):
        """Run one scenario's CONFIG against a fake board; return the lines."""
        sent = []

        class FakeCapture:
            returncode = 0

            def communicate(self, timeout=None):
                return ("", "")

        def fake_capture(*_args, **_kwargs):
            return FakeCapture()

        wire, channels, mask = run_hardware.wire_plan(scenario, dut_driver)
        with mock.patch.object(run_hardware, "send",
                               lambda ser, line, wait=1.0:
                               sent.append(line) or "OK CONFIG"), \
                mock.patch.object(run_hardware, "cold_boot",
                                  lambda port: mock.Mock()), \
                mock.patch.object(run_hardware.subprocess, "Popen",
                                  fake_capture), \
                mock.patch("time.sleep", lambda _s: None):
            run_hardware.run_segments(
                [(8, 640, True)], wire, channels, mask, "t",
                _FAKE_DUT.info())
        return sent

    def test_config_is_sent_exactly_once(self):
        for scenario in sorted(run_tests.SCENARIOS,
                               key=lambda s: int(s.split("_")[1])):
            sent = self._sent_to_board(scenario)
            self.assertTrue(sent, scenario)
            first = sent[0]
            # `scenario_wire`, not `config_wire`: the pin mode is `dir` for all
            # but one scenario, and building the line from a hardcoded `dir`
            # would score SR_31 against a CONFIG it never sends.
            self.assertEqual(first, run_tests.scenario_wire(scenario, "rmt"),
                             f"{scenario} sends {first!r}")
            self.assertEqual(first.count("CONFIG"), 1, first)
            self.assertEqual(first.split()[0], "CONFIG", first)

    def _stop_scenario_lines(self, scenario):
        """The lines run_hardware puts on the wire for a stop scenario."""
        sent = []

        class FakeCapture:
            returncode = 0

            def communicate(self, timeout=None):
                return ("", "")

            def kill(self):
                pass

        info = vf.Dut().info()
        segments = run_tests.SCENARIOS[scenario][1](info)
        wire, channels, mask = run_hardware.wire_plan(scenario, "rmt")

        def record(_ser, line, wait=1.0):
            sent.append(line)
            return "OK"

        def qfill(ser, m, entries=run_tests.QUEUE_FILL_ENTRIES):
            sent.append(f"QFILL {m} {entries}")
            return 14

        with mock.patch.object(run_hardware, "send", record), \
                mock.patch.object(run_hardware, "cold_boot",
                                  lambda port: mock.Mock()), \
                mock.patch.object(run_hardware.subprocess, "Popen",
                                  lambda *a, **k: FakeCapture()), \
                mock.patch.object(run_tests, "read_map",
                                  lambda ser: (
                                      run_tests.default_channel_map(1, 2),
                                      {"mode": "dir", "stride": 2,
                                       "pins": [2, 0], "marker": 7})), \
                mock.patch.object(run_tests, "fill_queue", qfill), \
                mock.patch("time.sleep", lambda _s: None):
            run_hardware.run_segments(
                segments, wire, channels, mask, scenario, info,
                stop_after=run_tests.STOP_AFTER.get(scenario),
                scenario=scenario)
        return sent

    def test_both_runners_issue_the_same_stop(self):
        """run_tests.py and run_hardware.py drive the same board.

        Separate capture paths, nothing to force agreement: an earlier revision
        of this harness gave the two runners different stops, and the one that
        was not updated could not have judged its scenario -- the marker was
        never designated either, so the evaluator found no marker edge and
        declined to judge. A runner that cannot judge a scenario is worse than
        one that fails it.

        So both stop scenarios are driven through run_hardware's own path here
        and the sequence is asserted: the fill and the marker are setup, both
        before QRUN, and the stop is the one the scenario is about.
        """
        for scenario, stop in (("SR_25", "STOP"), ("SR_30", "XSTOP")):
            with self.subTest(scenario=scenario):
                sent = self._stop_scenario_lines(scenario)
                self.assertEqual(sent[0].split()[0], "CONFIG")
                self.assertEqual(sent[-1], "POS")
                qrun = sent.index(f"QRUN 1")
                self.assertLess(sent.index("MARK 7"), qrun,
                                "MARK after QRUN puts a serial round-trip "
                                "between the start of the move and the stop")
                self.assertLess(sent.index(f"QFILL 1 "
                                           f"{run_tests.QUEUE_FILL_ENTRIES}"),
                                qrun, "QFILL must precede the run it fills for")
                self.assertLess(qrun, sent.index(stop),
                                f"{scenario} asserts a {stop} contract and must "
                                f"issue {stop}")
                # Only the one stop, and only after the run has started.
                self.assertEqual(sent.count(stop), 1)
                # Four segments of one queue-fill each, then the fill request.
                self.assertEqual(
                    [ln for ln in sent if ln.startswith("QSEG")],
                    [f"QSEG {run_tests.QUEUE_FILL_STEPS} 640 1"] * 4)


# The QINFO shapes this harness has actually met, one per driver family. They
# differ in `max_speed_ticks` by a factor of eight, which is the whole point of
# checking every builder against all of them: a builder that assumes a fast
# driver has a fast floor is legal on one and not on the other.
QINFO_SHAPES = {
    "rmt": {"ticks_per_s": 16_000_000, "min_cmd_ticks": 3200,
            "max_speed_ticks": 640, "max_speed_all_ticks": 640},
    "i2s_direct": {"ticks_per_s": 16_000_000, "min_cmd_ticks": 3200,
                   "max_speed_ticks": 80, "max_speed_all_ticks": 80},
    "avr_timer": {"ticks_per_s": 16_000_000, "min_cmd_ticks": 3200,
                  "max_speed_ticks": 426, "max_speed_all_ticks": 426},
}


class TestAlreadyIsNotAlwaysYes(unittest.TestCase):
    """A CONFIG that cannot be honoured must say so.

    `handle_config()` cannot move an already-connected stepper: a queue is
    allocated once and the engine has no release, so a reset is the only way
    back to an empty board. That is the reason for the `already` branch -- and
    it is not a reason to answer `OK`.

    Measured on ESP-IDF 5.5.3, before this was fixed:

    ```
    CONFIG 8 mcpwm_pcnt,... nodir -> ERR connect step 6 n=6
    CONFIG 7 mcpwm_pcnt,... nodir -> OK CONFIG n=6 mode=nodir already
    ```

    The first connects six and then fails, leaving `slot_count = 6`, so the
    second reported success for a configuration that was never established. The
    max-count probe believed it and recorded seven steppers on a board running
    six -- and the run *passed*, because `eval_scale` judges the six that are
    really there.

    Source checks, because the replies are firmware's. What the host does with
    them is checked in `TestMaxStepperCount`.
    """

    def _already_branch(self):
        src = (COMMON / "saleae_app.cpp").read_text()
        start = src.index("if (slot_count > 0) {")
        return src[start:src.index("\n  }", start)]

    def test_the_already_branch_compares_the_request_before_answering_ok(self):
        body = self._already_branch()
        # Every field a second CONFIG can disagree about: the count, the pin
        # mode (which is what sets the stride), and each stepper's driver. A
        # check that misses one of them reopens a way to be told OK falsely.
        self.assertIn("n == slot_count", body, "the count is compared")
        self.assertIn("stride == chan_stride", body,
                      "the pin mode is compared; it is what sets the stride")
        self.assertIn("slots[i].driver == drivers[i]", body,
                      "every stepper's driver is compared")
        # And OK is reachable only from inside that comparison.
        ok = body.index("OK CONFIG n=%u mode=%s already")
        self.assertGreater(ok, body.index("if (i == n)"),
                           "OK must be behind the full-match test, not before it")

    def test_a_mismatch_is_refused_and_names_what_is_connected(self):
        body = self._already_branch()
        self.assertIn("ERR CONFIG already n=%u stride=%u mode=%s", body)
        # The driver-mismatch refusal names the *index*, so a reader can see
        # which stepper disagrees rather than only that they all do.
        self.assertIn("ERR CONFIG already n=%u driver %u is %s", body)

    def test_the_branch_is_not_reachable_before_the_request_is_parsed(self):
        # Order matters as a property of the function, not just of the text: the
        # count, the mode and the driver list all have to be resolved first, or
        # the comparison is against whatever was left in them. `slot_count` is
        # zero until a CONFIG has connected something, so a fresh board never
        # reaches this branch at all.
        src = (COMMON / "saleae_app.cpp").read_text()
        setup = src[src.index("static void handle_config("):
                    src.index("if (slot_count > 0) {")]
        for needed in ("parse_pin_mode(mode_text", "strtol(count_text",
                       "parse_driver(tok"):
            self.assertIn(needed, setup,
                          f"the request is not fully parsed before the already "
                          f"check: {needed}")

    def test_the_probe_no_longer_depends_on_the_lie_being_absent(self):
        # The host fix is the load-bearing half: even with the firmware honest,
        # acceptance is MAP, because a CONFIG that says OK has still to be
        # cross-checked against what the board reports it connected. This is
        # what makes the probe right on a firmware that has not been reflashed.
        src = (SCRIPTS / "run_tests.py").read_text()
        probe = src[src.index("class MaxCountProbe:"):
                    src.index("def probe_for(")]
        self.assertIn("connected = len(read_map(ser)[0])", probe)
        self.assertIn("if connected == count:", probe,
                      "acceptance must be the board's own count")
        self.assertNotIn('if "OK CONFIG" in reply:',
                         probe.split("if connected == count:")[0],
                         "the reply must not be the acceptance test")


class TestNoRunawaySpin(unittest.TestCase):
    """No firmware loop may busy-wait long enough to starve IDLE into a WDT.

    Measured on this harness, ESP-IDF 5.5.3: a board sitting idle produced six
    `task_wdt` lines within ~5.3 s, with an IDLE0 backtrace naming `main` as the
    running task. A panic inside a capture window truncates the run, and SR_00
    then reported the truncated 1 Hz pattern as eight dead pins -- the one fault
    the wiring pre-check exists to catch.

    Two loops were at fault and both had to change, because neither alone fixed
    it:

    - `saleae_hal_idle()` was `saleae_hal_delay_ms(1)`, and that resolves to the
      *sub-tick spin* branch at FreeRTOS's 100 Hz, contradicting its own header,
      which says the idle path must block;
    - `saleae_test_loop()` spins once per millisecond for the whole second-long
      period, so SR_00 starves IDLE0 even with the idle path fixed.

    The second cannot simply stop spinning -- an edge has to land within SR_00's
    2 ms width tolerance -- so it sleeps between edges and keeps the spin only
    where the edge is. These are source checks because the defect is a *timing*
    property of firmware, which no unit test here can execute.
    """

    def _body(self, text, signature):
        """The body of `signature`'s definition, comments stripped."""
        start = text.index(signature)
        body = text[start:text.index("\n}", start)]
        return "\n".join(line for line in body.splitlines()
                         if not line.lstrip().startswith("//"))

    def test_the_idle_path_blocks_and_does_not_reuse_the_spin_delay(self):
        # The header names the required behaviour and the spin as the thing to
        # avoid; this is the check that the two agree. saleae_hal_delay_ms(1) is
        # the spin on ESP-IDF, so it appearing here is the bug.
        header = (COMMON / "saleae_hal.h").read_text()
        espidf = (COMMON / "saleae_hal_espidf.cpp").read_text()
        self.assertIn("Deliberately not `saleae_hal_delay_ms(1)`", header,
                      "the header must keep naming the trap, or the next "
                      "reader has no way to know what is being avoided")
        body = self._body(espidf, "void saleae_hal_idle(void)")
        self.assertNotIn("saleae_hal_delay_ms", body,
                         "the idle path is the sub-tick spin; it must block so "
                         "IDLE0 gets to run")
        self.assertIn("vTaskDelay", body)

    def test_the_sr00_pattern_sleeps_between_edges_and_spins_only_near_one(self):
        # The pattern is 1 Hz with edges every 50 ms, so there is nothing to do
        # between them. Spinning through all of it starves IDLE0, which is what
        # produced the watchdog panic.
        src = (COMMON / "saleae_test.cpp").read_text()
        body = self._body(src, "void saleae_test_loop(void)")
        self.assertIn("SPIN_WINDOW_MS", body,
                      "the spin has to be bounded to the window before an edge")
        self.assertIn("saleae_hal_delay_ms(ms_to_edge", body,
                      "the quiet time between edges must block, not spin")
        # And the spin itself must remain: it is where the edge lands, and
        # removing it is how SR_00's widths went to 428 us / 455.9 ms once
        # already (see the note in saleae_hal_espidf.cpp).
        self.assertIn("saleae_hal_delay_ms(1)", body)

    def test_the_spin_window_is_wider_than_the_tolerance_and_narrower_than_the_grid(self):
        # Both bounds are load-bearing. Narrower than the grid, or an edge is
        # reached by a timer wake-up instead of by spinning. Wider than SR_00's
        # tolerance, or the same. Measured tolerances, not preferences.
        src = (COMMON / "saleae_test.cpp").read_text()
        window = int(re.search(r"#define SPIN_WINDOW_MS (\d+)",
                               src).group(1))
        grid = int(re.search(r"#define EDGE_GRID_MS (\d+)", src).group(1))
        self.assertGreater(window, analyze_csv.EXPECTED_WIDTH_TOL_US / 1000.0)
        self.assertLess(window, grid)

    def test_the_edge_grid_matches_the_high_times_the_pattern_generates(self):
        # `saleae_test_loop()` computes the next edge as the next EDGE_GRID_MS
        # boundary. That is only right because every high time is a multiple of
        # the grid -- if one were not, the loop would sleep through its own edge.
        src = (COMMON / "saleae_test.cpp").read_text()
        grid = int(re.search(r"#define EDGE_GRID_MS (\d+)", src).group(1))
        table = re.search(r"saleae_high_ms\[SALEAE_PIN_COUNT\] SAL_PROGMEM = \{"
                          r"([^}]*)\}", src)
        self.assertIsNotNone(table, "the high-time table moved or changed shape")
        highs = [int(v) for v in re.findall(r"\d+", table.group(1))]
        self.assertTrue(highs)
        for high in highs:
            self.assertEqual(high % grid, 0,
                             f"high time {high} ms is not on the {grid} ms "
                             f"edge grid, so the sleep would cross its edge")

    def test_the_watchdog_is_subscribed_before_it_is_reset(self):
        # `esp_task_wdt_reset()` on a task that is not subscribed logs an error
        # on every call -- once per main-loop pass, on the UART the host
        # protocol uses. Measured: it flooded the console and the capture window.
        for name in ("saleae_hal_espidf.cpp", "saleae_hal_arduino.cpp"):
            hal = (COMMON / name).read_text()
            self.assertIn("saleae_hal_wdt_subscribe", hal, name)
            self.assertIn("saleae_hal_wdt_reset", hal, name)
        espidf = (COMMON / "saleae_hal_espidf.cpp").read_text()
        self.assertIn("wdt_subscribed", espidf,
                      "the reset must be guarded by the subscription")
        # Subscribe once, in setup, and before READY -- the host opens the port
        # and waits for READY, so anything before it is unobserved.
        app = (COMMON / "saleae_app.cpp").read_text()
        setup = self._body(app, "void saleae_app_setup(void)")
        self.assertIn("saleae_hal_wdt_subscribe()", setup)
        self.assertLess(setup.index("saleae_hal_wdt_subscribe()"),
                        setup.index("READY"),
                        "the watchdog must be live before the host is told the "
                        "board is ready")
        loop = self._body(app, "void saleae_app_loop(void)")
        self.assertIn("saleae_hal_wdt_reset()", loop)
        # Every pass, not inside one of the three branches: the pattern loop and
        # the feeder both end in a spin, so idle is not the only hungry path.
        reset = loop.index("saleae_hal_wdt_reset()")
        for branch in ("saleae_test_loop()", "qe_pump()", "saleae_hal_idle()"):
            self.assertLess(loop.index(branch), reset,
                            f"{branch} must come before the watchdog reset")

    def test_both_watchdog_calls_exist_on_every_hal(self):
        # Both HALs are linked into every build (link_app.sh) and app calls both
        # unconditionally, so a missing one is a link error on the other
        # platform -- found by building the wrong one first.
        for name in ("saleae_hal_espidf.cpp", "saleae_hal_arduino.cpp"):
            hal = (COMMON / name).read_text()
            for fn in ("saleae_hal_wdt_subscribe(void)",
                       "saleae_hal_wdt_reset(void)"):
                self.assertIn(f"void {fn}", hal, f"{name} lacks {fn}")
        header = (COMMON / "saleae_hal.h").read_text()
        for fn in ("saleae_hal_wdt_subscribe(void);", "saleae_hal_wdt_reset(void);"):
            self.assertIn(fn, header)


class TestResetBeforeEachTest(unittest.TestCase):
    """Every run starts on a board that has just rebooted.

    Measured on an ESP32-DevKitC with ESP-IDF 5.5.3: the firmware trips its own
    task watchdog while completely idle -- `open_board()`, then nothing at all,
    and six `task_wdt` lines arrive within ~5.3 s with an IDLE0 backtrace. No
    CONFIG, no program, no capture involved.

    That is a firmware property, and it is why this is load-bearing rather than
    hygiene. A panic inside a capture window truncates the run, and the pin
    self-test then reports the truncated pattern as eight dead cables -- which
    is the fault it exists to catch, so the pre-check becomes indistinguishable
    from the thing it is for.
    """

    def test_open_board_raises_when_the_board_did_not_reset(self):
        # Opening the port does not *guarantee* a reset: the DTR/RTS toggle does
        # not always fire, which is the same reason a flash occasionally comes
        # up in the wrong boot mode. The old loop fell through after its timeout
        # and handed back a stale board that answered commands from the previous
        # test, so a run measured that board's leftovers.
        class FakeSerial:
            def __init__(self, *a, **k):
                pass

            def read(self, _n):
                return b""

            def reset_input_buffer(self):
                pass

            def close(self):
                pass

        # A clock that advances: open_board() loops on `time.time() < deadline`,
        # so a frozen one would spin forever rather than time out.
        clock = itertools.count(0, 0.5)

        def now():
            return next(clock)

        with mock.patch("serial.Serial", FakeSerial), \
                mock.patch.object(run_tests.time, "time", now):
            with self.assertRaises(run_tests.BoardError) as ctx:
                run_tests.open_board("/dev/null", 115200, timeout=1.0)
        # The message has to name the cause, because "it did not reset" is the
        # one thing the reader can act on and "timed out" is not.
        self.assertIn("READY", str(ctx.exception))
        self.assertIn("not reset", str(ctx.exception))

    def test_open_board_returns_when_ready_is_seen(self):
        class FakeSerial:
            def __init__(self, *a, **k):
                self.written = []

            def read(self, _n):
                return b"I (290) main_task: Start\n... READY\n"

            def reset_input_buffer(self):
                pass

            def close(self):
                pass

        with mock.patch("serial.Serial", FakeSerial):
            ser = run_tests.open_board("/dev/null", 115200, timeout=1.0)
        self.assertIsInstance(ser, FakeSerial)

    def test_every_path_that_talks_to_a_board_goes_through_open_board(self):
        # One reset point, or the guarantee is only as good as the call site
        # somebody remembered. `measure()` and `run_sr00()` are the two, and
        # both are what every scenario and every mode point runs through.
        source = (SCRIPTS / "run_tests.py").read_text()
        code = "\n".join(line for line in source.splitlines()
                         if not line.lstrip().startswith("#"))
        for fn in ("def run_sr00(", "def measure("):
            start = code.index(fn)
            body = code[start:code.index("\ndef ", start + 10)]
            self.assertIn("open_board(", body,
                          f"{fn} does not open (and so reset) the board")

    def test_a_firmware_fault_is_not_reported_as_a_wiring_failure(self):
        # The two must be distinguishable, and it is the *verdict* that carries
        # it: `failed` on SR_00 is a statement about the cable, so it must not
        # be reachable from a firmware panic.
        self.assertEqual(
            run_tests.firmware_fault("OK SR00\nE (5300) task_wdt: ...\n"),
            ["task_wdt"])
        self.assertEqual(
            run_tests.firmware_fault(
                "Backtrace: 0x400DEDEE:0x3FFB1150 0x400DF1B0\n"),
            ["Backtrace:"])
        self.assertIsNone(run_tests.firmware_fault(
            "OK SR00\nOK CONFIG n=1 mode=dir\nPOS 0\n"))
        # A refused command is not a fault: the firmware is answering.
        self.assertIsNone(run_tests.firmware_fault(
            "ERR connect step 6 n=7 drv=mcpwm_pcnt nodir=1\n"))

    def test_sr00_records_a_fault_as_an_error_and_names_the_marker(self):
        # The path that was misreading one. A truncated capture makes every
        # channel's first high time a fragment of the commanded one -- measured
        # D0 19.3 ms against 50 ms, the pairs summing to exactly one 1000 ms
        # period -- which is indistinguishable from eight dead pins. So the
        # verdict is `error`, the marker is named, and the waveform is not
        # consulted for a wiring claim at all.
        args = harness.parse_args(["--arch", "esp32", "--driver", "rmt",
                                   "--tests", "SR_00", "--results-dir",
                                   "/tmp/sr00err", "--capture-dir",
                                   "/tmp/sr00err"])
        fault = "E (5300) task_wdt: Task watchdog got triggered.\n" \
                "Backtrace: 0x400DEDEE:0x3FFB1150\n"
        with tempfile.TemporaryDirectory() as tmp:
            args.capture_dir = str(Path(tmp) / "cap")
            with mock.patch.object(run_tests, "open_board",
                                   lambda *a, **k: mock.Mock()), \
                    mock.patch.object(run_tests, "send_line",
                                      lambda *a, **k: ""), \
                    mock.patch.object(run_tests, "start_capture",
                                      lambda *a, **k: mock.Mock(
                                          wait=lambda: None)), \
                    mock.patch.object(run_tests, "drain",
                                      lambda *a, **k: fault), \
                    mock.patch.object(run_tests.time, "sleep",
                                      lambda *a: None), \
                    mock.patch.object(
                        run_tests, "load_capture_for_eval",
                        lambda *a, **k: ({f"D{i}": [0] * 100 for i in range(8)},
                                         1_000_000)):
                status, detail = run_tests.run_sr00("t", args)
        self.assertEqual(status, "error")
        self.assertEqual(detail["firmware_fault"], ["task_wdt", "Backtrace:"])
        self.assertIn("truncated", detail["error"])
        self.assertIn("nothing about the wiring", detail["error"])

    def test_a_clean_sr00_is_still_passed_and_a_wiring_fault_still_failed(self):
        # The other direction, which matters more: the check must not turn a
        # real dead cable into an error, or the pre-check stops catching the one
        # thing it exists for.
        args = harness.parse_args(["--arch", "esp32", "--driver", "rmt",
                                   "--tests", "SR_00", "--results-dir",
                                   "/tmp/sr00ok", "--capture-dir",
                                   "/tmp/sr00ok"])
        quiet = {"D0": [0] * 100}
        with tempfile.TemporaryDirectory() as tmp:
            args.capture_dir = str(Path(tmp) / "cap")
            with mock.patch.object(run_tests, "open_board",
                                   lambda *a, **k: mock.Mock()), \
                    mock.patch.object(run_tests, "send_line",
                                      lambda *a, **k: ""), \
                    mock.patch.object(run_tests, "start_capture",
                                      lambda *a, **k: mock.Mock(
                                          wait=lambda: None)), \
                    mock.patch.object(run_tests, "drain",
                                      lambda *a, **k: "OK SR00\n"), \
                    mock.patch.object(run_tests.time, "sleep",
                                      lambda _s: None), \
                    mock.patch.object(run_tests,
                                      "load_capture_for_eval",
                                      lambda *a, **k: (quiet, 1_000_000)):
                status, _detail = run_tests.run_sr00("t", args)
        # A single quiet channel cannot pass the 8-channel pre-check, and it is
        # a wiring verdict, not a firmware one.
        self.assertEqual(status, "failed")

    def test_a_fault_during_a_scenario_capture_is_an_error_not_a_failure(self):
        # Same reasoning for a queue scenario: a panic truncates the run, and
        # the evaluator would otherwise report the short count as a driver that
        # drops steps. That is a defect claim, and it would be false.
        args = harness.parse_args(["--arch", "esp32", "--driver", "rmt",
                                   "--tests", "SR_01", "--results-dir",
                                   "/tmp/faultsr", "--capture-dir",
                                   "/tmp/faultsr"])
        info = {"ticks_per_s": 16_000_000, "min_cmd_ticks": 3200,
                "max_speed_ticks": 640, "queue_len": 32}
        fault = "OK QRUN\nE (5300) task_wdt: Task watchdog got triggered.\n"

        def _ok_config(_ser, line, **_kw):
            # CONFIG has to be accepted or the run is refused before it ever
            # reaches a capture, and this test is about what happens after.
            return ("OK CONFIG n=1 mode=dir" if line.startswith("CONFIG")
                    else "OK QCLR")

        with tempfile.TemporaryDirectory() as tmp:
            args.capture_dir = str(Path(tmp) / "cap")
            with mock.patch.object(run_tests, "open_board",
                                   lambda *a, **k: mock.Mock()), \
                    mock.patch.object(run_tests, "reply_of", _ok_config), \
                    mock.patch.object(run_tests, "send_line",
                                      lambda *a, **k: ""), \
                    mock.patch.object(run_tests, "drain",
                                      lambda *a, **k: fault), \
                    mock.patch.object(run_tests, "read_map",
                                      lambda ser: (run_tests
                                                   .default_channel_map(1, 2),
                                                   {"stride": 2})), \
                    mock.patch.object(run_tests, "read_qinfo",
                                      lambda ser: dict(info)), \
                    mock.patch.object(run_tests, "program",
                                      lambda *a: True), \
                    mock.patch.object(run_tests, "start_capture",
                                      lambda *a, **k: mock.Mock(
                                          wait=lambda: None)), \
                    mock.patch.object(run_tests.time, "sleep",
                                      lambda _s: None):
                status, detail = run_tests.measure(
                    "t", "sr01", "CONFIG 1 rmt dir", 1,
                    run_tests.sc_period_exact, "SR_01", args,
                    scenario="SR_01")
        self.assertEqual(status, "error")
        self.assertEqual(detail["firmware_fault"], ["task_wdt"])
        self.assertNotIn("per_stepper", detail)


class TestMaxStepperCount(unittest.TestCase):
    """SR_31: the catalogue's only test whose stepper count it does not know.

    Everything here is a pure function of the driver name and the board's
    replies, so the properties that matter -- the count comes from the board and
    not from a table, the wire is legal, the mask covers every stepper -- are
    checkable without hardware. What still needs a board is the *answer*, which
    is the measurement the scenario exists to record.
    """

    class FakeBoard:
        """A board that connects `n` steppers up to `limit` and refuses above.

        Refuses for the reason each real limit has: a driver with no queue left
        says `ERR connect step`, and the analyzer running out of channels says
        the count needs more channels than it has.

        Models the *partial* connect too, because it is the whole reason
        acceptance cannot be read off the reply: a refused CONFIG leaves the
        steppers it did connect in place, and the next CONFIG answers
        `OK CONFIG n=<connected> ... already`. `lying_again` turns that second
        half on and off, because a fake board that only ever refuses perfectly
        would pass a probe that trusted the reply.
        """

        def __init__(self, limit, channel_cap=run_tests.CHANNELS,
                     mux_word=run_tests.MUX_SLOT_COUNT, stride=1,
                     lying_again=True):
            self.limit = limit
            self.channel_cap = channel_cap
            self.mux_word = mux_word
            self.stride = stride
            self.lying_again = lying_again
            self.lines = []
            self.connected = 0

        def __call__(self, _ser, line, **_kw):
            self.lines.append(line)
            tokens = line.split()
            if tokens[0] != "CONFIG":
                return "OK"
            count = int(tokens[1])
            mux = all(d.startswith("i2s_mux") for d in tokens[2].split(","))
            if self.connected and self.lying_again:
                # The half-configured trap: whatever is asked for, the board
                # reports success for what a *previous* attempt left connected.
                return f"OK CONFIG n={self.connected} mode=nodir already"
            if mux and count * self.stride > self.mux_word:
                return (f"ERR CONFIG mux n={count} needs "
                        f"{count * self.stride} slots, max={self.mux_word}")
            if not mux and count * self.stride > self.channel_cap:
                return (f"ERR CONFIG n={count} needs "
                        f"{count * self.stride} channels, "
                        f"max={self.channel_cap}")
            if count > self.limit:
                # Partial connect: the prefix that fitted stays connected.
                self.connected = self.limit
                return f"ERR connect step {self.limit} n={self.limit}"
            self.connected = count
            return "OK CONFIG"

        def map(self, _ser):
            """read_map(), as the probe sees it: one entry per connected stepper."""
            return (run_tests.default_channel_map(self.connected, self.stride),
                    {"stride": self.stride})

    def _probe(self, driver, limit, reopen=True, reset=True, **kw):
        """Run the probe against a FakeBoard. Returns what find() returns.

        `reopen` models the caller reopening the port between attempts.
        `reset` is what that reopen *does* -- clearing the half-connected state
        an ESP32 reset clears. Both are switchable because they are two
        different mistakes: `reopen=False` means the probe never asked for a
        fresh session, and `reset=False` means it asked for one that did not
        reset, which is the silent half (the DTR/RTS toggle not firing, which
        AGENTS.md already records for the bootloader).
        """
        board = self.FakeBoard(limit, **kw)
        probe = run_tests.MaxCountProbe(driver)
        opens = []

        def do_reopen():
            opens.append(1)
            if reset:
                board.connected = 0
            return mock.Mock()

        with mock.patch.object(run_tests, "reply_of", board), \
                mock.patch.object(run_tests, "read_map", board.map):
            wire, mask, detail, reply, ser = probe.find(
                mock.Mock(), do_reopen if reopen else None)
        return wire, mask, detail, reply, board, opens

    def test_an_ok_reply_for_a_count_the_board_did_not_connect_is_not_believed(self):
        # The defect the first `--tests SR_31` run walked into, measured on the
        # board and reproduced by the fake's `lying_again` mode:
        #
        #   CONFIG 8 mcpwm_pcnt,... nodir -> ERR connect step 6 n=6
        #   CONFIG 7 mcpwm_pcnt,... nodir -> OK CONFIG n=6 mode=nodir already
        #
        # The second line is success for a configuration that was never
        # established: the first attempt left six steppers connected, and
        # handle_config() short-circuits any later CONFIG to "already". A host
        # that reads acceptance off the reply records 7 here -- and the run
        # *passes*, because eval_scale judges the six steppers that are really
        # there. Nothing downstream would have noticed.
        #
        # So acceptance is MAP, and the disagreement is recorded rather than
        # absorbed: `ok_but_short` is a different entry from `refused_above`
        # because "the board refuses 7" and "the board said yes and did not" are
        # different findings.
        _w, _m, detail, _r, _b, _o = self._probe("mcpwm_pcnt", 6, reset=False)
        self.assertEqual(detail["max_stepper_count"], 6,
                         "the board connected six; six is the answer")
        self.assertEqual([r["n"] for r in detail["refused_above"]], [8])
        self.assertEqual(detail["ok_but_short"],
                         [{"n": 7,
                           "reply": "OK CONFIG n=6 mode=nodir already",
                           "connected": 6}])
        # And the mask is the connected count's, not the one that said yes.
        _w, mask, _d, _r, _b, _o = self._probe("mcpwm_pcnt", 6, reset=False)
        self.assertEqual(mask, (1 << 6) - 1)

    def test_the_count_survives_a_reopen_that_does_not_reset_because_map_decides(self):
        # Worth stating because the first design assumed the opposite. Reading
        # acceptance off MAP rather than off the reply is what makes the probe
        # robust to the half-configured board, so the reset between attempts is
        # about *provenance*, not about reaching the number:
        #
        #   with a reset: 8 refused, 7 refused, 6 CONFIGured and connected
        #   without one: 8 refused, 7 answered "already" for the six that
        #                 attempt left, and 6 is accepted because MAP says six
        #
        # Both reach 6. What differs is that without the reset the accepted
        # state is a *leftover* from a refused attempt, which is only the right
        # answer because every attempt uses the same driver and the same pin
        # mode -- asserted below. A probe whose attempts could differ would need
        # the reset to mean anything.
        _w, _m, clean, _r, _b, opens = self._probe("mcpwm_pcnt", 6)
        self.assertEqual(len(opens), 2, "two resets between three attempts")
        self.assertEqual(clean["probe_attempts"], 3)
        self.assertEqual(clean["max_stepper_count"], 6)
        self.assertEqual([r["n"] for r in clean["refused_above"]], [8, 7])
        self.assertEqual(clean["ok_but_short"], [])
        _w, _m, dirty, _r, _b, opens = self._probe("mcpwm_pcnt", 6, reset=False)
        self.assertEqual(dirty["max_stepper_count"], 6)
        self.assertEqual(len(opens), 2)
        self.assertEqual([r["n"] for r in dirty["refused_above"]], [8])
        self.assertEqual(dirty["ok_but_short"],
                         [{"n": 7,
                           "reply": "OK CONFIG n=6 mode=nodir already",
                           "connected": 6}])

    def test_every_probe_attempt_uses_one_driver_and_one_pin_mode(self):
        # The invariant the previous test leans on: a leftover state from a
        # refused attempt is the right answer only because it was the same
        # request. If two attempts could differ, MAP matching by count would be
        # a coincidence, and the reset would be load-bearing again.
        probe = run_tests.MaxCountProbe("mcpwm_pcnt")
        wires = [probe.wire_for(n) for n in range(1, probe.bound + 1)]
        for wire in wires:
            tokens = wire.split()
            self.assertEqual(tokens[0], "CONFIG")
            self.assertEqual(tokens[3], run_tests.MAX_COUNT_PIN_MODE)
            self.assertEqual(set(tokens[2].split(",")), {"mcpwm_pcnt"})
            self.assertEqual(int(tokens[1]),
                             len(tokens[2].split(",")))

    def test_a_driver_that_takes_the_bound_outright_is_never_reset(self):
        # The common case must cost one CONFIG and no reset: `rmt` has eight
        # queues and eight channels, so the first attempt is the answer.
        _w, mask, detail, _r, _b, opens = self._probe("rmt", 8)
        self.assertEqual(opens, [])
        self.assertEqual(detail["probe_attempts"], 1)
        self.assertEqual(mask, 255)

    def test_the_count_is_the_board_s_answer_not_a_host_table_s(self):
        # The whole premise of the scenario, and the thing a host table would
        # quietly replace: three boards, three different answers, one probe.
        for limit, want in ((8, 8), (6, 6), (2, 2)):
            with self.subTest(limit=limit):
                _w, _m, detail, reply, _b, _o = self._probe("mcpwm_pcnt", limit)
                self.assertIn("OK CONFIG", reply)
                self.assertEqual(detail["max_stepper_count"], want)

    def test_the_probe_walks_down_and_stops_at_the_first_acceptance(self):
        wire, mask, detail, reply, board, opens = self._probe("mcpwm_pcnt", 6)
        self.assertIn("OK CONFIG", reply)
        self.assertEqual(mask, (1 << 6) - 1, "the mask must select every stepper")
        self.assertEqual(detail["search_bound"], 8)
        self.assertEqual(detail["probe_attempts"], 3, "8 refused, 7 refused, 6 ok")
        # Descending, one CONFIG per attempt, and the accepted one is last: the
        # run must not connect the same steppers a second time.
        counts = [int(ln.split()[1]) for ln in board.lines]
        self.assertEqual(counts, [8, 7, 6])
        self.assertEqual(wire, f"CONFIG 6 {','.join(['mcpwm_pcnt'] * 6)} nodir")

    def test_every_refusal_above_the_count_is_recorded_with_its_reason(self):
        # The refusal names *which* limit stopped it, and "stops at 6" and
        # "stops at 6 because the analyzer has no channel 7" are different
        # answers. So the transcript travels with the result.
        _w, _m, detail, _r, _b, _o = self._probe("mcpwm_pcnt", 6)
        self.assertEqual([r["n"] for r in detail["refused_above"]], [8, 7])
        for refused in detail["refused_above"]:
            self.assertTrue(refused["reply"].startswith("ERR"), refused)
            self.assertTrue(refused["reply"] != "", refused)
        # And the channel-budget reason is distinguishable from the queue one.
        _w, _m, channels_detail, _r, _b, _o = self._probe("rmt", 99)
        self.assertEqual(channels_detail["max_stepper_count"], 8)
        self.assertEqual(channels_detail["refused_above"], [])

    def test_the_search_starts_from_the_channel_budget_not_a_queue_table(self):
        self.assertEqual(run_tests.max_stepper_count_bound("rmt")[0], 8)
        self.assertEqual(run_tests.max_stepper_count_bound("timer")[0], 8)
        # A multiplexed stepper is a bit of the word, so the eight channels stop
        # being its limit and the 32-bit word takes over.
        count, why = run_tests.max_stepper_count_bound("i2s_mux")
        self.assertEqual(count, 32)
        self.assertIn("32-bit", why)
        # `dir` halves it either way.
        self.assertEqual(
            run_tests.max_stepper_count_bound("rmt", "dir")[0], 4)
        self.assertEqual(
            run_tests.max_stepper_count_bound("i2s_mux", "dir")[0], 16)

    def test_the_bound_is_a_bound_and_not_a_prediction(self):
        # It is the top of a descending search, so it may be above the answer --
        # but never below it, or the probe cannot reach the maximum. The
        # channel budget is the tightest cap the host knows of, and the mux word
        # is the tightest for a multiplexed stepper, because both are enforced
        # by the firmware itself.
        self.assertEqual(run_tests.max_stepper_count_bound("mcpwm_pcnt")[0], 8)
        self.assertGreaterEqual(
            run_tests.max_stepper_count_bound("mcpwm_pcnt")[0],
            harness.driver_max("esp32", "mcpwm_pcnt"))

    def test_measure_connects_once_and_qruns_every_stepper_it_found(self):
        # The end-to-end shape, with the board and the analyzer faked: one
        # CONFIG for the accepted count (the probe's reply is reused rather than
        # a second CONFIG sent), QRUN carrying a mask that selects all of them,
        # and the count in the result record.
        #
        # The double-CONFIG matters and is not cosmetic: CONFIG re-arms the
        # drivers through _initVars(), so a second one for the count the probe
        # just proved allocatable would discard the very measurement.
        sent = []

        class FakeCapture:
            returncode = 0

            def wait(self):
                return 0

            def kill(self):
                pass

        board = self.FakeBoard(6)

        def fake_reply(_ser, line, **_kw):
            return board(None, line)

        def fake_send(_ser, line, **_kw):
            sent.append(line)
            return ""

        info = dict(vf.Dut().info(), ticks_per_s=16_000_000,
                    min_cmd_ticks=3200, max_speed_ticks=640)
        with tempfile.TemporaryDirectory() as tmp:
            args = harness.parse_args(["--arch", "esp32", "--driver",
                                       "mcpwm_pcnt", "--tests", "SR_31",
                                       "--results-dir", tmp,
                                       "--capture-dir", tmp])
            args.dut_driver = args.driver
            probe = run_tests.MaxCountProbe("mcpwm_pcnt")

            def fake_open(*a, **k):
                # An ESP32 reset is what clears the half-connected state a
                # refused CONFIG leaves, so opening the port clears it here.
                board.connected = 0
                return mock.Mock()

            with mock.patch.object(run_tests, "open_board", fake_open), \
                    mock.patch.object(run_tests, "reply_of", fake_reply), \
                    mock.patch.object(run_tests, "send_line", fake_send), \
                    mock.patch.object(run_tests, "drain",
                                      lambda *a, **k: ""), \
                    mock.patch.object(run_tests, "read_map", board.map), \
                    mock.patch.object(run_tests, "read_qinfo",
                                      lambda ser: dict(info)), \
                    mock.patch.object(run_tests, "program",
                                      lambda *a: True), \
                    mock.patch.object(run_tests, "start_capture",
                                      lambda *a, **k: FakeCapture()), \
                    mock.patch.object(run_tests, "load_capture_for_eval",
                                      lambda *a, **k: ({"D0": [0] * 400},
                                                       4_000_000, None)), \
                    mock.patch("time.sleep", lambda _s: None):
                status, detail = run_tests.measure(
                    "t", "sr31", None, 0,
                    run_tests.sc_max_stepper_count, "SR_31", args,
                    scenario="SR_31", probe=probe)
        configs = [ln for ln in sent if ln.startswith("CONFIG")]
        self.assertEqual(status, "failed", "a quiet fake capture fails, which "
                                          "is not what this test is about")
        self.assertEqual(configs, [], "the CONFIGs go through reply_of")
        # Three CONFIG attempts: 8 refused, 7 refused, 6 connected. Each on a
        # reset board, so the third is a real attempt and not an "already".
        self.assertEqual([int(l.split()[1]) for l in board.lines], [8, 7, 6])
        self.assertIn("QRUN 63", sent,
                      "the mask must select every stepper the probe accepted")
        self.assertEqual(detail["max_stepper_count"], 6)
        self.assertEqual(detail["search_bound"], 8)
        self.assertEqual([r["n"] for r in detail["refused_above"]], [8, 7])
        self.assertEqual(detail["ok_but_short"], [])

    def test_a_refused_probe_is_a_refused_run_naming_the_board_s_own_words(self):
        # Not a crash and not a run against steppers that were never connected.
        board = self.FakeBoard(0)
        args = harness.parse_args(["--arch", "esp32", "--driver", "rmt",
                                   "--tests", "SR_31",
                                   "--results-dir", "/tmp/r31",
                                   "--capture-dir", "/tmp/r31"])
        args.dut_driver = args.driver
        probe = run_tests.MaxCountProbe("rmt")
        with mock.patch.object(run_tests, "open_board",
                               lambda *a, **k: mock.Mock()), \
                mock.patch.object(run_tests, "reply_of",
                                  lambda _s, l, **k: board(None, l)), \
                mock.patch.object(run_tests, "read_map", board.map), \
                mock.patch.object(run_tests, "send_line",
                                  lambda *a, **k: ""), \
                mock.patch.object(run_tests, "drain", lambda *a, **k: ""):
            status, detail = run_tests.measure(
                "t", "sr31", None, 0, run_tests.sc_max_stepper_count,
                "SR_31", args, scenario="SR_31", probe=probe)
        self.assertEqual(status, "refused")
        self.assertIn("ERR", detail["error"])
        self.assertEqual(detail["max_stepper_count"], 0)
        self.assertEqual(detail["wire"], f"CONFIG 1 rmt nodir")

    def test_the_mux_search_is_refused_by_the_word_and_not_by_the_channels(self):
        _w, _m, detail, reply, _b, _o = self._probe("i2s_mux", 99, channel_cap=5)
        self.assertIn("OK CONFIG", reply)
        self.assertEqual(detail["max_stepper_count"], 32)
        self.assertEqual(detail["refused_above"], [])

    def test_a_board_that_accepts_nothing_is_a_refusal_with_the_smallest_try(self):
        # Not a crash and not a zero mask: the caller's job is to report the
        # board's own words, and the smallest attempt's refusal is the one that
        # says why even one stepper would not connect.
        wire, mask, detail, reply, _b, _o = self._probe("rmt", 0)
        self.assertNotIn("OK CONFIG", reply)
        self.assertEqual(mask, 0)
        self.assertEqual(detail["max_stepper_count"], 0)
        self.assertEqual(detail["probe_attempts"], 8)
        # Every count down to one, and n=1's refusal is the one that names why.
        self.assertEqual([r["n"] for r in detail["refused_above"]],
                         list(range(8, 0, -1)))
        self.assertEqual(detail["refused_above"][-1]["reply"], reply.strip())
        self.assertEqual(int(wire.split()[1]), 1)

    def test_the_probe_never_sends_a_second_config_for_the_count_it_found(self):
        # One CONFIG per attempt, and the accepted attempt is the run's own.
        # measure() takes the reply `find()` hands back precisely so it does not
        # re-send it; re-CONFIG would reset the drivers the probe just proved
        # can be allocated, which is the measurement.
        _w, mask, _detail, _r, board, _o = self._probe("rmt", 8)
        self.assertEqual(len(board.lines), 1)
        self.assertEqual(mask, 255)

    def test_the_scenario_sends_nodir_not_dir(self):
        # `dir` spends two channels per stepper and stops at 4, which is below
        # MCPWM/PCNT's 6 queues -- so a `dir` max-count run would report the
        # analyzer's channel count as the driver's queue count.
        self.assertEqual(run_tests.scenario_pin_mode("SR_31"), "nodir")
        wire = run_tests.scenario_wire("SR_31", "rmt")
        self.assertTrue(wire.endswith(" nodir"), wire)
        # And it is the only scenario that is not `dir`, which is what makes
        # this a table rather than a constant.
        others = [s for s in run_tests.SCENARIOS
                  if run_tests.scenario_pin_mode(s) != "dir"]
        self.assertEqual(others, ["SR_31"])

    def test_the_mask_in_the_scenario_table_is_not_a_stepper_count(self):
        # 0 means "not known yet", and the probe overwrites it. A literal here
        # would be the catalogue asserting a count it exists to measure.
        _cfg, _builder, mask, _desc = run_tests.SCENARIOS["SR_31"]
        self.assertEqual(mask, 0)
        self.assertEqual(run_tests.PROBE_SCENARIOS, {"SR_31"})

    def test_only_the_max_count_scenario_is_probed(self):
        for scenario in run_tests.SCENARIOS:
            if scenario == "SR_31":
                self.assertIsNotNone(run_tests.probe_for(scenario, "rmt"))
            else:
                self.assertIsNone(
                    run_tests.probe_for(scenario, "rmt"), scenario)

    def test_the_channel_map_covers_every_channel_the_pin_mode_allows(self):
        # The fixture and report path judges the capture through this map, so a
        # map of 2 steppers would score steppers A and B and report the run
        # done while six channels went unread.
        pins = run_tests.Pins.for_scenario("SR_31")
        self.assertEqual(len(pins.map),
                         run_tests.MAX_STEPPERS_PER_MODE["nodir"])
        self.assertEqual([pins.step_of(c) for c in pins.letters],
                         [f"D{i}" for i in range(8)])
        self.assertEqual(pins.dir_of("A"), None, "nodir has no direction pin")
        self.assertEqual(pins.missing({f"D{i}": [] for i in range(8)}), [])
        self.assertEqual(pins.missing({f"D{i}": [] for i in range(7)}), ["H"])

    def test_the_program_is_the_shared_one_a_scale_point_sends(self):
        # One program, one period, every stepper. A per-stepper program would
        # let a driver that matched its own command and dragged the others off
        # pace look correct.
        info = {"min_cmd_ticks": 3200, "max_speed_ticks": 640,
                "ticks_per_s": 16_000_000}
        segs = run_tests.sc_max_stepper_count(info)
        self.assertEqual(len(segs), 1)
        self.assertEqual(segs[0][0], run_tests.SCALE_STEPS)
        self.assertFalse(run_tests.needs_per_stepper("SR_31"),
                         "a per-stepper program would let a driver that "
                         "matched its own command and dragged the others off "
                         "pace look correct")
        # And it is legal on every QINFO shape the harness has met -- the same
        # property the catalogue-wide test asserts, checked here because the
        # step count is this scenario's own.
        for name, qinfo in QINFO_SHAPES.items():
            with self.subTest(driver=name):
                self.assertIsNone(
                    run_tests.unprogrammable(
                        run_tests.sc_max_stepper_count(qinfo), qinfo),
                    name)

    def test_every_scenario_config_resolves_to_a_pin_mode(self):
        # The pin mode is read from a table now, so a scenario added without an
        # entry silently gets `dir` -- which is right for all of them today and
        # wrong for a future `nodir` one. This is the guard that says so.
        self.assertEqual(set(run_tests.SCENARIO_PIN_MODE), {"SR_31"})
        for scenario in run_tests.SCENARIOS:
            mode = run_tests.scenario_pin_mode(scenario)
            self.assertIn(mode, run_tests.CHANNELS_PER_STEPPER, scenario)
            self.assertIn(mode, run_tests.MAX_STEPPERS_PER_MODE, scenario)


class TestModes(unittest.TestCase):
    """The two generic modes (todo R3).

    Neither mode names an architecture, so what is worth testing here is the
    property that makes that true: a plan is a legal, fully-labelled set of
    CONFIG lines and programs, built before anything runs. Every one of these
    is a pure function of the plan arguments -- no board -- so a mode that
    would waste a capture on a point the firmware refuses, or measure a
    different configuration than it names, is caught here rather than by an
    hour of hardware.
    """

    # QINFO values good enough to plan against. Nothing here measures them;
    # these only have to make legal_ticks() produce distinct periods.
    INFO = {"ticks_per_s": 16_000_000, "min_cmd_ticks": 6400,
            "max_speed_ticks": 80, "queue_len": 32,
            "per_stepper_floor": 80}

    def dense(self, events, n):
        """Expand (index, level) changes into the per-sample array evaluators read."""
        s = [0] * n
        for i, (t, v) in enumerate(events):
            end = events[i + 1][0] if i + 1 < len(events) else n
            for j in range(t, min(end, n)):
                s[j] = v
        return s

    def square(self, period, n, offset=1000, high=8):
        out = []
        for i in range(n):
            out.append((i * period + offset, 1))
            out.append((i * period + offset + high, 0))
        return out

    # -- plan shape ------------------------------------------------------
    def test_scale_plan_covers_every_count_from_one(self):
        for pin_mode, cap in (("nodir", 8), ("dir", 4)):
            plan = run_tests.scale_plan("rmt", pin_mode, cap)
            self.assertEqual([p.count for p in plan], list(range(1, cap + 1)),
                             pin_mode)
            self.assertEqual(plan[0].count, 1, "the loop must start at one "
                             "stepper; a sweep that skipped 1 could not show "
                             "whether adding a stepper changed anything")

    def test_scale_run_selects_every_connected_stepper(self):
        # QRUN's mask is a bitmask over stepper slots. A mask that does not
        # cover them all leaves steppers idle, and the run then reports a
        # driver that emitted nothing for a stepper that was never asked to
        # run -- the same silent-wrong-answer shape as a wrong channel map.
        for plan in run_tests.scale_plan("rmt", "nodir", 8):
            self.assertEqual(plan.mask, (1 << plan.count) - 1, plan.label)

    def test_scale_names_the_configuration_it_actually_connects(self):
        # The wire is what the firmware gets, and the label is what the result
        # is filed under. If they disagree the table attributes a measurement
        # to a run that never happened.
        for plan in run_tests.scale_plan("mcpwm_pcnt", "dir", 4):
            tokens = plan.wire.split()
            self.assertEqual(tokens[0], "CONFIG", plan.wire)
            self.assertEqual(int(tokens[1]), plan.count, plan.wire)
            self.assertEqual(tokens[2].split(","), plan.drivers, plan.wire)
            self.assertEqual(tokens[3], plan.pin_mode, plan.wire)
            self.assertIn(f"n{plan.count}", plan.label)

    def test_scale_uses_one_shared_program(self):
        # A per-stepper program here would answer SR_15's question (does each
        # stepper keep its own speed) instead of scale's (does the driver still
        # emit this period with N attached), and the two would be conflated.
        for plan in run_tests.scale_plan("rmt", "nodir", 8):
            self.assertIsNone(plan.per_stepper_builder, plan.label)

    def test_sync_enumerates_combinations_with_repetition(self):
        drivers = ["rmt", "mcpwm_pcnt", "i2s_direct", "i2s_mux"]
        plan = run_tests.sync_plan(drivers, "dir", 2)
        pairs = [tuple(p.drivers) for p in plan]
        # n*(n+1)/2, and no reverse duplicates: rmt+mcpwm is one measurement.
        self.assertEqual(len(pairs), len(drivers) * (len(drivers) + 1) // 2)
        # No reverse duplicate: rmt+mcpwm_pcnt and mcpwm_pcnt+rmt are the same
        # measurement and the same two channels, so running both would report
        # one configuration twice.
        self.assertEqual(len(set(pairs)), len(pairs), pairs)
        for a, b in pairs:
            self.assertNotIn((b, a), set(pairs) - {(a, b)}, (a, b))

    def test_sync_includes_both_same_and_cross_driver_pairs(self):
        # The same-driver pair is the baseline a cross-driver skew is only
        # interpretable against, so a plan with only cross-driver pairs has no
        # reference and the column means nothing. R1's wrong finding was
        # exactly this shape: a "cross-driver" run that was really RMT+RMT.
        plan = run_tests.sync_plan(["rmt", "mcpwm_pcnt"], "dir", 2)
        pairs = {tuple(p.drivers) for p in plan}
        self.assertEqual(pairs, {("rmt", "rmt"),
                                 ("mcpwm_pcnt", "mcpwm_pcnt"),
                                 ("rmt", "mcpwm_pcnt")})

    def test_sync_gives_each_stepper_a_distinct_period(self):
        # Adherence is only checkable if each stepper was given a *different*
        # period. With one shared period a start that dragged every stepper
        # onto one speed would satisfy every check here, which is the exact
        # defect the mode exists to catch.
        for plan in run_tests.sync_plan(["rmt", "mcpwm_pcnt", "i2s_direct"],
                                       "dir", 3):
            programs = plan.per_stepper_builder(self.INFO)
            ticks = [programs[i][0][1] for i in sorted(programs)]
            self.assertEqual(len(ticks), 3, plan.label)
            self.assertEqual(len(set(ticks)), 3,
                             f"{plan.label}: steppers share a period {ticks}, "
                             f"so a rate collapse could not be detected")

    def test_sync_periods_are_legal_commands(self):
        # legal_ticks clamps to MIN_CMD_TICKS/steps; a plan that skipped it
        # would have the firmware refuse the program and the evaluator would
        # never see a waveform at all.
        for plan in run_tests.sync_plan(["rmt", "i2s_direct"], "dir", 2):
            for i, segs in plan.per_stepper_builder(self.INFO).items():
                ticks, steps = segs[0][1], segs[0][0]
                self.assertGreaterEqual(ticks * steps, self.INFO["min_cmd_ticks"],
                                        f"{plan.label} step {i}")

    def test_sync_needs_two_drivers_to_have_a_skew(self):
        with self.assertRaises(ValueError):
            run_tests.sync_plan(["timer"], "dir", 2)

    # -- one name per driver --------------------------------------------
    def test_one_name_per_driver(self):
        # `rmt_v2` used to be a second name for the RMT queue, and `mcpwm` and
        # `i2s` for the other two. No build has both RMT implementations, so
        # the second name could only ever be a spelling; enumerating spellings
        # as drivers would put rmt+rmt in the sync table as a *cross-driver*
        # combination, under a name claiming they differ, when the two are the
        # same run -- the tell that gave away R1's wrong finding (two supposedly
        # different configurations agreeing to four decimal places).
        for family, names in harness.DRIVERS.items():
            self.assertEqual(len(names), len(set(names)),
                             f"{family}: {names}")
        for gone in ("rmt_v2", "mcpwm", "i2s"):
            for family, names in harness.DRIVERS.items():
                self.assertNotIn(gone, names,
                                 f"{gone} is a second name, not a driver")

    def test_sync_plan_has_no_duplicate_run(self):
        drivers = harness.DRIVERS[harness.arch_family("esp32")]
        plan = run_tests.sync_plan(drivers, "dir", 2)
        labels = [p.label for p in plan]
        self.assertEqual(len(set(labels)), len(labels), labels)

    # -- every scenario must be programmable on every driver --------------
    #
    # The QINFO shapes below are the ones this harness has actually met. They
    # differ in `max_speed_ticks` by a factor of eight, which is the whole point:
    # a builder that assumes a fast driver has a fast floor is legal on one and
    # not on the other.
    QINFOS = QINFO_SHAPES

    def _programs(self, scenario, info):
        """Every program a scenario would send, as {stepper: segments}."""
        cfg, builder, _mask, _desc = run_tests.SCENARIOS[scenario]
        segments = builder(info)
        per_stepper = run_tests.per_stepper_builder(scenario)
        return per_stepper(info) if per_stepper else {0: segments}

    def test_every_scenario_is_programmable_on_every_driver_seen(self):
        """The bug this pins, exactly as it happened.

        SR_05 built `max(max_speed_ticks, 160)` instead of legal_ticks(). On
        rmt (floor 640) that gave 16 * 640 = 10240 and passed; on i2s_direct
        (floor 80) it gave 16 * 160 = 2560 against a MIN_CMD_TICKS of 3200, so
        the queue rejected the command and the run was reported as "the pin
        emitted 0 of 16 steps". One expression, two drivers, and the difference
        was invisible until a driver with a lower floor was tried.
        """
        offenders = []
        for scenario in run_tests.SCENARIOS:
            if scenario in run_tests.REJECTION_SCENARIOS:
                continue
            for name, info in self.QINFOS.items():
                for idx, segs in self._programs(scenario, info).items():
                    bad = run_tests.unprogrammable(segs, info)
                    if bad:
                        offenders.append(f"{scenario} on {name} "
                                         f"(stepper {idx}): {bad}")
        self.assertEqual(offenders, [], "unprogrammable scenarios:\n  "
                                        + "\n  ".join(offenders))

    def test_the_only_intentionally_illegal_scenario_is_the_rejection_test(self):
        """The exclusion list is a decision, so it is pinned.

        SR_13 exists to assert that a sub-MIN_CMD_TICKS command emits nothing, so
        it is illegal on purpose. Every other scenario has to be programmable --
        which is the assertion the previous test makes, and this one keeps the
        carve-out from quietly widening.
        """
        for name, info in self.QINFOS.items():
            for scenario in run_tests.REJECTION_SCENARIOS:
                self.assertIn(scenario, run_tests.SCENARIOS)
                bad = [run_tests.unprogrammable(segs, info)
                       for segs in self._programs(scenario, info).values()]
                self.assertTrue(any(b is not None for b in bad),
                                f"{scenario} on {name} is now programmable, so "
                                f"it no longer tests the rejection and should "
                                f"leave REJECTION_SCENARIOS")
        # And every remaining scenario is illegal on at least one driver, or it
        # is not testing what its name claims.
        self.assertEqual(run_tests.REJECTION_SCENARIOS, {"SR_13"})

    def test_the_sweep_covers_a_driver_floor_eight_times_lower(self):
        # Guards the guard: if QINFOS above ever collapsed to one shape, the
        # test above would pass for the wrong reason and quietly stop covering
        # the low-floor driver that broke SR_05.
        floors = {name: info["max_speed_ticks"] for name, info
                  in self.QINFOS.items()}
        self.assertEqual(max(floors.values()) / min(floors.values()), 8.0,
                         floors)

    def test_a_short_entry_is_refused_before_a_capture_is_spent(self):
        # Refusing after the capture costs seconds of analyzer time and leaves
        # only "the pin emitted nothing" to say -- which is what made this look
        # like a hardware fault for so long.
        sent = []
        with mock.patch.object(run_tests, "reply_of",
                               lambda ser, line: sent.append(line) or "OK QCLR"):
            self.assertFalse(run_tests.program(object(),
                                               [(16, 160, True)],
                                               self.QINFOS["i2s_direct"]))
        self.assertEqual(sent, [], "nothing should reach the board")

    def test_a_legal_entry_is_still_sent(self):
        sent = []
        replies = {"QCLR": "OK QCLR", "QSEG 16 640 1": "OK QSEG 1/1"}
        with mock.patch.object(run_tests, "reply_of",
                               lambda ser, line: sent.append(line)
                               or replies[line]):
            self.assertTrue(run_tests.program(object(), [(16, 640, True)],
                                             self.QINFOS["rmt"]))
        self.assertEqual(sent, ["QCLR", "QSEG 16 640 1"])

    def test_the_stop_delay_scales_with_the_fill_not_with_the_program(self):
        """A wall-clock constant is a different depth on every driver.

        The stop has exactly one requirement now that the feeder is stopped after
        the start: land inside the run, which is the fill and nothing more. A
        fixed 1 ms met that on rmt and mcpwm_pcnt and not on i2s_direct, whose
        first step arrives later than 1 ms after QRUN because it streams from a
        DMA buffer -- its marker edge came 16 us *before* the first pulse, so the
        scenario measured a stop that interrupted nothing.

        Scaled to the fill's own duration the same fraction is ~10 ms here, about
        a thousand steps in, and comfortably outside any driver's start latency.
        It also has to be a fraction of the *fill*, not of the program: the
        program is four fills long, so a quarter of it would land past the end of
        the run on any driver.
        """
        info = {"ticks_per_s": 16000000, "min_cmd_ticks": 3200,
                "max_speed_all_ticks": 640}
        for scenario in ("SR_25", "SR_30"):
            for floor in (640, 160, 80):
                dut = dict(info, max_speed_ticks=floor)
                segs = run_tests.sc_emergency_stop(dut)
                when = run_tests.stop_after_for(scenario, segs, dut)
                fill_s = (run_tests.QUEUE_FILL_STEPS * segs[0][1]
                          / info["ticks_per_s"])
                self.assertAlmostEqual(when, fill_s * 0.25, places=6)
                self.assertGreater(when, 0.001,
                                   "must clear every driver's start latency")
                # Inside the run, and not scaled to the program: four fills long,
                # a quarter of *that* would be past the end.
                self.assertLess(when, fill_s)
                self.assertLess(when, run_tests.scenario_seconds(
                    segs, info["ticks_per_s"]))
        # Nothing to stop in an ordinary scenario.
        self.assertIsNone(run_tests.stop_after_for(
            "SR_01", [(8, 640, True)], info))

    def test_the_stop_program_is_four_queue_fills_of_255_step_commands(self):
        """The program has to outlast the queue, or the stop proves nothing.

        QFILL puts 16 * 255 = 4080 steps in the queue before the run, and the
        program is four times that. The program being longer than the fill is
        what makes a run *longer* than the fill a detectable defect rather than
        something the feeder is entitled to do, so two ways this could rot: the
        segments dropping to one, and the command growing past 255 steps per
        entry (which the firmware splits itself, so the fill would no longer be
        16 entries of the size the scenario claims).
        """
        info = {"ticks_per_s": 16000000, "min_cmd_ticks": 3200,
                "queue_len": 32, "max_speed_ticks": 640,
                "max_speed_all_ticks": 640}
        segs = run_tests.sc_emergency_stop(info)
        self.assertEqual(len(segs), 4)
        for steps, ticks, _ in segs:
            self.assertEqual(steps, 16 * 255)
            self.assertLessEqual(steps, 255 * 16)
            self.assertIsNone(run_tests.unprogrammable([(steps, ticks, True)],
                                                      info))
        self.assertEqual(run_tests.requested_steps(segs), 4 * 16 * 255)
        # Four segments is all QE_MAX_SEG takes; a fifth would be refused.
        self.assertLessEqual(len(segs), 8)
        # The fill is what makes the queue bound 4080, and it is a whole queue
        # on AVR -- where QUEUE_LEN is 16 -- so the two boards differ.
        self.assertEqual(run_tests.QUEUE_FILL_STEPS, 4080)

    def test_the_queue_is_filled_before_the_run_and_the_depth_recorded(self):
        """QFILL's reply, not the request, is what the evaluator bounds with.

        QUEUE_LEN is 16 on AVR and 32 on ESP32, and the firmware keeps
        QE_ROOM_RESERVE entries back for a DIR-drain pause, so a request for 16
        entries cannot be met in full everywhere. The depth the board reports is
        recorded in `info`, where the evaluator reads it -- bounding the drain
        with the requested depth instead would assert against a queue state no
        board was ever in.
        """
        sent = []
        with mock.patch.object(
                run_tests, "reply_of",
                lambda ser, line: sent.append(line) or "OK QFILL q=14\n"):
            self.assertEqual(run_tests.fill_queue(object(), 1), 14)
        self.assertEqual(sent, ["QFILL 1 16"])
        # No reply, no fill: 0 is what measure() refuses to capture against.
        with mock.patch.object(run_tests, "reply_of", lambda ser, line: "ERR"):
            self.assertEqual(run_tests.fill_queue(object(), 3), 0)
        self.assertIn("SR_25", run_tests.SCENARIO_FILL)
        self.assertIn("SR_30", run_tests.SCENARIO_FILL)

    def test_the_requested_step_count_sums_every_segment(self):
        """`segments[0][0]` is one segment, not the program.

        The stop scenarios program four identical segments. Reading the first
        would ask for 4080 of 16320 steps and call a complete run a fourfold
        oversupply -- and would call SR_25's *contract* (nothing truncated)
        impossible to satisfy.
        """
        segs = [(4080, 640, True)] * 4
        self.assertEqual(run_tests.requested_steps(segs), 16320)
        self.assertEqual(run_tests.requested_steps([(1, 8, True)]), 1)
        # A pause contributes no steps, only ticks.
        self.assertEqual(run_tests.requested_steps([(0, 1600, True),
                                                    (255, 640, True)]), 255)

    def test_measure_reaches_the_capture_on_both_paths(self):
        """measure() itself, not just its helpers.

        The SR_13 exemption introduced a NameError in this function -- it read
        `test_id`, which is not a parameter here -- and every unit test passed
        because they all drove the helpers directly. Only a hardware run found
        it. So measure() is exercised with serial and capture mocked, on the
        ordinary path and on the rejection scenario: the two branches that can
        disagree about whether a command is legal.
        """
        info = {"ticks_per_s": 16000000, "min_cmd_ticks": 3200,
                "queue_len": 32, "max_speed_ticks": 640,
                "max_speed_all_ticks": 640, "max_speed_per_stepper": [640]}
        replies = {
            "CONFIG 1 rmt dir": "OK CONFIG n=1 mode=dir stride=2 "
                                    "drivers=rmt\n",
            "QCLR": "OK QCLR\n",
            "MAP": "OK MAP count=1 mode=dir stride=2 ch=2\n",
            "QINFO": "OK QINFO tps=16000000 mincmd=3200 qlen=32 maxall=640"
                     " maxspeed0=640\n",
            "QSEG 8 640 1": "OK QSEG 1/1\n",
            "QSEG 1 8 1": "OK QSEG 1/1\n",
            "QRUN 1": "OK QRUN\n",
            "POS": "POS 8\n",
            "STOP": "OK STOP\n",
        }
        args = argparse.Namespace(
            capture_dir="/tmp/none", results_dir="/tmp/none",
            sample_rate=4000000, port="/dev/null", baud=115200,
            dut_driver="rmt", seconds=1.0, sr00_sample_rate=1000000,
            force=True)
        outcomes = {}
        sent_lines = []

        fixture = next(fx for fx in vf.FIXTURES if fx.scenario == "SR_01")

        def drive(scenario, builder, post_qrun="OK QRUN\nPOS 8\n",
                  capture=None):
            # SR_13 asserts the *absence* of pulses, so it needs a capture with
            # no edges at all -- the SR_01 fixture has eight and would fail it.
            capture = capture if capture is not None else sp.load_vcd(
                str(vf.FIXTURE_DIR / f"{fixture.name}.vcd"))
            with mock.patch.object(run_tests, "reply_of",
                                   lambda ser, line: (
                                       sent_lines.append(line),
                                       replies.get(line, "OK"))[1]), \
                    mock.patch.object(run_tests, "drain",
                                      lambda *a, **k: post_qrun), \
                    mock.patch.object(run_tests, "read_qinfo",
                                      lambda ser: dict(info)), \
                    mock.patch.object(run_tests, "read_map",
                                      lambda ser: (
                                          run_tests.default_channel_map(1, 2),
                                          {"mode": "dir", "stride": 2,
                                           "pins": [2, 0]})), \
                    mock.patch.object(run_tests, "start_capture",
                                      lambda *a, **k: mock.Mock(
                                          wait=lambda: None,
                                          returncode=0)), \
                    mock.patch.object(run_tests.time, "sleep",
                                      lambda *a: None), \
                    mock.patch.object(run_tests, "open_board",
                                      lambda *a, **k: mock.Mock()), \
                    mock.patch.object(
                        run_tests, "load_capture_for_eval",
                        _two_or_three(capture)):
                outcomes[scenario] = run_tests.measure(
                    "t", scenario.lower(), "CONFIG 1 rmt dir", 1, builder,
                    scenario, args, None, scenario=scenario)

        drive("SR_01", lambda i: [(8, i["max_speed_ticks"], True)])
        # The rejection scenario must still reach the board: exempting it from
        # the legality pre-check is the point, and a mistake here would make it
        # "pass" by never programming anything at all.
        # The rejection arrives in the post-QRUN drain, from qe_pump in the main
        # loop -- which is exactly why program()'s "OK QSEG" check cannot see it.
        drive("SR_13", lambda i: [(1, 8, True)],
              post_qrun="ERR QE step0 rc=-1\nOK QRUN\nDONE 0\nPOS 0\n",
              capture=({"D0": [0, 0, 0, 0], "D1": [0, 0, 0, 0]}, 4000000))
        self.assertEqual(outcomes["SR_01"][0], "passed")
        self.assertEqual(outcomes["SR_13"][0], "passed")
        self.assertIn("POS 0", outcomes["SR_13"][1].get("reply", ""))
        # And the ordinary path must still *program*: a run that refuses to send
        # anything would report zero steps and look like a dead pin.
        self.assertIn("QSEG 8 640 1", sent_lines)

    def test_a_pause_counts_one_period_against_the_minimum(self):
        # A pause's tick field is its own bound, not ticks*steps, so it is a
        # separate case with the same threshold.
        info = self.QINFOS["i2s_direct"]
        self.assertIsNotNone(run_tests.unprogrammable([(0, 1600, True)], info))
        self.assertIsNone(run_tests.unprogrammable([(0, 3200, True)], info))

    # -- queue entries the board may silently discard --------------------
    def test_an_entry_shorter_than_min_cmd_ticks_is_named_in_the_result(self):
        """A scenario measuring zero steps has to say why.

        Measured on the ESP32 with i2s_direct: a 195 us entry produces zero
        pulses, a 200 us entry produces every one, and 200 us is exactly
        MIN_CMD_TICKS (3200 ticks at 16 MHz). Three 40 us entries totalling
        240 us also produce nothing, so the limit is per entry and not on the
        program as a whole.

        Without the note a zero-step result reads as a dead pin or a driver
        that emits nothing -- and neither is true: the same pin carries every
        longer move perfectly.
        """
        self.assertEqual(run_tests.sub_min_entries(
            [(16, 160, True)], {"ticks_per_s": 16_000_000,
                                "min_cmd_ticks": 3200}),
            [{"steps": 16, "ticks": 160, "us": 160.0}])
        # Exactly at the threshold is not below it.
        self.assertEqual(run_tests.sub_min_entries(
            [(20, 160, True)], {"ticks_per_s": 16_000_000,
                                "min_cmd_ticks": 3200}), [])
        # A pause counts one period, and the threshold applies to it too.
        self.assertEqual(run_tests.sub_min_entries(
            [(0, 1600, True)], {"ticks_per_s": 16_000_000,
                                "min_cmd_ticks": 3200}),
            [{"steps": 0, "ticks": 1600, "us": 100.0}])

    def test_per_stepper_programs_are_checked_for_short_entries_too(self):
        # SR_15 is the one scenario with per-stepper programs, and its steppers
        # are short by design. Checking only the shared program would let the
        # same silent drop through on the other half of the run.
        # One short entry and one long one, so the filter is exercised per entry
        # rather than per program -- checking only the shared program would let
        # the same silent drop through on the other half of the run.
        entries = run_tests.sub_min_entries(
            None, {"ticks_per_s": 16_000_000, "min_cmd_ticks": 3200},
            programs={0: [(4, 160, True)], 1: [(4, 1600, True)]})
        self.assertEqual(entries, [{"steps": 4, "ticks": 160, "us": 40.0}])

    # -- DRIVERS: asking the board what it has ----------------------------
    def test_drivers_is_read_as_name_value_pairs_not_by_position(self):
        # The set of names is build-dependent -- an AVR build emits only
        # `timer`, a Pico `timer` and `pio` -- so a positional parse reads a
        # different field as each driver on each target.
        esp = "OK DRIVERS mux=0 rmt=1 rmt=1 mcpwm_pcnt=1 i2s_direct=1 " \
              "i2s_mux=1 mux_init=0"
        avr = "OK DRIVERS mux=0 timer=1 mux_init=0"
        pico = "OK DRIVERS mux=0 timer=1 pio=1 mux_init=0"
        for text, expected in ((esp, {"rmt", "rmt", "mcpwm_pcnt",
                                      "i2s_direct", "i2s_mux"}),
                               (avr, {"timer"}),
                               (pico, {"timer", "pio"})):
            m = run_tests.DRIVERS_RE.search(text)
            self.assertIsNotNone(m, text)
            present = {k for k, v in re.findall(r"(\w+)=(\d)", m.group(2))
                       if v == "1"}
            self.assertEqual(present, expected, text)

    def test_a_mux_compiled_in_but_not_brought_up_is_distinguishable(self):
        """"Present" and "up" are different, and only one is enough to connect.

        Without the distinction a planner reads i2s_mux=1, builds a run, and
        every CONFIG naming it is refused -- which looks like a contradiction
        between two firmware replies rather than a pin assignment not made yet.
        """
        up = run_tests.DRIVERS_RE.search(
            "OK DRIVERS mux=1 rmt=1 rmt=1 mcpwm_pcnt=1 i2s_direct=1 "
            "i2s_mux=1 mux_init=1")
        down = run_tests.DRIVERS_RE.search(
            "OK DRIVERS mux=0 rmt=1 rmt=1 mcpwm_pcnt=1 i2s_direct=1 "
            "i2s_mux=1 mux_init=0")
        self.assertEqual(up.group(3), "1")
        self.assertEqual(down.group(3), "0")

    def test_read_drivers_parses_every_driver_not_just_the_first(self):
        """The function's own parsing, not just the regex the tests read.

        The regex tests above assert on DRIVERS_RE directly, which left
        read_drivers()'s parse of it untested -- so truncating the field list to
        the first two drivers broke nothing. Every driver this build reports has
        to reach the host, or a planner will read "i2s_mux absent" from a board
        that has it.
        """
        reply = ("OK DRIVERS mux=0 rmt=1 rmt=1 mcpwm_pcnt=1 i2s_direct=1 "
                 "i2s_mux=1 mux_init=0")
        with mock.patch.object(run_tests, "reply_of", lambda *a: reply):
            present, mux_init = run_tests.read_drivers(object())
        self.assertEqual(present, {"rmt": True, "rmt": True,
                                   "mcpwm_pcnt": True, "i2s_direct": True,
                                   "i2s_mux": True})
        self.assertFalse(mux_init)

    def test_read_drivers_reports_a_mux_that_is_up_as_up(self):
        # Reported-always-up would tell the planner the mux needs no pin
        # assignment, and every CONFIG naming it would then be refused.
        reply = ("OK DRIVERS mux=1 rmt=1 rmt=1 mcpwm_pcnt=1 i2s_direct=1 "
                 "i2s_mux=1 mux_init=1")
        with mock.patch.object(run_tests, "reply_of", lambda *a: reply):
            _present, mux_init = run_tests.read_drivers(object())
        self.assertTrue(mux_init)

    def test_read_drivers_survives_the_boot_log_in_front_of_the_reply(self):
        """Not hypothetical: the first DRIVERS after a reset arrives behind the
        ESP32's own boot banner, which is several hundred bytes of unrelated
        text. A parser that did not retry would read the wrong thing here."""
        boot = ("v:0x00\r\nmode:div:2\r\nload:0x3fff0030,len:4688\r\n"
                "READY\r\nDONE 0\r\nOK DRIVERS mux=0 rmt=1 rmt=1 "
                "mcpwm_pcnt=1 i2s_direct=1 i2s_mux=1 mux_init=0\r\n")
        replies = iter([boot, boot])
        with mock.patch.object(run_tests, "reply_of",
                               lambda *a: next(replies)), \
                mock.patch.object(run_tests.time, "sleep",
                                  lambda *a: None):
            present, mux_init = run_tests.read_drivers(object())
        self.assertTrue(present["i2s_mux"])
        self.assertFalse(mux_init)

    def test_read_drivers_raises_rather_than_guessing_when_nobody_answers(self):
        # The failure mode this whole item removes. If the board cannot be asked,
        # the old answer was the table -- and a silent fallback to the table is
        # how the wrong 6 got believed in the first place.
        with mock.patch.object(run_tests, "reply_of",
                               lambda *a: "OK QCLR"), \
                mock.patch.object(run_tests.time, "sleep",
                                  lambda *a: None):
            with self.assertRaises(RuntimeError):
                run_tests.read_drivers(object())

    def test_imux_is_sent_once_and_its_failure_is_not_retried(self):
        # initI2sMux() cannot run twice, so a retry loop would turn a first
        # success into a later failure.
        sent = []

        def reply(ser, line):
            sent.append(line)
            return "OK IMUX ch=5,6,7 pin=5,18,19\n" \
                if len(sent) == 1 else "ERR IMUX already up\n"

        with mock.patch.object(run_tests, "reply_of", reply):
            self.assertTrue(run_tests.send_imux(object()))
            self.assertFalse(run_tests.send_imux(object()))
        # No pins in the command, and no retry: the bus is the last three
        # analyzer channels and their GPIOs come from the firmware's own channel
        # table, so there is nothing for the host to name -- and a host that
        # named them would be naming this rig's cable.
        self.assertEqual(sent, ["IMUX", "IMUX"])

    @staticmethod
    def _plan_args(**over):
        base = dict(arch="esp32", framework="arduino", version="latest",
                    driver="rmt", pin_mode="nodir", count=4, speed_us=40,
                    sample_rate=0, mode=None, tests=None, drivers=None,
                    capture_seconds=None, tag_key=None)
        base.update(over)
        return argparse.Namespace(**base)

    def test_a_mux_run_is_sampled_fast_enough_to_read_the_bit_clock(self):
        # The I2S bus runs at 8 MHz and the 32-bit word is read out of it, so a
        # mux run is sampled for the bit clock and not for the step period.
        # Nothing else about it needs a fast rate -- the step signals are
        # ordinary -- so this floor is specific to it and would otherwise never
        # be applied. 40 us steps ask for 500 kHz on their own.
        _, _, _, rate = harness.derive(self._plan_args(driver="i2s_mux"))
        self.assertGreaterEqual(rate, harness.MUX_MIN_SAMPLE_RATE)
        self.assertGreaterEqual(rate / harness.MUX_BIT_CLOCK_HZ,
                                harness.MUX_MIN_SAMPLES_PER_BIT)

        # The same run without the mux keeps the rate the step period needs.
        _, _, _, plain = harness.derive(self._plan_args(driver="rmt"))
        self.assertLess(plain, harness.MUX_MIN_SAMPLE_RATE)

        # And a run that asks for too little is raised, not refused: the mux has
        # its own floor and nothing else in the plan would notice.
        _, _, _, low = harness.derive(
            self._plan_args(driver="i2s_mux", sample_rate=1_000_000))
        self.assertEqual(low, harness.MUX_MIN_SAMPLE_RATE)

    def test_the_mux_rate_is_capped_below_the_analyzers_best_resolution(self):
        # 48 MS/s resolves the bit clock better and is worse: this analyzer
        # truncates an eight-channel 48 MS/s capture to 0.18 ms, and a scenario
        # lasts milliseconds, so the capture covers a run that has not started.
        # A floor without a ceiling is how that gets chosen.
        self.assertLess(harness.MUX_MIN_SAMPLE_RATE, 48_000_000)
        # ...and the ceiling is the analyzer's truncation, not a preference.
        _, _, _, rate = harness.derive(
            self._plan_args(driver="i2s_mux", sample_rate=48_000_000))
        self.assertEqual(rate, harness.MUX_MIN_SAMPLE_RATE)

    def test_a_mux_tag_records_that_the_channels_were_decoded(self):
        # A physical 4-stepper nodir run and a 4-slot mux run both read D0..D3
        # after decoding, and their waveforms come from completely different
        # hardware. Without `/mux` in the tag, half the mux results read as
        # physical ones.
        mux_tag = harness.derive(self._plan_args(driver="i2s_mux"))[0]
        phy_tag = harness.derive(self._plan_args(driver="rmt"))[0]
        self.assertNotEqual(mux_tag, phy_tag)
        self.assertTrue(mux_tag.endswith("4_mux"), mux_tag)
        self.assertTrue(phy_tag.endswith("4_nodir"), phy_tag)

    def test_a_mux_run_may_ask_for_more_steppers_than_channels(self):
        # The point of the decoder: 32 steppers, three wires. The refusal for
        # the other direction has to stay a refusal.
        args = self._plan_args
        self.assertTrue(harness.derive(args(driver="i2s_mux", count=32))[0])
        with self.assertRaises(SystemExit):
            harness.derive(args(driver="rmt", count=32))
        # And the word still bounds it: 32 is all there is.
        with self.assertRaises(SystemExit):
            harness.derive(args(driver="i2s_mux", count=33))

    def test_a_mux_dir_run_is_bounded_by_the_word_not_the_channels(self):
        # 16, because a direction signal is a second bit of the same 32 -- even
        # though the analyzer could afford far more physical pairs.
        args = lambda **kw: self._plan_args(pin_mode="dir", driver="i2s_mux",
                                            **kw)
        self.assertTrue(harness.derive(args(count=16))[0])
        with self.assertRaises(SystemExit):
            harness.derive(args(count=17))

    def test_a_mode_tag_is_still_a_filename_at_32_steppers(self):
        # The tag lands in the capture's filename, the result's filename and the
        # decoded capture's filename. Spelling the driver out once per stepper
        # made that 32 x 8 characters for a mux run, which with the prefix and the
        # `_37ch` suffix the decoder appends passes 255 -- and the capture is
        # then simply not written, with the failure surfacing as an unrelated
        # "sr -> vcd conversion failed".
        for driver, mode, bound in (("i2s_mux", "nodir", 32),
                                    ("rmt", "dir", 4)):
            for plan in run_tests.scale_plan(driver, mode, bound):
                name = f"esp32_arduino_{driver}_scale{mode}_{plan.tag}"
                self.assertLessEqual(len(name) + len("_37ch.vcd"), 255,
                                     name)

    def test_a_mode_tag_names_each_driver_once(self):
        # ...and it still says which drivers, because a tag that cannot tell
        # one driver from two of the same characterizes nothing.
        self.assertEqual(
            run_tests.scale_plan("rmt", "dir", 4)[1].tag, "rmtdirn2")
        tags = {p.tag for p in run_tests.sync_plan(["rmt", "mcpwm_pcnt"],
                                                   "dir", 2)}
        self.assertEqual(tags, {"rmtdirn2", "mcpwm_pcntdirn2",
                                "rmt+mcpwm_pcntdirn2"})

    # -- the bound a scale run stops at ----------------------------------
    def test_the_sweep_limit_is_the_channel_budget_not_a_predicted_count(self):
        # The limit is the analyzer's budget and nothing else. It used to be
        # min(that, a DRIVER_MAXS entry) -- a copy of the library's declared
        # QUEUES_* constant. That constant is accurate about allocations (this
        # board really does allocate six MCPWM queues, refusing the seventh at
        # CONFIG) and silent about health: one of the six runs. A sweep bounded
        # by it reported "MCPWM reaches 6" directly above five runaways.
        for arch, driver, pin_mode, expected in (
                ("esp32", "rmt", "dir", 4),
                ("esp32", "rmt", "nodir", 8),
                ("esp32", "mcpwm_pcnt", "nodir", 8),   # not 6: measured
                ("esp32", "i2s_direct", "dir", 4),
                # The 328P is the clearest case: the analyzer affords 4 in
                # `dir`, and the board connects 2. The sweep now runs to 4 and
                # the firmware refuses 3 and 4 -- which is a better result than
                # the table's answer of 2, because it is the board's answer.
                ("nanoatmega328", "timer", "dir", 4),
                ("rpipico", "pio", "nodir", 8)):
            self.assertEqual(harness.scale_bound(arch, driver, pin_mode)[0],
                             expected, f"{arch}/{driver}/{pin_mode}")
            self.assertIn("channels",
                          harness.scale_bound(arch, driver, pin_mode)[1])

    def test_the_budget_may_exceed_what_the_board_connects(self):
        """The sweep limit and the driver's real reach are different numbers.

        Pinned because the temptation to restore the smaller of the two is
        exactly the bug: a sweep that stops where the driver stops reports
        "the driver reached N", when all it established is that it stopped
        looking.
        """
        for arch, driver, pin_mode, board_reaches in (
                ("esp32", "mcpwm_pcnt", "nodir", 1),
                ("nanoatmega328", "timer", "dir", 2)):
            budget = harness.scale_bound(arch, driver, pin_mode)[0]
            self.assertGreater(budget, board_reaches,
                               f"{arch}/{driver}: budget {budget} does not "
                               f"exceed the measured {board_reaches}, so this "
                               f"test would not notice the limit creeping back")

    def test_a_host_table_entry_cannot_change_the_sweep_limit(self):
        # Even an absurd entry is only reported, never obeyed. If a claim could
        # set the limit, the claim would again be the answer to the mode's
        # question, which is the whole thing being removed.
        with mock.patch.dict(harness.DRIVER_MAXS,
                             {"esp32": {"rmt": 1, "brand_new": 64}}):
            self.assertEqual(harness.scale_bound("esp32", "rmt", "nodir")[0],
                             8)
            self.assertEqual(harness.scale_bound("esp32", "brand_new",
                                                 "nodir")[0], 8)
            bound = harness.scale_bound("esp32", "rmt", "nodir")[1]
        self.assertIn("host table believed 1", bound)

    def test_an_unknown_driver_no_longer_stops_the_sweep(self):
        # It used to raise, on the grounds that a guessed bound is worse than
        # none. But the loop limit is the rig's channel budget, which is a fact
        # and not a guess, and where the driver stops is the board's to say.
        # Refusing to plan for a driver with no table entry would mean the table
        # still governs which runs are possible -- exactly the coupling being
        # removed.
        self.assertEqual(harness.scale_bound("esp32", "nonexistent", "dir")[0],
                         4)
        self.assertEqual(harness.scale_bound("not_an_arch", "rmt", "dir")[0], 4)
        self.assertIsNone(harness.driver_max("esp32", "nonexistent"))

    def test_scale_bound_agrees_with_the_librarys_queue_counts(self):
        # Cross-check the table against the library's own QUEUES_* values, so
        # it cannot drift away from the build it plans for.
        root = SCRIPTS.parents[3] / "src" / "pd_esp32"
        idf5 = (root / "pd_config_idf5.h").read_text()

        def queues(target, name):
            block = idf5.split(f"CONFIG_IDF_TARGET_{target}")[1]
            for line in block.splitlines():
                if line.startswith(f"#define QUEUES_{name} "):
                    return int(line.split()[2])
            self.fail(f"no QUEUES_{name} for {target}")

        self.assertEqual(harness.driver_max("esp32", "rmt"),
                         queues("ESP32", "RMT"))
        self.assertEqual(harness.driver_max("esp32", "mcpwm_pcnt"),
                         queues("ESP32", "MCPWM_PCNT"))
        self.assertEqual(harness.driver_max("esp32s2", "rmt"),
                         queues("ESP32S2", "RMT"))
        self.assertEqual(harness.driver_max("esp32c3", "rmt"),
                         queues("ESP32C3", "RMT"))
        # Under dynamic allocation I2S mux is 32 and I2S direct is
        # SOC_I2S_NUM (2 on the ESP32 -- measured: n=1,2 connect, n=3 is refused
        # inside i2s_new_channel()). A multiplexed stepper spends a slot of the
        # 32-bit word and no analyzer channel, so the channel budget no longer
        # caps it -- the word does. That is the whole reason the decoder exists:
        # 32 steppers on a rig whose steppers are the analyzer's channels, with
        # three of the eight carrying the bus.
        self.assertEqual(harness.driver_max("esp32", "i2s_mux"), 32)
        self.assertEqual(harness.driver_max("esp32", "i2s_direct"), 2)
        self.assertEqual(harness.scale_bound("esp32", "i2s_mux", "nodir")[0], 32)
        self.assertEqual(harness.scale_bound("esp32", "i2s_mux", "dir")[0], 16)
        # A PHYSICAL stepper beside the same bus is still capped by the five
        # channels left over: 5 in nodir, 2 in dir.
        self.assertEqual(harness.max_steppers("nodir", bus=True), 5)
        self.assertEqual(harness.max_steppers("dir", bus=True), 2)
        self.assertEqual(harness.max_steppers("nodir"), 8)
        # And the word bounds mux `dir` at 16, not 32: a direction signal is a
        # second bit of the same word.
        self.assertEqual(
            harness.mux_slot_bound("dir"), 16)
        self.assertEqual(harness.mux_slot_bound("nodir"), 32)

    def test_driver_max_is_known_for_every_architecture_offered(self):
        # `--arch` accepts these, so a scale run on any of them must be able
        # to say how far it goes. An architecture whose limit is missing is a
        # mode that cannot run at all.
        for arch in harness.ARCHS:
            for driver in harness.DRIVERS[harness.arch_family(arch)]:
                self.assertIsNotNone(
                    harness.driver_max(arch, driver),
                    f"{arch}/{driver} has no DRIVER_MAXS entry")

    # -- what the evaluators actually catch -----------------------------
    # The evaluator tests synthesize waveforms at 1 sample == 1 us, so their
    # DUT tick rate has to be 1 MHz for "160 ticks" to mean 160 samples. Using
    # the ESP32's 16 MHz here would ask for a 10 us period from a waveform that
    # measures 160 us, and every case would fail for the wrong reason.
    EVAL_INFO = {"ticks_per_s": 1_000_000, "min_cmd_ticks": 100,
                 "max_speed_ticks": 160, "queue_len": 32,
                 "per_stepper_floor": 160}

    def _eval_sync(self, a, b, chan_map=None, count=2):
        """eval_sync on two rendered stepper channels, 1 sample == 1 us.

        The map is passed in and nothing is set globally -- which is the point
        of R4. These tests previously saved, overwrote and restored two module
        globals, so they could only ever test one map at a time and would
        silently judge a `nodir` capture with the `dir` table if two of them ran
        together.
        """
        n = 20000
        channels = {"D0": self.dense(a, n)}
        for i in range(1, 4):
            channels[f"D{i}"] = [0] * n
        channels["D2"] = self.dense(b, n)
        pins = run_tests.Pins(chan_map or run_tests.default_channel_map(2, 2))
        return run_tests.eval_sync(channels, 1_000_000,
                                   [(40, 160, True)], self.EVAL_INFO, pins,
                                   {0: [(40, 160, True)],
                                    1: [(40, 320, True)]})

    def test_sync_accepts_each_stepper_at_its_own_period(self):
        ok, detail = self._eval_sync(self.square(160, 40),
                                     self.square(320, 40, offset=1100))
        self.assertTrue(ok, detail)
        self.assertAlmostEqual(
            detail["per_stepper"]["A"]["mean_period_us"], 160.0, places=3)
        self.assertAlmostEqual(
            detail["per_stepper"]["B"]["mean_period_us"], 320.0, places=3)

    def test_sync_catches_a_stepper_collapsed_onto_another_s_period(self):
        # The defect the distinct periods exist to expose: both steppers
        # stepped, the start was aligned, and one was running at the wrong
        # speed. With a shared expected period this would pass.
        ok, detail = self._eval_sync(self.square(160, 40),
                                     self.square(160, 40, offset=1100))
        self.assertFalse(ok, "a rate collapse passed a sync run")
        self.assertFalse(detail["per_stepper"]["B"]["period"]["ok"])

    def test_sync_catches_a_stepper_that_lost_steps(self):
        ok, detail = self._eval_sync(self.square(160, 40),
                                     self.square(320, 40, offset=1100)[:-400])
        self.assertFalse(ok, "a lost step passed a sync run")
        self.assertFalse(detail["per_stepper"]["B"]["steps"]["ok"])

    def test_sync_reports_skew_but_does_not_gate_on_it(self):
        # A stepper starting 40 us late has not malfunctioned; it is a
        # platform characteristic. It is reported in us AND in step periods,
        # because 40 us is nothing on one period and enormous on another.
        ok, detail = self._eval_sync(self.square(160, 40),
                                     self.square(320, 40, offset=1040))
        self.assertTrue(ok, "skew was gated on: " + repr(detail))
        self.assertAlmostEqual(detail["first_step_skew_us"], 40.0, places=3)
        self.assertAlmostEqual(detail["skew_periods"], 0.25, places=3)

    def _eval_scale(self, waveforms):
        n = 20000
        channels = {}
        for i, w in enumerate(waveforms):
            channels[f"D{i}"] = self.dense(w, n)
        pins = run_tests.Pins(run_tests.default_channel_map(len(waveforms), 1))
        return run_tests.eval_scale(channels, 1_000_000,
                                   [(40, 160, True)], self.EVAL_INFO, pins)

    def test_scale_accepts_every_stepper_at_the_shared_period(self):
        ok, detail = self._eval_scale([self.square(160, 40)
                                       for _ in range(4)])
        self.assertTrue(ok, detail)
        self.assertEqual(detail["stepper_count"], 4)
        self.assertEqual(detail["period_spread_us"], 0.0)

    def test_scale_catches_one_stepper_off_period(self):
        # The point of scale: stepper D runs at another stepper's rate while
        # the others are correct, so a check that only looked at the first
        # stepper would pass.
        ok, detail = self._eval_scale([self.square(160, 40), self.square(160, 40),
                                       self.square(160, 40), self.square(320, 40)])
        self.assertFalse(ok, "an off-period stepper passed a scale run")
        self.assertFalse(detail["per_stepper"]["D"]["period"]["ok"])

    def test_scale_catches_a_stepper_that_never_stepped(self):
        ok, detail = self._eval_scale([self.square(160, 40), self.square(160, 40),
                                       self.square(160, 40), []])
        self.assertFalse(ok, "a silent stepper passed a scale run")
        self.assertEqual(detail["per_stepper"]["D"]["steps"]["steps_measured"], 0)


class TestPerStepperPrograms(unittest.TestCase):
    """The scenarios that need one indexed QSEG program per stepper.

    SR_15 is the only one, and it is the only one because it is the only one
    that gives two steppers *different* periods. Both halves of that have been
    wrong in ways no test caught, so they are pinned here:

    * The runner used to answer "does this need per-stepper programs?" by
      calling `per_stepper_programs(scenario, None)`, which dereferences the
      QINFO dict and raised `TypeError`. SR_15 could therefore never run.
    * SR_15 derives its slow stepper from `max_speed_ticks * ratio`, and
      `legal_ticks()` has a 160-tick floor. The ESP32's real RMT floor is 80,
      so 80 and 160 both clamp to 160 and the ratio collapses to **1.0** -- the
      scenario would send two identical periods and report success. The fixture
      DUT's floor is 640, where the ratio survives by accident, which is why the
      recorded result was never a statement about the real chip.
    """

    REAL_ESP32 = {"min_cmd_ticks": 3200, "max_speed_ticks": 80,
                  "ticks_per_s": 16_000_000, "queue_len": 32}
    FIXTURE_DUT = {"min_cmd_ticks": 3200, "max_speed_ticks": 640,
                  "ticks_per_s": 16_000_000, "queue_len": 32}

    def test_sr15_is_the_only_scenario_needing_them(self):
        self.assertEqual(run_tests.PER_STEPPER_SCENARIOS, {"SR_15"})
        self.assertTrue(run_tests.needs_per_stepper("SR_15"))
        self.assertIsNotNone(run_tests.per_stepper_builder("SR_15"))
        for sid in run_tests.SCENARIOS:
            if sid == "SR_15":
                continue
            self.assertFalse(run_tests.needs_per_stepper(sid), sid)
            self.assertIsNone(run_tests.per_stepper_builder(sid), sid)

    def test_asking_needs_no_qinfo(self):
        # The runner asks before the board is wired, so this must not need an
        # info dict. It used to, and every catalogue run that reached SR_15
        # raised TypeError on the way.
        for sid in run_tests.SCENARIOS:
            run_tests.per_stepper_builder(sid)
        self.assertIsNone(run_tests.per_stepper_programs("SR_01", None))

    def test_sr15_ratio_survives_on_the_real_chip(self):
        for label, info in (("real ESP32 RMT", self.REAL_ESP32),
                            ("fixture DUT", self.FIXTURE_DUT)):
            programs = run_tests.per_stepper_programs("SR_15", info)
            a, b = programs[0][0][1], programs[1][0][1]
            self.assertEqual(b / a, run_tests.SR_15_RATIO,
                             f"{label}: A={a} B={b} -- the ratio collapsed to "
                             f"{b / a}, so both steppers ran at one speed and "
                             f"the scenario measured nothing")

    def test_sr15_periods_are_legal_at_the_real_chip_floor(self):
        programs = run_tests.per_stepper_programs("SR_15", self.REAL_ESP32)
        for idx, segs in programs.items():
            steps, ticks = segs[0][0], segs[0][1]
            self.assertGreaterEqual(ticks * steps,
                                    self.REAL_ESP32["min_cmd_ticks"],
                                    f"stepper {idx} would be refused")
            self.assertLessEqual(ticks, 65535, f"stepper {idx}")

    def test_every_synced_plan_keeps_its_ratio_on_the_real_chip(self):
        # sync_plan already derives its ladder from the clamped base; this pins
        # that at the real floor, where the SR_15 bug lived.
        for plan in run_tests.sync_plan(["rmt", "mcpwm_pcnt", "i2s_direct"],
                                        "dir", 3):
            programs = plan.per_stepper_builder(self.REAL_ESP32)
            ticks = [programs[i][0][1] for i in sorted(programs)]
            self.assertEqual(len(set(ticks)), 3,
                             f"{plan.label}: {ticks} -- a rate collapse could "
                             f"not be detected if two steppers share a period")


class TestMuxChannelMap(unittest.TestCase):
    """A multiplexed stepper is not on a channel at all.

    Its step signal is one bit of the 32-bit word the I2S bus carries, so it has
    no wire and no GPIO. The map has to name the *decoded* channel it becomes --
    S<slot> -- or the evaluators are handed a D channel, read whichever physical
    pin happens to own it, find nothing, and report a driver that emits nothing.
    That is the same silent wrong map the physical cases below guard against,
    one level of indirection further out.
    """

    BUS = " bus=5,6,7"

    def _read(self, reply):
        with mock.patch.object(run_tests, "reply_of", lambda ser, line: reply):
            return run_tests.read_map(object())

    def test_a_mux_stepper_maps_to_its_decoded_slot(self):
        chan_map, pins = self._read(
            f"MAP count=4 mode=nodir stride=1 ch= bus=5,6,7 "
            f"slots=0,1,2,3 marker=255\n")
        self.assertEqual(chan_map, {
            "A": {"step": "S0"}, "B": {"step": "S1"},
            "C": {"step": "S2"}, "D": {"step": "S3"},
        })
        self.assertEqual(pins["bus"], [5, 6, 7])
        # No analyzer channel was spent on them, which is the whole point.
        self.assertEqual(pins["pins"], [])
        self.assertEqual(pins["physical_channels"], 0)

    def test_a_mux_dir_stepper_maps_to_two_slots(self):
        # Direction on the mux is a second bit of the SAME word, so it is a
        # second slot rather than a second channel. That is what caps mux `dir`
        # at 16 steppers and not 32.
        #
        # `slots=0,2 dslots=1,3` is what the board sends for `CONFIG 2 i2s_mux,
        # i2s_mux dir` -- two entries in each field, one per STEPPER. The step
        # field used to be read as one entry per CHANNEL (four of them), which
        # agrees with itself in `nodir` and runs off the end in `dir`: every
        # stepper past the first was handed a GPIO channel and reported as
        # emitting nothing. The direction bit was then derived as
        # `step_slot + 1`, which held only while allocation stayed gapless.
        chan_map, pins = self._read(
            f"MAP count=2 mode=dir stride=2 ch= bus=5,6,7 "
            f"slots=0,2 dslots=1,3 marker=255\n")
        self.assertEqual(chan_map, {
            "A": {"step": "S0", "dir": "S1"},
            "B": {"step": "S2", "dir": "S3"},
        })
        self.assertEqual(pins["slots"], [0, 2])
        self.assertEqual(pins["dslots"], [1, 3])

    def test_the_direction_slot_is_read_not_derived(self):
        # The point of the `dslots` field: a non-gapless allocation must not be
        # misread. Here stepper A holds bit 5 and stepper B bit 2 -- B was
        # connected second but got a lower bit, which is exactly the case
        # `step_slot + 1` gets wrong (it would claim 6 and 3).
        chan_map, _ = self._read(
            "MAP count=2 mode=dir stride=2 ch= bus=5,6,7 "
            "slots=5,2 dslots=30,3 marker=255\n")
        self.assertEqual(chan_map, {
            "A": {"step": "S5", "dir": "S30"},
            "B": {"step": "S2", "dir": "S3"},
        })

    def test_a_mux_dir_stepper_on_a_firmware_without_dslots_is_refused(self):
        # Guessing is what this field removed, so a reply that lacks it has to
        # stop the run rather than fall back to `step_slot + 1`. The message
        # names the reflashing, because the alternative reading -- "the host is
        # wrong" -- is what made the old assumption survive.
        with self.assertRaises(RuntimeError) as cm:
            self._read("MAP count=2 mode=dir stride=2 ch= bus=5,6,7 "
                       "slots=0,2 marker=255\n")
        self.assertIn("dslots", str(cm.exception))

    def test_a_mux_dir_stepper_beside_a_gpio_one(self):
        # The case `sync --imux` runs, and the one that was measured wrong: a
        # multiplexed stepper and a GPIO stepper in one CONFIG. The GPIO one
        # takes a physical channel, the mux one takes none, and the mux one is
        # still reported by slot.
        #
        # Read as one-per-channel this gave stepper B the channel D2 -- a pin
        # nothing was connected to -- and the run reported 0 of 64 steps for a
        # stepper the capture shows stepping 64 times.
        chan_map, pins = self._read(
            "MAP count=2 mode=dir stride=2 ch=2,0 bus=5,6,7 "
            "slots=-,0 dslots=-,1 marker=255\n")
        self.assertEqual(chan_map, {
            "A": {"step": "D0", "dir": "D1"},
            "B": {"step": "S0", "dir": "S1"},
        })
        self.assertEqual(pins["physical_channels"], 2)

    def test_mux_and_physical_steppers_share_one_map(self):
        # The mixed case the harness exists for: a mux run and a physical run in
        # the same capture, evaluated from one map.
        chan_map, pins = self._read(
            f"MAP count=3 mode=nodir stride=1 ch=2 bus=5,6,7 "
            f"slots=0,1,- dslots=-,-,- marker=255\n")
        self.assertEqual(chan_map, {
            "A": {"step": "S0"}, "B": {"step": "S1"}, "C": {"step": "D0"},
        })
        self.assertEqual(pins["physical_channels"], 1)
        self.assertEqual(pins["pins"], [2])

    def test_a_non_mux_board_still_reports_no_bus(self):
        # `bus=-` and `slots=-` are what every non-ESP32 build reports. The
        # fields exist in the regex but must not turn a physical run into a mux
        # one.
        chan_map, pins = self._read(
            "MAP count=2 mode=dir stride=2 ch=2,0,4,16 marker=255\n")
        self.assertEqual(chan_map, {
            "A": {"step": "D0", "dir": "D1"},
            "B": {"step": "D2", "dir": "D3"},
        })
        self.assertEqual(pins["bus"], [])
        self.assertFalse(pins["slots"])

    def test_a_mux_dir_stepper_whose_dir_bit_leaves_the_word_is_refused(self):
        # `dslots` names a real bit, so a well-formed board cannot put it outside
        # the 32-bit word. The check stays anyway: a bit the decoder has no
        # channel for would otherwise make every direction scenario measure a
        # channel that does not exist, and report it as a direction failure.
        with self.assertRaises(RuntimeError) as cm:
            self._read("MAP count=1 mode=dir stride=2 ch= bus=5,6,7 "
                       f"slots=0 dslots={run_tests.MUX_SLOT_COUNT} "
                       f"marker=255\n")
        self.assertIn("outside the", str(cm.exception))

    def test_a_mux_stepper_in_dir_with_no_direction_bit_is_refused(self):
        # `-` in `dslots` for a multiplexed stepper in `dir` mode is a firmware
        # that allocated no direction bit, and the evaluator would then be handed
        # a stepper with no dir channel while the mode says every stepper has
        # one. Reported rather than defaulted.
        with self.assertRaises(RuntimeError) as cm:
            self._read("MAP count=1 mode=dir stride=2 ch= bus=5,6,7 "
                       "slots=0 dslots=- marker=255\n")
        self.assertIn("no direction bit", str(cm.exception))

    def test_the_bus_costs_three_channels_of_the_marker_budget(self):
        # MARK cannot sit on a bus channel: its edges are the bus protocol, not
        # an event. And three channels of the eight are the bus, so a 5-stepper
        # `nodir` mux run has no marker left where a 5-stepper physical run does.
        self.assertEqual(run_tests.marker_channel_for(5, 1), 7)
        self.assertIsNone(run_tests.marker_channel_for(5, 1, bus_channels=3))
        self.assertEqual(run_tests.marker_channel_for(2, 2, bus_channels=3), 7)
        self.assertIsNone(run_tests.marker_channel_for(5, 2, bus_channels=3))


class TestMuxPinWriteGuard(unittest.TestCase):
    """A mux direction or enable slot must never reach the GPIO driver.

    `PIN_I2S_FLAG` is 0x40, so a mux slot is a pin number of 64 or above -- not a
    pin on any ESP32. `FastAccelStepper::setDirectionPin()` masks the flag off for
    I2S_DIRECT (where a slot is meaningless) but must KEEP it for I2S_MUX, since
    the queue needs it to find the bit in the frame. Keeping it meant the initial
    level went through `PIN_OUTPUT(PIN_I2S_FLAG | slot, ...)`, and the board said
    so once per stepper:

        CONFIG 2 i2s_mux,i2s_mux dir
        E (916) gpio: gpio_set_direction(321): GPIO number error
        E (916) gpio: gpio_set_level(251): GPIO output gpio_num error
        OK CONFIG n=2 mode=dir stride=2 drivers=i2s_mux,i2s_mux

    Measured on an ESP32-DevKitC, ESP-IDF 6.13: 2 errors per stepper, at every n
    from 1 to 16, and 0 after the guard. Spurious rather than corrupting -- the
    queue's own setDirPin() applies the level through i2sMuxSetBit() either way --
    which is why `dir` still stepped correctly and only the log was wrong. A
    capture-based test cannot see it (the wrong write changes no wire) and neither
    can pc_based (it stubs digitalWrite/pinMode to empty blocks), so this is a
    source-level guard.

    The harness is what finds a library defect like this, and the guard belongs
    with the mux tests rather than in pc_based; the fix is in
    src/FastAccelStepper.cpp.
    """

    def _body(self, func):
        src = (LIB / "FastAccelStepper.cpp").read_text()
        start = src.index(func)
        # Up to the next top-level function definition.
        rest = src[start + len(func):]
        end = rest.index("\nvoid FastAccelStepper::")
        return src[start:start + len(func) + end]

    def test_direction_pin_never_drives_a_mux_slot_as_a_gpio(self):
        body = self._body("void FastAccelStepper::setDirectionPin(")
        self.assertIn("PIN_OUTPUT", body)
        # The comment block between the guard and the write is part of why the
        # guard is load-bearing, so the match allows for it rather than pinning
        # the two lines together.
        self.assertRegex(
            body,
            r"else if \(!isI2sMuxPin\(_dirPin\)\) \{[^{}]*?PIN_OUTPUT",
            "setDirectionPin() reaches PIN_OUTPUT without asking "
            "isI2sMuxPin() first, so a mux direction slot is written as GPIO "
            "0x40|slot. The flag has to be KEPT here (the queue needs it), so "
            "the test cannot live on the argument -- see the class docstring.")

    def test_enable_pin_never_drives_a_mux_slot_as_a_gpio(self):
        body = self._body("void FastAccelStepper::setEnablePin(")
        self.assertIn("PIN_OUTPUT", body)
        # Both polarities, or the guard covers only one of them.
        guards = len(re.findall(r"else if \(!mux_pin\) \{\s*\n\s*PIN_OUTPUT",
                                body))
        self.assertEqual(guards, 2,
                         "setEnablePin() has two PIN_OUTPUT sites (one per "
                         "polarity) and both must be guarded; found "
                         f"{guards}")

    def test_the_mux_initial_level_still_goes_somewhere(self):
        # The guard must not silence the level along with the bad write: the
        # queue applies it for a mux slot, so setDirPin() still has to be called
        # with the FLAGGED pin (stripping it there would lose the slot).
        body = self._body("void FastAccelStepper::setDirectionPin(")
        self.assertRegex(
            body, r"_queue\(\)->setDirPin\(dirPin, dirHighCountsUp\)",
            "setDirectionPin() no longer hands the flagged pin to the queue, so "
            "the initial direction level is never applied to the mux word")

    def test_the_helper_is_defined_once_and_degrades_off_esp32(self):
        hdr = (LIB / "FastAccelStepper.h").read_text()
        self.assertEqual(hdr.count("isI2sMuxPin"), 1,
                         "isI2sMuxPin() must be defined exactly once")
        self.assertRegex(hdr, r"#if defined\(PIN_I2S_FLAG\)")
        self.assertRegex(hdr, r"#else\s*\n\s*\(void\)pin;\s*\n\s*return false;",
                         "off ESP32 there is no PIN_I2S_FLAG, so the helper has "
                         "to compile to a constant false rather than fail to "
                         "build -- AVR has no mux and must still build")


class TestMuxDecode(unittest.TestCase):
    """The 8 -> 37 decode the evaluators actually run on."""

    def setUp(self):
        self._tmp = tempfile.TemporaryDirectory()
        self.tmp = Path(self._tmp.name)
        self.addCleanup(self._tmp.cleanup)

    def _capture(self, words, passthrough_n=0):
        """A synthetic 8-channel capture of a mux bus, as run_tests sees it."""
        bus = MuxBus(words)
        path = self.tmp / "cap.vcd"
        passthrough = [f"D{i}" for i in range(passthrough_n)]
        write_mux_bus_vcd(path, bus, passthrough=passthrough)
        return path, sp.load_vcd(str(path)), passthrough

    def test_decode_turns_three_bus_wires_into_slot_channels(self):
        path, (channels, rate), _ = self._capture([0x00000005, 0, 0x80000004])
        pin_map = {"bus": [5, 6, 7], "slots": [0, 1, 2, None],
                   "mode": "nodir", "stride": 1}
        decoded, decoded_rate, record = run_tests.decode_mux_capture(
            path, channels, rate, pin_map)
        self.assertEqual(decoded_rate, rate)
        # 0x00000005 sets slots 0 and 2; 0x80000004 sets slots 31 and 2. So S0
        # pulses once, S2 twice, and S1 never -- which is also the only reason to
        # trust a word-level decode over "the data line moved".
        fs = MuxBus([1]).frame_samples
        self.assertEqual(sum(decoded["S0"]), fs)
        self.assertEqual(sum(decoded["S2"]), 2 * fs)
        self.assertEqual(sum(decoded["S1"]), 0)
        self.assertEqual(record["bus_channels"], [5, 6, 7])
        self.assertTrue(Path(record["vcd"]).exists())

    def test_the_decoder_config_indexes_slots_per_stepper(self):
        # The second half of the one-per-channel misreading: the decoder config
        # used to index `slots[j * stride]` as well, so in `dir` it handed the
        # decoder stepper A's bit 0 and stepper B's bit 2 *from the wrong
        # entries* -- A read entry 0 (correct by luck) and B read entry 2, which
        # does not exist for two steppers. The decoder and read_map() disagreed
        # about what the field meant, and only the `nodir` runs, where the two
        # readings coincide, had ever been compared.
        pin_map = {"bus": [5, 6, 7], "slots": [0, 2], "dslots": [1, 3],
                   "mode": "dir", "stride": 2}
        self.assertEqual(run_tests._mux_map_with_slots(pin_map), {
            "A": {"slot": 0, "step": "S0", "dir": "S1"},
            "B": {"slot": 2, "step": "S2", "dir": "S3"},
        })

    def test_the_decoder_config_carries_a_non_gapless_allocation(self):
        # The property that killed the `j * stride` indexing: a bit assignment
        # the stride arithmetic cannot produce. Both step and dir bits have to
        # come from the reply, or the decoder config and the evaluator's channel
        # map name different channels for the same stepper.
        pin_map = {"bus": [5, 6, 7], "slots": [7, 2], "dslots": [8, 3],
                   "mode": "dir", "stride": 2}
        self.assertEqual(run_tests._mux_map_with_slots(pin_map), {
            "A": {"slot": 7, "step": "S7", "dir": "S8"},
            "B": {"slot": 2, "step": "S2", "dir": "S3"},
        })

    def test_decode_passes_physical_channels_through_untouched(self):
        # A mux run and a physical run in one capture: the physical steppers'
        # channels have to come out the other side unchanged, or the mux work
        # silently damages the measurements next to it.
        path, (channels, rate), passthrough = self._capture(
            [0x1, 0, 0x0], passthrough_n=2)
        before = {n: bytes(channels[n]) for n in passthrough}
        pin_map = {"bus": [5, 6, 7], "slots": [0, None], "mode": "nodir",
                   "stride": 1}
        chan_map = {"A": {"step": "S0"}, "B": {"step": "D0"},
                    "C": {"step": "D1"}}
        decoded, _, _ = run_tests.decode_mux_capture(path, channels, rate,
                                                     pin_map, chan_map)
        for n in passthrough:
            self.assertEqual(bytes(decoded[n]), before[n])

    def test_the_decoded_capture_evaluates_through_the_ordinary_path(self):
        # The end the whole design rests on: a mux run goes through `evaluate()`
        # with no special case anywhere, because a decoded slot channel is an
        # ordinary step channel. If this needs a mux-aware evaluator, the
        # channel-agnostic claim in white paper 7.4 is false.
        # 640 ticks at 16 MHz is 40 us and a frame is 4 us, so a step lands on
        # every tenth frame: nine idle frames then one with slot 0 set. The
        # decoded channel is a GPIO step channel, so SR_01's own expectations
        # have to hold against it untouched.
        path, (channels, rate), _ = self._capture(([0] * 9 + [1]) * 4)
        pin_map = {"bus": [5, 6, 7], "slots": [0], "mode": "nodir",
                   "stride": 1}
        decoded, decoded_rate, _ = run_tests.decode_mux_capture(
            path, channels, rate, pin_map)
        chan_map = {"A": {"step": "S0"}}
        info = dict(vf.Dut().info())
        segments = [(4, 640, True)]
        passed, detail = run_tests.evaluate(
            "SR_01", decoded, decoded_rate, segments, info, chan_map)
        self.assertTrue(passed, detail)
        self.assertEqual(detail["steps"]["steps_measured"], 4)
        self.assertEqual(detail["adherence"]["mean_period_us"], 40.0)


class TestMuxFrameGrid(unittest.TestCase):
    """A multiplexed step cannot start between frames, so its period is a set.

    The I2S mux sets a step bit for whole frames: the pulse is 4 us and it goes
    out in the frame that contains its instant. A 400-tick command is therefore
    not a steady 25 us period -- it is 6, 6, 6, then 7 frames, i.e. 24, 24, 24,
    28 us, averaging exactly 25. Judged against the harness's ordinary +-5% band
    around 25 us, one period in four reads 12 % long and the run fails while
    measuring the behaviour it exists to describe.
    """

    RATE = 1_000_000
    GRID = dict(frame_grid_ticks=run_tests.I2S_TICKS_PER_FRAME)
    INFO = dict(vf.Dut().info(), **GRID)

    def _channels(self, periods_us, high_us=1.0):
        """A step channel with the given inter-step periods in microseconds.

        Each period is placed on the sample grid, so the measured periods are the
        commanded ones to within a sample -- which at 1 MS/s is 1 us against a
        4 us frame, so the grid check is not being asked to resolve finer than it
        can.
        """
        high = max(1, int(round(high_us * self.RATE / 1e6)))
        wave = bytearray(int(sum(periods_us) * self.RATE / 1e6) + 8)
        at = 0.0
        for p in periods_us:
            lo = int(round(at * self.RATE / 1e6))
            wave[lo:lo + high] = b"\x01" * high
            at += p
        # One leading low sample. detect_edges() never counts the first sample as
        # an edge, so a waveform that starts high reports one step fewer than it
        # has -- which is a real property of the parser, not a fixture quirk, and
        # the tests below count steps.
        return {"S0": bytes(1) + bytes(wave)}, self.RATE

    def test_the_library_frame_is_the_const_the_harness_asserts_against(self):
        # Read from the source, not from the copy in run_tests.py: if the driver
        # changes its frame length, the harness must find out here rather than by
        # judging every mux run against a stale grid.
        root = SCRIPTS.parents[3] / "src" / "pd_esp32" / "i2s_constants.h"
        m = re.search(r"#define I2S_TICKS_PER_FRAME (\d+)",
                      root.read_text())
        self.assertIsNotNone(m, "I2S_TICKS_PER_FRAME moved or was renamed")
        self.assertEqual(run_tests.I2S_TICKS_PER_FRAME, int(m.group(1)))

    def test_the_frame_quantised_period_is_legal_and_the_mean_is_exact(self):
        channels, rate = self._channels([24.0, 24.0, 24.0, 28.0] * 4)
        m = sp.channel_metrics(channels["S0"], rate)
        d = sp.grid_period_defects(m.inter_step_us, 400, 16_000_000,
                                   run_tests.I2S_TICKS_PER_FRAME)
        self.assertTrue(d["ok"], d)
        self.assertEqual(d["legal_periods_us"], [24.0, 28.0])

    def test_a_period_on_the_wrong_frame_count_still_fails(self):
        # The grid is not a free pass. 6 and 7 frames are the legal pair for a
        # 400-tick command; 5 frames is 20 us and is not one of them.
        channels, rate = self._channels([20.0] * 8)
        m = sp.channel_metrics(channels["S0"], rate)
        d = sp.grid_period_defects(m.inter_step_us, 400, 16_000_000,
                                   run_tests.I2S_TICKS_PER_FRAME)
        self.assertFalse(d["ok"])
        self.assertTrue(d["n_off_grid"])

    def test_a_mean_that_has_drifted_fails_even_on_the_grid(self):
        # Every period legal but the average wrong is a driver losing time, and
        # the grid does not excuse it: the extra frame is paid every fourth step,
        # not on every step.
        channels, rate = self._channels([24.0, 24.0, 24.0, 32.0] * 4)
        m = sp.channel_metrics(channels["S0"], rate)
        d = sp.grid_period_defects(m.inter_step_us, 400, 16_000_000,
                                   run_tests.I2S_TICKS_PER_FRAME)
        self.assertFalse(d["ok"])
        self.assertLess(d["mean_period_us"], 26.0)
        self.assertFalse(d["ok"])

    def test_scale_uses_the_grid_only_when_the_run_has_one(self):
        # The same evaluator, both ways. A non-mux run must keep its +-5% band --
        # widening it for every driver to accommodate the mux would let a real
        # rate error through everywhere else.
        segments = [(8, 400, True)]
        chan_map = {"A": {"step": "S0"}}
        # The frame-quantised pattern the mux really emits at 400 ticks, which a
        # +-5% band around 25 us rejects...
        wave, rate = self._channels([24.0, 24.0, 24.0, 28.0] * 2)
        on_grid, detail = run_tests.evaluate(
            "SR_01", wave, rate, segments,
            dict(vf.Dut().info(), **self.GRID), chan_map)
        self.assertTrue(on_grid, detail)
        self.assertEqual(detail["period"]["legal_periods_us"], [24.0, 28.0])
        # ...and the same evaluator WITHOUT the grid still catches a real rate
        # error, which is what keeping the ordinary band everywhere else is for.
        ok, detail2 = run_tests.evaluate(
            "SR_01", wave, rate, segments, dict(vf.Dut().info()), chan_map)
        self.assertFalse(ok)
        self.assertTrue(detail2["period"]["long_periods_us"], detail2)

    def test_the_grid_reaches_the_evaluator_through_info(self):
        # Not a module-level flag and not a sixth evaluator argument threaded
        # through all of them: `measure()` puts it in `info`, and `eval_scale`
        # reads it there. Both halves named, because either alone leaves the
        # other able to disappear unnoticed.
        src = (SCRIPTS / "run_tests.py").read_text()
        self.assertIn('info.get("frame_grid_ticks")', src)
        self.assertIn("info = dict(info, frame_grid_ticks=", src)


class TestPins(unittest.TestCase):
    """The channel map as a value passed to the evaluator, not a global.

    This is todo R4. The map used to be two module-level dicts that `evaluate()`
    overwrote per run, which meant a *wrong* map was not an error but a silent
    substitution: with `nodir`, stepper B is on `D1`, so reading it on `D2`
    measures a quiet pin and reports a driver that emits nothing.

    The tests below are mostly about that substitution not being able to
    happen any more, and the two that matter most are `test_a_nodir_map_cannot
    _be_judged_as_a_dir_map` and its inverse -- they assert the *conclusion*
    changes with the map, which is what makes the map load-bearing rather than
    decorative.
    """

    # 1 sample == 1 us, and a tick is a sample, so 160 ticks is a 160 us period.
    INFO = {"ticks_per_s": 1_000_000, "min_cmd_ticks": 100,
            "max_speed_ticks": 160, "queue_len": 32, "per_stepper_floor": 160}

    def dense(self, events, n):
        s = [0] * n
        for i, (t, v) in enumerate(events):
            end = events[i + 1][0] if i + 1 < len(events) else n
            for j in range(t, min(end, n)):
                s[j] = v
        return s

    def square(self, period, n, offset=1000, high=8):
        out = []
        for i in range(n):
            out.append((i * period + offset, 1))
            out.append((i * period + offset + high, 0))
        return out

    def two_steppers(self, b_period=320, b_offset=1000):
        """A and B both stepping, B at `b_period`, all eight channels present."""
        n = 20000
        ch = {f"D{i}": [0] * n for i in range(8)}
        ch["D0"] = self.dense(self.square(160, 40), n)
        ch["D2"] = self.dense(self.square(b_period, 40, offset=b_offset), n)
        return ch

    def test_pins_exposes_both_shapes(self):
        d = run_tests.Pins(run_tests.default_channel_map(2, 2))
        self.assertEqual(d.letters, ["A", "B"])
        self.assertEqual(d.step_of("A"), "D0")
        self.assertEqual(d.dir_of("A"), "D1")
        self.assertEqual(d.step_of("B"), "D2")
        self.assertEqual(d.count, 2)

        n = run_tests.Pins(run_tests.default_channel_map(8, 1))
        self.assertEqual(n.letters, list("ABCDEFGH"))
        self.assertEqual(n.step_of("B"), "D1")
        # nodir has no direction pin at all, and that has to be None rather than
        # a repeat of the step pin: comparing a pin against itself would either
        # find every dir change or none, both meaningless.
        self.assertIsNone(n.dir_of("B"))
        self.assertEqual(n.dir, {})

    def test_nodir_8_stepper_map_is_d0_through_d7(self):
        # The todo's verification case, stated directly: a nodir 8-stepper
        # result maps A..H to D0..D7.
        pins = run_tests.Pins(run_tests.default_channel_map(8, 1))
        self.assertEqual([pins.step_of(c) for c in "ABCDEFGH"],
                         [f"D{i}" for i in range(8)])

    def test_a_nodir_map_cannot_be_judged_as_a_dir_map(self):
        # A on D0, B on D1 -- the `nodir` shape. Judged with the `dir` map, B is
        # read on D2, which is quiet, so a working pair of steppers is reported
        # as one working stepper and one dead driver.
        # Both steppers at the same period, and B present on D1 *only*: SR_16
        # judges every stepper against the shared command, so a differing rate
        # would fail for a second unrelated reason, and a waveform mirrored onto
        # D2 as well would make the wrong map pass. D2 stays quiet, which is the
        # whole situation being tested.
        ch = self.two_steppers(b_period=160, b_offset=1100)
        ch["D1"] = ch.pop("D2")
        nodir = run_tests.default_channel_map(2, 1)
        wrong = run_tests.default_channel_map(2, 2)

        ok, right = run_tests.evaluate(
            run_tests.EVALUATORS["SR_16"], ch, 1_000_000,
            [(40, 160, True)], self.INFO, nodir)
        ok_wrong, wrong_detail = run_tests.evaluate(
            run_tests.EVALUATORS["SR_16"], ch, 1_000_000,
            [(40, 160, True)], self.INFO, wrong)

        self.assertTrue(ok, right)
        self.assertEqual(right["per_stepper"]["B"]["steps"]["steps_measured"],
                         40)
        self.assertEqual(right["per_stepper"]["B"]["channel"], "D1")

        # The wrong map does not merely report B as quiet -- it says the capture
        # has no B at all, which is the true reason and cannot be mistaken for a
        # driver finding.
        self.assertFalse(ok_wrong,
                         "the wrong map still passed -- the map is decorative")
        self.assertIn("incomplete_capture", wrong_detail)
        self.assertEqual(wrong_detail["incomplete_capture"]["missing_steppers"],
                         ["B"])
        self.assertEqual(wrong_detail["incomplete_capture"]["missing_channels"],
                         ["D2"])

    def test_two_maps_can_be_judged_in_one_process(self):
        # The globals could not do this: whichever map was installed last won,
        # so two results with different shapes could never be compared. This is
        # the property the fixtures and the mode runs both need.
        nodir_ch = self.two_steppers(b_period=160, b_offset=1100)
        nodir_ch["D1"] = nodir_ch.pop("D2")
        dir_ch = self.two_steppers(b_period=160, b_offset=1000)

        nodir_ok, _ = run_tests.evaluate(
            run_tests.EVALUATORS["SR_16"], nodir_ch, 1_000_000,
            [(40, 160, True)], self.INFO,
            run_tests.default_channel_map(2, 1))
        dir_ok, dir_detail = run_tests.evaluate(
            run_tests.EVALUATORS["SR_16"], dir_ch, 1_000_000,
            [(40, 160, True)], self.INFO,
            run_tests.default_channel_map(2, 2))
        # And back again, to catch an evaluator that cached the first map.
        nodir_again, _ = run_tests.evaluate(
            run_tests.EVALUATORS["SR_16"], nodir_ch, 1_000_000,
            [(40, 160, True)], self.INFO,
            run_tests.default_channel_map(2, 1))

        self.assertTrue(nodir_ok and dir_ok and nodir_again,
                        "interleaved maps gave different answers")
        self.assertEqual(dir_detail["per_stepper"]["B"]["channel"], "D2")

    def test_no_evaluator_reads_a_module_level_channel_table(self):
        # The structural half of R4. A grep-based test, deliberately: the
        # failure mode was a table that exists and is correct for the shape in
        # front of you, so the only way to keep it gone is to notice it coming
        # back.
        source = (SCRIPTS / "run_tests.py").read_text()
        code = "\n".join(line for line in source.splitlines()
                          if not line.lstrip().startswith("#"))
        for name in ("STEP_CHANNELS", "DIR_CHANNELS"):
            body = code.split('"""', 2)[-1] if '"""' in code else code
            self.assertNotIn(f"{name} =", code,
                             f"a module-level {name} is back; the map must be "
                             f"passed to the evaluator, not installed globally")

    def test_every_evaluator_takes_the_map(self):
        # Every entry in EVALUATORS has the same signature, so a new one cannot
        # be added that quietly reads a default instead of the run's own map.
        import inspect
        for test_id, fn in run_tests.EVALUATORS.items():
            params = list(inspect.signature(fn).parameters)
            # The first five exactly; further keyword arguments are allowed,
            # because two evaluators genuinely need one more -- eval_sync the
            # per-stepper programs, eval_abort_queue the marker channel. What
            # must never happen is one that reads a *default* instead of the
            # run's own data, which is what the first five guarantee.
            self.assertEqual(params[:5],
                             ["channels", "rate", "segments", "info", "pins"],
                             f"{test_id} does not take the channel map")
            extra = params[5:]
            self.assertTrue(all(inspect.signature(fn).parameters[n].default
                                is not inspect.Parameter.empty
                                for n in extra),
                            f"{test_id} has required extras {extra}; a run "
                            f"would have to supply them unconditionally")

    def test_wire_plan_captures_every_channel_the_config_can_reach(self):
        # Sparse selections drop channels on this clone, so the capture must be
        # contiguous -- and it must cover all of them. Capturing only D0..D3 left
        # a four-stepper scenario's C and D uncaptured, which used to score them
        # as silent drivers and now fails as an incomplete capture.
        import run_hardware as hw
        for scenario, want_channels, want_mask in (
                ("SR_01", ["D0", "D1"], "1"),
                ("SR_14", ["D0", "D1", "D2", "D3"], "3")):
            _wire, channels, mask = hw.wire_plan(scenario, "rmt")
            self.assertEqual(channels.split(","), want_channels, scenario)
            self.assertEqual(mask, want_mask, scenario)
        # The mask has to select every stepper the config connected, or QRUN
        # leaves one idle and the run reports a driver that was never asked.
        cfg = run_tests.SCENARIOS["SR_14"][0]
        count = len(run_tests.config_drivers(cfg, "rmt"))
        _w, _c, mask = hw.wire_plan("SR_14", "rmt")
        self.assertEqual(int(mask), (1 << count) - 1)

        # No catalogue scenario reaches four steppers today, so the 4-channel
        # cap is unreachable through SR ids -- which is exactly why a test that
        # only walked the catalogue would not have caught it. Exercise the rule
        # with the scenario table widened instead of pretending it is reachable.
        with mock.patch.dict(run_tests.CONFIGS, {"4ch": (4, "native")}), \
                mock.patch.dict(run_tests.SCENARIOS,
                                {"SR_99": ("4ch", None, 0, "four steppers")}):
            _w, channels, mask = hw.wire_plan("SR_99", "rmt")
        self.assertEqual(channels.split(","), [f"D{i}" for i in range(8)])
        self.assertEqual(mask, "15",
                         "a four-stepper run needs all 8 channels and mask 15")

    def test_dir_change_still_maps_a_to_d0_and_b_to_d2(self):
        # The todo's other verification case, and the guard against R4 breaking
        # the one shape every recorded result uses.
        pins = run_tests.Pins.default()
        self.assertEqual(pins.step_of("A"), "D0")
        self.assertEqual(pins.dir_of("A"), "D1")
        self.assertEqual(pins.step_of("B"), "D2")
        self.assertEqual(pins.dir_of("B"), "D3")

    def test_pin_invariants_skip_nodir_rather_than_comparing_a_pin_to_itself(self):
        # In nodir there is no dir pin. Passing the step pin as both arguments
        # would make every step edge look like a dir edge during step-high, and
        # the invariant would then fail every nodir run for no reason.
        ch = {f"D{i}": [0] * 20000 for i in range(8)}
        ch["D0"] = self.dense(self.square(160, 40), 20000)
        pins = run_tests.Pins(run_tests.default_channel_map(2, 1))
        inv = run_tests.check_pin_invariants(ch, 1_000_000, pins)
        self.assertTrue(inv["ok"], inv)
        self.assertEqual(inv["n_dir_while_step_high"], 0)

    def test_pin_invariants_still_catch_a_dir_edge_inside_a_pulse(self):
        # ...and the check is not vacuous for them: a dir change during step-high
        # is a real defect, so it has to be found in `dir` mode.
        n = 20000
        ch = {f"D{i}": [0] * n for i in range(8)}
        ch["D0"] = self.dense(self.square(160, 40), n)
        ch["D1"] = self.dense([(1000 + 160 * 5, 1), (1000 + 160 * 5 + 4, 0)], n)
        pins = run_tests.Pins(run_tests.default_channel_map(1, 2))
        inv = run_tests.check_pin_invariants(ch, 1_000_000, pins)
        self.assertFalse(inv["ok"], "a dir edge inside a pulse went unnoticed")
        self.assertEqual(inv["n_dir_while_step_high"], 1)
        self.assertIn("A", inv["dir_while_step_high"])

    def test_a_missing_channel_is_not_a_quiet_one(self):
        # A capture can legitimately omit a channel. `step_wave` returns None
        # for that, so an evaluator skips the stepper rather than measuring an
        # absent channel as a quiet one -- which would read as a driver that
        # never fired.
        pins = run_tests.Pins(run_tests.default_channel_map(2, 2))
        self.assertIsNone(pins.step_wave({"D0": [1]}, "B"))
        self.assertIsNotNone(pins.step_wave({"D0": [1]}, "A"))
        self.assertIsNone(pins.dir_wave({"D0": [1]}, "A"))

    def test_results_carry_the_map_they_were_judged_with(self):
        # R4 is only half done if the map is used and then thrown away: the
        # record has to say which map produced it, or a reader cannot tell a
        # nodir result from a dir one.
        import json
        import tempfile
        with tempfile.TemporaryDirectory() as tmp:
            out = Path(tmp) / "results"
            args = argparse.Namespace(
                port="/dev/null", baud=115200, sample_rate=1_000_000,
                sr00_sample_rate=1_000_000, seconds=1.0,
                capture_dir=str(Path(tmp) / "cap"),
                results_dir=str(out), force=True)
            seen = {}

            def fake_measure(key, name, wire, mask, builder, ev, a,
                             per_stepper_for=None):
                chan_map = run_tests.default_channel_map(3, 1)
                seen["pins"] = run_tests.Pins(chan_map)
                return "passed", {"channel_map": chan_map, "pin_map": {}}

            with mock.patch.object(run_tests, "measure", fake_measure), \
                    mock.patch.object(run_tests, "load_index", lambda f: {}), \
                    mock.patch.object(run_tests, "start_capture",
                                      lambda *a, **k: mock.Mock()), \
                    mock.patch.object(run_tests, "send_line",
                                      lambda *a, **k: ""), \
                    mock.patch.object(run_tests, "reply_of",
                                      lambda *a: "OK CONFIG"), \
                    mock.patch.object(run_tests, "read_map",
                                      lambda ser: (seen["chan_map"], {})), \
                    mock.patch.object(run_tests, "read_qinfo",
                                      lambda ser: dict(vf.Dut().info())), \
                    mock.patch.object(run_tests, "program",
                                      lambda *a: True), \
                    mock.patch.object(run_tests, "drain",
                                      lambda *a, **k: ""), \
                    mock.patch.object(run_tests, "open_board",
                                      lambda *a: mock.Mock()), \
                    mock.patch.object(run_tests, "load_capture_for_eval",
                                      lambda *a: ({"D0": [0]}, 1000)):
                seen["chan_map"] = run_tests.default_channel_map(3, 1)
                plan = run_tests.scale_plan("rmt", "nodir", 3)
                run_tests.run_modes("r4_probe", plan, args, "scale")
            records = [json.loads(f.read_text())
                       for f in sorted(out.glob("*.json"))
                       if "tag_index" not in f.name]
            self.assertEqual(len(records), 3)
            for rec in records:
                self.assertEqual(rec["channel_map"]["B"], {"step": "D1"},
                                 rec["tag_key"])
                self.assertEqual(rec["pin_mode"], "nodir")


if __name__ == "__main__":
    unittest.main(verbosity=2)
