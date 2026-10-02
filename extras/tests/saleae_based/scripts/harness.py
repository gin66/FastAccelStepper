#!/usr/bin/env python3
"""
harness.py — high-level, target-agnostic front-end for the Saleae harness.

Describe the target and the test; the harness derives the rest and drives the
low-level orchestrator (run_tests.py).

    # ESP32, Arduino framework, four steppers on RMT
    python3 scripts/harness.py --arch esp32 --framework arduino \
        --driver rmt_v2 --count 4 --pin-mode dir --tests SR_01 \
        --steps 4000 --speed-us 5 --flash

    # Same board, one stepper on RMT and one on MCPWM/PCNT
    python3 scripts/harness.py --arch esp32 --framework idf --version 5.3 \
        --drivers rmt,mcpwm_pcnt --count 2 --tests SR_17 --flash

    # ESP32, ESP-IDF 5.3, MCPWM
    python3 scripts/harness.py --arch esp32 --framework idf --version 5.3 \
        --driver mcpwm_pcnt --tests SR_01 --speed-us 5 --flash

    # AVR at 40 kSteps/s (25 us/step), Timer driver
    python3 scripts/harness.py --arch nanoatmega328 --driver timer \
        --tests SR_01 --steps 2000 --speed-us 25 --flash

    # Pico at 200 kHz (5 us/step), PIO driver
    python3 scripts/harness.py --arch rpipico --driver pio \
        --tests SR_01 --steps 4000 --speed-us 5 --flash

From --arch/--framework/--version/--driver(s)/--count/--pin-mode it computes the
tag key, the PlatformIO project dir + env, and (unless given) a capture sample
rate that is >= 20 samples per step period. --dry-run prints the mapping only.

There is no automatic driver selection anywhere. --driver names the pulse driver
for every stepper, or --drivers gives one name per stepper; both land in the
firmware's `CONFIG <count> <drv>[,<drv>...] [dir|nodir]`, which refuses anything
it cannot provide rather than falling back.

The capture sample rate is automatically snapped to a rate the analyzer
supports (fx2lafw: 48 MHz / n). Use --sample-rate to override.
"""

import argparse
import subprocess
import sys
from pathlib import Path

SCRIPTS = Path(__file__).resolve().parent
ROOT = SCRIPTS.parents[3]  # repo root
sys.path.insert(0, str(SCRIPTS))

import run_tests  # noqa: E402

ESP_ARCHS = ["esp32", "esp32s2", "esp32s3", "esp32c3", "esp32c6", "esp32h2",
             "esp32p4"]
AVR_ARCHS = ["nanoatmega168", "nanoatmega328", "atmega2560", "atmega32u4"]
PICO_ARCHS = ["rpipico", "rpipico2"]
SAM_ARCHS = ["atmelsam", "samd51"]
ARCHS = ESP_ARCHS + AVR_ARCHS + PICO_ARCHS + SAM_ARCHS

DRIVERS = {
    "esp": ["rmt_v2", "rmt", "mcpwm_pcnt", "i2s_direct", "i2s_mux"],
    "avr": ["timer"],
    "pico": ["pio"],
    "sam": ["timer"],
}

# Driver *name* -> driver *identity*. `rmt` and `rmt_v2` are two spellings of
# one driver: the firmware's parse_driver() maps both to SA_RMT and reports both
# as `rmt`, because the RMT generation is a property of the SDK, which is a tag
# on the run, not of the driver. They must be collapsed before a sync plan is
# enumerated.
#
# This is not tidiness. R1 recorded a finding that was wrong for exactly this
# reason: the pre-R1 firmware's `mixed` config parsed a driver list and then
# overwrote it with its automatic choice, so "cross-driver" was measured as
# RMT+RMT -- and the tell was that two supposedly different configurations
# agreed to four decimal places. Enumerating `rmt_v2+rmt` as a *combination*
# would put that same identical pair in the table under a name claiming they are
# different drivers, and reading it as a cross-driver measurement is the mistake
# the table exists to prevent.
DRIVER_IDENTITY = {
    "rmt_v2": "rmt",
    "rmt": "rmt",
    "mcpwm": "mcpwm_pcnt",
    "mcpwm_pcnt": "mcpwm_pcnt",
    "i2s": "i2s_direct",
    "i2s_direct": "i2s_direct",
    "i2s_mux": "i2s_mux",
    "timer": "timer",
    "pio": "pio",
}


def driver_identities(drivers):
    """The distinct driver identities in `drivers`, first-seen order.

    `sync` enumerates over identities, because a combination of two spellings of
    one driver is not a combination -- it is the same-driver case, which is
    already in the plan under the canonical name.
    """
    seen = []
    for d in drivers:
        ident = DRIVER_IDENTITY.get(d, d)
        if ident not in seen:
            seen.append(ident)
    return seen

# The pin mode, the other half of the firmware's CONFIG grammar. It is not
# cosmetic: the analyzer has 8 channels and `dir` spends two per stepper, so
# `dir` reaches 4 steppers and `nodir` reaches 8 (white paper 3.3/10.1).
PIN_MODES = ["dir", "nodir"]

# The two generic test modes (white paper 1.2). Neither names an architecture:
# arch/framework/version/driver are *tags on a run*, which is what makes the
# same two commands the whole cross-architecture matrix.
MODES = ["scale", "sync"]

# Analyzer channels, and the stepper count each pin mode can therefore carry.
CHANNELS = 8
CHANNELS_PER_STEPPER = {"dir": 2, "nodir": 1}


def max_steppers(pin_mode):
    """How many steppers this pin mode can put on CHANNELS channels."""
    return CHANNELS // CHANNELS_PER_STEPPER[pin_mode]

# One generic CONFIG covers every channel configuration there is: a count, a
# driver list and a pin mode. The eight named presets the firmware used to take
# (`8ch_step_only`, `4ch_rmt`, `mixed`, ...) are gone -- naming the combinations
# only means a new name for every one (white paper §3.2). --count and --pin-mode
# say the first and third terms; --driver says the second.
DEFAULT_COUNT = 4

# fx2lafw-supported samplerates (48 MHz / n).
SUPPORTED_RATES = [20000, 25000, 50000, 100000, 200000, 250000, 500000,
                   1_000_000, 2_000_000, 3_000_000, 4_000_000, 6_000_000,
                   8_000_000, 12_000_000, 16_000_000, 24_000_000, 48_000_000]


# How many steppers each driver can hand out at once.
#
# These are the library's `QUEUES_*` values for the architecture, and they are
# the *driver's* bound -- distinct from the channel budget above, which is the
# analyzer's. A `scale` run stops at the smaller of the two and says which, so
# both have to be known before anything runs: a sweep point the firmware would
# refuse spends a capture measuring its refusal.
#
# AVR/SAM/Pico have one driver and one number each, which is the whole of what
# `scale` needs to know about them -- that is why `scale` is the single command
# the cross-architecture matrix is made of.
#
# i2s_mux is 32 under dynamic allocation but the channel budget caps the run at
# 8 first, so its row is the analyzer's bound, not the driver's. i2s_direct is 3
# *channels that are not one pin each* -- it is an internal demux, not a
# separate GPIO -- so it gets one channel per stepper like any other driver and
# is limited by its own 3.
# A 0 is not a missing entry: it is a driver this chip has no queues for
# (`QUEUES_MCPWM_PCNT 0` on the S2, C3 and P4, which have no MCPWM/PCNT at
# all). Omitting those would make the table's absence ambiguous between "not
# applicable here" and "nobody filled this in", and the second is the one that
# silently guesses a bound.
DRIVER_MAXS = {
    "esp32": {"rmt": 8, "rmt_v2": 8, "mcpwm_pcnt": 6,
              "i2s_direct": 3, "i2s_mux": 32},
    "esp32s2": {"rmt": 4, "rmt_v2": 4, "mcpwm_pcnt": 0,
                "i2s_direct": 0, "i2s_mux": 0},
    "esp32s3": {"rmt": 4, "rmt_v2": 4, "mcpwm_pcnt": 4,
                "i2s_direct": 0, "i2s_mux": 0},
    "esp32c3": {"rmt": 2, "rmt_v2": 2, "mcpwm_pcnt": 0,
                "i2s_direct": 0, "i2s_mux": 0},
    "esp32c6": {"rmt": 2, "rmt_v2": 2, "mcpwm_pcnt": 2,
                "i2s_direct": 0, "i2s_mux": 0},
    "esp32h2": {"rmt": 2, "rmt_v2": 2, "mcpwm_pcnt": 2,
                "i2s_direct": 0, "i2s_mux": 0},
    "esp32p4": {"rmt": 8, "rmt_v2": 8, "mcpwm_pcnt": 0,
                "i2s_direct": 0, "i2s_mux": 0},
    "nanoatmega328": {"timer": 2},
    "nanoatmega168": {"timer": 2},
    "atmega2560": {"timer": 3},
    "atmega32u4": {"timer": 3},
    # Pico: NUM_QUEUES = 4 * NUM_PIOS, and NUM_PIOS is the PIO block count --
    # 2 on RP2040, 3 on RP2350 (pico-sdk platform_defs.h).
    "rpipico": {"pio": 8},
    "rpipico2": {"pio": 12},
    # SAMD: MAX_STEPPER is the TCC instance count, which is 3 on SAMD51G.
    "atmelsam": {"timer": 6},
    "samd51": {"timer": 3},
}

# A driver the board has not got connected -- `i2s_mux` today -- is one flag
# away. It is named here, in a list, and nothing in the mode logic switches on
# it, so a plan that includes it either connects or records the refusal.
def driver_max(arch, driver):
    """A *claimed* queue count for `driver` on `arch`, or None. Cross-check only.

    Kept so a run can be compared against what the host believed, and so a
    disagreement is visible. It is no longer consulted to decide the sweep:
    see scale_bound().
    """
    return DRIVER_MAXS.get(arch, {}).get(driver)


def scale_bound(arch, driver, pin_mode):
    """(count_max, bound) for a `scale` run: the loop limit, and what set it.

    The limit is the analyzer's channel budget and nothing else. It used to be
    min(that, a DRIVER_MAXS entry), and the entry -- `QUEUES_MCPWM_PCNT` = 6 --
    is not wrong about what it counts. The board really does allocate six MCPWM
    queues; CONFIG accepts six and refuses the seventh with
    `ERR connect step 6`. What the constant cannot say is how many of those six
    *run*, and on this board exactly one does: two through six connect and then
    emit ~21 000 steps where 64 were commanded.

    So the old bound was not a wrong number, it was the wrong question. A sweep
    that stopped at the queue count reported "MCPWM reaches 6" over a column of
    five runaway steppers. Sweeping past it lets the refusals land as refusals
    instead, and the run then says three separate things: one stepper passes,
    five connect and misbehave, and a seventh will not connect at all.

    So the loop now runs to the channel budget and the board decides where it
    stops: run_modes() records a refusal and carries on, which for `scale` is
    exactly the measurement. The bound in the summary is then measured rather
    than predicted, and it is right about drivers no table has ever heard of.
    """
    chan_cap = max_steppers(pin_mode)
    claimed = driver_max(arch, driver)
    note = ""
    if claimed is not None and claimed != chan_cap:
        # Said once, and only when it would have mattered -- a host belief that
        # differs from the rig's own budget is not by itself an error.
        note = f" [host table believed {claimed} queues for {driver} on " \
               f"{arch}; the sweep measures it]"
    return chan_cap, f"channels ({CHANNELS} / {CHANNELS_PER_STEPPER[pin_mode]}" \
                     f" per stepper){note}"


def arch_family(arch):
    if arch in ESP_ARCHS:
        return "esp"
    if arch in AVR_ARCHS:
        return "avr"
    if arch in PICO_ARCHS:
        return "pico"
    return "sam"


def version_tag(version):
    """'5.3' / 'V5_3_0' / 'latest' -> 'V5_3_0' or None for latest."""
    if version in (None, "", "latest"):
        return None
    v = version.lstrip("V")
    parts = v.split(".")
    while len(parts) < 3:
        parts.append("0")
    return "V" + "_".join(parts[:3])


def snap_rate(desired):
    for r in SUPPORTED_RATES:
        if r >= desired:
            return r
    return SUPPORTED_RATES[-1]


def derive(args):
    """Return (tag_key, project_dir, env, sample_rate)."""
    family = arch_family(args.arch)
    vt = version_tag(args.version)

    # Checked here rather than left to the firmware, because the firmware's
    # refusal names the bounds but by then the host has already picked a sample
    # rate and opened a capture.
    cap = max_steppers(args.pin_mode)
    if args.count < 1 or args.count > cap:
        raise SystemExit(
            f"--count {args.count} does not fit {args.pin_mode}: "
            f"{CHANNELS} channels / {CHANNELS_PER_STEPPER[args.pin_mode]} per "
            f"stepper = {cap}")

    if args.framework == "idf":
        if family != "esp":
            raise SystemExit("--framework idf is only valid for ESP32 targets")
        proj = "pio_espidf/saleae"
        env = f"{args.arch}_idf_{vt}" if vt else f"{args.arch}_idf_V5_3_0"
        fw = "idf" + (vt.replace("V", "").replace("_", ".") if vt else "latest")
    else:
        proj = "pio_dirs/saleae"
        if family == "esp":
            env = "esp32" if (args.arch == "esp32" and vt is None) \
                else f"{args.arch}_{vt}"
        else:
            env = args.arch  # avr / pico / sam envs are the plain names
        fw = "arduino" if vt is None else "arduino" + vt.replace("V", "").replace("_", ".")

    if args.mode:
        # A mode's tag key must not carry a stepper count: the count is the
        # sweep axis, so embedding the --count default would put a fixed
        # number in every run's tag and make `scale_n4` and `scale_n7` look like
        # the same configuration. Each generated run gets its own key suffix
        # (ModeRun.tag) that does say which count it was.
        raw = f"{args.arch}_{fw}_{args.driver}_{args.mode}{args.pin_mode}"
    else:
        raw = f"{args.arch}_{fw}_{driver_list(args).replace('+', '_')}" \
              f"{args.count}{args.pin_mode}"
    tag_key = "".join(c if (c.isalnum() or c == "_") else "_" for c in raw)

    rate = args.sample_rate
    if rate <= 0:
        # Must resolve the step PULSE, not just the period: the high time can
        # be a few microseconds, so use a floor of 4 MS/s (0.25 us/sample) and
        # at least 20 samples per step period.
        step_freq = 1_000_000.0 / args.speed_us
        rate = snap_rate(max(4_000_000, int(20 * step_freq)))
    return tag_key, proj, env, rate


def driver_list(args):
    """One driver name per stepper, which is what CONFIG wants.

    --drivers overrides --driver and is the way to say "stepper A on RMT,
    stepper B on MCPWM": every run names its drivers, because a result that does
    not record which driver produced it characterizes nothing.
    """
    if args.drivers:
        names = [d.strip() for d in args.drivers.split(",") if d.strip()]
    else:
        names = [args.driver] * args.count
    if len(names) != args.count:
        raise SystemExit(f"--drivers lists {len(names)} names but --count is "
                         f"{args.count}; CONFIG refuses a mismatched list")
    unknown = [d for d in names if d not in DRIVERS[arch_family(args.arch)]]
    if unknown:
        raise SystemExit(f"unknown driver(s) for {args.arch}: "
                         f"{', '.join(unknown)}; known: "
                         f"{', '.join(DRIVERS[arch_family(args.arch)])}")
    return "+".join(names)


def preflight(args, port, baud=115200):
    """Ask the board which drivers it accepts, and bring the mux up if asked.

    The host keeps no authority on what the target has. It used to: a per-arch
    name table and DRIVER_MAXS, both maintained by hand, and the queue counts
    in DRIVER_MAXS were a faithful copy of the library's declared constants --
    which is exactly the problem, because on this board QUEUES_MCPWM_PCNT is 6
    and the hardware connects one. A copy of a claim is not evidence.

    So the question goes to the only party that knows. And a board that cannot
    be asked is an error, never a fallback to the table: silently reverting to
    the belief is how the wrong number got believed in the first place.
    """
    import serial
    ser = serial.Serial(port, baud, timeout=1)
    try:
        present, mux_init = run_tests.read_drivers(ser)
        absent = sorted(d for d, ok in present.items() if not ok)
        if absent:
            print(f"drivers: this build does not accept {', '.join(absent)}")
        if args.imux:
            if "i2s_mux" not in present:
                raise SystemExit(f"--imux needs an ESP32 I2S build; this one "
                                 f"does not accept i2s_mux")
            if mux_init:
                print("mux     : already up (nothing sent)")
            else:
                data, bclk, ws = (int(x) for x in args.imux.split(","))
                if not run_tests.send_imux(ser, data, bclk, ws):
                    raise SystemExit(
                        f"--imux {args.imux} was refused; the three pins must "
                        f"be free, and initI2sMux() cannot run twice")
                print(f"mux     : up, data={data} bclk={bclk} ws={ws}")
        return present, mux_init
    finally:
        ser.close()


def build_and_flash(proj, env, port, do_build, do_flash):
    if do_build or do_flash:
        subprocess.run(["bash", "extras/scripts/build-pio-dirs.sh"],
                       cwd=ROOT, check=True)
    cmd = ["pio", "run", "-d", proj, "-e", env]
    if do_flash:
        cmd += ["-t", "upload", "--upload-port", port]
    print("Build:", " ".join(cmd))
    subprocess.run(cmd, cwd=ROOT, check=True)


def build_parser():
    """The argument parser, split out so the tests can call it.

    The channel-budget refusal in derive() is only reachable through the CLI,
    and a rule nothing can reach is a rule that silently stops being true.
    """
    p = argparse.ArgumentParser(description="Target-agnostic Saleae harness.")
    p.add_argument("--arch", choices=ARCHS, default="esp32")
    p.add_argument("--framework", choices=["arduino", "idf"], default="arduino")
    p.add_argument("--imux", default=None,
                   help="comma list data,bclk,ws: bring the ESP32 I2S "
                        "multiplexer up over serial before the run. Wired at "
                        "runtime rather than compiled in, because "
                        "initI2sMux() must precede any mux stepper and cannot "
                        "run twice -- so this is three pins on existing "
                        "firmware, not a rebuild")
    p.add_argument("--version", default="latest",
                   help="build version, e.g. 5.3 / V6_13_0 / latest (ESP only)")
    p.add_argument("--driver", default=None,
                   help="driver family; defaults per arch "
                        "(esp: rmt_v2, avr: timer, pico: pio)")
    p.add_argument("--drivers", default=None,
                   help="per-stepper drivers, one per stepper, e.g. "
                        "rmt,mcpwm; overrides --driver")
    p.add_argument("--count", type=int, default=DEFAULT_COUNT,
                   help="number of steppers to connect (1..4 with dir, "
                        "1..8 with nodir: 8 channels, 2 or 1 per stepper)")
    p.add_argument("--pin-mode", choices=PIN_MODES, default="dir",
                   help="two channels per stepper (dir) or one (nodir)")
    # The two generic modes. Both generate runs from a rule rather than naming
    # them, and neither names an architecture: `scale` is how a single driver
    # behaves from 1 up to its stepper limit, `sync` is how every driver-list
    # combination on a multi-driver board starts and whether each stepper keeps
    # its own speed. Without --mode the catalogue scenarios run instead.
    p.add_argument("--mode", choices=MODES, default=None,
                   help="scale: 1..driver-max on one driver. sync: every "
                        "driver-list combination. (default: the SR catalogue)")
    p.add_argument("--sync-count", type=int, default=2,
                   help="steppers per sync combination (default: 2; skew "
                        "needs two to exist)")
    p.add_argument("--tests", help="comma list (default: all)")
    p.add_argument("--flash", action="store_true", help="build + flash first")
    p.add_argument("--build", action="store_true", help="build first (no flash)")
    p.add_argument("--dry-run", action="store_true")
    p.add_argument("--list", action="store_true", help="list recorded results")
    # pass-through to run_tests
    p.add_argument("--port", default="/dev/cu.usbserial-0001")
    p.add_argument("--baud", type=int, default=115200)
    p.add_argument("--sample-rate", type=int, default=0,
                   help="0 = auto from --speed-us (>=20 samples/step)")
    p.add_argument("--seconds", type=float, default=5.0)
    # run_tests.py names the capture *directory* and has a separate rate for
    # SR_00's 1 Hz pin pattern, which needs no more than 1 MS/s. Both were
    # missing here, so any non-dry-run invocation died on the first capture
    # with an AttributeError -- the documented `--flash` example had never
    # actually run. The vestigial --capture (a .sr *file*) is kept so an old
    # command line does not become an unknown-option error; its directory is
    # what gets written.
    p.add_argument("--capture-dir", default="capture",
                   help="where .sr/.vcd captures go")
    p.add_argument("--capture", default=None,
                   help=argparse.SUPPRESS)
    p.add_argument("--sr00-sample-rate", type=int, default=1_000_000,
                   help="SR_00 needs no more than 1 MS/s (default: 1000000)")
    p.add_argument("--results-dir", default="results")
    p.add_argument("--steps", type=int, default=400)
    p.add_argument("--speed-us", type=int, default=400,
                   help="us per step; 25 = 40 kHz, 5 = 200 kHz")
    p.add_argument("--force", action="store_true")
    return p


def parse_args(argv=None):
    args = build_parser().parse_args(argv)
    if args.driver is None:
        args.driver = DRIVERS[arch_family(args.arch)][0]
    return args


def plan_scale(args):
    """The `scale` plan: 1..bound on one driver, plus which bound stopped it.

    Only one driver is swept per invocation, because that is what makes the
    result attributable: "RMT reaches 8 steppers in nodir" is a statement about
    RMT, and a sweep over several drivers at once would produce the same number
    without saying which driver it came from. --driver names the one.

    `--count` is deliberately ignored here: the count *is* the sweep axis, so
    honouring it would silently start the loop somewhere other than 1.
    """
    count_max, bound = scale_bound(args.arch, args.driver, args.pin_mode)
    plans = run_tests.scale_plan(args.driver, args.pin_mode, count_max)
    print(f"scale  : {args.driver}, {args.pin_mode}, counts 1..{count_max}")
    print(f"         sweeping 1..{count_max}, bound: {bound}")
    print(f"         the board decides where it stops: a CONFIG refusal is the "
          f"measured bound")
    return plans


def plan_sync(args):
    """The `sync` plan: every driver-list combination this board could connect.

    The driver list is the architecture's, from DRIVERS -- every name it
    offers, with repetition, two at a time. Including every name is what makes
    "a driver nobody has connected is one flag away" true rather than
    aspirational: `i2s_mux` is in the ESP32 list, so a sync run attempts it,
    and if the build cannot connect it that is recorded as a refusal next to
    the combinations that did measure, rather than being quietly excluded.

    A board with a single driver has no combinations, and saying so is more
    useful than emitting one "combination" of that driver with itself: `scale`
    is the mode that applies there, and the sync table would otherwise show a
    same-driver row indistinguishable from a real cross-driver measurement.
    """
    named = DRIVERS[arch_family(args.arch)]
    # Over identities, not spellings: `rmt` and `rmt_v2` are one driver, so
    # enumerating both would put rmt_v2+rmt in the table as if it were a
    # cross-driver combination. See DRIVER_IDENTITY.
    drivers = driver_identities(named)
    print(f"sync   : {args.arch} drivers {drivers} "
          f"(from {', '.join(named)}), "
          f"{args.sync_count} steppers per combination")
    if len(drivers) < args.sync_count:
        raise SystemExit(
            f"--mode sync needs at least {args.sync_count} drivers on "
            f"{args.arch}, which has {len(drivers)} ({', '.join(drivers)}). "
            f"That is what --mode scale is for: one driver is not a "
            f"combination, and printing one would look like a cross-driver "
            f"measurement.")
    return run_tests.sync_plan(drivers, args.pin_mode, args.sync_count)


def main():
    args = parse_args()

    if args.mode == "sync" and args.driver is not None and args.count != \
            DEFAULT_COUNT:
        raise SystemExit("--mode sync builds its own driver lists; --count "
                         "and --driver do not apply")

    tag_key, proj, env, rate = derive(args)
    args.sample_rate = rate

    if args.list:
        run_tests.show_list(args)
        return 0

    print(f"target : {args.arch} / {args.framework} / {args.version}")
    if args.mode:
        print(f"mode   : {args.mode}")
    else:
        print(f"driver : {driver_list(args)}  "
              f"config: {args.count} steppers, {args.pin_mode}")
    print(f"tag key: {tag_key}")
    print(f"project: {proj}  env: {env}")
    print(f"rate   : {rate} Hz  ({rate / 1_000_000 * args.speed_us:.0f} "
          f"samples per {args.speed_us} us step)")

    if args.dry_run:
        if args.mode:
            plans = plan_scale(args) if args.mode == "scale" \
                else plan_sync(args)
            print()
            print(run_tests.summarize([{"label": p.label,
                                        "drivers": p.drivers,
                                        "count": p.count,
                                        "result": p.wire} for p in plans]))
        return 0

    build_and_flash(proj, env, args.port, args.build, args.flash)

    args.dut_driver = args.driver
    # run_tests.run_modes() records these into every mode result; the report
    # groups its tables by them. harness owns them, so set them from here.
    args.sdk_version = args.version
    # The board is asked what it accepts, after the flash, so the answer is
    # about the firmware that is actually running. The answer is recorded in
    # every mode result, so a table can be read without the hardware attached.
    args.board_drivers, args.mux_init = preflight(args, args.port, args.baud)
    if args.mode:
        plans = plan_scale(args) if args.mode == "scale" else plan_sync(args)
        print()
        summary = run_tests.run_modes(tag_key, plans, args, args.mode)
        print()
        print(run_tests.summarize(summary))
        return 0

    tests = run_tests.parse_tests(args.tests)
    run_tests.run(tag_key, tests, args)
    return 0


if __name__ == "__main__":
    sys.exit(main())
