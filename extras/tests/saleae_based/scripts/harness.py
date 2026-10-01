#!/usr/bin/env python3
"""
harness.py — high-level, target-agnostic front-end for the Saleae harness.

Describe the target and the test; the harness derives the rest and drives the
low-level orchestrator (run_tests.py).

    # ESP32, Arduino framework, latest
    python3 scripts/harness.py --arch esp32 --framework arduino \
        --driver rmt_v2 --channel-config 8ch_step_only --tests SR_01 \
        --steps 4000 --speed-us 5 --flash

    # ESP32, ESP-IDF 5.3, MCPWM
    python3 scripts/harness.py --arch esp32 --framework idf --version 5.3 \
        --driver mcpwm_pcnt --tests SR_01 --speed-us 5 --flash

    # AVR at 40 kSteps/s (25 us/step), Timer driver
    python3 scripts/harness.py --arch nanoatmega328 --driver timer \
        --tests SR_01 --steps 2000 --speed-us 25 --flash

    # Pico at 200 kHz (5 us/step), PIO driver
    python3 scripts/harness.py --arch rpipico --driver pio \
        --tests SR_01 --steps 4000 --speed-us 5 --flash

From --arch/--framework/--version/--driver/--channel-config it computes the tag
key, the PlatformIO project dir + env, and (unless given) a capture sample rate
that is >= 20 samples per step period. --dry-run prints the mapping only.

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
    "esp": ["rmt_v2", "rmt", "mcpwm_pcnt", "i2s_direct", "i2s_mux", "mixed"],
    "avr": ["timer"],
    "pico": ["pio"],
    "sam": ["timer"],
}

CHANNEL_CONFIGS = ["8ch_step_only", "7ch_shared_dir", "4ch_rmt", "4ch_mcpwm",
                   "2rmt_2i2s", "4ch_i2s_extender", "6ch_i2s_mux", "mixed"]

# fx2lafw-supported samplerates (48 MHz / n).
SUPPORTED_RATES = [20000, 25000, 50000, 100000, 200000, 250000, 500000,
                   1_000_000, 2_000_000, 3_000_000, 4_000_000, 6_000_000,
                   8_000_000, 12_000_000, 16_000_000, 24_000_000, 48_000_000]


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

    raw = f"{args.arch}_{fw}_{args.driver}_{args.channel_config}"
    tag_key = "".join(c if (c.isalnum() or c == "_") else "_" for c in raw)

    rate = args.sample_rate
    if rate <= 0:
        # Must resolve the step PULSE, not just the period: the high time can
        # be a few microseconds, so use a floor of 4 MS/s (0.25 us/sample) and
        # at least 20 samples per step period.
        step_freq = 1_000_000.0 / args.speed_us
        rate = snap_rate(max(4_000_000, int(20 * step_freq)))
    return tag_key, proj, env, rate


def build_and_flash(proj, env, port, do_build, do_flash):
    if do_build or do_flash:
        subprocess.run(["bash", "extras/scripts/build-pio-dirs.sh"],
                       cwd=ROOT, check=True)
    cmd = ["pio", "run", "-d", proj, "-e", env]
    if do_flash:
        cmd += ["-t", "upload", "--upload-port", port]
    print("Build:", " ".join(cmd))
    subprocess.run(cmd, cwd=ROOT, check=True)


def main():
    p = argparse.ArgumentParser(description="Target-agnostic Saleae harness.")
    p.add_argument("--arch", choices=ARCHS, default="esp32")
    p.add_argument("--framework", choices=["arduino", "idf"], default="arduino")
    p.add_argument("--version", default="latest",
                   help="build version, e.g. 5.3 / V6_13_0 / latest (ESP only)")
    p.add_argument("--driver", default=None,
                   help="driver family; defaults per arch "
                        "(esp: rmt_v2, avr: timer, pico: pio)")
    p.add_argument("--drivers", default=None,
                   help="per-stepper drivers for --channel-config mixed, "
                        "e.g. rmt,mcpwm (sent as 'CONFIG mixed <list>')")
    p.add_argument("--channel-config", choices=CHANNEL_CONFIGS,
                   default="4ch_rmt")
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
    p.add_argument("--capture", default="capture.sr")
    p.add_argument("--results-dir", default="results")
    p.add_argument("--steps", type=int, default=400)
    p.add_argument("--speed-us", type=int, default=400,
                   help="us per step; 25 = 40 kHz, 5 = 200 kHz")
    p.add_argument("--force", action="store_true")
    args = p.parse_args()

    if args.driver is None:
        args.driver = DRIVERS[arch_family(args.arch)][0]

    tag_key, proj, env, rate = derive(args)
    args.sample_rate = rate

    if args.list:
        run_tests.show_list(args)
        return 0

    print(f"target : {args.arch} / {args.framework} / {args.version}")
    print(f"driver : {args.driver}  config: {args.channel_config}")
    print(f"tag key: {tag_key}")
    print(f"project: {proj}  env: {env}")
    print(f"rate   : {rate} Hz  ({rate / 1_000_000 * args.speed_us:.0f} "
          f"samples per {args.speed_us} us step)")

    if args.dry_run:
        return 0

    build_and_flash(proj, env, args.port, args.build, args.flash)

    tests = run_tests.parse_tests(args.tests)
    run_tests.run(tag_key, tests, args)
    return 0


if __name__ == "__main__":
    sys.exit(main())
