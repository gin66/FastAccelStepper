# Pico — registry `platform = raspberrypi` selects mbed, and that is issue #339

Closes the item tracked as `extras/todo/034_pico_registry_platform.md`. Resolved
on a clean PlatformIO core, built and measured on an RP2350
(`--arch rpipico2 --driver pio`, GPIO2..GPIO9 = D0..D7, Saleae clone).

## Summary

`platform = raspberrypi` from the PlatformIO registry is the **mbed** path, not
the earlephilhower/arduino-pico core that FastAccelStepper's PIO driver needs.
Version **1.20.0** (published 2026-06-25) declares exactly one Arduino
framework:

```
"frameworks": {
  "arduino": {
    "package": "framework-arduino-mbed",
    "script": "builder/frameworks/arduino/mbed-core/arduino-core-mbed.py"
  }
}
```

and ships **two boards**, `pico` and `nanorp2040connect`. `rpipico2` and
`bigtreetech_SKR_Pico` are not among them. So the registry is not a path that
"may now select earlephilhower by default": it has no earlephilhower core
package at all, and the boards the issue and this repository name do not exist
in it.

The fork every CI and Saleae Pico env uses —
`https://github.com/maxgerhardt/platform-raspberrypi.git` — is what adds
`framework-arduinopico` and the board list. It builds and runs the PIO driver;
the registry does not. The warning in `extras/doc/platforms/pico.md` stands, and
is now specific about the core and the boards.

## Measured

Registry package, isolated in a fresh `PLATFORMIO_CORE_DIR`
(`pio pkg install -g -p platformio/raspberrypi@1.20.0`), Saleae app from the
same sources:

| env | result |
|---|---|
| `board = rpipico2` | `UnknownBoard: Unknown board ID 'rpipico2'` |
| `board = bigtreetech_SKR_Pico` | `UnknownBoard: Unknown board ID 'bigtreetech_SKR_Pico'` |
| `board = pico` (mbed core) | framework builds; the app does not: `'TICKS_PER_S' was not declared in this scope`, `'rp2040' was not declared in this scope` |

The `pico` row is the core answer: the mbed Arduino core does not define the
`PICO_RP2040` / `PICO_RP2350` symbols `fas_arch/common.h` keys the Pico port on,
and it has no arduino-pico `rp2040` object. `SUPPORT_RP_PICO` is never selected,
so there is no PIO driver to run. It is not a missing header or a link error
that a flag could fix; the port is absent.

The board list is not a recent regression. Registry **1.18.0** (published
2025-10-23, the release contemporaneous with issue #339) also has only `pico`
and `nanorp2040connect` and only `framework-arduino-mbed`. A clean registry
install therefore could not have resolved `board = bigtreetech_SKR_Pico` at
all; #339's environment must have taken the board from the fork (or a local
board definition). The advice it produced — use the fork — is still right.

### The fork is the working path

```
platform = https://github.com/maxgerhardt/platform-raspberrypi.git
```

resolves to `raspberrypi@1.20.0+sha.5d4561a`, which adds
`framework-arduinopico@1.60100.0+sha.fd65f6d4` and the board list. With it:

| env | result |
|---|---|
| `rpipico2` | builds, flashes the Pico 2 |
| `rpipico` | builds |
| `bigtreetech_SKR_Pico` | builds |

On the RP2350 flashed from that build, SR_00, SR_01 (one stepper) and SR_16 (two
steppers, one period) all **pass** — the #339 shape, green on both counts. They
were run with `--no-build` against the freshly flashed firmware.

## A resolution footgun worth recording

`platform = raspberrypi` does not force a registry download when a platform
folder named `raspberrypi` is already installed. On a machine that has the fork
installed, a new `platform = raspberrypi` env **silently reuses the fork** and
builds — which is how the first scratch build here appeared to succeed. The
registry result above was obtained only by pointing `PLATFORMIO_CORE_DIR` at a
clean directory. Any future check of "does the registry work" has to start
there.

## Changes

- `extras/doc/platforms/pico.md` — the warning now names the core
  (`framework-arduino-mbed`), the two-board list and the missing
  `PICO_RP2040`/`PICO_RP2350` port, instead of "appears to not work".
- No config change: `extras/ci/platformio.ini` and the generated
  `pio_dirs/saleae/platformio.ini` already use the fork, and the item asked for
  them to stay that way.

## References

- `extras/doc/platforms/pico.md` — the warning and the two example envs.
- Issue #339, commit `00462de`.
- `extras/ci/platformio.ini` — `[env:rpipico2]` uses the git URL.
- `src/fas_arch/common.h` — `PICO_RP2040` / `PICO_RP2350` select the Pico port.
