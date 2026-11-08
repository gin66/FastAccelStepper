# Pico — registry `platform = raspberrypi` and issue #339

Priority: **034**. `extras/doc/platforms/pico.md` tells users the
standard PlatformIO platform does not work with PIO.

Status: open. Every CI and saleae Pico env uses
`https://github.com/maxgerhardt/platform-raspberrypi.git`. The registry
id `platform = raspberrypi` has not been built or captured for the
current tree.

## What #339 was

Issue #339 (2025-10, SKR Pico) used:

```
platform = raspberrypi
board = bigtreetech_SKR_Pico
framework = arduino
```

Each motor ran alone. The two together were jerky. The warning was
added in `00462de`, with the git URL as the alternative, for both
`rpipico` and `rpipico2`.

"The registry package runs" and "the Arduino mbed core runs" are
different outcomes. The registry package may now select earlephilhower
by default. The warning is about the core that `platform = raspberrypi`
actually compiles.

## Work items

- Resolve `platform = raspberrypi` in a scratch env. Record the package
  version and which core it selects. Leave
  `pio_dirs/saleae/platformio.ini` unchanged.
- Build the saleae app for this Pico 2 from that platform. A missing
  `rpipico2` board, a missing PIO header, or a link error is the result.
- If it flashes, capture SR_01 (one stepper) and SR_16 (two steppers,
  one period). SR_16 is the #339 shape: a green SR_01 with a bad SR_16
  is the old failure, and both green means this registry platform runs
  the PIO driver on RP2350.
- If the registry default is earlephilhower, also build the mbed core
  (`board_build.core` left at that platform's mbed default, or named
  explicitly). Record that result separately.
- Keep or rewrite the warning in `pico.md` from whichever of those two
  cores fails, and name the core.

## References

- `extras/doc/platforms/pico.md` — the warning and the two example envs.
- Issue #339, commit `00462de`.
- `extras/ci/platformio.ini` — `[env:rpipico2]` uses the git URL.
- `extras/tests/saleae_based/scripts/run_tests.py` — SR_01, SR_16.
