# 174 AVR RAM was 51% string literals

## Priority

**MEDIUM** — not a defect, but a critical resource constraint that was found
and fixed.  Important for any future AVR builds.

## Finding

AVR `.rodata` was **1040 B of 2048 B** (51%) — the whole string pool.  avr-gcc
puts `.rodata` in RAM, so every string literal consumes precious SRAM.

Before fix: `.data` 1136 B (96 B variables + **1040 B** string pool), `.bss`
832 B, **80 B free**.

After fix: `.data` 20 B, `.bss` 832 B, **1196 B free**.

## Fix

`common/saleae_str.h` gives target-independent macros:

- `SAL_PSTR` — flash-string macro
- `sal_strcmp`, `sal_sscanf`, `sal_snprintf` — flash-string variants
- `sal_to_ram` — copy into caller's buffer
- `sal_pgm_read_*` — flash-memory accessors
- `saleae_hal_serial_write_p` — carries a flash string to the port

`const` tables went too — `kChanPin`, `saleae_pins`, `saleae_high_ms` were
52 B of SRAM for data nothing writes.

`.text` grew 22664 → 24210 B for the `_P` libc variants, which is the right
trade (6.5 KB of flash spare).

## Two things worth remembering

1. **`const` is not free on AVR either** — the fix was not only about strings.
2. **Not even a delimiter is free:** `strtok(driver_list, ",")` was 2 bytes of
   SRAM, so it is now a `char[2]` built from char constants.

## Test

`TestAvrRamBudget` (`scripts/tests/test_saleae.py`) enforces: no bare literal
in `common/` outside `SAL_PSTR`, no literal reply through `reply()`,
`SAL_PROGMEM` on the `const` tables.  Each assertion was checked against a
planted regression.  **This matters because no host-side test can see it and
a 328P build only fails once the part is full.**
