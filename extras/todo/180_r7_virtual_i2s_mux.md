# 177 R7 — Flexible channel count for the test harness

## Goal

Make the entire test harness (not just the Saleae capture) able to handle an
arbitrary number of channels — currently the harness is hard-coded to **8
channels** (the Saleae clone's limit).  R7 must let the harness work with
**8 + 32 channels** (the virtual I2S mux output) and any other count, without
new code paths for each configuration.

## Scope

### What must change

The harness currently assumes:

- Exactly 8 analyzer channels (D0–D7)
- A fixed channel-to-stepper mapping (4 steppers in `dir`, 8 in `nodir`)
- Hard-coded channel names in evaluators (`D0`, `D2`, `D4`, `D6` for step
  pins)
- No concept of mux-slot channels (S0–S31) or I2S bus channels (data, bclk,
  ws)

R7 must:

1. **Parse any number of channels** from the VCD — no 8-channel constant.
2. **Accept a channel map** (from `MAP` or a config file) that assigns each
   stepper to any channel by name, not by index.
3. **Support mux-slot channels** (S0–S31) and I2S bus channels (data, bclk,
   ws) as first-class channels in the harness.
4. **Make evaluators channel-agnostic** — they receive a `Pins` object that
   maps steppers to channels, not hardcoded channel names.
5. **The virtual mux script** (`virtual_i2s_mux.py`) must produce a VCD the
   harness can ingest without modifications.

### What stays the same

- The Saleae capture itself still records 8 channels (hardware limit).
- The virtual mux script synthesises 32 mux-slot signals + 3 I2S bus signals
  from those 8 channels.
- The harness evaluates whatever the VCD contains.

## Implementation

`scripts/virtual_i2s_mux.py` exists and produces a 43-channel VCD.  The
harness must be updated to:

- Read the VCD regardless of channel count.
- Accept a channel map that includes S0–S31 and I2S bus channels.
- Pass the map to every evaluator so they can look up channels by stepper,
  not by hard-coded name.

## Status

**Script exists, harness not yet updated.**  The harness still hard-codes 8
channels and will reject or misinterpret a 43-channel VCD.
