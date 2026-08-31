# Step timebase is exactly 16 MHz from a dedicated GCLK generator

The queue TCCs are clocked at exactly 16 MHz by a dedicated generic clock
generator (default GEN6, override `FAS_SAMD_GCLK_GEN`) sourced from the 48 MHz
DFLL with divider 3 — not from the natural 120 MHz GCLK0 divisions. The ramp
generator's fixed-point log2 math ships precomputed constants only for the
16 MHz (AVR/ESP32/Pico) and 21 MHz (SAM Due) tick rates; every other value
routes into a generic runtime-computed branch that has never been used by any
platform and does not currently compile (static/extern declaration rot). Using
16 MHz keeps the port on the same well-tested math path as all proven ports,
makes tick values 1:1 comparable with upstream docs and tests, and leaves the
library core untouched. Cost: the port claims one of the SAMD51's 12 clock
generators (the Adafruit core startup uses 0–5, including GEN5 as the 1 MHz
DPLL reference — clobbering that crashes the chip; 6–11 are free).
