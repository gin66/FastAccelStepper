# SR_22 — RMT V2 encoder

**Status:** not implemented (skipped in all platform runs)

**White paper description (§5.2):**
> RMT V2 fill-encoder output has no irregular gap.

**Program:** `QSEG 200 <ticks> 1`

**Why skipped:** Requires an ESP32 with RMT V2 hardware. Current test hardware has RMT V1.

**What implementing it means:**
- Run on an ESP32 variant with RMT V2 (e.g. ESP32-S3 or ESP32-C6).
- Send a 200-step command at the speed floor.
- Verify the capture shows no irregular inter-step gaps introduced by the V2 fill-encoder.
- Compare against SR_21 (RMT V1 buffer split) to confirm V2 does not introduce the same defect.

**Dependencies:**
- Hardware: ESP32 with RMT V2.
- Driver: `rmt` (not `i2s_*`, not `mcpwm_pcnt`).
- Channel config: `1ch` (single stepper, `dir` mode).

**References:**
- White paper: `white_paper_saleae_test_harness.md` §5.2, row SR_22.
- Implemented note: "SR_22 needs RMT V2 (this ESP32 has V1)" (line ~1250).
- Related: SR_21 (RMT V1 buffer split) — already implemented.