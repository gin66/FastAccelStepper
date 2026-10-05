# SR_24 — AVR timer timings

**Status:** not implemented (skipped in all platform runs)

**White paper description (§5.2):**
> Each AVR timer channel produces the commanded period on its OC pin.

**Program:** `QSEG 64 <ticks> 1` on each of Timer1/3/4/5

**Why skipped:** Requires an AVR board. Current test hardware is ESP32-only.

**What implementing it means:**
- Run on an AVR (e.g. 328P, as in the `saleae_avr` platformio env).
- For each timer channel (Timer1, Timer3, Timer4, Timer5), send a 64-step command.
- Verify each timer's output compare (OC) pin produces the commanded period.
- Must cover all timer channels available on the target AVR.

**Dependencies:**
- Hardware: AVR board (328P or similar).
- Driver: `timer` (AVR-specific).
- Channel config: one channel per timer (stepper mapped to the OC pin).

**References:**
- White paper: `white_paper_saleae_test_harness.md` §5.2, row SR_24.
- Implemented note: "SR_24 needs an AVR board" (line ~1250).
- Related: the `saleae_avr` platformio env already exists in `apps/arduino/platformio.ini`.