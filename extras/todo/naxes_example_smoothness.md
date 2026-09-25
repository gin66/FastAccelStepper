# naxes example smoothness (hardware / simavr)

Priority: **P3** — end-to-end validation; depends on P1 and P2.

Status: detector in place (`detect_geometry.py` `path-stops` check).
The planner cruises a 7.5° helix (PC, and on hardware after P1). The
feeder batches fast equal-period steps (P2, PC). simavr `test_naxes`
has not been re-run since those changes; it last failed with
`stops=156`.

## Problem

On hardware the `examples/naxes/` run stops both motors at every helix
chord (`~1 stop/s`); the square alternates the axes. This is the Linear
per-vertex path-stop, not pump timing or an AVR speed problem — the slow
points land exactly on the helix vertices and the run is underrun-free
(whitepaper §6.3).

## Deliverable

1. `linear_junction_carry.md` makes the helix cruise (matched-role
   masters keep `P`); the square corners stay legitimate stops.
2. `feeder_command_batching.md` keeps AVR/ESP32 queues from draining at
   the application's `pump()` interval.
3. Re-run `extras/tests/simavr_based/test_naxes` and tune
   `MAX_PATH_STOPS` in `detect_geometry.py` to the legitimate stops
   (expect: start + path end + the square/idle corners, not the 144
   helix chords).
4. Re-check on hardware; keep the fixed path/geometry constants.

## References

- `examples/naxes/`, `extras/tests/simavr_based/test_naxes/`
- `extras/doc/n_axes_whitepaper.md` §6.3 worked circle model
