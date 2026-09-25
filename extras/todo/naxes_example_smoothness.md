# naxes example smoothness (hardware / simavr)

Priority: **P3** — end-to-end validation; depends on P1 and P2.

Status: **implemented**. simavr `test_naxes` runs under the geometry
detector and passes with `stops=11` (`MAX_PATH_STOPS = 11`). The 11 are
the legitimate stops: rest at the start, the helix entry (the incoming
master reverses), the four hexagon corners where an in-line axis goes
idle, the four axis-aligned square corners, and the return to the
origin. No helix chord stops.

## Problem

On hardware the `examples/naxes/` run stops both motors at every helix
chord (`~1 stop/s`); the square alternates the axes. This is the Linear
per-vertex path-stop, not pump timing or an AVR speed problem — the slow
points land exactly on the helix vertices and the run is underrun-free
(whitepaper §6.3).

## Finding: the block ring was not sliding

After P1 and P2 the simavr run still showed 15 stops, and the excess four
were not ramp or geometry: they sat on helix vertices 63/64 and 127/128.
`FasNAxis`'s block array only grew (`_n_blk++` in `addLine`) and was
never compacted, so after `HORIZON` points had *ever* been appended
`addLine` back-pressured until the whole buffered plan drained and
`pump()` reset the ring. The 155-point path was therefore executed in
chunks of `HORIZON = 64`, each ramping to rest at its end (the entry
into the next chunk re-accelerated from 0). `compact_ring()` now drops
executed blocks and rebases `_head` to 0, so the array is a sliding
window of at most `HORIZON` *pending* points as whitepaper §8 describes.

## Deliverable

1. `linear_junction_carry.md` makes the helix cruise (matched-role
   masters keep `P`); the square corners stay legitimate stops. Done.
2. `feeder_command_batching.md` keeps AVR/ESP32 queues from draining at
   the application's `pump()` interval. Done.
3. `FasNAxis::addLine()` slides the block ring, so a path longer than
   `HORIZON` no longer ramps to rest at each ring boundary. Done.
4. Re-run `extras/tests/simavr_based/test_naxes` and tune
   `MAX_PATH_STOPS` in `detect_geometry.py` to the legitimate stops
   (11, not the 144 helix chords). Done.
5. Re-check on hardware; keep the fixed path/geometry constants. Open —
   the simavr trace and the PC ring behaviour are the evidence so far.

## References

- `examples/naxes/`, `extras/tests/simavr_based/test_naxes/`
- `extras/doc/n_axes_whitepaper.md` §6.3 worked circle model
