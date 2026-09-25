#ifndef NAXES_PATH_H
#define NAXES_PATH_H

#include <stdint.h>

// Fixed, deterministic path for the naxes example. Coordinates are absolute
// stepper positions in steps. The path runs, in order:
//
//   1. helix      - a full circle in the XY plane while Z climbs steadily,
//                   approximated by NAXES_HELIX_SEGMENTS straight chords.
//   2. hexagon    - six straight sides in the XY plane (Z held at the helix
//   end).
//   3. square     - four straight sides (a square in the XY plane; on a 2-axis
//                   build this is the "cube" degenerate, still four corners).
//   4. return     - a direct line back to the origin, the start of the path.
//
// The geometry is exact-chord (Line mode): each addLine() target is a vertex,
// so the realized curve is the polygon through the vertices. The simavr/PC test
// reconstructs this polygon from the Step/Dir pin traces and checks that the
// realized vertices match NAXES_PATH[] and that the run ends at the origin.

// Circle / polygon radius in steps.
#define NAXES_RADIUS 400
// Number of straight chords per quarter of the helix circle. A full turn is
// 4 * NAXES_QSAMPLES vertices.
#define NAXES_QSAMPLES 12
// Number of full turns during the helix.
#define NAXES_HELIX_TURNS 3
// Z climb per helix turn, in steps.
#define NAXES_HELIX_Z_PER_TURN 100

// Hexagon vertices (XY, at Z = end-of-helix Z).
// A regular hexagon has six vertices at angles 0, 60, 120, 180, 240, 300.
// int(400*cos(a)), int(400*sin(a)) for a in {0,60,120,180,240,300} degrees.
//   0   deg: ( 400,   0)
//   60  deg: ( 200, 346)   sin60=0.8660 -> 346
//   120 deg: (-200, 346)
//   180 deg: (-400,   0)
//   240 deg: (-200,-346)
//   300 deg: ( 200,-346)

// Square vertices (XY, at Z = end-of-helix Z), radius NAXES_SQUARE_HALF=300.
// A square inscribed on the axes: (+/-300, +/-300).
#define NAXES_SQUARE_HALF 300

// Short-side height of the hexagon corners, in steps: int(R * sin(60)) =
// int(R * 866 / 1000). The L keeps the product in 32 bits so it does not
// overflow on 16-bit-int platforms (AVR).
#define NAXES_HEX_Y (NAXES_RADIUS * 866L / 1000)

// The path is built at runtime in naxes.ino from these constants so the same
// constants drive both the commanded addLine() targets and the expected
// reconstruction, keeping the two sides identical.

// The helix is emitted as a table of vertices by the caller; this file only
// carries the constants. The caller (naxes.ino) generates the helix vertices
// with a fixed quarter-sine table (NAXES_SIN_QUAD, 91 entries, like
// MoveTimed.ino) so it stays float-free and deterministic.

// Quarter sine / cosine tables for a radius of NAXES_RADIUS, sampled at
// x = k * 90 / NAXES_QSAMPLES degrees for k in 0..NAXES_QSAMPLES-1:
//   NAXES_SIN_Q[k] = int(NAXES_RADIUS * sin(x*PI/180))
//   NAXES_COS_Q[k] = int(NAXES_RADIUS * cos(x*PI/180))
// Together they place every helix vertex exactly on the circle of radius
// NAXES_RADIUS (a rounding error of well under one step).
static const int16_t NAXES_SIN_Q[NAXES_QSAMPLES] = {
    0, 52, 104, 153, 200, 244, 283, 317, 346, 370, 386, 397};
static const int16_t NAXES_COS_Q[NAXES_QSAMPLES] = {
    400, 397, 386, 370, 346, 317, 283, 244, 200, 153, 104, 52};

#endif
