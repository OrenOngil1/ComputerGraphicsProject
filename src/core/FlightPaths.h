#pragma once

#include <cstddef>

#include "Simulation.h"

// Canned camera paths: geometry only, writing a flight into sim.waypoints and
// sim.pathPoints. Shared by RECORD's O key and FEATURE MATCH's Ctrl+G; every
// radius, altitude and stop count is calibrated -- the .cpp says by what.

// The three shapes Ctrl+G can fly. Their default stop counts differ because
// only the circle couples to SIFT's viewpoint tolerance (see the .cpp).
enum class AutoPathMode { Arc, Circle, Scattered };

struct AutoPathSpec {
    const char *name;          // as printed in the build summary
    size_t      defaultViews;
    const char *flyingHint;    // where recognition is strongest, printed after the build
};

// Indexed by AutoPathMode.
extern const AutoPathSpec kAutoPathSpecs[3];

// Lay `views` stops of the chosen shape, clearing the existing flight first: a
// path and its waypoints are one recording.
void layOutAutoPath(Simulation &sim, AutoPathMode mode, size_t views);

// Lay the calibrated hand-build ring (O in RECORD): 16 stations at the arc's
// measured-good radius and altitude, all aimed at the centre. Lays the path
// ONLY -- the anchoring stays entirely by hand.
void layOutBuildRing(Simulation &sim);
