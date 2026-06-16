#pragma once

#include <optional>
#include <vector>

#include "Camera.h"   // Waypoint

// One captured timestep of a pose-comparison mode (TRACKERS / feature
// matching): the player camera's true pose at the moment 'B' was pressed,
// paired with the pose PnP computed from that frame alone. computedPose is
// empty when the solve was skipped (too few correspondences) or failed --
// the timestep is logged anyway, so the true and computed timelines stay
// aligned index-for-index.
struct PoseEntry {
    Waypoint truePose;
    std::optional<Waypoint> computedPose;
};

// The captured timeline plus a review cursor for stepping through it (N/M).
// Owned by the mode state that fills it, so it is born and dies with the mode
// -- the same lifetime choice as PlaybackState's index.
struct PoseLog {
    std::vector<PoseEntry> entries;
    size_t current = 0;   // entry under review; meaningful only when entries is non-empty

    void add(PoseEntry entry)
    {
        entries.push_back(std::move(entry));
        current = entries.size() - 1;   // review follows the newest capture
    }

    // Step the review cursor one entry forward (+) or back (-), wrapping.
    void step(int direction)
    {
        const size_t n = entries.size();
        if (n == 0)
            return;
        current = (direction > 0) ? (current + 1) % n
                                  : (current + n - 1) % n;   // n - 1 is safe: n >= 1
    }
};
