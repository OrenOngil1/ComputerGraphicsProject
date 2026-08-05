#pragma once

#include <cmath>
#include <optional>
#include <vector>

#include <glm/glm.hpp>

#include "Camera.h"   // Waypoint

// One captured timestep of a pose-comparison mode: the player camera's true
// pose when 'B' was pressed, paired with the pose PnP computed from that frame
// alone. computedPose is empty when the solve was skipped or failed -- the
// timestep is logged anyway, so the true and computed timelines stay aligned
// index-for-index.
struct PoseEntry {
    Waypoint truePose;
    std::optional<Waypoint> computedPose;
};

// How far a computed pose is from the truth it was solved for.
//
// Two numbers, because a pose is two things. A camera can sit in exactly the
// right place looking the wrong way -- a real failure of these modes, since PnP
// solves position and orientation together and a contaminated consensus can
// trade one against the other -- and position error alone would report that as
// a success.
struct PoseError {
    float positionUnits;    // distance between the two eyes, in world units
    float headingDegrees;   // angle between the two view directions
};

inline PoseError poseError(const Waypoint &truth, const Waypoint &estimate)
{
    const glm::vec3 trueForward     = truth.target - truth.position;
    const glm::vec3 estimateForward = estimate.target - estimate.position;

    // A degenerate pose (eye sitting on its own target) has no direction to
    // compare; report the position half rather than a NaN angle.
    float headingDegrees = 0.0f;
    if (glm::length(trueForward) > 0.0f && glm::length(estimateForward) > 0.0f) {
        const float cosAngle = glm::clamp(glm::dot(glm::normalize(trueForward),
                                                   glm::normalize(estimateForward)),
                                          -1.0f, 1.0f);
        headingDegrees = glm::degrees(std::acos(cosAngle));
    }

    return { glm::length(estimate.position - truth.position), headingDegrees };
}

// The captured timeline plus a review cursor for stepping through it (N/M).
// Owned by the mode state that fills it, so it is born and dies with the mode.
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
                                  : (current + n - 1) % n;
    }
};
