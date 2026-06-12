#pragma once

#include <optional>
#include <vector>

#include "PoseComparisonState.h"
#include "../core/Scene.h"    // Tracker
#include "../core/Camera.h"   // Waypoint

// Mode 3: automatic pose estimation from colored tracker spheres. onEnter
// scatters uniquely colored spheres across the terrain surface; on each 'B'
// the inherited capture flow records the true pose and asks computePose for
// the estimate -- which it finds with no user clicks at all: render the
// detection frame, locate each tracker's color blob, pair blob centroids with
// the known sphere centers, and solve PnP. Flight, N/M timestep review, and
// both comparison overlays come from PoseComparisonState.
//
// In its own file (not States.h) per the guidance there: substantial modes
// split out.
class TrackersState : public PoseComparisonState {
public:
    void onEnter(Simulation &sim) override;   // scatter the trackers
    void renderGlobalOverlay(const Simulation &sim, Renderer &renderer,
                             const glm::mat4 &mvp) const override;
    void renderPlayerOverlay(const Simulation &sim, Renderer &renderer,
                             const glm::mat4 &mvp) const override;

protected:
    // The automatic correspondence + PnP described above. Empty when fewer
    // than 4 trackers are visible or the solver fails.
    std::optional<Waypoint> computePose(Simulation &sim, Renderer &renderer) override;

private:
    std::vector<Tracker> m_trackers;   // placed once per mode entry
};
