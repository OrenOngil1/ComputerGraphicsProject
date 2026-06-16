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
    // Palette bounds, public so the T-key prompt (Callbacks.cpp) and this class
    // share one source of truth for the valid range and its default. kMaxCount
    // must equal the palette size in the .cpp -- a static_assert there enforces
    // it, so growing the palette without updating this constant fails the build.
    static constexpr size_t kDefaultCount = 7;
    static constexpr size_t kMaxCount     = 20;

    // The tracker count is transition-time configuration: chosen once at the
    // T-key prompt, fixed for the mode's lifetime (press T again to change it).
    explicit TrackersState(size_t count = kDefaultCount);

    // The T-key terminal prompt (1..kMaxCount, plain Enter = the default).
    // Lives with the state so the range policy has one home; the transition
    // table just writes TrackersState(TrackersState::promptCount()).
    static size_t promptCount();

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
    size_t                m_count;      // how many trackers onEnter scatters
    std::vector<Tracker>  m_trackers;   // placed once per mode entry
};
