#pragma once

#include <optional>
#include <vector>

#include "PoseComparisonState.h"
#include "../core/Scene.h"    // Tracker
#include "../core/Camera.h"   // Waypoint

// Mode C: automatic pose estimation from colored tracker spheres. onEnter
// scatters uniquely colored spheres across the terrain; computePose needs no
// user clicks -- render the detection frame, locate each tracker's color blob,
// pair blob centroids with the known sphere centers, and solve PnP. Flight,
// N/M review, and the comparison overlays come from PoseComparisonState.
class TrackersState : public PoseComparisonState {
public:
    static constexpr size_t kDefaultCount = 7;
    // Must equal the palette size in the .cpp (static_assert enforced there).
    static constexpr size_t kMaxCount     = 20;

    // count is fixed for the mode's lifetime; press T again to change it.
    explicit TrackersState(size_t count = kDefaultCount);

    // The T-key terminal prompt (1..kMaxCount, plain Enter = the default).
    static size_t promptCount();

    void onEnter(Simulation &sim) override;   // scatter the trackers
    void renderGlobalOverlay(const Simulation &sim, Renderer &renderer,
                             const glm::mat4 &mvp) const override;
    void renderPlayerOverlay(const Simulation &sim, Renderer &renderer,
                             const glm::mat4 &mvp) const override;

protected:
    // Blob detection + PnP as described above. Empty when fewer than 4
    // trackers are visible or the solver fails.
    std::optional<Waypoint> computePose(Simulation &sim, Renderer &renderer) override;

private:
    size_t                m_count;      // how many trackers onEnter scatters
    std::vector<Tracker>  m_trackers;   // placed once per mode entry
};
