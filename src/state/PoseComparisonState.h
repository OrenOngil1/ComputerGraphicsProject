#pragma once

#include <optional>

#include "State.h"
#include "../core/Camera.h"    // Waypoint
#include "../core/PoseLog.h"

// Shared backbone of the automatic pose-estimation modes (TRACKERS, FEATURE
// MATCH). Both fly the player camera freely, capture (true pose, computed
// pose) pairs per timestep, and display the comparison identically; only HOW a
// pose is computed from the rendered frame differs -- the one pure virtual.
//
// Keys: 'B' captures a timestep (the camera's current pose is the ground
// truth; computePose estimates it from the frame alone). 'N'/'M' step the
// review cursor through the captured timesteps, snapping the camera to each
// one's true pose.
//
// Display: the global view draws both fly-through paths (true in RECORD's
// visual language, computed in the estimate color); the player view, while the
// camera sits on the reviewed true pose, overlays the computed pose as a
// translucent ghost.
class PoseComparisonState : public State {
public:
    void handleKey(Simulation &sim, Renderer &renderer, int key, int mods) override;
    void tick(Simulation &sim, GLFWwindow *window, float dt) override;   // free flight
    void renderGlobalOverlay(const Simulation &sim, Renderer &renderer,
                             const glm::mat4 &mvp) const override;
    void renderPlayerOverlay(const Simulation &sim, Renderer &renderer,
                             const glm::mat4 &mvp) const override;

protected:
    // Estimate the player camera's pose from the current frame. Empty when it
    // can't (too few correspondences, solver failure); 'B' logs the timestep
    // either way, so the true and computed timelines stay aligned.
    virtual std::optional<Waypoint> computePose(Simulation &sim, Renderer &renderer) = 0;

    // Help text for the keys this class handles, for subclasses' onEnter banners.
    static constexpr const char *kCaptureHelp =
        "B = capture timestep, N/M = step through timesteps";

    PoseLog m_log;

private:
    // Put the player camera on the reviewed timestep's true pose.
    void snapToCurrent(Simulation &sim) const;
};
