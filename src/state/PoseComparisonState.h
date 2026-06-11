#pragma once

#include <optional>

#include "State.h"
#include "../core/Camera.h"    // Waypoint
#include "../core/PoseLog.h"

// Shared backbone of the automatic pose-estimation modes (Mode 3 TRACKERS,
// Mode 4 feature matching). Both fly the player camera freely, capture
// (true pose, computed pose) pairs per timestep, and display the comparison
// identically; the only thing that differs is HOW a pose is computed from the
// rendered frame -- so that is the one pure virtual (computePose), and
// everything else lives here, built once.
//
// Keys: 'B' captures a timestep -- the camera's pose right now is recorded as
// ground truth, and computePose estimates the same pose from the frame alone.
// 'N' / 'M' step the review cursor forward / back through the captured
// timesteps, snapping the player camera to that timestep's true pose.
//
// Display: the global view draws both fly-through paths -- true (blue path,
// red/green waypoint dots, exactly RECORD's language) and computed (estimate
// orange) -- so the two trajectories can be compared at a glance. The player
// view, while the camera sits on the reviewed true pose, overlays the computed
// pose as a translucent ghost: the closer the alignment, the better that
// timestep's estimate.
class PoseComparisonState : public State {
public:
    void handleKey(Simulation &sim, Renderer &renderer, int key, int mods) override;
    void tick(Simulation &sim, GLFWwindow *window, float dt) override;   // free flight
    void renderGlobalOverlay(const Simulation &sim, Renderer &renderer,
                             const glm::mat4 &mvp) const override;
    void renderPlayerOverlay(const Simulation &sim, Renderer &renderer,
                             const glm::mat4 &mvp) const override;

protected:
    // Estimate the player camera's pose from the current frame -- the one step
    // the modes implement differently. Returns empty when it can't (too few
    // correspondences, solver failure); 'B' logs the timestep either way, so
    // the timelines stay aligned.
    virtual std::optional<Waypoint> computePose(Simulation &sim, Renderer &renderer) = 0;

    PoseLog m_log;

private:
    // Put the player camera on the reviewed timestep's true pose, so the view
    // shows what the camera truly saw there and the ghost diff reads against it.
    void snapToCurrent(Simulation &sim) const;
};
