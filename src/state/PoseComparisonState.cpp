#include "PoseComparisonState.h"

#include <iomanip>
#include <iostream>
#include <sstream>
#include <string>
#include <vector>

#include <GLFW/glfw3.h>

#include "OverlayStyle.h"
#include "../core/Simulation.h"
#include "../input/CameraControls.h"   // fly, MovementIntent
#include "../input/Callbacks.h"        // pollMovementIntent (GLFW glue)
#include "../render/Renderer.h"

namespace {

// One phrase for "how did this capture do", shared by the single capture, the
// review cursor, and the all-views table, so the three cannot drift apart.
//
// The percentage is the point of it. A raw "position error 8.4" says nothing
// until you know how big the terrain is -- it is a tight result on a 633-unit
// DEM and a hopeless one on a 60-unit DEM, and the number alone reads the same.
std::string describe(const PoseEntry &entry, float terrainSize)
{
    if (!entry.computedPose)
        return "no pose computed";

    const PoseError error = poseError(entry.truePose, *entry.computedPose);

    std::ostringstream out;
    out << std::fixed << std::setprecision(1)
        << "position error " << error.positionUnits << " units";
    if (terrainSize > 0.0f)
        out << " (" << (100.0f * error.positionUnits / terrainSize) << "% of terrain)";
    out << ", heading off by " << error.headingDegrees << " deg";
    return out.str();
}

}  // namespace

void PoseComparisonState::tick(Simulation &sim, GLFWwindow *window, float dt)
{
    // fly only changes the pose while movement keys are held, so a pose
    // snapped to by N/M stays put until the user flies off.
    fly(sim.playerView.camera, pollMovementIntent(window), sim.terrainSize, dt);
}

void PoseComparisonState::snapToCurrent(Simulation &sim) const
{
    if (m_log.entries.empty())
        return;

    sim.playerView.camera.applyPose(m_log.entries[m_log.current].truePose);
}

// One capture per recorded waypoint, as a table.
//
// A single check of a pose estimator costs a flight plus a keypress per stop,
// which is slow enough that it discourages the very thing a change needs --
// measuring it again. Every row here is exactly what B would have logged at
// that waypoint, so the timesteps and the drawn paths stay the same kind of
// data; the only thing added is that the camera drives itself.
void PoseComparisonState::evaluateAllWaypoints(Simulation &sim, Renderer &renderer)
{
    if (sim.waypoints.empty()) {
        std::cout << "No recorded views to evaluate -- record some in RECORD mode"
                  << std::endl;
        return;
    }

    std::cout << "EVALUATE all " << sim.waypoints.size() << " recorded views:" << std::endl;

    float  errorSum = 0.0f;
    size_t solved   = 0;
    for (size_t i = 0; i < sim.waypoints.size(); i++) {
        sim.playerView.camera.applyPose(sim.waypoints[i]);
        m_log.add({ sim.playerView.camera.pose(), computePose(sim, renderer) });

        const PoseEntry &entry = m_log.entries.back();
        if (entry.computedPose) {
            errorSum += poseError(entry.truePose, *entry.computedPose).positionUnits;
            solved++;
        }
        std::cout << "  view " << (i + 1) << " of " << sim.waypoints.size() << ": "
                  << describe(entry, sim.terrainSize) << std::endl;
    }

    std::ostringstream summary;
    summary << std::fixed << std::setprecision(1)
            << "  solved " << solved << " of " << sim.waypoints.size();
    if (solved > 0) {
        const float mean = errorSum / (float)solved;
        summary << ", mean position error " << mean << " units";
        if (sim.terrainSize > 0.0f)
            summary << " (" << (100.0f * mean / sim.terrainSize) << "% of a "
                    << std::setprecision(0) << sim.terrainSize << "-unit terrain)";
    }
    std::cout << summary.str() << std::endl;
}

// Capture a timestep: the camera's pose right now is the ground truth;
// computePose estimates it from the rendered frame alone.
void PoseComparisonState::captureTimestep(Simulation &sim, Renderer &renderer)
{
    m_log.add({ sim.playerView.camera.pose(), computePose(sim, renderer) });

    const PoseEntry &entry = m_log.entries.back();
    std::cout << "CAPTURE " << m_log.entries.size() << " of " << m_log.entries.size()
              << ": " << describe(entry, sim.terrainSize);
    if (!entry.computedPose)
        std::cout << " -- see the reason above";
    std::cout << std::endl;
}

void PoseComparisonState::reviewStep(Simulation &sim, int direction)
{
    if (m_log.entries.empty()) {
        std::cout << "No captures yet -- press B to take one" << std::endl;
        return;
    }
    m_log.step(direction);
    snapToCurrent(sim);

    const PoseEntry &entry = m_log.entries[m_log.current];
    std::cout << "REVIEW " << (m_log.current + 1) << " of " << m_log.entries.size()
              << ": ";
    if (entry.computedPose)
        std::cout << describe(entry, sim.terrainSize) << std::endl;
    else
        std::cout << "no pose was computed here -- nothing to compare against"
                  << std::endl;

    // Say what reviewing IS, once. Without it this is a number that changes
    // and nothing else visibly happens -- the whole point is off to the side,
    // in the player pane.
    if (!m_reviewHintShown) {
        m_reviewHintShown = true;
        std::cout << "        (the player view is back on this capture's true pose; "
                  << "the orange ghost is the pose PnP computed from it)" << std::endl;
    }
}

void PoseComparisonState::handleKey(Simulation &sim, Renderer &renderer, int key, int mods)
{
    switch (key) {
        case GLFW_KEY_B:
            if (mods & GLFW_MOD_CONTROL) {
                const double now = glfwGetTime();
                if (now - m_lastBulkRun >= 0.5) {   // drop autorepeat
                    m_lastBulkRun = now;
                    evaluateAllWaypoints(sim, renderer);
                }
            } else {
                captureTimestep(sim, renderer);
            }
            break;

        // N = next capture, M = previous (mnemonic: minus). The camera snaps
        // back to that capture's true pose, so the ghost shows its diff.
        case GLFW_KEY_N:
        case GLFW_KEY_M:
            reviewStep(sim, key == GLFW_KEY_N ? +1 : -1);
            break;
    }
}

void PoseComparisonState::renderGlobalOverlay(const Simulation &sim, Renderer &renderer,
                                              const glm::mat4 &mvp) const
{
    if (m_log.entries.empty())
        return;

    // Split the log into the two trajectories. The computed one may be shorter
    // (skipped solves leave gaps); its path connects the poses that exist.
    std::vector<Waypoint>  truePoses;
    std::vector<glm::vec3> truePositions, computedPositions;
    truePoses.reserve(m_log.entries.size());
    truePositions.reserve(m_log.entries.size());
    for (const PoseEntry &entry : m_log.entries) {
        truePoses.push_back(entry.truePose);
        truePositions.push_back(entry.truePose.position);
        if (entry.computedPose)
            computedPositions.push_back(entry.computedPose->position);
    }

    // True fly-through in RECORD's visual language; computed in the estimate color.
    renderer.drawPath(truePositions, overlay::truePathColor, mvp);
    renderer.drawWaypoints(truePoses, sim.playerView.camera.position, mvp);

    renderer.drawPath(computedPositions, overlay::estimateColor, mvp);
    renderer.drawPoints(computedPositions,
                        std::vector<glm::vec3>(computedPositions.size(),
                                               overlay::estimateColor),
                        overlay::estimateMarkerSize, mvp);
}

void PoseComparisonState::renderPlayerOverlay(const Simulation &sim, Renderer &renderer,
                                              const glm::mat4 &mvp) const
{
    (void)mvp;

    if (m_log.entries.empty())
        return;

    const PoseEntry &entry = m_log.entries[m_log.current];
    if (!entry.computedPose)
        return;

    // Show the diff only while the camera actually sits on the reviewed true
    // pose (the pose was applied by copy, so Waypoint's exact equality holds).
    // Once the user moves, the ghost would compare against the wrong view.
    if (sim.playerView.camera.pose() != entry.truePose)
        return;


    Camera estimated = sim.playerView.camera;   // inherit fov/near/far/up
    estimated.applyPose(*entry.computedPose);
    renderer.drawGhost(estimated, sim.playerView.viewport, overlay::estimateColor,
                       overlay::estimateGhostAlpha, overlay::estimateGhostTint);
}
