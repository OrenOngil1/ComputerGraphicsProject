#include "PoseComparisonState.h"

#include <iostream>
#include <vector>

#include <GLFW/glfw3.h>

#include "OverlayStyle.h"
#include "../core/Simulation.h"
#include "../input/Movement.h"
#include "../render/Renderer.h"

void PoseComparisonState::tick(Simulation &sim, GLFWwindow *window, float dt)
{
    // Free flight, same as NAVIGATION: the user positions the camera for the
    // next capture. moveCamera only changes the pose while movement keys are
    // held, so a pose snapped to by N/M stays put until the user flies off.
    moveCamera(sim.playerView.camera, sim.terrainSize, window, dt);
}

void PoseComparisonState::snapToCurrent(Simulation &sim) const
{
    if (m_log.entries.empty())
        return;

    const Waypoint &pose = m_log.entries[m_log.current].truePose;
    sim.playerView.camera.position = pose.position;
    sim.playerView.camera.target   = pose.target;
}

void PoseComparisonState::handleKey(Simulation &sim, Renderer &renderer, int key, int mods)
{
    (void)mods;

    switch (key) {
        case GLFW_KEY_B: {
            // Capture a timestep: the camera's pose right now is the ground
            // truth; computePose estimates that same pose from the rendered
            // frame alone. Both go in the log -- computed may be empty.
            Waypoint truePose{ sim.playerView.camera.position,
                               sim.playerView.camera.target };
            m_log.add({ truePose, computePose(sim, renderer) });

            const PoseEntry &entry = m_log.entries.back();
            std::cout << "Timestep " << m_log.entries.size() - 1 << ": ";
            if (entry.computedPose) {
                const glm::vec3 err = entry.computedPose->position - truePose.position;
                std::cout << "pose computed, position error " << glm::length(err) << std::endl;
            } else {
                std::cout << "pose NOT computed (see message above)" << std::endl;
            }
            break;
        }

        // N steps to the next captured timestep, M back to the previous one
        // (mnemonic: N = next, M = minus). The camera snaps to the timestep's
        // true pose, so the player view + ghost show that capture's diff.
        case GLFW_KEY_N:
        case GLFW_KEY_M: {
            if (m_log.entries.empty()) {
                std::cout << "No timesteps captured yet -- press B to capture one" << std::endl;
                return;
            }
            m_log.step(key == GLFW_KEY_N ? +1 : -1);
            snapToCurrent(sim);
            std::cout << "Reviewing timestep " << m_log.current
                      << " of " << m_log.entries.size() << std::endl;
            break;
        }
    }
}

void PoseComparisonState::renderGlobalOverlay(const Simulation &sim, Renderer &renderer,
                                              const glm::mat4 &mvp) const
{
    if (m_log.entries.empty())
        return;

    // Split the log into the two trajectories. The computed one may be shorter
    // (skipped solves leave gaps); its path simply connects the poses that exist.
    std::vector<Waypoint>  truePoses;
    std::vector<glm::vec3> truePositions, computedPositions, computedColors;
    truePoses.reserve(m_log.entries.size());
    truePositions.reserve(m_log.entries.size());
    for (const PoseEntry &entry : m_log.entries) {
        truePoses.push_back(entry.truePose);
        truePositions.push_back(entry.truePose.position);
        if (entry.computedPose) {
            computedPositions.push_back(entry.computedPose->position);
            computedColors.push_back(overlay::estimateColor);
        }
    }

    // True fly-through, in RECORD's visual language: blue path, red waypoint
    // dots, green for the timestep the camera sits on (the one under review).
    renderer.drawPath(truePositions, overlay::truePathColor, mvp);
    renderer.drawWaypoints(truePoses, sim.playerView.camera.position, mvp);

    // Computed fly-through, in the estimate's signature shade.
    renderer.drawPath(computedPositions, overlay::estimateColor, mvp);
    renderer.drawPoints(computedPositions, computedColors, 5.0f, mvp);
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
    // pose (B just captured it, or N/M snapped to it) -- the same position-match
    // trick as the waypoint highlight. Once the user flies off toward the next
    // capture, the ghost would be compared against the wrong view, so it hides.
    if (sim.playerView.camera.position != entry.truePose.position)
        return;

    Camera estimated = sim.playerView.camera;   // inherit fov/near/far/up
    estimated.position = entry.computedPose->position;
    estimated.target   = entry.computedPose->target;
    renderer.drawGhost(estimated, sim.playerView.viewport, overlay::estimateColor,
                       overlay::estimateGhostAlpha, overlay::estimateGhostTint);
}
