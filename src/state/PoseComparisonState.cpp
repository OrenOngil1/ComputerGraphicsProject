#include "PoseComparisonState.h"

#include <iostream>
#include <vector>

#include <GLFW/glfw3.h>

#include "OverlayStyle.h"
#include "../core/Simulation.h"
#include "../input/CameraControls.h"   // fly, MovementIntent
#include "../input/Callbacks.h"        // pollMovementIntent (GLFW glue)
#include "../render/Renderer.h"

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

void PoseComparisonState::handleKey(Simulation &sim, Renderer &renderer, int key, int mods)
{
    (void)mods;

    switch (key) {
        case GLFW_KEY_B: {
            // Capture a timestep: the camera's pose right now is the ground
            // truth; computePose estimates it from the rendered frame alone.
            Waypoint truePose = sim.playerView.camera.pose();
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

        // N = next timestep, M = previous (mnemonic: minus). The camera snaps
        // to the timestep's true pose, so the ghost shows that capture's diff.
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
