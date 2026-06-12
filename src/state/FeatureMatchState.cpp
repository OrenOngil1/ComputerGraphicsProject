#include "FeatureMatchState.h"

#include <iostream>
#include <vector>

#include <GLFW/glfw3.h>

#include "../core/Simulation.h"
#include "../render/Renderer.h"
#include "../vision/FeatureMatching.h"

// Where FeatureDb is complete; the defaulted special members must live here
// (see the header).
FeatureMatchState::FeatureMatchState() = default;
FeatureMatchState::~FeatureMatchState() = default;

void FeatureMatchState::onEnter(Simulation &sim)
{
    std::cout << "FEATURE MATCH: " << sim.waypoints.size()
              << " recorded views available. G = build the feature database "
              << "(under the current light -- L to change it), B = capture "
              << "timestep, N/M = step through timesteps" << std::endl;
}

// Pre-phase: one capture per recorded waypoint, rendered with the player
// view's lens (fov/near/far) at the waypoint's pose under the current light.
// The scene frame provides the descriptors, the id frame anchors them to 3D.
void FeatureMatchState::buildDatabase(Simulation &sim, Renderer &renderer)
{
    m_db = std::make_unique<FeatureDb>();

    for (const Waypoint &waypoint : sim.waypoints) {
        View view = sim.playerView;   // lens + viewport; pose swapped in below
        view.camera.position = waypoint.position;
        view.camera.target   = waypoint.target;

        FramePixels frame = renderer.captureSceneFrame(view, sim.light);
        std::vector<int> vertexIds = renderer.captureVertexIdFrame(view);
        harvestViewFeatures(*m_db, frame, vertexIds, sim.mesh);
    }

    std::cout << "FEATURES: database built from " << m_db->views << " views, "
              << m_db->anchors.size() << " anchored descriptors" << std::endl;
}

void FeatureMatchState::handleKey(Simulation &sim, Renderer &renderer, int key, int mods)
{
    // G rebuilds the database from scratch under the current light -- "from
    // scratch" is the point: mixing two lightings in one database would
    // muddy the pre/run experiment.
    if (key == GLFW_KEY_G) {
        buildDatabase(sim, renderer);
        return;
    }
    PoseComparisonState::handleKey(sim, renderer, key, mods);   // B / N / M
}

std::optional<Waypoint> FeatureMatchState::computePose(Simulation &sim, Renderer &renderer)
{
    if (!m_db || m_db->empty()) {
        std::cout << "FEATURES: no database yet -- press G to build it first"
                  << std::endl;
        return std::nullopt;
    }

    FramePixels frame = renderer.captureSceneFrame(sim.playerView, sim.light);
    const Viewport &viewport = sim.playerView.viewport;
    return estimatePoseFromFeatures(*m_db, frame, sim.playerView.camera.fov,
                                    viewport.width, viewport.height);
}
