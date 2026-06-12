#include "TrackersState.h"

#include <iostream>

#include "../core/Simulation.h"
#include "../core/Utils.h"               // randomIndex
#include "../render/Renderer.h"
#include "../vision/Pnp.h"               // computeCameraPose
#include "../vision/TrackerDetection.h"  // findTrackerCentroids

// The tracker palette doubles as the tracker count. Every color differs from
// every other in at least one full-intensity channel -- and from the capture
// background (black) likewise -- so blob classification is unambiguous even
// with the detector's tolerance. 7 trackers sits comfortably above the
// 4-correspondence PnP minimum, leaving headroom for occlusion.
static const glm::vec3 trackerPalette[] = {
    { 1.0f, 0.0f, 0.0f },  // red
    { 0.0f, 1.0f, 0.0f },  // green
    { 0.0f, 0.0f, 1.0f },  // blue
    { 1.0f, 1.0f, 0.0f },  // yellow
    { 1.0f, 0.0f, 1.0f },  // magenta
    { 0.0f, 1.0f, 1.0f },  // cyan
    { 1.0f, 1.0f, 1.0f },  // white
};
static const size_t trackerCount = sizeof(trackerPalette) / sizeof(trackerPalette[0]);

void TrackersState::onEnter(Simulation &sim)
{
    m_trackers.clear();

    // Both knobs scale with the terrain so any DEM gets sensibly sized,
    // sensibly spread trackers.
    const float radius        = sim.terrainSize * 0.012f;
    const float minSeparation = sim.terrainSize * 0.08f;

    const Mesh &mesh = sim.mesh;
    const glm::vec3 center(mesh.cols / 2.0f, 0.0f, mesh.rows / 2.0f);

    for (size_t i = 0; i < trackerCount; i++) {
        // Rest each sphere on a random terrain vertex: recentered to the world
        // the cameras live in, raised by the radius so it sits ON the surface.
        // Re-roll a bounded number of times to keep trackers apart -- clumped
        // trackers hand PnP a near-degenerate point configuration. Best effort:
        // if the rolls run out, the last candidate stands rather than failing.
        glm::vec3 position(0.0f);
        for (int attempt = 0; attempt < 40; attempt++) {
            const Vertex &v = mesh.vertices[randomIndex(mesh.vertices.size())];
            position = v.position - center + glm::vec3(0.0f, radius, 0.0f);

            bool farEnough = true;
            for (const Tracker &placed : m_trackers) {
                if (glm::distance(placed.center, position) < minSeparation) {
                    farEnough = false;
                    break;
                }
            }
            if (farEnough)
                break;
        }
        m_trackers.push_back({ position, radius, trackerPalette[i] });
    }

    std::cout << "TRACKERS: placed " << m_trackers.size() << " trackers. "
              << "Fly freely; B = capture timestep, N/M = step through timesteps"
              << std::endl;
}

std::optional<Waypoint> TrackersState::computePose(Simulation &sim, Renderer &renderer)
{
    // 2D side: render the detection frame and locate each tracker's blob.
    FramePixels frame = renderer.captureTrackersFrame(sim.playerView, m_trackers);
    std::vector<std::optional<glm::vec2>> centroids =
        findTrackerCentroids(frame, m_trackers);

    // The automatic correspondences: each visible centroid pairs with its
    // sphere's known 3D center. Occluded/off-screen trackers simply drop out.
    std::vector<PickedPoint> correspondences;
    for (size_t i = 0; i < m_trackers.size(); i++) {
        if (centroids[i])
            correspondences.push_back({ m_trackers[i].center, *centroids[i] });
    }

    std::cout << "TRACKERS: " << correspondences.size() << " of "
              << m_trackers.size() << " trackers visible" << std::endl;
    if (correspondences.size() < 4) {
        std::cout << "TRACKERS: PnP needs at least 4 -- aim at more trackers"
                  << std::endl;
        return std::nullopt;
    }

    const Viewport &viewport = sim.playerView.viewport;
    return computeCameraPose(correspondences, sim.playerView.camera.fov,
                             viewport.width, viewport.height);
}

void TrackersState::renderGlobalOverlay(const Simulation &sim, Renderer &renderer,
                                        const glm::mat4 &mvp) const
{
    // Spheres first (depth-tested scene objects), comparison overlay on top.
    for (const Tracker &tracker : m_trackers)
        renderer.drawTracker(tracker, mvp);
    PoseComparisonState::renderGlobalOverlay(sim, renderer, mvp);
}

void TrackersState::renderPlayerOverlay(const Simulation &sim, Renderer &renderer,
                                        const glm::mat4 &mvp) const
{
    for (const Tracker &tracker : m_trackers)
        renderer.drawTracker(tracker, mvp);
    PoseComparisonState::renderPlayerOverlay(sim, renderer, mvp);   // ghost over all
}
