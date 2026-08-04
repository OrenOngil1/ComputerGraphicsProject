#include "TrackersState.h"

#include <algorithm>
#include <iostream>

#include "../core/Menu.h"                // promptCount
#include "../core/Simulation.h"
#include "../core/Random.h"              // randomIndex
#include "../render/Renderer.h"
#include "../vision/Pnp.h"               // computeCameraPose
#include "../vision/TrackerDetection.h"  // findTrackerCentroids

namespace {
// Placement tuning, as fractions of terrain size so any DEM gets sensibly
// sized, sensibly spread trackers.
constexpr float kRadiusFraction        = 0.012f;   // sphere radius
constexpr float kMinSeparationFraction = 0.08f;    // pairwise spacing floor
constexpr int   kPlacementAttempts     = 40;       // re-rolls before a clumped spot stands
}

// 20 distinct tracker colors on the {0, 0.5, 1} lattice, under two constraints
// the blob detector depends on: every pair differs by >= 0.5 (~128/255) in at
// least one channel, and none lies within channelTolerance of any color the lit
// terrain can render. The second bars the achromatic lattice points -- sunlit
// snow saturates to white, half-lit snow passes through grey.
static const glm::vec3 trackerPalette[] = {
    { 1.0f, 0.0f, 0.0f },  // red
    { 0.0f, 1.0f, 0.0f },  // green
    { 0.0f, 0.0f, 1.0f },  // blue
    { 1.0f, 1.0f, 0.0f },  // yellow
    { 1.0f, 0.0f, 1.0f },  // magenta
    { 0.0f, 1.0f, 1.0f },  // cyan
    { 1.0f, 0.5f, 1.0f },  // orchid
    { 1.0f, 0.5f, 0.0f },  // orange
    { 0.5f, 0.0f, 1.0f },  // violet
    { 0.0f, 0.5f, 0.0f },  // dark green
    { 1.0f, 0.0f, 0.5f },  // rose
    { 0.0f, 1.0f, 0.5f },  // spring green
    { 0.5f, 1.0f, 0.0f },  // lime
    { 0.0f, 0.5f, 1.0f },  // sky blue
    { 1.0f, 0.5f, 0.5f },  // salmon
    { 0.5f, 1.0f, 0.5f },  // mint
    { 0.5f, 0.5f, 1.0f },  // periwinkle
    { 0.5f, 0.5f, 0.0f },  // olive
    { 0.5f, 0.0f, 0.5f },  // plum
    { 0.0f, 0.5f, 0.5f },  // teal
};
static const size_t paletteSize = sizeof(trackerPalette) / sizeof(trackerPalette[0]);
static_assert(paletteSize == TrackersState::kMaxCount,
              "kMaxCount (TrackersState.h) must track the palette size");

// Clamp rather than trust: the T-key prompt validates its input, but the
// constructor is the last line of defense for any other caller.
TrackersState::TrackersState(size_t count)
    : m_count(std::clamp(count, size_t(1), TrackersState::kMaxCount))
{}

size_t TrackersState::promptCount()
{
    return ::promptCount("Number of trackers", kMaxCount, kDefaultCount);
}

void TrackersState::onEnter(Simulation &sim)
{
    m_trackers.clear();

    const float radius        = sim.terrainSize * kRadiusFraction;
    const float minSeparation = sim.terrainSize * kMinSeparationFraction;

    const Mesh &mesh = sim.mesh;

    for (size_t i = 0; i < m_count; i++) {
        // Rest each sphere on a random terrain vertex, raised by its radius.
        // Re-roll a bounded number of times to keep trackers apart (clumped
        // trackers hand PnP a near-degenerate configuration); if the rolls run
        // out, the last candidate stands rather than failing.
        glm::vec3 position(0.0f);
        for (int attempt = 0; attempt < kPlacementAttempts; attempt++) {
            position = mesh.worldPos(randomIndex(mesh.vertices.size()))
                     + glm::vec3(0.0f, radius, 0.0f);

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
              << "Fly freely; " << kCaptureHelp << std::endl;
}

std::optional<Waypoint> TrackersState::computePose(Simulation &sim, Renderer &renderer)
{
    // 2D side: render the detection frame and locate each tracker's blob.
    FramePixels frame = renderer.captureTrackersFrame(sim.playerView, m_trackers,
                                                      sim.light());
    std::vector<std::optional<glm::vec2>> centroids =
        findTrackerCentroids(frame, m_trackers);

    // Each visible centroid pairs with its sphere's known 3D center;
    // occluded/off-screen trackers simply drop out.
    std::vector<Correspondence> correspondences;
    const glm::vec2 frameSize((float)frame.width, (float)frame.height);
    for (size_t i = 0; i < m_trackers.size(); i++) {
        if (centroids[i])
            // Centroid pixel -> the [0,1] fraction Correspondence stores.
            correspondences.push_back({ m_trackers[i].center, *centroids[i] / frameSize });
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
    renderer.drawTrackers(m_trackers, mvp);
    PoseComparisonState::renderGlobalOverlay(sim, renderer, mvp);
}

void TrackersState::renderPlayerOverlay(const Simulation &sim, Renderer &renderer,
                                        const glm::mat4 &mvp) const
{
    renderer.drawTrackers(m_trackers, mvp);
    PoseComparisonState::renderPlayerOverlay(sim, renderer, mvp);   // ghost over all
}
