#include "TrackersState.h"

#include <algorithm>
#include <iostream>
#include <string>

#include "../core/Simulation.h"
#include "../core/Utils.h"               // randomIndex
#include "../render/Renderer.h"
#include "../vision/Pnp.h"               // computeCameraPose
#include "../vision/TrackerDetection.h"  // findTrackerCentroids

// 20 perceptually distinct colors for tracker identification. The first 7 use
// only 0/1 channel values (all binary primaries/secondaries except black); the
// remaining 13 use 0.5 as a mid-step. Every pair differs by ≥0.5 (≈128/255)
// in at least one channel -- far above the blob detector's channelTolerance=3,
// so misclassification is impossible even with mild rendering imprecision.
static const glm::vec3 trackerPalette[] = {
    { 1.0f, 0.0f, 0.0f },  // red
    { 0.0f, 1.0f, 0.0f },  // green
    { 0.0f, 0.0f, 1.0f },  // blue
    { 1.0f, 1.0f, 0.0f },  // yellow
    { 1.0f, 0.0f, 1.0f },  // magenta
    { 0.0f, 1.0f, 1.0f },  // cyan
    { 1.0f, 1.0f, 1.0f },  // white
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

// Reads one terminal line; anything that isn't a number in range falls back
// to the default with a message. Relies on every earlier cin reader
// discarding its own trailing newline (see selectTerrain), so an empty line
// here really is the user pressing Enter for the default.
size_t TrackersState::promptCount()
{
    std::cout << "Number of trackers (1-" << kMaxCount
              << ", Enter = " << kDefaultCount << "): ";
    std::string line;
    std::getline(std::cin, line);

    if (line.empty())
        return kDefaultCount;
    try {
        const int n = std::stoi(line);
        if (n >= 1 && (size_t)n <= kMaxCount)
            return (size_t)n;
    } catch (...) {}   // stoi: not a number at all

    std::cout << "Invalid count -- using " << kDefaultCount << std::endl;
    return kDefaultCount;
}

void TrackersState::onEnter(Simulation &sim)
{
    m_trackers.clear();

    // Both knobs scale with the terrain so any DEM gets sensibly sized,
    // sensibly spread trackers.
    const float radius        = sim.terrainSize * 0.012f;
    const float minSeparation = sim.terrainSize * 0.08f;

    const Mesh &mesh = sim.mesh;

    for (size_t i = 0; i < m_count; i++) {
        // Rest each sphere on a random terrain vertex: recentered to the world
        // the cameras live in, raised by the radius so it sits ON the surface.
        // Re-roll a bounded number of times to keep trackers apart -- clumped
        // trackers hand PnP a near-degenerate point configuration. Best effort:
        // if the rolls run out, the last candidate stands rather than failing.
        glm::vec3 position(0.0f);
        for (int attempt = 0; attempt < 40; attempt++) {
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
    renderer.drawTrackers(m_trackers, mvp);
    PoseComparisonState::renderGlobalOverlay(sim, renderer, mvp);
}

void TrackersState::renderPlayerOverlay(const Simulation &sim, Renderer &renderer,
                                        const glm::mat4 &mvp) const
{
    renderer.drawTrackers(m_trackers, mvp);
    PoseComparisonState::renderPlayerOverlay(sim, renderer, mvp);   // ghost over all
}
