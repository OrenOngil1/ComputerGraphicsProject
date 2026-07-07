#include "FeatureMatchState.h"

#include <algorithm>
#include <iostream>
#include <vector>

#include <GLFW/glfw3.h>
#include <glm/gtc/matrix_transform.hpp>   // glm::ortho

#include "OverlayStyle.h"
#include "../core/Menu.h"         // promptCount
#include "../core/Simulation.h"
#include "../input/Callbacks.h"   // cursorPosPixels
#include "../render/Renderer.h"
#include "../vision/FeatureMatching.h"

// Pre-phase scratch: which recorded view is being anchored, which suggestion
// is active, the top-N ORB suggestions for the current view, and their
// descriptors (one row each, aligned with markers). In the .cpp so OpenCV
// types stay out of the state header.
//
// markers is DISPLAY ONLY -- the 2D ORB position is just the dot the user aims
// from; it never enters the database or PnP. The build stores only
// (descriptor, user-picked 3D); run-phase PnP pairs each match with the LIVE
// frame's keypoint pixel. That is what makes Mode D manual.
struct BuildScratch {
    size_t                 waypoint = 0;
    size_t                 active   = 0;
    std::vector<glm::vec2> markers;       // [0,1] screen fractions of the suggestions
    cv::Mat                descriptors;   // one ORB descriptor row per marker
};

// FeatureDb + BuildScratch are complete in this TU, so the unique_ptr members'
// special members are defined here.
FeatureMatchState::FeatureMatchState(size_t featureCount)
    : m_featureCount(std::clamp(featureCount, size_t(1), kMaxFeatures))
{}
FeatureMatchState::~FeatureMatchState() = default;

size_t FeatureMatchState::promptCount()
{
    return ::promptCount("Features per view", kMaxFeatures, kDefaultFeatures);
}

void FeatureMatchState::onEnter(Simulation &sim)
{
    std::cout << "FEATURE MATCH: " << sim.waypoints.size() << " recorded views, "
              << m_featureCount << " features each. G = build the database by hand: "
              << "for each view ORB highlights a point (red) in the player (right) view "
              << "-- color-pick its 3D spot in the global (left) map; X skips one. "
              << "Global map: scroll = zoom, middle-drag = pan, right-drag = rotate. "
              << "Then " << kCaptureHelp << std::endl;
}

// Pose the player camera at the current build waypoint (so the right pane
// shows that recorded view live) and detect its top-N ORB suggestions. Views
// with no features are skipped; the build finishes when the waypoints run out.
void FeatureMatchState::loadCurrentView(Simulation &sim, Renderer &renderer)
{
    while (m_build->waypoint < sim.waypoints.size()) {
        sim.playerView.camera.applyPose(sim.waypoints[m_build->waypoint]);

        FramePixels frame = renderer.captureSceneFrame(sim.playerView, sim.light());
        std::vector<cv::KeyPoint> kps;
        cv::Mat desc;
        detectTopFeatures(frame, (int)m_featureCount, kps, desc);

        if (!kps.empty()) {
            m_build->markers.clear();
            m_build->markers.reserve(kps.size());
            for (const cv::KeyPoint &kp : kps)
                m_build->markers.push_back(glm::vec2(kp.pt.x / frame.width,
                                                     kp.pt.y / frame.height));
            m_build->descriptors = desc;
            m_build->active = 0;
            std::cout << "FEATURES: view " << (m_build->waypoint + 1) << "/"
                      << sim.waypoints.size() << " -- place " << kps.size()
                      << " features on the map." << std::endl;
            return;
        }
        std::cout << "FEATURES: view " << (m_build->waypoint + 1)
                  << " has no features -- skipping." << std::endl;
        m_build->waypoint++;
    }
    finishBuild();
}

// G: discard any previous database and start a fresh hand-build at view 0.
// Rebuilding from scratch is deliberate: a database must be anchored under one
// light, so mixing two lightings (the experiment) would muddy it.
void FeatureMatchState::startBuild(Simulation &sim, Renderer &renderer)
{
    m_db = std::make_unique<FeatureDb>();
    m_build = std::make_unique<BuildScratch>();
    loadCurrentView(sim, renderer);
}

void FeatureMatchState::finishBuild()
{
    const size_t rows = m_db ? m_db->anchors.size() : 0;
    std::cout << "FEATURES: database built -- " << rows
              << " hand-anchored descriptors. " << kCaptureHelp << std::endl;
    m_build.reset();   // back to run-phase: tick flies again, B/N/M work
}

void FeatureMatchState::advance(Simulation &sim, Renderer &renderer)
{
    m_build->active++;
    if (m_build->active < m_build->markers.size())
        return;
    m_build->waypoint++;
    loadCurrentView(sim, renderer);   // next view, or finishBuild() when exhausted
}

void FeatureMatchState::handleKey(Simulation &sim, Renderer &renderer, int key, int mods)
{
    if (key == GLFW_KEY_G) {       // (re)start the manual build from scratch
        startBuild(sim, renderer);
        return;
    }
    if (building()) {
        if (key == GLFW_KEY_X) {   // skip an unplaceable suggestion
            std::cout << "FEATURES: skipped a suggestion." << std::endl;
            advance(sim, renderer);
        }
        return;                    // B/N/M and the rest are inert mid-build
    }
    PoseComparisonState::handleKey(sim, renderer, key, mods);
}

void FeatureMatchState::tick(Simulation &sim, GLFWwindow *window, float dt)
{
    if (building())                // camera is pinned to the view being anchored
        return;
    PoseComparisonState::tick(sim, window, dt);   // free flight in the run-phase
}

void FeatureMatchState::handleMouseButton(Simulation &sim, Renderer &renderer,
                                          GLFWwindow *window, int button, int action)
{
    // Left-click only: right/middle drive the global-map controls (intercepted
    // in Callbacks), and skipping a suggestion is the X key.
    if (!building() || action != GLFW_PRESS || button != GLFW_MOUSE_BUTTON_LEFT)
        return;

    const glm::dvec2 cursor = cursorPosPixels(window);

    // The 3D half is color-picked in the global (left) map, exactly like PICK.
    const Viewport &map = sim.globalView.viewport;
    if (!map.contains(cursor.x, cursor.y)) {
        std::cout << "FEATURES: color-pick the 3D point in the global (left) map."
                  << std::endl;
        return;
    }
    int id = renderer.pickVertex((int)cursor.x, (int)cursor.y, sim.globalView);
    if (id < 0) {
        std::cout << "FEATURES: no terrain under the cursor -- try again." << std::endl;
        return;
    }

    // The manual anchor: the active suggestion's descriptor paired with the
    // user's 3D pick -- the single step that would otherwise be automatic.
    m_db->descriptors.push_back(m_build->descriptors.row((int)m_build->active));
    m_db->anchors.push_back(sim.mesh.worldPos(id));
    advance(sim, renderer);
}

std::optional<Waypoint> FeatureMatchState::computePose(Simulation &sim, Renderer &renderer)
{
    if (!m_db || m_db->empty()) {
        std::cout << "FEATURES: no database yet -- press G to build it first"
                  << std::endl;
        return std::nullopt;
    }

    FramePixels frame = renderer.captureSceneFrame(sim.playerView, sim.light());
    const Viewport &viewport = sim.playerView.viewport;
    return estimatePoseFromFeatures(*m_db, frame, sim.playerView.camera.fov,
                                    viewport.width, viewport.height);
}

void FeatureMatchState::renderGlobalOverlay(const Simulation &sim, Renderer &renderer,
                                            const glm::mat4 &mvp) const
{
    if (!building()) {
        PoseComparisonState::renderGlobalOverlay(sim, renderer, mvp);
        return;
    }

    // Build phase: flight context plus the 3D anchors placed so far (green).
    renderer.drawPath(sim.pathPoints, overlay::truePathColor, mvp);
    renderer.drawWaypoints(sim.waypoints, sim.playerView.camera.position, mvp);
    if (m_db && !m_db->anchors.empty()) {
        const std::vector<glm::vec3> colors(m_db->anchors.size(), overlay::anchoredColor);
        renderer.drawPoints(m_db->anchors, colors, overlay::pickMarkerSize, mvp);
    }
}

void FeatureMatchState::renderPlayerOverlay(const Simulation &sim, Renderer &renderer,
                                            const glm::mat4 &mvp) const
{
    if (!building()) {
        PoseComparisonState::renderPlayerOverlay(sim, renderer, mvp);
        return;
    }
    (void)mvp;

    // Only the active suggestion is shown -- one at a time keeps the anchoring
    // unambiguous. A screen-space marker over the live waypoint view: an ortho
    // over the unit square maps the stored [0,1] fraction straight to the
    // viewport (the same screen matrix PICK uses).
    if (m_build->active < m_build->markers.size()) {
        const glm::mat4 screen = glm::ortho(0.0f, 1.0f, 1.0f, 0.0f);
        const glm::vec3 pos(m_build->markers[m_build->active], 0.0f);
        renderer.drawPoints({ pos }, { overlay::suggestionColor },
                            overlay::suggestionMarkerSize, screen);
    }
}
