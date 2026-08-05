#include "FeatureMatchState.h"

#include <algorithm>
#include <cmath>
#include <iostream>
#include <random>
#include <vector>

#include <GLFW/glfw3.h>
#include <glm/gtc/matrix_transform.hpp>   // glm::ortho

#include "OverlayStyle.h"
#include "../core/Menu.h"         // promptCount
#include "../core/Simulation.h"
#include "../input/Callbacks.h"   // cursorPosPixels
#include "../render/Renderer.h"
#include "../vision/FeatureDbIo.h"
#include "../vision/FeatureMatching.h"

// Pre-phase scratch: which recorded view is being anchored, which suggestion
// is active, the top-N SIFT suggestions for the current view, and their
// descriptors (one row each, aligned with markers). In the .cpp so OpenCV
// types stay out of the state header.
//
// markers is DISPLAY ONLY -- the 2D SIFT position is just the dot the user aims
// from; it never enters the database or PnP. The build stores only
// (descriptor, user-picked 3D); run-phase PnP pairs each match with the LIVE
// frame's keypoint pixel. That is what makes Mode D manual.
struct BuildScratch {
    size_t                 waypoint = 0;
    size_t                 active   = 0;
    std::vector<glm::vec2> markers;       // [0,1] screen fractions of the suggestions
    cv::Mat                descriptors;   // one SIFT descriptor row per marker

    // Which markers have been anchored in THIS view, newest last -- the undo
    // stack for 'U'. Marker indices rather than a count because skips leave
    // gaps: stepping `active` back by one would land on a skipped suggestion,
    // not the one just placed. Cleared whenever a new view loads.
    std::vector<size_t> anchoredAt;
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
              << "for each view SIFT highlights a point (red) in the player (right) view "
              << "-- color-pick its 3D spot in the global (left) map; X skips one, "
              << "U undoes the last one.\n"
              << "               The map shows the view's cyan cone and a red line from "
              << "the camera: the point you are placing lies somewhere ALONG that line, "
              << "and your click is snapped onto it -- so only how far along it you "
              << "click matters. Orbit to judge that. V hides the aids.\n"
              << "               Global map: scroll = zoom, middle-drag = pan, "
              << "right-drag = rotate.\n"
              << "               Then " << kCaptureHelp
              << ". Ctrl+S saves the database, Ctrl+O loads it back. Ctrl+G auto-builds"
              << " and saves one (orbit path, simulated aim error) when you just need"
              << " a database to test against." << std::endl;
}

// Pose the player camera at the current build waypoint (so the right pane
// shows that recorded view live) and detect its top-N SIFT suggestions. Views
// with no features are skipped; the build finishes when the waypoints run out.
void FeatureMatchState::loadCurrentView(Simulation &sim, Renderer &renderer)
{
    while (m_build->waypoint < sim.waypoints.size()) {
        sim.playerView.camera.applyPose(sim.waypoints[m_build->waypoint]);

        const Viewport vp = captureViewport(sim);
        FramePixels frame = renderer.captureSceneFrameAt(vp.width, vp.height,
                                                         sim.playerView.camera, sim.light());
        std::vector<cv::KeyPoint> kps;
        cv::Mat desc;
        detectSpreadFeatures(frame, (int)m_featureCount, kps, desc);

        if (!kps.empty()) {
            m_build->markers.clear();
            m_build->markers.reserve(kps.size());
            for (const cv::KeyPoint &kp : kps)
                m_build->markers.push_back(glm::vec2(kp.pt.x / frame.width,
                                                     kp.pt.y / frame.height));
            m_build->descriptors = desc;
            m_build->active = 0;
            m_build->anchoredAt.clear();   // undo is scoped to the current view
            std::cout << "FEATURES: view " << (m_build->waypoint + 1) << "/"
                      << sim.waypoints.size() << " -- place " << kps.size()
                      << " features on the map. (" << m_db->anchors.size()
                      << " anchored so far)" << std::endl;
            return;
        }
        std::cout << "FEATURES: view " << (m_build->waypoint + 1)
                  << " has no features -- skipping." << std::endl;
        m_build->waypoint++;
    }
    finishBuild(sim, renderer);
}

// G: discard any previous database and start a fresh hand-build at view 0.
// Rebuilding from scratch is deliberate: a database must be anchored under one
// light, so mixing two lightings (the experiment) would muddy it.
void FeatureMatchState::startBuild(Simulation &sim, Renderer &renderer)
{
    m_db = std::make_unique<FeatureDb>();
    m_build = std::make_unique<BuildScratch>();
    // A fresh build adopts the live pane's size as the database's capture
    // resolution; every capture from here on renders at it, whatever the
    // window does later.
    m_captureWidth  = sim.playerView.viewport.width;
    m_captureHeight = sim.playerView.viewport.height;
    refreshPlaces();   // the old database's markers go with it
    loadCurrentView(sim, renderer);
}

Viewport FeatureMatchState::captureViewport(const Simulation &sim) const
{
    if (m_captureWidth > 0 && m_captureHeight > 0)
        return { 0, 0, m_captureWidth, m_captureHeight };
    return sim.playerView.viewport;
}

void FeatureMatchState::refreshPlaces()
{
    m_places = m_db ? m_db->places() : std::vector<glm::vec3>();
    m_inView.clear();
    m_outOfView.clear();
}

// Collect each anchored point's appearance in the OTHER recorded views.
//
// A human places one point in one view, so the hand-build leaves exactly one
// descriptor per anchor: one appearance, from one angle. SIFT is not
// viewpoint-invariant, and this terrain's appearance is shading rather than
// texture, so from anywhere else that single appearance often simply does not
// match. It is why capturing from a recorded waypoint works and free flight
// between them does not.
//
// Nothing here invents a correspondence. The recorded poses are known exactly
// and the anchor is the user's own 3D point, so where that point falls in every
// other recorded view is plain projection -- no mesh is read, no 3D is chosen
// by machine. The same hand-placed point simply gains the descriptors of its
// other appearances, which is what the removed automatic variant had for free
// and what the manual one has been missing.
//
// It is also self-limiting, which is what makes it safe. An appearance counts
// only if there is a keypoint within a few pixels of the projection whose
// descriptor still resembles the anchored one. A point whose depth was
// misjudged projects somewhere else in every other view, finds nothing that
// looks like it, and gains nothing -- a bad placement cannot poison the
// database, it just stays as weak as it was.
void FeatureMatchState::addOtherViewAppearances(Simulation &sim, Renderer &renderer)
{
    if (!m_db || m_db->empty())
        return;

    const std::vector<glm::vec3> places = m_db->places();
    const size_t placed = m_db->anchors.size();
    const Viewport vp = captureViewport(sim);

    // The RECORDED views only, per the course brief: the database describes
    // what was actually seen at the recorded waypoints, and nothing is
    // synthesized in between. (A variant that also collected from midpoint and
    // look-around poses along the path was tried and reverted -- it widened
    // free-flight coverage, but the spec asks for the recorded views alone.)
    for (const Waypoint &waypoint : sim.waypoints) {
        // A throwaway Camera so the player camera is left where the build put it.
        Camera camera = sim.playerView.camera;
        camera.applyPose(waypoint);

        const FramePixels frame = renderer.captureSceneFrameAt(vp.width, vp.height,
                                                               camera, sim.light());
        if (frame.rgb.empty())
            continue;
        std::vector<cv::KeyPoint> keypoints;
        cv::Mat descriptors;
        detectAllFeatures(frame, keypoints, descriptors);
        if (keypoints.empty())
            continue;

        // How far the projection may miss its keypoint. Big enough for SIFT's
        // own localisation slop and a slightly-off anchor, small enough that it
        // cannot reach a different feature.
        const float radius = std::max(6.0f, 0.01f * (float)frame.height);

        for (const glm::vec3 &place : places) {
            if (!isInFrame(camera, vp, place))
                continue;

            const glm::vec2 pixel = rasterize(camera, vp, place);

            int nearest = -1;
            float nearestDistance = radius;
            for (size_t i = 0; i < keypoints.size(); i++) {
                const float d = glm::length(glm::vec2(keypoints[i].pt.x, keypoints[i].pt.y) - pixel);
                if (d < nearestDistance) {
                    nearestDistance = d;
                    nearest = (int)i;
                }
            }
            if (nearest < 0)
                continue;

            // Must still look like the point the user anchored, or this view is
            // seeing something else there -- a ridge in front of it, most often.
            if (!resemblesAnchoredPoint(*m_db, place, descriptors.row(nearest)))
                continue;

            // Idempotence: a load re-runs this pass, and the same view of the
            // same place recomputes the same descriptor. Skip rows the place
            // already owns, so re-collection tops a database up instead of
            // duplicating it.
            bool alreadyStored = false;
            for (size_t i = 0; i < m_db->anchors.size() && !alreadyStored; i++)
                alreadyStored = m_db->anchors[i] == place &&
                                cv::norm(m_db->descriptors.row((int)i),
                                         descriptors.row(nearest), cv::NORM_L2) < 1.0;
            if (alreadyStored)
                continue;

            m_db->descriptors.push_back(descriptors.row(nearest));
            m_db->anchors.push_back(place);
        }
    }

    std::cout << "FEATURES: " << places.size() << " placed points seen "
              << m_db->anchors.size() << " times across the " << sim.waypoints.size()
              << " recorded views (" << (m_db->anchors.size() - placed)
              << " appearances added by this pass)" << std::endl;
}

void FeatureMatchState::finishBuild(Simulation &sim, Renderer &renderer)
{
    addOtherViewAppearances(sim, renderer);

    std::cout << "FEATURES: database built -- " << m_db->places().size()
              << " hand-placed points. Ctrl+S saves it so this need not be redone. "
              << kCaptureHelp << std::endl;
    m_build.reset();   // back to run-phase: tick flies again, B/N/M work
    refreshPlaces();
}

void FeatureMatchState::advance(Simulation &sim, Renderer &renderer)
{
    m_build->active++;
    if (m_build->active < m_build->markers.size())
        return;
    m_build->waypoint++;
    loadCurrentView(sim, renderer);   // next view, or finishBuild() when exhausted
}

// Take back the last anchor placed in the CURRENT view: drop its database row
// and 3D point, and make that suggestion active again.
//
// Scoped to this view on purpose. Crossing back over a view boundary would
// mean re-posing the camera and re-running SIFT to rebuild the markers, and the
// mistake this exists for -- noticing a misclick right after making it -- never
// needs it.
void FeatureMatchState::undoAnchor()
{
    if (m_build->anchoredAt.empty()) {
        std::cout << "FEATURES: nothing to undo in this view." << std::endl;
        return;
    }

    m_build->active = m_build->anchoredAt.back();
    m_build->anchoredAt.pop_back();

    // The two halves of an anchor are stored in parallel and must stay that
    // way: anchors[i] belongs to descriptor row i.
    m_db->anchors.pop_back();
    m_db->descriptors = m_db->descriptors.rowRange(0, m_db->descriptors.rows - 1).clone();

    std::cout << "FEATURES: undid the last anchor -- place feature "
              << (m_build->active + 1) << " again." << std::endl;
}

// Ctrl+S: write the placed database out. Run-phase only, which the mid-build
// return below enforces -- a half-finished build has nothing meaningful to
// save, and the count it would write disagrees with what the user is still
// placing.
void FeatureMatchState::saveDatabase(const Simulation &sim) const
{
    if (!m_db || m_db->empty() || m_db->descriptors.rows != (int)m_db->anchors.size()) {
        std::cout << "FEATURES: no database to save -- press G to build one first"
                  << std::endl;
        return;
    }
    saveFeatureDb(featureDbPath(sim.terrainFile), *m_db, sim.waypoints, sim.terrainFile,
                  m_captureWidth, m_captureHeight);
}

// Ctrl+O: replace the database with the saved one. Read into a fresh FeatureDb
// so a refused load leaves the current one untouched.
void FeatureMatchState::loadDatabase(Simulation &sim, Renderer &renderer)
{
    auto loaded = std::make_unique<FeatureDb>();
    std::vector<Waypoint> waypoints;
    int captureWidth = 0, captureHeight = 0;
    if (!loadFeatureDb(featureDbPath(sim.terrainFile), *loaded, waypoints, sim.terrainFile,
                       captureWidth, captureHeight))
        return;

    m_db = std::move(loaded);

    // The file's capture size is the resolution its descriptors were computed
    // at; every capture from here renders at it. A file from before the size
    // was recorded adopts the live pane -- the next Ctrl+S makes that
    // permanent (and its descriptors were built on some other session's pane
    // anyway, so a rebuild is the real fix for those).
    if (captureWidth > 0 && captureHeight > 0) {
        m_captureWidth  = captureWidth;
        m_captureHeight = captureHeight;
    } else {
        m_captureWidth  = sim.playerView.viewport.width;
        m_captureHeight = sim.playerView.viewport.height;
        std::cout << "FEATURES: this file predates capture-size pinning -- adopting "
                  << m_captureWidth << "x" << m_captureHeight
                  << "; Ctrl+S records it (a G rebuild gives exact matching)" << std::endl;
    }

    // The waypoints come back too: the anchors were placed from those views, so
    // restoring them is what makes Ctrl+B mean anything right after a load. The
    // flight path between them was never saved -- it is only drawn -- so it is
    // dropped rather than left describing a flight these waypoints are not on.
    if (!waypoints.empty()) {
        sim.waypoints = std::move(waypoints);
        sim.pathPoints.clear();
    }

    // Collect whatever appearances the file does not yet have -- an older file
    // may hold only the hand-placed rows, or predate the synthesized
    // collection poses. Collection skips rows a place already owns, so running
    // it on every load upgrades any vintage of file without bloating a
    // current one, and never costs the user a re-placement.
    if (!m_db->empty())
        addOtherViewAppearances(sim, renderer);

    refreshPlaces();
}

// ── The automated stand-in for the manual build (Ctrl+G) ──────
//
// A hand-built database is minutes of clicking, which taxes exactly the thing
// the mode needs most: experiments. This runs the whole G workflow without the
// human and saves the result -- and it SIMULATES the human rather than
// replacing the pipeline. The run phase still consumes nothing but
// (descriptor, 3D) pairs and cannot tell the two builds apart; the manual
// build remains the assignment's mode, this produces test databases for it.
//
// The simulation is honest because ray-snapping already reduces a real
// person's click to a single number: the depth along the suggestion's sight
// line. So the simulated "human" reads the true ray-terrain intersection and
// disturbs that depth with Gaussian aim error. Reading the terrain here plays
// the human's eyes, not the estimator's -- the estimator never sees it. A ray
// that misses the terrain is skipped, as a person would press X.

// Aim error, in units along the sight line -- the one dimension a human
// actually supplies. Sized to what measured hand-built databases achieved
// (poses 2-4 units off from carefully placed anchors).
static constexpr float kSimulatedDepthErrorUnits = 4.0f;

static constexpr size_t kDefaultAutoViews    = 8;    // ring stops
static constexpr size_t kMaxAutoViews        = 20;
static constexpr size_t kDefaultAutoFeatures = 10;   // denser than the manual default:
                                                     // clicks are free here

void FeatureMatchState::autoBuild(Simulation &sim, Renderer &renderer)
{
    const size_t views    = ::promptCount("Auto-path waypoints", kMaxAutoViews,
                                          kDefaultAutoViews);
    const size_t features = ::promptCount("Features per view", kMaxFeatures,
                                          kDefaultAutoFeatures);

    // The path: eyes on a ring around the terrain's middle, all looking at it.
    // This is the shape the successful manual sessions kept converging on --
    // every frame filled with relief at a consistent scale, and adjacent view
    // cones overlapping over the middle so appearance collection has overlap
    // to work with.
    const float tau      = 6.28318530718f;
    const float radius   = 0.30f * sim.terrainSize;
    const float altitude = 0.25f * sim.terrainSize;

    sim.waypoints.clear();
    sim.pathPoints.clear();
    for (size_t i = 0; i < views; i++) {
        const float angle = tau * (float)i / (float)views;
        const glm::vec3 eye(radius * std::cos(angle), altitude, radius * std::sin(angle));
        sim.waypoints.push_back({ eye, glm::vec3(0.0f) });
        sim.pathPoints.push_back(eye);
    }
    sim.pathPoints.push_back(sim.waypoints.front().position);   // close the ring on the map

    // Same lifecycle as startBuild: fresh database, fresh capture resolution,
    // and any manual build in progress is discarded exactly as G would.
    m_db = std::make_unique<FeatureDb>();
    m_build.reset();
    m_captureWidth  = sim.playerView.viewport.width;
    m_captureHeight = sim.playerView.viewport.height;

    // A fixed seed, so two auto-builds with the same parameters are the same
    // database -- "change one thing and measure" stays possible.
    std::mt19937 rng(20260805u);
    std::normal_distribution<float> aim(0.0f, kSimulatedDepthErrorUnits);

    const Viewport vp = captureViewport(sim);
    const float step = std::max(0.25f, sim.terrainSize / 1200.0f);
    size_t skipped = 0;

    for (const Waypoint &waypoint : sim.waypoints) {
        Camera camera = sim.playerView.camera;   // throwaway; the player stays put
        camera.applyPose(waypoint);

        FramePixels frame = renderer.captureSceneFrameAt(vp.width, vp.height,
                                                         camera, sim.light());
        if (frame.rgb.empty())
            continue;
        std::vector<cv::KeyPoint> kps;
        cv::Mat desc;
        detectSpreadFeatures(frame, (int)features, kps, desc);

        for (size_t i = 0; i < kps.size(); i++) {
            const glm::vec2 fraction(kps[i].pt.x / frame.width,
                                     kps[i].pt.y / frame.height);
            const glm::vec3 direction =
                rayDirection(camera, fractionToRay(fraction, camera.fov, vp.aspect()));

            const std::optional<float> depth =
                raycastTerrain(sim.mesh, camera.position, direction,
                               3.0f * sim.terrainSize, step);
            if (!depth) {
                skipped++;
                continue;
            }
            const float judged = *depth + aim(rng);
            if (judged <= 0.0f) {
                skipped++;
                continue;
            }
            m_db->descriptors.push_back(desc.row((int)i));
            m_db->anchors.push_back(camera.position + direction * judged);
        }
    }

    if (m_db->empty()) {
        std::cout << "FEATURES: auto-build anchored nothing -- every suggestion missed"
                     " the terrain" << std::endl;
        refreshPlaces();
        return;
    }

    const size_t placed = m_db->anchors.size();
    addOtherViewAppearances(sim, renderer);
    refreshPlaces();
    std::cout << "FEATURES: auto-built " << placed << " points from "
              << sim.waypoints.size() << " views on an orbit (" << skipped
              << " suggestions skipped; simulated aim error sigma "
              << kSimulatedDepthErrorUnits << " units along the sight line). "
              << kCaptureHelp << std::endl;
    saveDatabase(sim);
}

void FeatureMatchState::handleKey(Simulation &sim, Renderer &renderer, int key, int mods)
{
    if (key == GLFW_KEY_G) {
        // Plain G (re)starts the manual build; Ctrl+G runs the automated
        // stand-in. Either discards whatever build was in progress.
        if (mods & GLFW_MOD_CONTROL)
            autoBuild(sim, renderer);
        else
            startBuild(sim, renderer);
        return;
    }
    if (building()) {
        if (key == GLFW_KEY_X) {   // skip an unplaceable suggestion
            std::cout << "FEATURES: skipped a suggestion." << std::endl;
            advance(sim, renderer);
        } else if (key == GLFW_KEY_U) {
            undoAnchor();
        } else if ((mods & GLFW_MOD_CONTROL) &&
                   (key == GLFW_KEY_S || key == GLFW_KEY_O)) {
            // Answered rather than ignored: silence here reads as "saving is
            // broken". A half-built database is also genuinely unsaveable --
            // its count disagrees with what is still being placed.
            std::cout << "FEATURES: mid-build -- finish placing the views first;"
                         " Ctrl+S / Ctrl+O work once the build is done" << std::endl;
        }
        return;                    // B/N/M and the rest are inert mid-build
    }

    // The base class ignores modifiers on its own keys, so the database's
    // Ctrl combinations are claimed before delegating.
    if (mods & GLFW_MOD_CONTROL) {
        if (key == GLFW_KEY_S) {
            saveDatabase(sim);
            return;
        }
        if (key == GLFW_KEY_O) {
            loadDatabase(sim, renderer);
            return;
        }
    }

    PoseComparisonState::handleKey(sim, renderer, key, mods);
}

void FeatureMatchState::tick(Simulation &sim, GLFWwindow *window, float dt)
{
    if (building())                // camera is pinned to the view being anchored
        return;
    PoseComparisonState::tick(sim, window, dt);   // free flight in the run-phase

    // The map's answer to "is this a good place to capture from" has to follow
    // the camera, so it is recomputed as the camera moves rather than on demand.
    refreshAnchorVisibility(sim);
    reportAnchorCount();
}

// Can the player's current view actually use this anchor?
//
// Being inside the frustum is not enough. SIFT matches what was drawn, and a
// feature behind a ridge was not drawn -- counting it would promise matches the
// capture cannot deliver, which is exactly the wrong answer for a display whose
// job is to say whether this position is worth capturing from.
static bool anchorVisible(const Simulation &sim, const glm::vec3 &anchor)
{
    const View &view = sim.playerView;
    if (!isInFrame(view.camera, view.viewport, anchor))
        return false;

    const glm::vec3 toAnchor = anchor - view.camera.position;
    const float distance = glm::length(toAnchor);

    // Stop the march short of the anchor itself: an anchor sits on or near the
    // surface, so a hit at its own distance is the ground it belongs to rather
    // than something standing in front of it. The margin is generous because
    // the snap can leave an anchor slightly under the surface (a click past
    // where its ray meets the ground), and that should still read as usable.
    // An anchor buried far deeper does read as hidden -- which is honest: it
    // was placed badly, and the run phase will not match it well either.
    const float margin = std::max(1.0f, sim.terrainSize / 100.0f);
    const float step   = std::max(0.5f, sim.terrainSize / 300.0f);
    if (distance <= margin)
        return true;

    return !raycastTerrain(sim.mesh, view.camera.position, toAnchor / distance,
                           distance - margin, step);
}

void FeatureMatchState::refreshAnchorVisibility(const Simulation &sim)
{
    m_inView.clear();
    m_outOfView.clear();

    for (const glm::vec3 &place : m_places)
        (anchorVisible(sim, place) ? m_inView : m_outOfView).push_back(place);
}

void FeatureMatchState::reportAnchorCount()
{
    if (m_places.empty())
        return;

    // Only on a change, and no more than twice a second. The count moves
    // continuously while flying, and a line per frame would bury every other
    // message in the console -- which is the opposite of the point.
    const double now = glfwGetTime();
    if (m_inView.size() == m_reportedInView || now - m_lastCountReport < 0.5)
        return;

    m_reportedInView  = m_inView.size();
    m_lastCountReport = now;
    std::cout << "FEATURES: " << m_inView.size() << " of " << m_places.size()
              << " anchors in view" << std::endl;
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
    //
    // The click is snapped onto the suggestion's viewing ray first. That ray is
    // known exactly -- this view's pose is a recorded waypoint and the keypoint's
    // pixel is exact -- so the true point lies somewhere along it, and whatever
    // sideways offset the click has is pure aim error. What the user is actually
    // being asked for is the DEPTH, the one thing a single image cannot supply.
    //
    // Dropping the sideways part is worth more than it looks: an error along the
    // ray reprojects onto the same pixel and costs the solve almost nothing,
    // while a sideways slip of the same size reprojects tens of pixels away and
    // gets a perfectly good correspondence voted out as an outlier.
    const Camera &camera = sim.playerView.camera;
    const glm::vec2 ray = fractionToRay(m_build->markers[m_build->active], camera.fov,
                                        captureViewport(sim).aspect());
    const std::optional<glm::vec3> anchor =
        snapToViewRay(camera, ray, sim.mesh.worldPos(id));
    if (!anchor) {
        std::cout << "FEATURES: that spot is behind the view -- pick along the red line."
                  << std::endl;
        return;
    }

    m_db->descriptors.push_back(m_build->descriptors.row((int)m_build->active));
    m_db->anchors.push_back(*anchor);
    m_build->anchoredAt.push_back(m_build->active);   // so U can take it back
    advance(sim, renderer);
}

std::optional<Waypoint> FeatureMatchState::computePose(Simulation &sim, Renderer &renderer)
{
    if (!m_db || m_db->empty()) {
        std::cout << "FEATURES: no database yet -- press G to build it first"
                  << std::endl;
        return std::nullopt;
    }

    // What was AVAILABLE, before what was found. Read together, the two lines
    // separate a matching problem from a positioning one: 2 of 7 matched is a
    // matcher story, 7 of 7 anchors in frame and 2 matched is a different one
    // from 2 in frame and 2 matched. Recomputed here rather than trusted from
    // the last tick, because Ctrl+B poses the camera itself between captures.
    refreshAnchorVisibility(sim);
    std::cout << "FEATURES: " << m_inView.size() << " of " << m_places.size()
              << " anchors in frame" << std::endl;

    if (!m_captureHintShown) {
        m_captureHintShown = true;
        std::cout << "        (a capture never adds to the database -- only G does;"
                     " the red dot it leaves on the map just marks where you tested"
                     " from)" << std::endl;
    }

    const Viewport vp = captureViewport(sim);
    FramePixels frame = renderer.captureSceneFrameAt(vp.width, vp.height,
                                                     sim.playerView.camera, sim.light());
    if (frame.rgb.empty())
        return std::nullopt;
    std::optional<Waypoint> estimate =
        estimatePoseFromFeatures(*m_db, frame, sim.playerView.camera.fov,
                                 vp.width, vp.height);

    // Plausibility, judged on the estimate alone -- the true pose is never
    // consulted. A camera whose frame this terrain fills cannot be many
    // terrain-widths away from it, yet a coalition of lookalike matches that
    // survives every gate tends to produce exactly that: not a slightly-wrong
    // pose but an impossible one (measured once at 24 terrain-widths out).
    // The terrain is centered on the origin, so distance from it is the test.
    if (estimate) {
        const float distance = glm::length(estimate->position);
        const float limit    = 2.0f * sim.terrainSize;
        if (distance > limit) {
            std::cout << "FEATURES: pose rejected as implausible -- it puts the"
                         " camera " << (int)distance << " units from the terrain"
                         " (nothing seeing this terrain can be past "
                      << (int)limit << ")" << std::endl;
            return std::nullopt;
        }
    }
    return estimate;
}

// The build phase's map aids. The active suggestion's sight line is drawn in
// the same red as its marker in the player view, so the dot to place and the
// line to place it on are visibly one object; this view's earlier anchors get
// a dim line each, which doubles as a check -- a green anchor sitting off its
// own line was misplaced, and U takes it back.
//
// The lines stop at a fixed reach rather than at the terrain. Intersecting one
// with the surface would BE the correspondence the user is here to supply: a
// pixel fixes the direction, and choosing the depth along it by eye is the
// manual anchoring the mode is built around (and what the brief means by
// "map features to 3D using picking").
void FeatureMatchState::drawSightAids(const Simulation &sim, Renderer &renderer,
                                      const glm::mat4 &mvp) const
{
    if (!sim.showViewAids || m_build->markers.empty())
        return;

    const Camera &camera = sim.playerView.camera;
    const float fov    = camera.fov;
    // The markers are fractions of the CAPTURE frame, so their rays are built
    // with its aspect -- the live pane's only coincides until a mid-build
    // window resize.
    const float aspect = captureViewport(sim).aspect();
    const float reach  = sim.terrainSize * 1.5f;   // clears the map from anywhere on it

    if (!m_build->anchoredAt.empty()) {
        std::vector<glm::vec2> rays;
        rays.reserve(m_build->anchoredAt.size());
        for (size_t marker : m_build->anchoredAt)
            rays.push_back(fractionToRay(m_build->markers[marker], fov, aspect));

        renderer.drawSightLines(camera, rays, reach, overlay::sightLineDimColor,
                                overlay::sightLineDimWidth, mvp);
    }

    if (m_build->active < m_build->markers.size())
        renderer.drawSightLines(camera,
                                { fractionToRay(m_build->markers[m_build->active], fov, aspect) },
                                reach, overlay::suggestionColor, overlay::sightLineWidth, mvp);
}

// One color for a whole set of points -- the map's markers are uniform batches,
// so the per-point color vector drawPoints takes is always a fill.
static void drawDots(Renderer &renderer, const std::vector<glm::vec3> &points,
                     const glm::vec3 &color, float size, const glm::mat4 &mvp)
{
    if (points.empty())
        return;
    renderer.drawPoints(points, std::vector<glm::vec3>(points.size(), color), size, mvp);
}

// Where the database came from: the recorded flight and the views the anchors
// were placed from. Grey and dim on purpose -- it is provenance, not a
// measurement, and the run phase's own captures are already drawn in blue and
// red by the base class. Without the split the same dots would mean two
// different things on one map.
static void drawProvenance(const Simulation &sim, Renderer &renderer, const glm::mat4 &mvp)
{
    renderer.drawPath(sim.pathPoints, overlay::provenanceColor, mvp);

    std::vector<glm::vec3> views;
    views.reserve(sim.waypoints.size());
    for (const Waypoint &waypoint : sim.waypoints)
        views.push_back(waypoint.position);
    drawDots(renderer, views, overlay::provenanceColor, overlay::provenanceMarkerSize, mvp);
}

void FeatureMatchState::renderGlobalOverlay(const Simulation &sim, Renderer &renderer,
                                            const glm::mat4 &mvp) const
{
    // Everything here draws with depth off, so paint order is the layering.
    if (!building()) {
        // Run phase, back to front: where the database came from, the anchors
        // this view cannot use, what B has measured, and on top the anchors it
        // CAN use -- the last being the thing the user is reading the map for.
        drawProvenance(sim, renderer, mvp);
        drawDots(renderer, m_outOfView, overlay::anchorDimColor,
                 overlay::anchorDimMarkerSize, mvp);
        PoseComparisonState::renderGlobalOverlay(sim, renderer, mvp);
        drawDots(renderer, m_inView, overlay::anchorColor, overlay::anchorMarkerSize, mvp);
        return;
    }

    // Build phase: flight context (the current build view marked green by
    // drawWaypoints), the sight lines, then the anchors placed so far on top.
    // All bright -- the camera is pinned to the view being anchored, so the
    // in-view split the run phase draws has nothing to say here.
    renderer.drawPath(sim.pathPoints, overlay::truePathColor, mvp);
    renderer.drawWaypoints(sim.waypoints, sim.playerView.camera.position, mvp);
    drawSightAids(sim, renderer, mvp);
    if (m_db)
        drawDots(renderer, m_db->anchors, overlay::anchorColor, overlay::anchorMarkerSize, mvp);
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
