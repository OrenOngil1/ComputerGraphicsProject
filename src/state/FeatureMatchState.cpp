#include "FeatureMatchState.h"

#include <algorithm>
#include <cmath>
#include <iostream>
#include <vector>

#include <GLFW/glfw3.h>
#include <glm/gtc/matrix_transform.hpp>   // glm::ortho

#include "ManualBuild.h"
#include "OverlayStyle.h"
#include "../core/FlightPaths.h"  // AutoPathMode, kAutoPathSpecs, layOutAutoPath
#include "../core/Menu.h"         // promptCount, promptOptionalCount
#include "../core/Simulation.h"
#include "../input/Callbacks.h"   // cursorPosPixels
#include "../render/Renderer.h"
#include "../vision/AnchorVisibility.h"
#include "../vision/AppearanceCollection.h"
#include "../vision/FeatureAutoBuild.h"
#include "../vision/FeatureDbIo.h"
#include "../vision/FeatureMatching.h"
#include "../vision/PosePlausibility.h"

// FeatureDb + ManualBuild are complete in this TU, so the unique_ptr members'
// special members are defined here.
FeatureMatchState::FeatureMatchState(size_t featureCount, std::optional<size_t> minInliers)
    : m_featureCount(std::clamp(featureCount, size_t(1), kMaxFeatures)),
      m_minInliers(minInliers)
{}
FeatureMatchState::~FeatureMatchState() = default;

size_t FeatureMatchState::promptCount()
{
    return ::promptCount("Features per view", kMaxFeatures, kDefaultFeatures);
}

std::optional<size_t> FeatureMatchState::promptMinInliers()
{
    return ::promptOptionalCount("Minimum agreeing matches", kMinConsensus, kMaxConsensus);
}

void FeatureMatchState::onEnter(Simulation &sim)
{
    std::cout << "FEATURE MATCH: " << sim.waypoints.size() << " recorded views, "
              << m_featureCount << " features each";
    if (m_minInliers)
        std::cout << ", " << *m_minInliers << " agreeing matches per pose";
    std::cout << ". G = build the database by hand: "
              << "for each view SIFT highlights a point (red) in the player (right) view "
              << "-- color-pick its 3D spot in the global (left) map; X skips one, "
              << "Ctrl+X skips the rest of the view, U undoes the last one.\n"
              << "               The map shows the view's cyan cone and a red line from "
              << "the camera: the point you are placing lies somewhere ALONG that line, "
              << "and your click is snapped onto it -- so only how far along it you "
              << "click matters. Orbit to judge that. V cycles the aids: full / cone "
              << "only / off -- below full there is no line and no snap, so clicks "
              << "land exactly where you pick.\n"
              << "               Global map: scroll = zoom, middle-drag = pan, "
              << "right-drag = rotate.\n"
              << "               Then " << kCaptureHelp
              << ". Ctrl+S saves the database, Ctrl+O loads it back. Ctrl+G auto-builds"
              << " and saves one (arc / high circle / scattered path, simulated aim"
              << " error) when you just need a database to test against." << std::endl;
}

// Bind this renderer into the FrameCapture the vision passes and the build take.
static FrameCapture captureWith(Renderer &renderer, const Simulation &sim)
{
    return [&renderer, &sim](const Camera &camera, const Viewport &vp) {
        return renderer.captureSceneFrameAt(vp.width, vp.height, camera, sim.light());
    };
}

// What a build step needs from the session, gathered per call so the build
// holds no references into it. A free function, not a member: naming
// ManualBuild::Context in the state header would drag OpenCV in with it.
static ManualBuild::Context buildContext(Simulation &sim, Renderer &renderer,
                                         size_t featureCount)
{
    return { sim.mesh, sim.terrainSize, sim.playerView.camera, featureCount,
             captureWith(renderer, sim) };
}

// G: discard any previous database and start a fresh hand-build at view 0.
// Rebuilding from scratch is deliberate: a database must be anchored under one
// light, so mixing two lightings (the experiment) would muddy it.
void FeatureMatchState::startBuild(Simulation &sim, Renderer &renderer)
{
    m_db.reset();      // the old database goes with the old build
    refreshPlaces();   // and so do its markers

    // A fresh build adopts the live pane's size as the database's capture
    // resolution; every capture from here on renders at it, whatever the window
    // does later.
    const Viewport frame{ 0, 0, sim.playerView.viewport.width,
                                sim.playerView.viewport.height };
    m_build = std::make_unique<ManualBuild>(sim.waypoints, frame,
                                            buildContext(sim, renderer, m_featureCount));
    followBuild(sim, renderer);
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

void FeatureMatchState::addOtherViewAppearances(Simulation &sim, Renderer &renderer)
{
    if (!m_db)
        return;
    collectAppearances(*m_db, sim.mesh, sim.terrainSize, sim.waypoints,
                       sim.playerView.camera, captureViewport(sim),
                       captureWith(renderer, sim));
}

// The debrief promised at placement time: distance from each hand anchor to
// its ray's true terrain point, disclosed only now that no placement remains
// to assist. Tenth-of-a-unit precision -- whole units are the scale that
// matters against the solver's tolerances.
static void reportPlacementDebrief(const std::vector<float> &errors)
{
    std::vector<float> landed;
    for (float e : errors)
        if (e >= 0.0f)
            landed.push_back(e);
    if (landed.empty())
        return;

    std::sort(landed.begin(), landed.end());
    std::cout << "FEATURES: placement debrief -- median "
              << std::round(landed[landed.size() / 2] * 10.0f) / 10.0f
              << " units off the true terrain point, worst "
              << std::round(landed.back() * 10.0f) / 10.0f << " (" << landed.size()
              << " anchors; diagnostic only, nothing stored)" << std::endl;
}

// reportDbAudit's companion that needs the mesh: how many hand-placed points
// the snap left under the surface. Diagnostic only -- visibility already
// forgives burial by judging the surface point (anchorVisibleFrom) -- but a
// buried anchor's DEPTH is 3D error PnP still pays for, so the count is worth
// reading next to the debrief.
static void reportBuriedAnchors(const Simulation &sim, const FeatureDb &db)
{
    const std::vector<glm::vec3> places = db.places();
    size_t buried = 0;
    float deepest = 0.0f;
    for (const glm::vec3 &place : places) {
        const std::optional<float> ground = sim.mesh.heightAt(place.x, place.z);
        if (ground && *ground > place.y) {
            buried++;
            deepest = std::max(deepest, *ground - place.y);
        }
    }
    if (buried > 0)
        std::cout << "FEATURES: " << buried << " of " << places.size()
                  << " anchors sit below the terrain surface (deepest "
                  << std::round(deepest * 10.0f) / 10.0f
                  << " units) -- visibility is judged at the surface above them"
                  << std::endl;
}

void FeatureMatchState::followBuild(Simulation &sim, Renderer &renderer)
{
    if (m_build->finished()) {
        finishBuild(sim, renderer);
        return;
    }
    // The right pane shows the view being anchored, so the suggestion marker
    // sits over the terrain the user is placing.
    sim.playerView.camera.applyPose(m_build->currentPose());
}

void FeatureMatchState::finishBuild(Simulation &sim, Renderer &renderer)
{
    // The database's frame size outlives the build that chose it: every later
    // capture must render at what these descriptors were computed at.
    m_captureWidth  = m_build->frame().width;
    m_captureHeight = m_build->frame().height;

    ManualBuild::Result built = m_build->takeResult();
    m_db = std::make_unique<FeatureDb>(std::move(built.db));
    m_build.reset();   // back to run-phase: tick flies again, B/N/M work

    addOtherViewAppearances(sim, renderer);
    reportPlacementDebrief(built.placementErrors);
    reportDbAudit(*m_db);
    reportBuriedAnchors(sim, *m_db);

    std::cout << "FEATURES: database built -- " << m_db->places().size()
              << " hand-placed points. Ctrl+S saves it so this need not be redone. "
              << kCaptureHelp << std::endl;
    refreshPlaces();
}

// Ctrl+S: write the placed database out. Run-phase only (handleKey enforces
// it): a half-finished build would write a count that disagrees with what the
// user is still placing.
void FeatureMatchState::saveDatabase(const Simulation &sim) const
{
    if (!m_db || m_db->empty() || m_db->descriptors.rows != (int)m_db->anchors.size()) {
        std::cout << "FEATURES: no database to save -- press G to build one first"
                  << std::endl;
        return;
    }
    saveFeatureDb(featureDbPath(sim.terrainFile), *m_db, sim.waypoints, sim.pathPoints,
                  sim.terrainFile, m_captureWidth, m_captureHeight);
}

// Ctrl+O: replace the database with the saved one. Read into a fresh FeatureDb
// so a refused load leaves the current one untouched.
void FeatureMatchState::loadDatabase(Simulation &sim, Renderer &renderer)
{
    auto loaded = std::make_unique<FeatureDb>();
    std::vector<Waypoint> waypoints;
    std::vector<glm::vec3> pathPoints;
    int captureWidth = 0, captureHeight = 0;
    if (!loadFeatureDb(featureDbPath(sim.terrainFile), *loaded, waypoints, pathPoints,
                       sim.terrainFile, captureWidth, captureHeight))
        return;

    m_db = std::move(loaded);

    // The file's capture size is the resolution its descriptors were computed
    // at; every capture from here renders at it. A file from before the size
    // was recorded adopts the live pane, and the next Ctrl+S makes that stick.
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

    // The waypoints come back too -- the anchors were placed from those views,
    // so restoring them is what makes Ctrl+B mean anything after a load.
    if (!waypoints.empty())
        sim.waypoints = std::move(waypoints);

    // The path is replaced unconditionally, whatever the waypoints did: what
    // is on screen after a load must be THIS database's provenance, and a path
    // left over from the session's own flying would be drawn as if it were.
    // A file from before paths were saved falls back to joining the views in
    // order (straight segments, unclosed -- whether the flight looped is not
    // knowable from the stops), and the next Ctrl+S records that stand-in.
    if (!pathPoints.empty()) {
        sim.pathPoints = std::move(pathPoints);
    } else {
        sim.pathPoints.clear();
        for (const Waypoint &waypoint : sim.waypoints)
            sim.pathPoints.push_back(waypoint.position);
        if (!sim.pathPoints.empty())
            std::cout << "FEATURES: this file predates saved flight paths -- drawing"
                      << " the recorded views joined in order; Ctrl+S records it"
                      << std::endl;
    }

    // Top up whatever appearances the file lacks -- an older one may hold only
    // the hand-placed rows. Collection skips rows a place already owns, so this
    // upgrades any vintage of file without bloating a current one.
    if (!m_db->empty()) {
        addOtherViewAppearances(sim, renderer);
        reportDbAudit(*m_db);
        reportBuriedAnchors(sim, *m_db);
    }

    refreshPlaces();
}

// The build's closing words: what was built on which path, then where flying
// will recognise it best. captureHelp arrives as a parameter -- it is a
// protected member of the base class, out of a file-static function's reach.
static void reportAutoBuild(const Simulation &sim, const AutoPathSpec &spec,
                            float mapSpacing, size_t placed, size_t aimError,
                            const char *captureHelp)
{
    std::cout << "FEATURES: auto-built " << placed << " points from "
              << sim.waypoints.size() << " views on " << spec.name
              << " (map spacing ~" << (int)mapSpacing
              << " units; simulated aim error sigma " << aimError
              << " units along the sight line). " << captureHelp << std::endl;
    std::cout << spec.flyingHint << std::endl;
}

void FeatureMatchState::autoBuild(Simulation &sim, Renderer &renderer)
{
    const auto mode = (AutoPathMode)(::promptCount(
        "Auto-path mode (1 = arc, 2 = high circle, 3 = scattered)", 3, 2) - 1);
    const AutoPathSpec &spec = kAutoPathSpecs[(size_t)mode];
    const size_t views    = ::promptCount("Auto-path waypoints", kMaxAutoViews,
                                          spec.defaultViews);
    const size_t features = ::promptCount("Features per view", kMaxFeatures,
                                          kDefaultAutoFeatures);
    const size_t aimError = ::promptCount("Simulated aim error (sigma, units of depth)",
                                          kMaxAimErrorUnits, kDefaultAimErrorUnits);

    layOutAutoPath(sim, mode, views);

    // Same lifecycle as startBuild: fresh database, any build in progress
    // discarded. The capture resolution does NOT follow the window here -- with
    // no on-screen markers to stay aligned with, one canonical size means a
    // small window cannot quietly starve SIFT of keypoints.
    m_build.reset();
    m_captureWidth  = kAutoCaptureWidth;
    m_captureHeight = kAutoCaptureHeight;

    AutoBuildResult built = autoBuildDatabase(sim.mesh, sim.terrainSize, sim.waypoints,
                                              sim.playerView.camera, captureViewport(sim),
                                              { features, aimError },
                                              captureWith(renderer, sim));
    m_db = std::make_unique<FeatureDb>(std::move(built.db));

    if (m_db->empty()) {
        std::cout << "FEATURES: auto-build anchored nothing -- every suggestion missed"
                     " the terrain" << std::endl;
        refreshPlaces();
        return;
    }

    const size_t placed = m_db->anchors.size();
    addOtherViewAppearances(sim, renderer);
    refreshPlaces();
    reportAutoBuild(sim, spec, built.mapSpacing, placed, aimError, kCaptureHelp);
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
        if (key == GLFW_KEY_X && (mods & GLFW_MOD_CONTROL)) {
            m_build->skipRestOfView(buildContext(sim, renderer, m_featureCount));
            followBuild(sim, renderer);
        } else if (key == GLFW_KEY_X) {   // skip an unplaceable suggestion
            m_build->skipSuggestion(buildContext(sim, renderer, m_featureCount));
            followBuild(sim, renderer);
        } else if (key == GLFW_KEY_U) {
            m_build->undo();
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
    m_nearestView = nearestRecordedView(sim.waypoints, sim.playerView.camera);
    reportAnchorCount();
}

// The run phase's asker: the player's live view (see anchorVisibleFrom).
static bool anchorVisible(const Simulation &sim, const glm::vec3 &anchor)
{
    return anchorVisibleFrom(sim.mesh, sim.terrainSize, sim.playerView.camera,
                             sim.playerView.viewport, anchor);
}

std::optional<NearestView> nearestRecordedView(const std::vector<Waypoint> &views,
                                               const Camera &camera)
{
    // Choosing runs every frame, so it uses SQUARED distance -- no square root
    // per waypoint, same ordering. Only the winner then pays for poseError,
    // whose heading half (two normalises and an acos) is the expensive part
    // and is wanted for exactly one view.
    size_t best = views.size();
    float  bestSquared = 0.0f;
    for (size_t i = 0; i < views.size(); i++) {
        const glm::vec3 gap = views[i].position - camera.position;
        const float squared = glm::dot(gap, gap);
        if (best == views.size() || squared < bestSquared) {
            best = i;
            bestSquared = squared;
        }
    }
    if (best == views.size())
        return std::nullopt;

    const PoseError gap = poseError(views[best], camera.pose());
    return NearestView{ best, views[best].position, gap.positionUnits, gap.headingDegrees };
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

    // Only on a change, and no more than twice a second: these numbers move
    // continuously while flying, and a line per frame would bury every other
    // message in the console. The nearest-view pair counts as changed when it
    // crosses a 5-unit / 5-degree bucket, not on every drifted decimal.
    const int distanceBucket = m_nearestView ? (int)(m_nearestView->distanceUnits / 5.0f) : -1;
    const int headingBucket  = m_nearestView ? (int)(m_nearestView->headingOffDegrees / 5.0f) : -1;

    const double now = glfwGetTime();
    const bool unchanged = m_inView.size() == m_reportedInView &&
                           distanceBucket == m_reportedDistanceBucket &&
                           headingBucket == m_reportedHeadingBucket;
    if (unchanged || now - m_lastCountReport < 0.5)
        return;

    m_reportedInView          = m_inView.size();
    m_reportedDistanceBucket  = distanceBucket;
    m_reportedHeadingBucket   = headingBucket;
    m_lastCountReport         = now;

    std::cout << "FEATURES: " << m_inView.size() << " of " << m_places.size()
              << " anchors in view";
    if (m_nearestView)
        std::cout << " | nearest view " << (m_nearestView->index + 1) << ": "
                  << std::lround(m_nearestView->distanceUnits) << "u away, heading "
                  << std::lround(m_nearestView->headingOffDegrees) << " deg off";
    std::cout << std::endl;
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
    // user's 3D pick -- the one step that would otherwise be automatic. At
    // Full aids the click first snaps onto the suggestion's exact viewing
    // ray: the user supplies only the DEPTH, and the sideways part of a pick
    // is aim error a solve would punish (see snapToViewRay in Camera.h).
    // Below Full the snap is off with the line -- the pick is stored exactly
    // where it landed, sideways error and all.
    const Camera &camera = sim.playerView.camera;
    const glm::vec2 ray = m_build->activeRay(camera.fov);
    glm::vec3 anchor = sim.mesh.worldPos(id);
    if (sim.viewAids == ViewAids::Full) {
        const std::optional<glm::vec3> snapped = snapToViewRay(camera, ray, anchor);
        if (!snapped) {
            std::cout << "FEATURES: that spot is behind the view -- pick along the red line."
                      << std::endl;
            return;
        }
        anchor = *snapped;
    }

    m_build->place(anchor, ray, buildContext(sim, renderer, m_featureCount));
    followBuild(sim, renderer);
}


std::optional<Waypoint> FeatureMatchState::computePose(Simulation &sim, Renderer &renderer)
{
    if (!m_db || m_db->empty()) {
        std::cout << "FEATURES: no database yet -- press G to build it first"
                  << std::endl;
        return std::nullopt;
    }

    // What was AVAILABLE, before what was found: 7 anchors in frame and 2
    // matched is a matcher problem, 2 in frame and 2 matched is a positioning
    // one. Recomputed here, not trusted from the last tick -- Ctrl+B poses the
    // camera itself between captures.
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
    std::vector<Correspondence> inliers;
    std::optional<Waypoint> estimate =
        estimatePoseFromFeatures(*m_db, frame, sim.playerView.camera.fov,
                                 vp.width, vp.height, m_minInliers, &inliers,
                                 &m_pinnedFloorNoted);

    if (estimate && !poseIsPlausible(sim.mesh, sim.terrainSize, sim.playerView.camera,
                                     *estimate, vp, inliers))
        return std::nullopt;
    return estimate;
}

// The active suggestion's sight line shares its player-view marker's red, so
// the dot to place and the line to place it on read as one object; earlier
// anchors get dim lines that double as a check (a green anchor off its own
// line was misplaced -- U takes it back). Lines stop at a fixed reach, never
// at the terrain: that intersection would BE the correspondence the user is
// here to supply.
void FeatureMatchState::drawSightAids(const Simulation &sim, Renderer &renderer,
                                      const glm::mat4 &mvp) const
{
    const std::vector<glm::vec2> &markers = m_build->markers();
    if (sim.viewAids != ViewAids::Full || markers.empty())
        return;

    const Camera &camera = sim.playerView.camera;
    const float fov    = camera.fov;
    // The markers are fractions of the BUILD's frame, so their rays are built
    // with its aspect -- the live pane's only coincides until a mid-build
    // window resize.
    const float aspect = m_build->frame().aspect();
    const float reach  = sim.terrainSize * 1.5f;   // clears the map from anywhere on it

    const std::vector<size_t> &anchored = m_build->anchoredMarkers();
    if (!anchored.empty()) {
        std::vector<glm::vec2> rays;
        rays.reserve(anchored.size());
        for (size_t marker : anchored)
            rays.push_back(fractionToRay(markers[marker], fov, aspect));

        renderer.drawSightLines(camera, rays, reach, overlay::sightLineDimColor,
                                overlay::sightLineDimWidth, mvp);
    }

    if (m_build->activeMarker() < markers.size())
        renderer.drawSightLines(camera, { m_build->activeRay(fov) },
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
// were placed from. Grey and dim on purpose -- provenance, not a measurement,
// and the run phase's own captures are already blue and red on the same map.
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

        // The envelope tether, an aid like the view cone (V hides both):
        // camera to the nearest recorded view, cyan while the heading is
        // inside the recognition range, provenance-grey once it has turned
        // past it -- distance is the line, the color is the angle. Drawn to
        // the position CACHED with the readout, never to a waypoint looked up
        // by index: the list is rebuilt by loads and auto-builds.
        if (sim.viewAids != ViewAids::Off && m_nearestView) {
            const bool aligned =
                m_nearestView->headingOffDegrees <= kViewpointToleranceDegrees;
            renderer.drawLine(sim.playerView.camera.position, m_nearestView->position,
                              aligned ? overlay::tetherColor : overlay::sightLineDimColor,
                              overlay::tetherWidth, mvp);
        }
        return;
    }

    // Build phase: flight context (drawWaypoints marks the current build view
    // green), the sight lines, then the anchors placed so far on top. All
    // bright -- the camera is pinned, so the run phase's in-view split has
    // nothing to say here.
    renderer.drawPath(sim.pathPoints, overlay::truePathColor, mvp);
    renderer.drawWaypoints(sim.waypoints, sim.playerView.camera.position, mvp);
    drawSightAids(sim, renderer, mvp);
    drawDots(renderer, m_build->database().anchors, overlay::anchorColor,
             overlay::anchorMarkerSize, mvp);
}

void FeatureMatchState::renderPlayerOverlay(const Simulation &sim, Renderer &renderer,
                                            const glm::mat4 &mvp) const
{
    if (!building()) {
        PoseComparisonState::renderPlayerOverlay(sim, renderer, mvp);
        return;
    }
    (void)mvp;

    // Only the active suggestion -- one at a time keeps the anchoring
    // unambiguous. An ortho over the unit square maps the stored [0,1] fraction
    // straight to the viewport (the same screen matrix PICK uses).
    if (m_build->activeMarker() < m_build->markers().size()) {
        const glm::mat4 screen = glm::ortho(0.0f, 1.0f, 1.0f, 0.0f);
        const glm::vec3 pos(m_build->markers()[m_build->activeMarker()], 0.0f);
        renderer.drawPoints({ pos }, { overlay::suggestionColor },
                            overlay::suggestionMarkerSize, screen);
    }
}
