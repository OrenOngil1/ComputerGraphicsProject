#include "FeatureMatchState.h"

#include <algorithm>
#include <cmath>
#include <iostream>
#include <limits>
#include <random>
#include <vector>

#include <GLFW/glfw3.h>
#include <glm/gtc/matrix_transform.hpp>   // glm::ortho

#include "OverlayStyle.h"
#include "../core/Menu.h"         // promptCount, promptOptionalCount
#include "../core/Simulation.h"
#include "../input/Callbacks.h"   // cursorPosPixels
#include "../render/Renderer.h"
#include "../vision/FeatureDbIo.h"
#include "../vision/FeatureMatching.h"

// Pre-phase scratch: which recorded view is being anchored, which suggestion is
// active, that view's top-N SIFT suggestions, and their descriptors. In the
// .cpp so OpenCV types stay out of the state header.
//
// markers is DISPLAY ONLY -- the 2D SIFT position is the dot the user aims
// from; only (descriptor, user-picked 3D) enters the database, and run-phase
// PnP pairs each match with the LIVE frame's keypoint pixel.
struct BuildScratch {
    size_t                 waypoint = 0;
    size_t                 active   = 0;
    std::vector<glm::vec2> markers;       // [0,1] screen fractions of the suggestions
    cv::Mat                descriptors;   // one SIFT descriptor row per marker

    // Markers anchored in THIS view, newest last -- the undo stack for 'U'.
    // Indices rather than a count because skips leave gaps: stepping `active`
    // back by one would land on a skipped suggestion. Cleared on a new view.
    std::vector<size_t> anchoredAt;

    // Distance from each placed anchor to its ray's true terrain point, whole
    // build, placement order -- the finishBuild debrief's data, measured at
    // placement and shown only after the last view. Negative marks a ray that
    // missed the terrain (kept so undo stays aligned with anchoredAt).
    std::vector<float> placementErrors;
};

// FeatureDb + BuildScratch are complete in this TU, so the unique_ptr members'
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

// The build's duplicate guard: drop suggestions that already resemble an
// anchored point. Left in, they would be re-anchored as SECOND places --
// identities the matcher cannot tell apart (measured as twin pairs a couple
// of units apart). Re-sightings belong to the appearance pass; the
// auto-build's selectSpacedAnchors is this same guard in map space.
static size_t dropAnchoredLookalikes(const FeatureDb &db,
                                     std::vector<cv::KeyPoint> &kps, cv::Mat &desc)
{
    if (db.empty() || kps.empty())
        return 0;

    std::vector<cv::KeyPoint> kept;
    cv::Mat keptDesc;
    for (int i = 0; i < desc.rows; i++) {
        if (resemblesAnyAnchoredPoint(db, desc.row(i)))
            continue;
        kept.push_back(kps[(size_t)i]);
        keptDesc.push_back(desc.row(i));
    }

    const size_t dropped = kps.size() - kept.size();
    kps  = std::move(kept);
    desc = keptDesc;
    return dropped;
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

        const size_t dropped = dropAnchoredLookalikes(*m_db, kps, desc);
        if (dropped)
            std::cout << "FEATURES: view " << (m_build->waypoint + 1) << " -- " << dropped
                      << " suggestion(s) are re-detections of already-anchored points;"
                         " skipped (the appearance pass collects re-sightings)" << std::endl;

        if (kps.empty()) {
            // Which of the two emptied it matters: no features is the terrain
            // being featureless here, all-dropped is the guard, and calling
            // the second one "no features" hid the guard from the one person
            // who could judge it.
            std::cout << "FEATURES: view " << (m_build->waypoint + 1)
                      << (dropped ? " has nothing left to place -- every suggestion was"
                                    " already anchored; skipping."
                                  : " has no features -- skipping.") << std::endl;
            m_build->waypoint++;
            continue;
        }

        m_build->markers.clear();
        m_build->markers.reserve(kps.size());
        for (const cv::KeyPoint &kp : kps) {
            m_build->markers.push_back(glm::vec2(kp.pt.x / frame.width,
                                                 kp.pt.y / frame.height));
        }
        m_build->descriptors = desc;
        m_build->active = 0;
        m_build->anchoredAt.clear();   // undo is scoped to the current view
        std::cout << "FEATURES: view " << (m_build->waypoint + 1) << "/"
                  << sim.waypoints.size() << " -- place " << kps.size()
                  << " features on the map. (" << m_db->anchors.size()
                  << " anchored so far)" << std::endl;
        return;
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

// Descriptor distance under which two rows are the same appearance rather than
// a similar one -- a recomputed row is bit-identical, so this only absorbs
// float noise.
static constexpr double kIdenticalRowDistance = 1.0;

// The keypoint closest to `pixel` within `radius`, or -1 when none reaches.
static int nearestKeypointTo(const glm::vec2 &pixel,
                             const std::vector<cv::KeyPoint> &keypoints, float radius)
{
    int nearest = -1;
    float nearestDistance = radius;
    for (size_t i = 0; i < keypoints.size(); i++) {
        const float d = glm::length(glm::vec2(keypoints[i].pt.x, keypoints[i].pt.y) - pixel);
        if (d < nearestDistance) {
            nearestDistance = d;
            nearest = (int)i;
        }
    }
    return nearest;
}

// Can a camera drawing through `viewport` actually use this anchor? Frustum is
// not enough: SIFT matches what was DRAWN, and a feature behind a ridge was
// not. One occlusion test serves three askers -- the run phase's in-view
// split, the appearance pass, and the plausibility check on an estimate.
static bool anchorVisibleFrom(const Mesh &mesh, float terrainSize, const Camera &camera,
                              const Viewport &viewport, const glm::vec3 &anchor)
{
    // Judge the surface feature the anchor DENOTES, not the stored point: the
    // snap can leave a point under the surface, and a buried point reads
    // "hidden" from every camera alike -- true pose and garbage both -- so its
    // verdict carries no information. The stored 3D never moves; only the
    // visibility question is lifted to the surface.
    glm::vec3 feature = anchor;
    const std::optional<float> ground = mesh.heightAt(anchor.x, anchor.z);
    if (ground && *ground > feature.y)
        feature.y = *ground;

    if (!isInFrame(camera, viewport, feature))
        return false;

    const glm::vec3 toFeature = feature - camera.position;
    const float distance = glm::length(toFeature);

    // Stop the march short of the feature itself: a hit at its own distance
    // is the ground it belongs to, not something standing in front of it.
    const float margin = std::max(1.0f, terrainSize / 100.0f);
    const float step   = std::max(0.5f, terrainSize / 300.0f);
    if (distance <= margin)
        return true;

    return !raycastTerrain(mesh, camera.position, toFeature / distance,
                           distance - margin, step);
}

// Where a pass's (place, view) pairs went, one counter per gate. Printed for
// the LAST fixpoint pass, whose tallies describe the steady state: what stays
// out of reach and which gate holds it -- the pass's own losses line, so a
// starved collection names its starver instead of just reporting a low count.
struct CollectTally {
    size_t pairs = 0, hidden = 0, noKeypoint = 0, unlike = 0,
           claimedAway = 0, alreadyStored = 0, added = 0;
    std::vector<double> unlikeNearest;   // nearest-row distances of `unlike`
};

// One place's bid for one keypoint: how far the place's projection missed it.
// The miss is what orders the bids, since the nearest projection has the best
// claim on a keypoint two places both reach.
struct AppearanceBid { float missPx; size_t place; int keypoint; };

// Phase one, geometry: which places this view can see, and which keypoint each
// one lands on. The search radius around a projection is a constant WORLD
// budget at the place's range (projectionRadiusPx), not a fixed pixel count.
static std::vector<AppearanceBid> gatherAppearanceBids(
    const std::vector<glm::vec3> &places, const Mesh &mesh, float terrainSize,
    const Camera &camera, const Viewport &vp,
    const std::vector<cv::KeyPoint> &keypoints, CollectTally &tally)
{
    const float fy = 0.5f * (float)vp.height / std::tan(glm::radians(camera.fov) * 0.5f);

    std::vector<AppearanceBid> bids;
    tally.pairs += places.size();
    for (size_t p = 0; p < places.size(); p++) {
        if (!anchorVisibleFrom(mesh, terrainSize, camera, vp, places[p])) {
            tally.hidden++;   // occluded: a ridge is in front, geometry says so
            continue;
        }
        const float radius = projectionRadiusPx(kAnchorSlopUnits, fy,
                                                glm::distance(places[p], camera.position),
                                                (float)vp.height);
        const glm::vec2 pixel = rasterize(camera, vp, places[p]);
        const int keypoint = nearestKeypointTo(pixel, keypoints, radius);
        if (keypoint < 0) {
            tally.noKeypoint++;
            continue;
        }
        const glm::vec2 kp(keypoints[(size_t)keypoint].pt.x, keypoints[(size_t)keypoint].pt.y);
        bids.push_back({ glm::length(kp - pixel), p, keypoint });
    }
    return bids;
}

// Phase two, appearance: award each keypoint to its nearest bidder and store
// the row if the descriptor agrees. A claimed keypoint is spent even when its
// resemblance check then fails -- handing it to a farther place is exactly the
// shared-row failure the ordering exists to prevent.
static void adoptWinningBids(FeatureDb &db, const std::vector<glm::vec3> &places,
                             std::vector<AppearanceBid> &bids, size_t keypointCount,
                             const cv::Mat &descriptors, CollectTally &tally)
{
    std::sort(bids.begin(), bids.end(),
              [](const AppearanceBid &a, const AppearanceBid &b) { return a.missPx < b.missPx; });

    std::vector<bool> claimed(keypointCount, false);
    for (const AppearanceBid &bid : bids) {
        if (claimed[(size_t)bid.keypoint]) {
            tally.claimedAway++;
            continue;
        }
        claimed[(size_t)bid.keypoint] = true;

        // One distance answers both descriptor questions: too far from every
        // row the place owns is a different feature (the collection bar --
        // geometry already pinned WHERE, see kCollectResemblanceDistance);
        // near-zero is the same row recomputed, which keeps a load's re-run
        // topping the database up instead of duplicating it.
        const double apart = nearestAppearanceDistance(db, places[bid.place],
                                                       descriptors.row(bid.keypoint));
        if (apart > kCollectResemblanceDistance) {
            tally.unlike++;
            tally.unlikeNearest.push_back(apart);
        } else if (apart <= kIdenticalRowDistance) {
            tally.alreadyStored++;
        } else {
            db.descriptors.push_back(descriptors.row(bid.keypoint));
            db.anchors.push_back(places[bid.place]);
            tally.added++;
        }
    }
}

// One view's contribution, hardened three ways (docs/pose-estimation-modes.md,
// "Hardening the database"): only unoccluded places collect, so "is a ridge in
// front" is answered by geometry rather than descriptor resemblance; the
// search radius is a world budget rather than a pixel count; and a keypoint
// feeds at most one place. The two phases above are that order -- geometry
// first, descriptors only over what geometry already vouched for.
static void collectAppearancesInView(FeatureDb &db, const std::vector<glm::vec3> &places,
                                     const Mesh &mesh, float terrainSize,
                                     const Camera &camera, const Viewport &vp,
                                     const std::vector<cv::KeyPoint> &keypoints,
                                     const cv::Mat &descriptors, CollectTally &tally)
{
    std::vector<AppearanceBid> bids = gatherAppearanceBids(places, mesh, terrainSize,
                                                           camera, vp, keypoints, tally);
    adoptWinningBids(db, places, bids, keypoints.size(), descriptors, tally);
}

// Render one recorded view offscreen and let it contribute appearances. A
// throwaway Camera so the player camera is left where the build put it.
static void collectAppearancesFromWaypoint(FeatureDb &db, const std::vector<glm::vec3> &places,
                                           const Simulation &sim, Renderer &renderer,
                                           const Waypoint &waypoint, const Viewport &vp,
                                           CollectTally &tally)
{
    Camera camera = sim.playerView.camera;
    camera.applyPose(waypoint);

    const FramePixels frame = renderer.captureSceneFrameAt(vp.width, vp.height,
                                                           camera, sim.light());
    if (frame.rgb.empty())
        return;
    std::vector<cv::KeyPoint> keypoints;
    cv::Mat descriptors;
    detectAllFeatures(frame, keypoints, descriptors);
    if (keypoints.empty())
        return;

    collectAppearancesInView(db, places, sim.mesh, sim.terrainSize, camera, vp,
                             keypoints, descriptors, tally);
}

// Nothing is invented here: the recorded poses are exact and the anchor is the
// user's own 3D point, so where it falls in another view is plain projection --
// no mesh is read for a 3D choice, no 3D is chosen by machine. The pass is
// self-limiting, which is what keeps it that way: an appearance counts only if
// a keypoint sits near the projection AND still resembles the anchored
// descriptor, so a misjudged depth gains nothing instead of poisoning the
// database. (docs/pose-estimation-modes.md, "Mode D")
void FeatureMatchState::addOtherViewAppearances(Simulation &sim, Renderer &renderer)
{
    if (!m_db || m_db->empty())
        return;

    const std::vector<glm::vec3> places = m_db->places();
    const size_t placed = m_db->anchors.size();
    const Viewport vp = captureViewport(sim);

    // The RECORDED views only, per the course brief: the database describes
    // what was actually seen at the waypoints, with nothing synthesized
    // between. Up to three passes: resemblance is judged against the rows a
    // place owns SO FAR, so a place anchored late in the flight reaches early
    // views only through rows added by later ones -- one pass in waypoint
    // order leaves such chains half-built. Idempotent rows make repeats safe;
    // stop as soon as a pass adds nothing.
    CollectTally tally;
    for (int pass = 0; pass < 3; pass++) {
        tally = CollectTally{};
        const size_t before = m_db->anchors.size();
        for (const Waypoint &waypoint : sim.waypoints)
            collectAppearancesFromWaypoint(*m_db, places, sim, renderer, waypoint, vp,
                                           tally);
        if (m_db->anchors.size() == before)
            break;
    }

    std::cout << "FEATURES: " << places.size() << " placed points seen "
              << m_db->anchors.size() << " times across the " << sim.waypoints.size()
              << " recorded views (" << (m_db->anchors.size() - placed)
              << " appearances added by this pass)" << std::endl;

    // The last pass's tallies are the steady state: what stays uncollected and
    // which gate holds it. Read `unlike` misses against the collection bar the
    // way capture losses are read against the match bar.
    std::sort(tally.unlikeNearest.begin(), tally.unlikeNearest.end());
    std::cout << "        (final pass: of " << tally.pairs << " place-view pairs -- "
              << tally.hidden << " hidden, " << tally.noKeypoint
              << " no keypoint in reach, " << tally.unlike
              << " unlike their place's rows";
    for (size_t i = 0; i < tally.unlikeNearest.size() && i < 3; i++)
        std::cout << (i ? ", " : " -- nearest ") << (int)tally.unlikeNearest[i];
    std::cout << ", " << tally.claimedAway << " claimed by a nearer place, "
              << tally.alreadyStored << " already stored, "
              << tally.added << " added)" << std::endl;
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

void FeatureMatchState::finishBuild(Simulation &sim, Renderer &renderer)
{
    addOtherViewAppearances(sim, renderer);
    reportPlacementDebrief(m_build->placementErrors);
    reportDbAudit(*m_db);
    reportBuriedAnchors(sim, *m_db);

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

// Ctrl+X: skip the REST of the current view in one press. The dense-ring
// workflow anchors in a few spread views and lets the appearance pass harvest
// the others -- walking each unanchored 8-suggestion view out with X was the
// friction that made recording more views expensive. Anchors already placed
// in the view stay placed.
void FeatureMatchState::skipView(Simulation &sim, Renderer &renderer)
{
    const size_t unplaced = m_build->markers.size() - m_build->active;
    std::cout << "FEATURES: skipped the rest of view " << (m_build->waypoint + 1)
              << " -- " << unplaced << " suggestion(s) left unplaced." << std::endl;
    m_build->waypoint++;
    loadCurrentView(sim, renderer);   // next view, or finishBuild() when exhausted
}

// Take back the last anchor placed in the CURRENT view: drop its database row
// and 3D point, and make that suggestion active again.
//
// Scoped to this view: crossing a view boundary would mean re-posing the camera
// and re-running SIFT to rebuild the markers, and the mistake this exists for --
// a misclick noticed right away -- never needs it.
void FeatureMatchState::undoAnchor()
{
    if (m_build->anchoredAt.empty()) {
        std::cout << "FEATURES: nothing to undo in this view." << std::endl;
        return;
    }

    m_build->active = m_build->anchoredAt.back();
    m_build->anchoredAt.pop_back();
    m_build->placementErrors.pop_back();   // pushed together, popped together

    // The two halves of an anchor are stored in parallel and must stay that
    // way: anchors[i] belongs to descriptor row i.
    m_db->anchors.pop_back();
    m_db->descriptors = m_db->descriptors.rowRange(0, m_db->descriptors.rows - 1).clone();

    std::cout << "FEATURES: undid the last anchor -- place feature "
              << (m_build->active + 1) << " again." << std::endl;
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

// ── The automated stand-in for the manual build (Ctrl+G) ──────
//
// The human is SIMULATED, not skipped: the true ray-terrain depth, disturbed
// by Gaussian aim error, plays the user's EYES, never the estimator's -- the
// run phase still sees nothing but (descriptor, 3D) pairs and cannot tell the
// two builds apart. (Why this is honest: docs/pose-estimation-modes.md, "Ctrl+G".)

// Aim error sigma, in units along the sight line -- the one dimension a human
// actually supplies. Keep it modest: a human clicks a terrain VERTEX, so real
// anchors sit on the surface, while this noise leaves the ray where it may --
// at sigma 15 a third of the anchors ended up buried past the visibility
// margin and the run phase (rightly) distrusted every pose. Large values
// distort rather than degrade. The Ctrl+G prompt overrides the default; the
// honest setting is your own number from the manual build's debrief.
static constexpr size_t kDefaultAimErrorUnits = 4;
static constexpr size_t kMaxAimErrorUnits     = 40;

static constexpr size_t kMaxAutoViews        = 20;
static constexpr size_t kDefaultAutoFeatures = 10;   // denser than the manual default:
                                                     // clicks are free here

// One seed for everything the auto build randomises (aim error, scattered
// stations), so two builds with the same parameters are the same database --
// "change one thing and measure" stays possible.
static constexpr unsigned int kAutoBuildSeed = 20260805u;

static constexpr float kTau = 6.28318530718f;

// Arc: span around the terrain's middle, orbit radius, and flight altitude
// (fractions of the terrain's width). Low and tight: its ~13 degree steps
// reproduce the geometry of the manual corridors that work.
static constexpr float kArcSpanDegrees      = 120.0f;
static constexpr float kArcRadiusFraction   = 0.30f;
static constexpr float kArcAltitudeFraction = 0.25f;

// High survey circle: orbit radius, flight altitude, and how far past the
// centre each stop aims (fractions of the terrain's width), plus the stop
// count below which the azimuth step outgrows SIFT's viewpoint tolerance.
static constexpr float  kCircleRadiusFraction       = 0.22f;
static constexpr float  kCircleAltitudeFraction     = 0.50f;
static constexpr float  kCircleAimOvershootFraction = 0.17f;
static constexpr size_t kCircleComfortViews         = 12;

// Scattered stations: the sampling extent and each station's look-ahead
// (fractions of the terrain's width), the altitude band, and how many random
// candidates compete for each station's spot (best-candidate spacing).
static constexpr float kScatterExtentFraction    = 0.35f;
static constexpr float kScatterLookaheadFraction = 0.45f;
static constexpr float kScatterAltitudeMin       = 0.30f;
static constexpr float kScatterAltitudeMax       = 0.50f;
static constexpr int   kScatterCandidates        = 16;

// The three path shapes Ctrl+G can fly. Each mode's default view count suits
// its geometry: the circle needs ~12 stops to keep its azimuth steps inside
// SIFT's viewpoint tolerance; the arc and the scattered stations have no such
// coupling.
enum class AutoPathMode { Arc, Circle, Scattered };

struct AutoPathSpec {
    const char *name;          // as printed in the build summary
    size_t      defaultViews;
    const char *flyingHint;    // where recognition is strongest, printed after the build
};

static const AutoPathSpec kAutoPathSpecs[] = {
    { "an arc", 8,
      "FEATURES: recognition is strongest along the arc's own corridor -- fly"
      " the low sweep around the middle, looking at the centre" },
    { "a high survey circle", kCircleComfortViews,
      "FEATURES: recognition is strongest from viewpoints like the orbit's own"
      " -- fly near the grey ring, high, looking across the middle" },
    { "scattered survey stations", 10,
      "FEATURES: recognition is strongest near the recorded stations -- their"
      " spread trades per-spot depth for coverage of the whole map" },
};

// The canonical frame size an auto build pins its database to.
static constexpr int kAutoCaptureWidth  = 1280;
static constexpr int kAutoCaptureHeight = 720;

// Anchor selection: the target map spacing as a fraction of what would tile the
// terrain with the requested points, how far along a sight line terrain is
// still worth anchoring, the frame rim a descriptor patch must clear, and how
// many times the spacing may halve before a view settles for what it has.
static constexpr float kMapSpacingFactor   = 0.8f;
static constexpr float kReachLimitFraction = 1.25f;
static constexpr float kRimMarginPx        = 16.0f;
static constexpr int   kSpacingRetries     = 3;

// One raycast per keypoint: its sight ray, how far along that ray the terrain
// is, and the world hit where that ground is anchorable. Selection reads the
// hits; anchoring reads the ray and the depth. hits[i] is empty where keypoint
// i has no anchorable ground under it (the ray missed, or the surface is past
// the reach limit -- grazing-angle mush whose keypoints are not repeatable).
struct KeypointGround {
    std::vector<glm::vec3>                directions;
    std::vector<std::optional<float>>     depths;
    std::vector<std::optional<glm::vec3>> hits;
};

static KeypointGround raycastKeypointGround(const Simulation &sim, const Camera &camera,
                                            const std::vector<cv::KeyPoint> &kps,
                                            const FramePixels &frame, float aspect,
                                            float step, float reachLimit)
{
    KeypointGround ground;
    ground.directions.resize(kps.size());
    ground.depths.resize(kps.size());
    ground.hits.resize(kps.size());
    for (size_t i = 0; i < kps.size(); i++) {
        const glm::vec2 fraction(kps[i].pt.x / frame.width,
                                 kps[i].pt.y / frame.height);
        ground.directions[i] = rayDirection(camera, fractionToRay(fraction, camera.fov,
                                                                  aspect));
        ground.depths[i] = raycastTerrain(sim.mesh, camera.position, ground.directions[i],
                                          3.0f * sim.terrainSize, step);
        if (ground.depths[i] && *ground.depths[i] <= reachLimit)
            ground.hits[i] = camera.position + ground.directions[i] * *ground.depths[i];
    }
    return ground;
}

// The keypoints one view contributes, strongest first: taken only when its
// terrain hit clears `spacing` from every hit already taken -- this view's and
// `taken`, every earlier view's -- pushing later views onto unclaimed ground.
// Spacing is judged on the MAP, not in the frame: perspective piles
// pixel-uniform picks onto the ground nearest the camera (docs, "Ctrl+G").
// hits[i] is empty where keypoint i has no anchorable ground under it.
// Returns at most `count` indices into `kps`.
static std::vector<size_t> selectSpacedAnchors(
        const std::vector<cv::KeyPoint> &kps,
        const std::vector<std::optional<glm::vec3>> &hits,
        const std::vector<glm::vec3> &taken,
        size_t count, float startSpacing, int frameWidth, int frameHeight)
{
    // Strongest first, so the spread is paid for with the weaker of any two
    // crowded keypoints.
    const std::vector<size_t> order = rankByResponse(kps);

    std::vector<size_t> picked;
    float spacing = startSpacing;
    // Halve and retry when a view cannot fill its quota that sparsely -- a
    // tighter set beats a short one, exactly as detectSpreadFeatures does in
    // frame space.
    for (int attempt = 0; attempt < kSpacingRetries && picked.size() < count;
         attempt++, spacing *= 0.5f) {
        picked.clear();
        for (size_t i : order) {
            if (!hits[i])
                continue;

            // A rim keypoint's descriptor patch is half off-frame.
            const cv::Point2f &pt = kps[i].pt;
            if (pt.x < kRimMarginPx || pt.y < kRimMarginPx ||
                pt.x > (float)frameWidth - kRimMarginPx ||
                pt.y > (float)frameHeight - kRimMarginPx)
                continue;

            const auto farEnough = [&](const glm::vec3 &other) {
                return glm::distance(*hits[i], other) >= spacing;
            };
            const bool clearOfEarlierViews =
                std::all_of(taken.begin(), taken.end(), farEnough);
            const bool clearOfThisView =
                std::all_of(picked.begin(), picked.end(),
                            [&](size_t j) { return farEnough(*hits[j]); });
            if (!clearOfEarlierViews || !clearOfThisView)
                continue;

            picked.push_back(i);
            if (picked.size() == count)
                break;
        }
    }
    return picked;
}

// An ARC around the terrain's middle, not a full ring -- low, every stop aimed
// at the centre. On a low ring neighbouring views of the same spot sit ~36
// degrees apart, past SIFT's ~15-20 degree tolerance on 3D relief, and nothing
// matches across them; the arc's ~13 degree steps reproduce the geometry of
// every manual corridor that worked. (docs/pose-estimation-modes.md, "Ctrl+G")
static void layOutArc(Simulation &sim, size_t views)
{
    const float arcSpan  = glm::radians(kArcSpanDegrees);
    const float radius   = kArcRadiusFraction   * sim.terrainSize;
    const float altitude = kArcAltitudeFraction * sim.terrainSize;

    sim.waypoints.clear();
    sim.pathPoints.clear();
    for (size_t i = 0; i < views; i++) {
        const float t     = views > 1 ? (float)i / (float)(views - 1) : 0.5f;
        const float angle = (t - 0.5f) * arcSpan;
        const glm::vec3 eye(radius * std::cos(angle), altitude, radius * std::sin(angle));
        sim.waypoints.push_back({ eye, glm::vec3(0.0f) });
        sim.pathPoints.push_back(eye);
    }
}

// The calibrated hand-build ring (O in RECORD mode): the arc's radius and
// altitude -- the corridor geometry of every measured-good manual flight --
// closed into a full circle of 16 stops, every one aimed at the centre. The
// count is the calibration: 16 stops step 22.5 degrees, inside the ~25-degree
// recognition range a 21-degree flight demonstrated, and safely under the
// 36-degree steps of a hand-flown 10-station ring, where cross-view
// collection died (95 of its pairs missed the collection bar). Unlike
// Ctrl+G this lays ONLY the path: the anchoring stays entirely by hand.
static constexpr size_t kBuildRingViews = 16;

void layOutBuildRing(Simulation &sim)
{
    const float radius   = kArcRadiusFraction   * sim.terrainSize;
    const float altitude = kArcAltitudeFraction * sim.terrainSize;

    sim.waypoints.clear();
    sim.pathPoints.clear();
    for (size_t i = 0; i < kBuildRingViews; i++) {
        const float angle = kTau * (float)i / (float)kBuildRingViews;
        sim.waypoints.push_back({ glm::vec3(radius * std::cos(angle), altitude,
                                            radius * std::sin(angle)),
                                  glm::vec3(0.0f) });
        sim.pathPoints.push_back(sim.waypoints.back().position);
    }
    sim.pathPoints.push_back(sim.waypoints.front().position);   // close the ring on the map
}

// A full circle flown HIGH, each stop aimed at a ground point PAST the centre.
// Height is the legality condition: a low ring's azimuth steps blow SIFT's
// viewpoint budget (see layOutArc), but from high up a step is mostly an
// in-plane rotation, which SIFT absorbs -- the harmful residue shrinks with
// every extra stop (hence the 12-view default and the warning below). The
// past-centre aim fans each stop's footprint across the middle so the union
// covers the map. (Full geometry: docs/pose-estimation-modes.md, "Ctrl+G".)
static void layOutSurveyCircle(Simulation &sim, size_t views)
{
    const float radius    = kCircleRadiusFraction       * sim.terrainSize;
    const float altitude  = kCircleAltitudeFraction     * sim.terrainSize;
    const float overshoot = kCircleAimOvershootFraction * sim.terrainSize;

    if (views < kCircleComfortViews)
        std::cout << "FEATURES: note -- with fewer than ~" << kCircleComfortViews
                  << " stops a circle's azimuth steps outgrow SIFT's viewpoint"
                     " tolerance; expect weaker cross-view collection" << std::endl;

    sim.waypoints.clear();
    sim.pathPoints.clear();
    for (size_t i = 0; i < views; i++) {
        const float angle = kTau * (float)i / (float)views;
        const glm::vec3 out(std::cos(angle), 0.0f, std::sin(angle));
        sim.waypoints.push_back({ out * radius + glm::vec3(0.0f, altitude, 0.0f),
                                  -out * overshoot });
        sim.pathPoints.push_back(sim.waypoints.back().position);
    }
    sim.pathPoints.push_back(sim.waypoints.front().position);   // close the ring on the map
}

// Best-candidate spacing: a station takes the best of kScatterCandidates
// random spots -- the one farthest from every station already placed -- which
// spreads the set without a tuning parameter to get wrong.
static glm::vec2 bestSpacedSpot(std::mt19937 &rng, const std::vector<Waypoint> &placed,
                                const glm::vec2 &extent)
{
    std::uniform_real_distribution<float> coordX(-extent.x, extent.x);
    std::uniform_real_distribution<float> coordZ(-extent.y, extent.y);
    glm::vec2 best(0.0f);
    float bestScore = -1.0f;
    for (int c = 0; c < kScatterCandidates; c++) {
        const glm::vec2 candidate(coordX(rng), coordZ(rng));
        float nearest = std::numeric_limits<float>::max();
        for (const Waypoint &w : placed)
            nearest = std::min(nearest, glm::distance(candidate,
                                                      glm::vec2(w.position.x, w.position.z)));
        if (nearest > bestScore) {
            bestScore = nearest;
            best = candidate;
        }
    }
    return best;
}

// Headings from an evenly divided compass, jittered and shuffled: the angular
// spread is guaranteed, the order and exact bearings are not, so the set reads
// as random without ever clustering half the stations on one bearing.
static std::vector<float> spreadHeadings(std::mt19937 &rng, size_t views)
{
    std::uniform_real_distribution<float> jitter(-0.5f, 0.5f);
    std::vector<float> headings(views);
    for (size_t i = 0; i < views; i++)
        headings[i] = kTau * ((float)i + jitter(rng)) / (float)views;
    std::shuffle(headings.begin(), headings.end(), rng);
    return headings;
}

// Scattered survey stations: well-spaced random positions over the map, each
// at its own altitude, looking along its own compass heading at a ground point
// ahead. Unlike the arc and the circle no geometry ties neighbouring views
// together, so the cross-view collection pass finds little -- the mode trades
// per-spot depth for coverage of positions and angles.
static void layOutScatteredStations(Simulation &sim, size_t views)
{
    std::mt19937 rng(kAutoBuildSeed);   // deterministic, like the aim error
    // Per-axis: the grid spans +-cols/2 in x by +-rows/2 in z, NOT a
    // terrainSize square (that is max(cols, rows)); a non-square DEM would put
    // square-sized stations over empty space where nothing anchors. The
    // lookahead follows the short axis so the re-aim below stays on the grid.
    const glm::vec2 grid((float)sim.mesh.cols, (float)sim.mesh.rows);
    const glm::vec2 extent  = kScatterExtentFraction * grid;
    const glm::vec2 mapEdge = 0.5f * grid;
    const float lookahead = kScatterLookaheadFraction * std::min(grid.x, grid.y);
    std::uniform_real_distribution<float> altitude(kScatterAltitudeMin * sim.terrainSize,
                                                   kScatterAltitudeMax * sim.terrainSize);
    const std::vector<float> headings = spreadHeadings(rng, views);

    sim.waypoints.clear();
    sim.pathPoints.clear();
    for (size_t i = 0; i < views; i++) {
        const glm::vec2 spot = bestSpacedSpot(rng, sim.waypoints, extent);
        glm::vec2 target = spot + glm::vec2(std::cos(headings[i]),
                                            std::sin(headings[i])) * lookahead;
        // A look target off the map anchors nothing, so a station that would
        // stare outward looks inward across the centre instead (the circle's
        // past-centre trick). Always on the grid: shrinking toward the centre
        // stays inside the station's own box, and crossing it ends within
        // lookahead of the centre -- under half of either axis. Off-map
        // implies |spot| > 0, so the normalize is safe.
        if (std::abs(target.x) > mapEdge.x || std::abs(target.y) > mapEdge.y)
            target = spot - lookahead * glm::normalize(spot);
        sim.waypoints.push_back({ glm::vec3(spot.x, altitude(rng), spot.y),
                                  glm::vec3(target.x, 0.0f, target.y) });
        sim.pathPoints.push_back(sim.waypoints.back().position);
    }
    std::cout << "FEATURES: note -- scattered stations rarely revisit a spot"
                 " within SIFT's viewpoint tolerance; expect weaker cross-view"
                 " collection than the circle's" << std::endl;
}

static void layOutAutoPath(Simulation &sim, AutoPathMode mode, size_t views)
{
    switch (mode) {
        case AutoPathMode::Arc:       layOutArc(sim, views);               break;
        case AutoPathMode::Circle:    layOutSurveyCircle(sim, views);      break;
        case AutoPathMode::Scattered: layOutScatteredStations(sim, views); break;
    }
}

// The knobs every simulated view consumes, derived once per build.
struct AutoBuildPlan {
    Viewport vp;             // the canonical capture frame
    size_t   features;       // clicks per view
    float    idealSpacing;   // target map distance between anchors
    float    reachLimit;     // anchorable ground ends here
    float    step;           // terrain ray-march step
};

static AutoBuildPlan makeAutoBuildPlan(const Simulation &sim, const Viewport &vp,
                                       size_t views, size_t features)
{
    AutoBuildPlan plan;
    plan.vp       = vp;
    plan.features = features;
    // The spacing selection starts from: what would tile the whole terrain with
    // this many points. See selectSpacedAnchors for why it is a map distance.
    plan.idealSpacing =
        kMapSpacingFactor * sim.terrainSize / std::sqrt((float)(views * features));
    plan.reachLimit = kReachLimitFraction * sim.terrainSize;
    // Ray-march step: about a thousandth of the terrain -- fine enough to land
    // on the right hillside, coarse enough to stay cheap over a few thousand rays.
    plan.step = std::max(0.25f, sim.terrainSize / 1200.0f);
    return plan;
}

// One stop of the simulated build: capture the view, detect, raycast every
// keypoint, then "click" the spaced selection -- each chosen sight line's true
// depth, disturbed by aim error. A "human" whose error aimed behind the camera
// skips the suggestion, as pressing X would.
static void simulateViewClicks(FeatureDb &db, std::vector<glm::vec3> &taken,
                               const Simulation &sim, Renderer &renderer,
                               const Waypoint &waypoint, const AutoBuildPlan &plan,
                               std::mt19937 &rng, std::normal_distribution<float> &aim)
{
    Camera camera = sim.playerView.camera;   // throwaway; the player stays put
    camera.applyPose(waypoint);

    FramePixels frame = renderer.captureSceneFrameAt(plan.vp.width, plan.vp.height,
                                                     camera, sim.light());
    if (frame.rgb.empty())
        return;
    std::vector<cv::KeyPoint> kps;
    cv::Mat desc;
    detectAllFeatures(frame, kps, desc);
    if (kps.empty())
        return;

    const KeypointGround ground = raycastKeypointGround(sim, camera, kps, frame,
                                                        plan.vp.aspect(), plan.step,
                                                        plan.reachLimit);
    for (size_t i : selectSpacedAnchors(kps, ground.hits, taken, plan.features,
                                        plan.idealSpacing, frame.width, frame.height)) {
        const float judged = *ground.depths[i] + aim(rng);
        if (judged <= 0.0f)
            continue;
        db.descriptors.push_back(desc.row((int)i));
        db.anchors.push_back(camera.position + ground.directions[i] * judged);
        taken.push_back(*ground.hits[i]);
    }
}

// The build's closing words: what was built on which path, then where flying
// will recognise it best. captureHelp arrives as a parameter -- it is a
// protected member of the base class, out of a file-static function's reach.
static void reportAutoBuild(const Simulation &sim, const AutoBuildPlan &plan,
                            const AutoPathSpec &spec, size_t placed, size_t aimError,
                            const char *captureHelp)
{
    std::cout << "FEATURES: auto-built " << placed << " points from "
              << sim.waypoints.size() << " views on " << spec.name
              << " (map spacing ~" << (int)plan.idealSpacing
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
    m_db = std::make_unique<FeatureDb>();
    m_build.reset();
    m_captureWidth  = kAutoCaptureWidth;
    m_captureHeight = kAutoCaptureHeight;

    std::mt19937 rng(kAutoBuildSeed);
    std::normal_distribution<float> aim(0.0f, (float)aimError);

    const AutoBuildPlan plan = makeAutoBuildPlan(sim, captureViewport(sim),
                                                 views, features);
    std::vector<glm::vec3> taken;   // true positions of every anchor placed so far
    for (const Waypoint &waypoint : sim.waypoints)
        simulateViewClicks(*m_db, taken, sim, renderer, waypoint, plan, rng, aim);

    if (m_db->empty()) {
        std::cout << "FEATURES: auto-build anchored nothing -- every suggestion missed"
                     " the terrain" << std::endl;
        refreshPlaces();
        return;
    }

    const size_t placed = m_db->anchors.size();
    addOtherViewAppearances(sim, renderer);
    refreshPlaces();
    reportAutoBuild(sim, plan, spec, placed, aimError, kCaptureHelp);
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
            skipView(sim, renderer);   // the rest of this view, one press
        } else if (key == GLFW_KEY_X) {   // skip an unplaceable suggestion
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

// The debrief's measurement: how far the placed anchor sits from the point
// the suggestion's ray actually meets the terrain -- the answer the user was
// estimating. Measured at placement, reported only by finishBuild, stored
// nowhere the run phase can see. Negative marks a ray that missed the terrain
// within reach (nothing to compare against).
static float placementError(const Simulation &sim, const Camera &camera,
                            const glm::vec2 &ray, const glm::vec3 &anchor)
{
    const glm::vec3 direction = rayDirection(camera, ray);
    const float reach = sim.terrainSize * 1.5f;
    const float step  = std::max(0.5f, sim.terrainSize / 300.0f);
    const std::optional<float> depth =
        raycastTerrain(sim.mesh, camera.position, direction, reach, step);
    if (!depth)
        return -1.0f;
    return glm::distance(anchor, camera.position + direction * *depth);
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
    const glm::vec2 ray = fractionToRay(m_build->markers[m_build->active], camera.fov,
                                        captureViewport(sim).aspect());
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

    m_db->descriptors.push_back(m_build->descriptors.row((int)m_build->active));
    m_db->anchors.push_back(anchor);
    m_build->anchoredAt.push_back(m_build->active);   // so U can take it back
    m_build->placementErrors.push_back(placementError(sim, camera, ray, anchor));
    advance(sim, renderer);
}

// The four plausibility checks, one predicate each. All judge the ESTIMATE
// alone -- the true pose is never consulted -- and each reports its own
// refusal, so the console names which one fired.

// Within reach of the terrain at all: how far from the origin-centered map an
// estimate may sit, in terrain widths.
static bool withinTerrainReach(const Simulation &sim, const Waypoint &estimate)
{
    constexpr float kPlausibleDistanceWidths = 2.0f;

    const float distance = glm::length(estimate.position);
    const float limit    = kPlausibleDistanceWidths * sim.terrainSize;
    if (distance <= limit)
        return true;

    std::cout << "FEATURES: pose rejected as implausible -- it puts the camera "
              << (int)distance << " units from the terrain (nothing seeing this"
                 " terrain can be past " << (int)limit << ")" << std::endl;
    return false;
}

// Under the map is not a pose a camera can hold. heightAt only answers over
// the grid, so an estimate past the edge is left to the other checks.
static bool sitsAboveSurface(const Simulation &sim, const Waypoint &estimate)
{
    const std::optional<float> ground =
        sim.mesh.heightAt(estimate.position.x, estimate.position.z);
    if (!ground || estimate.position.y >= *ground + std::max(1.0f, sim.terrainSize / 100.0f))
        return true;

    std::cout << "FEATURES: pose rejected as implausible -- it puts the camera"
                 " under the terrain" << std::endl;
    return false;
}

// Near the terrain is not the same as looking at it. Probe the ray through the
// lower-third centre -- terrain lives in a frame's lower half, so even a
// pitched-up-but-valid view passes.
static bool looksAtTerrain(const Simulation &sim, const Camera &estimated, const Viewport &vp)
{
    const glm::vec3 probe = rayDirection(
        estimated, fractionToRay(glm::vec2(0.5f, 0.75f), estimated.fov, vp.aspect()));
    if (raycastTerrain(sim.mesh, estimated.position, probe, 3.0f * sim.terrainSize,
                       std::max(1.0f, sim.terrainSize / 200.0f)))
        return true;

    std::cout << "FEATURES: pose rejected as implausible -- a camera there, aimed"
                 " that way, would be looking at no terrain at all" << std::endl;
    return false;
}

// The coalition killer: the estimate must be able to SEE its own evidence.
// Each consensus anchor reprojects acceptably by construction, but a lookalike
// coalition puts the camera somewhere ridges hide the very anchors it claims.
// The allowance scales with the consensus -- anchor noise buries the odd
// anchor just under a slope, and refusing a 25-inlier pose over two such
// burials was measured to throw away good captures. One hidden in eight is
// placement noise; more is a coalition.
static bool seesItsOwnInliers(const Simulation &sim, const Camera &estimated,
                              const Viewport &vp,
                              const std::vector<Correspondence> &inliers)
{
    size_t hidden = 0;
    for (const Correspondence &inlier : inliers)
        if (!anchorVisibleFrom(sim.mesh, sim.terrainSize, estimated, vp, inlier.worldPos))
            hidden++;
    if (hidden <= std::max<size_t>(1, inliers.size() / 8))
        return true;

    std::cout << "FEATURES: pose rejected as implausible -- " << hidden << " of its "
              << inliers.size() << " agreeing anchors would be hidden behind"
                 " ridges from there" << std::endl;
    return false;
}

// Could a camera at this pose have taken the frame? A coalition of lookalike
// matches can clear every gate and still put the camera an impossible distance
// out, under the ground, aimed at empty sky, or somewhere ridges hide its own
// evidence -- one check each, and passing all four is what lets the consensus
// floor sit as low as it does.
static bool poseIsPlausible(const Simulation &sim, const Waypoint &estimate,
                            const Viewport &vp,
                            const std::vector<Correspondence> &inliers)
{
    Camera estimated = sim.playerView.camera;
    estimated.applyPose(estimate);

    return withinTerrainReach(sim, estimate)
        && sitsAboveSurface(sim, estimate)
        && looksAtTerrain(sim, estimated, vp)
        && seesItsOwnInliers(sim, estimated, vp, inliers);
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

    if (estimate && !poseIsPlausible(sim, *estimate, vp, inliers))
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
    if (sim.viewAids != ViewAids::Full || m_build->markers.empty())
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

    // Only the active suggestion -- one at a time keeps the anchoring
    // unambiguous. An ortho over the unit square maps the stored [0,1] fraction
    // straight to the viewport (the same screen matrix PICK uses).
    if (m_build->active < m_build->markers.size()) {
        const glm::mat4 screen = glm::ortho(0.0f, 1.0f, 1.0f, 0.0f);
        const glm::vec3 pos(m_build->markers[m_build->active], 0.0f);
        renderer.drawPoints({ pos }, { overlay::suggestionColor },
                            overlay::suggestionMarkerSize, screen);
    }
}
