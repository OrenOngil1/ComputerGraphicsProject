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
              << "U undoes the last one.\n"
              << "               The map shows the view's cyan cone and a red line from "
              << "the camera: the point you are placing lies somewhere ALONG that line, "
              << "and your click is snapped onto it -- so only how far along it you "
              << "click matters. Orbit to judge that. V hides the aids.\n"
              << "               Global map: scroll = zoom, middle-drag = pan, "
              << "right-drag = rotate.\n"
              << "               Then " << kCaptureHelp
              << ". Ctrl+S saves the database, Ctrl+O loads it back. Ctrl+G auto-builds"
              << " and saves one (arc / high circle / scattered path, simulated aim"
              << " error) when you just need a database to test against." << std::endl;
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

        if (kps.empty()) {
            std::cout << "FEATURES: view " << (m_build->waypoint + 1)
                      << " has no features -- skipping." << std::endl;
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

// One view's contribution: project each place through the known pose, and store
// the descriptor of the keypoint found there -- if one is close enough AND still
// resembles the anchored point (otherwise this view is seeing something else
// there, a ridge in front of it most often). `radius` is how far the projection
// may miss its keypoint: big enough for SIFT's localisation slop and a
// slightly-off anchor, small enough that it cannot reach a different feature.
static void collectAppearancesInView(FeatureDb &db, const std::vector<glm::vec3> &places,
                                     const Camera &camera, const Viewport &vp,
                                     const std::vector<cv::KeyPoint> &keypoints,
                                     const cv::Mat &descriptors, float radius)
{
    for (const glm::vec3 &place : places) {
        if (!isInFrame(camera, vp, place))
            continue;

        const int nearest = nearestKeypointTo(rasterize(camera, vp, place),
                                              keypoints, radius);
        if (nearest < 0)
            continue;
        if (!resemblesAnchoredPoint(db, place, descriptors.row(nearest)))
            continue;

        // Idempotence: a load re-runs this pass, and the same view of the same
        // place recomputes the same descriptor. Skipping rows the place already
        // owns tops a database up instead of duplicating it.
        if (hasAppearanceWithin(db, place, descriptors.row(nearest),
                                kIdenticalRowDistance))
            continue;

        db.descriptors.push_back(descriptors.row(nearest));
        db.anchors.push_back(place);
    }
}

// Nothing is invented here: the recorded poses are exact and the anchor is the
// user's own 3D point, so where it falls in another view is plain projection --
// no mesh is read, no 3D is chosen by machine. The pass is self-limiting, which
// is what keeps it that way: an appearance counts only if a keypoint sits near
// the projection AND still resembles the anchored descriptor, so a misjudged
// depth gains nothing instead of poisoning the database.
// (docs/pose-estimation-modes.md, "Mode D")
void FeatureMatchState::addOtherViewAppearances(Simulation &sim, Renderer &renderer)
{
    if (!m_db || m_db->empty())
        return;

    const std::vector<glm::vec3> places = m_db->places();
    const size_t placed = m_db->anchors.size();
    const Viewport vp = captureViewport(sim);

    // The RECORDED views only, per the course brief: the database describes
    // what was actually seen at the waypoints, with nothing synthesized between.
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

        collectAppearancesInView(*m_db, places, camera, vp, keypoints, descriptors,
                                 std::max(6.0f, 0.01f * (float)frame.height));
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
    // so restoring them is what makes Ctrl+B mean anything after a load. The
    // path between them was only ever drawn, never saved, so it is dropped.
    if (!waypoints.empty()) {
        sim.waypoints = std::move(waypoints);
        sim.pathPoints.clear();
    }

    // Top up whatever appearances the file lacks -- an older one may hold only
    // the hand-placed rows. Collection skips rows a place already owns, so this
    // upgrades any vintage of file without bloating a current one.
    if (!m_db->empty())
        addOtherViewAppearances(sim, renderer);

    refreshPlaces();
}

// ── The automated stand-in for the manual build (Ctrl+G) ──────
//
// The human is SIMULATED, not skipped. Ray-snapping already reduces a real
// click to one number -- the depth along the suggestion's sight line -- so this
// reads the true ray-terrain hit and disturbs that depth with Gaussian aim
// error. Reading the terrain plays the user's EYES, never the estimator's: the
// run phase still sees nothing but (descriptor, 3D) pairs and cannot tell the
// two builds apart. A ray that misses is skipped, as a person would press X.

// Aim error, in units along the sight line -- the one dimension a human
// actually supplies. Sized to what hand-built databases achieve.
static constexpr float kSimulatedDepthErrorUnits = 4.0f;

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
// terrain hit is at least `spacing` from every hit already taken -- this view's
// and `taken`, every earlier view's -- so later views are pushed onto unclaimed
// ground and the circle's union covers the map.
//
// Spacing is judged on the MAP, not in the frame: perspective squeezes most of
// the terrain into a frame's upper half, so pixel-uniform picks pile onto the
// ground nearest the camera.
//
// `hits[i]` is empty where keypoint i has no anchorable ground under it.
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

// A full circle flown HIGH, each stop aimed at a ground point PAST the centre.
// Height is what makes a circle legal at all: a low ring's azimuth steps blow
// SIFT's viewpoint budget (see layOutArc), but from high up a step is mostly
// an IN-PLANE rotation of the same picture, which SIFT absorbs by design --
// the harmful out-of-plane residue, 2*asin(sin(step/2)*sin(tilt)), is ~18
// degrees at 12 stops and this tilt, shrinking with every extra stop (hence
// the 12-view default and the warning below it). The past-centre aim stretches
// each stop's footprint from its own nadir across the middle to the far edge,
// so the strips fan around the compass and their union covers essentially the
// whole map. (docs/pose-estimation-modes.md, "Ctrl+G")
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
                                float extent)
{
    std::uniform_real_distribution<float> coord(-extent, extent);
    glm::vec2 best(0.0f);
    float bestScore = -1.0f;
    for (int c = 0; c < kScatterCandidates; c++) {
        const glm::vec2 candidate(coord(rng), coord(rng));
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
    const float extent    = kScatterExtentFraction    * sim.terrainSize;
    const float lookahead = kScatterLookaheadFraction * sim.terrainSize;
    const float mapEdge   = 0.5f * sim.terrainSize;
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
        // past-centre trick). ||spot| - lookahead| < half the map, so the
        // re-aim always lands on it; off-map implies |spot| > 0, so the
        // normalize is safe.
        if (std::abs(target.x) > mapEdge || std::abs(target.y) > mapEdge)
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
                            const AutoPathSpec &spec, size_t placed,
                            const char *captureHelp)
{
    std::cout << "FEATURES: auto-built " << placed << " points from "
              << sim.waypoints.size() << " views on " << spec.name
              << " (map spacing ~" << (int)plan.idealSpacing
              << " units; simulated aim error sigma " << kSimulatedDepthErrorUnits
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
    std::normal_distribution<float> aim(0.0f, kSimulatedDepthErrorUnits);

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
    reportAutoBuild(sim, plan, spec, placed, kCaptureHelp);
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

// Can the player's current view actually use this anchor? Frustum is not
// enough: SIFT matches what was DRAWN, and a feature behind a ridge was not --
// counting it would promise matches the capture cannot deliver.
static bool anchorVisible(const Simulation &sim, const glm::vec3 &anchor)
{
    const View &view = sim.playerView;
    if (!isInFrame(view.camera, view.viewport, anchor))
        return false;

    const glm::vec3 toAnchor = anchor - view.camera.position;
    const float distance = glm::length(toAnchor);

    // Stop the march short of the anchor itself: a hit at its own distance is
    // the ground it belongs to, not something standing in front of it. The
    // margin is generous because the snap can leave an anchor slightly under
    // the surface. One buried far deeper does read as hidden -- honestly so.
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

    // Only on a change, and no more than twice a second: the count moves
    // continuously while flying, and a line per frame would bury every other
    // message in the console.
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
    // user's 3D pick -- the one step that would otherwise be automatic.
    //
    // The click is snapped onto the suggestion's viewing ray first. That ray is
    // exact (a recorded pose, an exact keypoint pixel), so the true point lies
    // along it and any sideways offset is pure aim error; what the user supplies
    // is the DEPTH, the one thing a single image cannot. Dropping the sideways
    // part matters: an error along the ray reprojects onto the same pixel, while
    // a sideways slip of the same size lands tens of pixels off and gets a
    // perfectly good correspondence voted out as an outlier.
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

// Could a camera at this pose have taken the frame? Judged on the ESTIMATE
// alone -- the true pose is never consulted. A coalition of lookalike matches
// can clear every gate and still put the camera an impossible distance out or
// aimed at empty sky, which is what lets the consensus floor sit at five.
// Refusals are reported on the console.
static bool poseIsPlausible(const Simulation &sim, const Waypoint &estimate, float aspect)
{
    // How far from the origin-centered terrain an estimate may sit, in terrain widths.
    constexpr float kPlausibleDistanceWidths = 2.0f;

    const float distance = glm::length(estimate.position);
    const float limit    = kPlausibleDistanceWidths * sim.terrainSize;
    if (distance > limit) {
        std::cout << "FEATURES: pose rejected as implausible -- it puts the camera "
                  << (int)distance << " units from the terrain (nothing seeing this"
                     " terrain can be past " << (int)limit << ")" << std::endl;
        return false;
    }

    // Near it is not the same as looking at it. Probe the ray through the
    // lower-third centre -- terrain lives in a frame's lower half, so even a
    // pitched-up-but-valid view passes.
    Camera estimated = sim.playerView.camera;
    estimated.applyPose(estimate);
    const glm::vec3 probe = rayDirection(
        estimated, fractionToRay(glm::vec2(0.5f, 0.75f), estimated.fov, aspect));
    if (!raycastTerrain(sim.mesh, estimated.position, probe, 3.0f * sim.terrainSize,
                        std::max(1.0f, sim.terrainSize / 200.0f))) {
        std::cout << "FEATURES: pose rejected as implausible -- a camera there, aimed"
                     " that way, would be looking at no terrain at all" << std::endl;
        return false;
    }
    return true;
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
    std::optional<Waypoint> estimate =
        estimatePoseFromFeatures(*m_db, frame, sim.playerView.camera.fov,
                                 vp.width, vp.height, m_minInliers);

    if (estimate && !poseIsPlausible(sim, *estimate, vp.aspect()))
        return std::nullopt;
    return estimate;
}

// The active suggestion's sight line is drawn in the same red as its marker in
// the player view, so the dot to place and the line to place it on read as one
// object; earlier anchors in this view get a dim line each, which doubles as a
// check -- a green anchor off its own line was misplaced, and U takes it back.
//
// The lines stop at a fixed reach rather than at the terrain: intersecting one
// with the surface would BE the correspondence the user is here to supply.
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
