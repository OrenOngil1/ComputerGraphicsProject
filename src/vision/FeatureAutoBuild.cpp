#include "FeatureAutoBuild.h"

#include <algorithm>
#include <cmath>
#include <optional>
#include <random>

#include <opencv2/core.hpp>

// Fixed, so two builds with the same parameters are the same database.
static constexpr unsigned int kAutoBuildSeed = 20260805u;

// Target map spacing as a fraction of what would tile the terrain with the
// requested points, and how far along a sight line terrain is still worth
// anchoring. The rim margin and retry budget are shared (FeatureMatching.h).
static constexpr float kMapSpacingFactor   = 0.8f;
static constexpr float kReachLimitFraction = 1.25f;

// One raycast per keypoint. Selection reads the hits; anchoring reads the ray
// and the depth. hits[i] is empty where keypoint i has no anchorable ground --
// the ray missed, or the surface is past the reach limit (grazing-angle mush
// whose keypoints are not repeatable).
struct KeypointGround {
    std::vector<glm::vec3>                directions;
    std::vector<std::optional<float>>     depths;
    std::vector<std::optional<glm::vec3>> hits;
};

static KeypointGround raycastKeypointGround(const Mesh &mesh, float terrainSize,
                                            const Camera &camera,
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
        ground.depths[i] = raycastTerrain(mesh, camera.position, ground.directions[i],
                                          3.0f * terrainSize, step);
        if (ground.depths[i] && *ground.depths[i] <= reachLimit)
            ground.hits[i] = camera.position + ground.directions[i] * *ground.depths[i];
    }
    return ground;
}

// Up to `count` indices into `kps`, strongest first, each taken only when its
// terrain hit clears `spacing` from every hit already taken -- this view's and
// `taken`, every earlier view's. Spacing is judged on the MAP, not in the
// frame: perspective piles pixel-uniform picks onto the nearest ground.
static std::vector<size_t> selectSpacedAnchors(
        const std::vector<cv::KeyPoint> &kps,
        const std::vector<std::optional<glm::vec3>> &hits,
        const std::vector<glm::vec3> &taken,
        size_t count, float startSpacing, int frameWidth, int frameHeight)
{
    const std::vector<size_t> order = rankByResponse(kps);

    std::vector<size_t> picked;
    float spacing = startSpacing;
    for (size_t attempt = 0; attempt < kSpacingRetries && picked.size() < count;
         attempt++, spacing *= 0.5f) {
        picked.clear();
        for (size_t i : order) {
            if (!hits[i])
                continue;

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

// The knobs every simulated view consumes, derived once per build.
struct AutoBuildPlan {
    Viewport vp;             // the canonical capture frame
    size_t   features;       // clicks per view
    float    idealSpacing;   // target map distance between anchors
    float    reachLimit;     // anchorable ground ends here
    float    step;           // terrain ray-march step
};

static AutoBuildPlan makeAutoBuildPlan(float terrainSize, const Viewport &vp,
                                       size_t views, size_t features)
{
    AutoBuildPlan plan;
    plan.vp       = vp;
    plan.features = features;
    // Start from what would tile the whole terrain with this many points.
    plan.idealSpacing =
        kMapSpacingFactor * terrainSize / std::sqrt((float)(views * features));
    plan.reachLimit = kReachLimitFraction * terrainSize;
    // About a thousandth of the terrain: fine enough to land on the right
    // hillside, coarse enough to stay cheap over a few thousand rays.
    plan.step = std::max(0.25f, terrainSize / 1200.0f);
    return plan;
}

// One stop: capture, detect, raycast every keypoint, then "click" the spaced
// selection at each sight line's true depth plus aim error. A "human" whose
// error aimed behind the camera skips the suggestion, as pressing X would.
static void simulateViewClicks(FeatureDb &db, std::vector<glm::vec3> &taken,
                               const Mesh &mesh, float terrainSize,
                               const Camera &prototype, const Waypoint &waypoint,
                               const AutoBuildPlan &plan, const FrameCapture &capture,
                               std::mt19937 &rng, std::normal_distribution<float> &aim)
{
    Camera camera = prototype;   // throwaway; the caller's camera stays put
    camera.applyPose(waypoint);

    FramePixels frame = capture(camera, plan.vp);
    if (frame.rgb.empty())
        return;
    std::vector<cv::KeyPoint> kps;
    cv::Mat desc;
    detectAllFeatures(frame, kps, desc);
    if (kps.empty())
        return;

    const KeypointGround ground = raycastKeypointGround(mesh, terrainSize, camera, kps,
                                                        frame, plan.vp.aspect(), plan.step,
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

AutoBuildResult autoBuildDatabase(const Mesh &mesh, float terrainSize,
                                  const std::vector<Waypoint> &views,
                                  const Camera &prototype, const Viewport &vp,
                                  const AutoBuildSettings &settings,
                                  const FrameCapture &capture)
{
    std::mt19937 rng(kAutoBuildSeed);
    std::normal_distribution<float> aim(0.0f, (float)settings.aimErrorUnits);

    const AutoBuildPlan plan = makeAutoBuildPlan(terrainSize, vp, views.size(),
                                                 settings.features);

    AutoBuildResult result;
    result.mapSpacing = plan.idealSpacing;
    std::vector<glm::vec3> taken;   // true positions of every anchor placed so far
    for (const Waypoint &waypoint : views)
        simulateViewClicks(result.db, taken, mesh, terrainSize, prototype, waypoint,
                           plan, capture, rng, aim);
    return result;
}
