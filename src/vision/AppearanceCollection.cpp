#include "AppearanceCollection.h"

#include <algorithm>
#include <cmath>
#include <iostream>

#include <opencv2/core.hpp>

#include "AnchorVisibility.h"

// Below this, two rows are the same appearance recomputed rather than a similar
// one -- a recomputed row is bit-identical, so this only absorbs float noise.
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

// Where a pass's (place, view) pairs went, one counter per gate. Printed for
// the LAST fixpoint pass, so a starved collection names its starver.
struct CollectTally {
    size_t pairs = 0, hidden = 0, noKeypoint = 0, unlike = 0,
           claimedAway = 0, alreadyStored = 0, added = 0;
    std::vector<double> unlikeNearest;   // nearest-row distances of `unlike`
};

// One place's bid for one keypoint; the miss orders the bids, since the nearest
// projection has the best claim on a keypoint two places both reach.
struct AppearanceBid { float missPx; size_t place; int keypoint; };

// Phase one, geometry: which places this view can see, and which keypoint each
// lands on. The search radius is a constant WORLD budget at the place's range
// (projectionRadiusPx), not a fixed pixel count.
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

        // One distance, two questions: too far from every row the place owns is
        // a different feature (kCollectResemblanceDistance); near-zero is the
        // same row recomputed, which keeps a reload topping up rather than
        // duplicating.
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

// One view's contribution: geometry first, descriptors only over what geometry
// already vouched for (docs/pose-estimation-modes.md, "Hardening the database").
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

// Render one recorded view offscreen and let it contribute. Throwaway Camera,
// so the caller's is left where it stood.
static void collectAppearancesFromWaypoint(FeatureDb &db, const std::vector<glm::vec3> &places,
                                           const Mesh &mesh, float terrainSize,
                                           const Camera &prototype, const Waypoint &waypoint,
                                           const Viewport &vp, const FrameCapture &capture,
                                           CollectTally &tally)
{
    Camera camera = prototype;
    camera.applyPose(waypoint);

    const FramePixels frame = capture(camera, vp);
    if (frame.rgb.empty())
        return;
    std::vector<cv::KeyPoint> keypoints;
    cv::Mat descriptors;
    detectAllFeatures(frame, keypoints, descriptors);
    if (keypoints.empty())
        return;

    collectAppearancesInView(db, places, mesh, terrainSize, camera, vp,
                             keypoints, descriptors, tally);
}

// Read `unlike` misses against the collection bar the way capture losses are
// read against the match bar.
static void reportTally(CollectTally &tally)
{
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

void collectAppearances(FeatureDb &db, const Mesh &mesh, float terrainSize,
                        const std::vector<Waypoint> &views, const Camera &prototype,
                        const Viewport &vp, const FrameCapture &capture)
{
    if (db.empty())
        return;

    const std::vector<glm::vec3> places = db.places();
    const size_t placed = db.anchors.size();

    // RECORDED views only, per the course brief -- nothing synthesized between.
    // Up to three passes: resemblance is judged against the rows a place owns SO
    // FAR, so a place anchored late reaches early views only through rows a
    // later pass adds. Idempotent rows make repeats safe.
    CollectTally tally;
    for (int pass = 0; pass < 3; pass++) {
        tally = CollectTally{};
        const size_t before = db.anchors.size();
        for (const Waypoint &waypoint : views)
            collectAppearancesFromWaypoint(db, places, mesh, terrainSize, prototype,
                                           waypoint, vp, capture, tally);
        if (db.anchors.size() == before)
            break;
    }

    std::cout << "FEATURES: " << places.size() << " placed points seen "
              << db.anchors.size() << " times across the " << views.size()
              << " recorded views (" << (db.anchors.size() - placed)
              << " appearances added by this pass)" << std::endl;
    reportTally(tally);
}
