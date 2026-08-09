#pragma once

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <optional>
#include <vector>

#include <glm/glm.hpp>
#include <opencv2/core.hpp>

#include "../core/Scene.h"    // FramePixels
#include "../core/Camera.h"   // Waypoint

// The pre-phase product: SIFT descriptors, each anchored to a 3D terrain point
// the user hand-placed on the map; anchors[i] is the world point behind
// descriptor row i. A row is one APPEARANCE, not one place: a point is stored
// once per recorded view that sees it (its appearance moves with viewpoint on
// shading-driven terrain), so `anchors` repeats positions and its size is the
// appearance count, not the number of points placed.
struct FeatureDb {
    cv::Mat descriptors;              // CV_32F, one 128-float SIFT descriptor per row
    std::vector<glm::vec3> anchors;   // centered world space, like Correspondence::worldPos

    bool empty() const { return anchors.empty(); }

    // The distinct places behind those appearances, in first-seen order --
    // what the user actually placed, for counting and for drawing on the map.
    std::vector<glm::vec3> places() const
    {
        std::vector<glm::vec3> distinct;
        for (const glm::vec3 &anchor : anchors) {
            bool seen = false;
            for (const glm::vec3 &place : distinct)
                seen = seen || place == anchor;   // exact: appearances copy one vec3
            if (!seen)
                distinct.push_back(anchor);
        }
        return distinct;
    }
};

// Pre-phase suggestion step: up to maxCount strong SIFT keypoints SPREAD
// ACROSS THE FRAME, with aligned descriptor rows (both outputs cleared first).
// Spread, not merely strong: top responses cluster a few pixels apart, where
// the user cannot tell the dots apart on the map and crowded points barely
// constrain a pose however accurately they are placed.
void detectSpreadFeatures(const FramePixels &frame, int maxCount,
                          std::vector<cv::KeyPoint> &keypoints, cv::Mat &descriptors);

// Every SIFT feature in a frame -- the full population the run phase matches
// against, and the same detection the suggestions are drawn from, so pre-phase
// and run-phase descriptors are always comparable. Exposed because the build
// collects an anchored point's other appearances from these keypoints.
void detectAllFeatures(const FramePixels &frame,
                       std::vector<cv::KeyPoint> &keypoints, cv::Mat &descriptors);

// Keypoint indices ranked by SIFT response (contrast of the scale-space
// extremum), strongest first: the most repeatable, and the ones a user would
// naturally single out. Shared so the suggestion spread (frame space) and the
// auto-build's anchor selection (map space) walk the same ranking.
std::vector<size_t> rankByResponse(const std::vector<cv::KeyPoint> &keypoints);

// How near (L2) `descriptor` comes to the closest appearance row `place`
// owns; a large sentinel if it owns none. The one scan behind every "is this
// that place?" question -- the bars below are where each asker draws the line.
double nearestAppearanceDistance(const FeatureDb &db, const glm::vec3 &place,
                                 const cv::Mat &descriptor);

// nearestAppearanceDistance against a caller-chosen bar.
bool hasAppearanceWithin(const FeatureDb &db, const glm::vec3 &place,
                         const cv::Mat &descriptor, double maxDistance);

// Is `descriptor` a re-detection of a point already anchored? The build's
// duplicate guard: the same feature anchored twice becomes two identities the
// matcher cannot tell apart. Deliberately NOT the match bar -- see
// kDuplicateSuggestionDistance.
bool resemblesAnyAnchoredPoint(const FeatureDb &db, const cv::Mat &descriptor);

// The two descriptor bars. Both measured on hand-built databases; the numbers
// and their failed alternatives are in docs/pose-estimation-modes.md
// ("Hardening the database").
//
// MATCH bar -- how close a frame descriptor must come to a stored row before
// a capture may claim it saw that place. Genuine cross-view re-sightings
// measured 250-310, and an earlier bar at 250 refused the database's own
// cross-view matches; 320 admits the measured band while staying under the
// lookalike cloud (different places' rows come no nearer than ~337 on a
// 33-place build, two audited outliers aside). Every capture prints accepted
// distances and nearest misses -- read both against this number before
// turning it.
constexpr float kMaxDescriptorDistance = 320.0f;

// COLLECTION bar -- how close a keypoint must come to a place's stored rows
// for the appearance pass to adopt it as a new row. Deliberately looser than
// the match bar: at collection time geometry has already pinned WHERE (the
// occlusion ray, the world-budget radius, one keypoint per place), so this
// check only rejects a DIFFERENT feature at that spot, and a wrong corner
// inside the radius sits within the slop budget anyway. Reusing the match bar
// here starved the pass -- same-place cross-view rows measured 106-480
// (median 250), so half the genuine re-sightings were refused and 26 of 33
// places never grew past one row.
constexpr float kCollectResemblanceDistance = 400.0f;

// DUPLICATE bar -- how close a fresh suggestion must come to an existing row
// before the build refuses to offer it for anchoring. This one must be TIGHT,
// and for the opposite reason the others are loose: it is the only bar whose
// verdict the user cannot overrule, so every false positive is a terrain
// feature that can never be anchored at all. The match bar is far too wide for
// it -- reportDbAudit measured 45 pairs of genuinely DISTINCT places whose
// rows sit under 320 on a 64-place build, so a guard at the match bar
// withholds real places, and the more the database grows the more it
// withholds. 150 is the bottom of the measured same-place band (106-480), so
// it still catches the near-identical re-detection this guard exists for --
// the same feature suggested again from a nearby view -- while staying far
// under anything two distinct places have ever measured. Duplicates that slip
// past it are visible afterwards in the audit's confusable-pair count.
constexpr float kDuplicateSuggestionDistance = 150.0f;

// How far a carefully hand-placed anchor may sit from the true point, in world
// units -- the appearance pass's error budget. Calibrated by measurement: the
// build debrief put a careful snap-assisted hand at median 6.4 units, and the
// original guess of 3 -- half that -- left the pass finding almost nothing to
// collect. Recalibrate from the debrief if placement technique changes.
constexpr float kAnchorSlopUnits = 7.0f;

// That budget as a search radius in pixels at a given range: a fixed pixel
// radius is a VARIABLE world budget (~1 unit from a low view), which starved
// collection for hand-built databases. Floored so SIFT's localisation slop
// always fits; capped only against degenerate cases (a place almost at the
// camera), because keeping keypoints apart is the claim step's job, not the
// radius's -- an earlier 2.5% cap silently re-clamped the world budget below
// the measured hand error at typical ranges and made the slop constant inert.
inline float projectionRadiusPx(float slopUnits, float focalPx, float range,
                                float frameHeight)
{
    const float px = slopUnits * focalPx / std::max(range, 1.0f);
    return std::clamp(px, 6.0f, 0.05f * frameHeight);
}

// Console hygiene report for a freshly built or loaded database: appearance
// coverage per place, and how many place PAIRS sit under the descriptor bar --
// identities the matcher cannot separate, each a built-in outlier source.
void reportDbAudit(const FeatureDb &db);

// The run-phase match step alone: each database anchor this frame appears to
// contain, paired with the pixel it was found at (a [0,1] fraction). Matched
// database -> frame, which guarantees at most one correspondence per anchor
// and no anchor claimed at two places; asked the other way round -- once per
// keypoint -- a 20-anchor database can answer "yes" 140 times, and PnP builds
// its consensus out of contradictions. See the .cpp.
std::vector<Correspondence> matchFeaturesToDb(const FeatureDb &db, const FramePixels &frame);

// The consensus floor's legal range. RANSAC fits each candidate on a 4-point
// sample that always votes for itself, so witnesses start at five -- and one
// witness measured as not enough: across two hand-built flights, every
// accepted-but-wrong pose carried exactly 5 inliers (135-702 units off),
// while every pose at 6+ was either exact or caught by the plausibility
// checks. Six -- two independent witnesses -- is where the confident garbage
// stopped. Above 25 no single frame can be expected to reach the floor.
constexpr size_t kMinConsensus = 6;
constexpr size_t kMaxConsensus = 25;

// Run-phase: match the frame against the database and solve the surviving
// 2D-3D pairs with RANSAC PnP. nullopt when there are too few confident
// matches or no consensus pose.
//
// minInliers is how many matches must agree on the pose, clamped to
// [kMinConsensus, kMaxConsensus]; nullopt takes a sixth of what this frame
// matched -- the same demand at every database size, see the .cpp.
//
// inliersOut, when given, receives the consensus correspondences of a
// successful solve -- the caller's plausibility check asks whether the
// estimate could actually see them.
//
// pinnedFloorNoted is the caller's once-per-pin latch for the small-pinned-
// floor warning: raised the first time it fires. Owned by the caller rather
// than a static here so a new mode entry with a new pin warns again.
std::optional<Waypoint> estimatePoseFromFeatures(const FeatureDb &db,
                                                 const FramePixels &frame,
                                                 float fov, int viewportWidth,
                                                 int viewportHeight,
                                                 std::optional<size_t> minInliers = std::nullopt,
                                                 std::vector<Correspondence> *inliersOut = nullptr,
                                                 bool *pinnedFloorNoted = nullptr);
