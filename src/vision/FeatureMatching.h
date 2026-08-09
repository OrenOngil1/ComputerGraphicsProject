#pragma once

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <functional>
#include <optional>
#include <vector>

#include <glm/glm.hpp>
#include <opencv2/core.hpp>

#include "../core/Scene.h"    // FramePixels
#include "../core/Camera.h"   // Waypoint

// How a pass renders a view without knowing what a Renderer is: the caller
// binds one in. Keeps this layer free of render/, and its passes headless.
using FrameCapture = std::function<FramePixels(const Camera &, const Viewport &)>;

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
void detectSpreadFeatures(const FramePixels &frame, size_t maxCount,
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

// The spread-selection policy, obeyed by both selectors in their own spaces.
// A rim keypoint is a poor anchor: half its descriptor patch is off-frame.
constexpr float kRimMarginPx = 16.0f;

// Neither selector can always fill its quota at the spacing it would prefer, so
// both halve and retry -- a tighter spread beats a short set.
constexpr size_t kSpacingRetries = 3;

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

// The three descriptor bars, all measured on hand-built databases. The numbers
// and the alternatives that failed: docs/pose-estimation-modes.md ("Hardening
// the database").

// MATCH bar -- how close a frame descriptor must come to a stored row before a
// capture may claim it saw that place. Genuine re-sightings measured 250-310;
// distinct places come no nearer than ~337. Every capture prints its accepted
// distances and nearest misses -- read both against this before turning it.
constexpr float kMaxDescriptorDistance = 320.0f;

// COLLECTION bar -- the same question for the appearance pass, looser on
// purpose: geometry has already pinned WHERE, so this only rejects a DIFFERENT
// feature at that spot. At the match bar the pass starved -- same-place rows
// measured 106-480, and 26 of 33 places never grew past one row.
constexpr float kCollectResemblanceDistance = 400.0f;

// DUPLICATE bar -- how close a fresh suggestion must come before the build
// refuses to offer it. TIGHT, because it is the only bar the user cannot
// overrule: a false positive is terrain that can never be anchored at all. At
// the match bar, 45 pairs of DISTINCT places were withheld on a 64-place build.
// 150 is the bottom of the measured same-place band; leaks past it show up in
// the audit's confusable-pair count.
constexpr float kDuplicateSuggestionDistance = 150.0f;

// How far a hand-placed anchor may sit from the true point -- the appearance
// pass's error budget. The build debrief put a careful snap-assisted hand at
// median 6.4 units; the original guess of 3 collected almost nothing.
constexpr float kAnchorSlopUnits = 7.0f;

// That budget as a pixel radius at a given range. A fixed pixel radius is a
// VARIABLE world budget (~1 unit from a low view), which starved collection.
// Floored so SIFT's localisation slop fits; the cap only guards a place almost
// at the camera, since keeping keypoints apart is the claim step's job.
inline float projectionRadiusPx(float slopUnits, float focalPx, float range,
                                float frameHeight)
{
    const float px = slopUnits * focalPx / std::max(range, 1.0f);
    return std::clamp(px, 6.0f, std::max(0.05f * frameHeight, 6.0f));
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

// Run-phase: match the frame against the database and solve the surviving 2D-3D
// pairs with RANSAC PnP. nullopt when too few matches are confident or nothing
// reaches consensus.
//
// minInliers: how many matches must agree, clamped to [kMinConsensus,
// kMaxConsensus]; nullopt takes a sixth of what this frame matched (see .cpp).
// inliersOut: the winning consensus, for the caller's plausibility check.
// pinnedFloorNoted: the caller's once-per-pin latch for the small-floor
// warning, owned there so a new mode entry with a new pin warns again.
std::optional<Waypoint> estimatePoseFromFeatures(const FeatureDb &db,
                                                 const FramePixels &frame,
                                                 float fov, int viewportWidth,
                                                 int viewportHeight,
                                                 std::optional<size_t> minInliers = std::nullopt,
                                                 std::vector<Correspondence> *inliersOut = nullptr,
                                                 bool *pinnedFloorNoted = nullptr);
