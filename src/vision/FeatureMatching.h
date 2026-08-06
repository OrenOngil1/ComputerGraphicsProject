#pragma once

#include <cstddef>
#include <optional>
#include <vector>

#include <glm/glm.hpp>
#include <opencv2/core.hpp>

#include "../core/Scene.h"    // FramePixels
#include "../core/Camera.h"   // Waypoint

// The feature-matching mode's pre-phase product: SIFT descriptors, each anchored
// to a 3D terrain point the user hand-placed on the map. anchors[i] is the
// world-space point behind descriptor row i.
//
// A row is one APPEARANCE, not one place: the same point is stored once per
// recorded view it can be seen in, because on shading-driven terrain a feature's
// appearance moves with the viewpoint. So `anchors` repeats positions, and its
// size is the appearance count, not the number of points the user placed.
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

// Pre-phase suggestion step: detect SIFT on the frame and return up to maxCount
// strong keypoints, SPREAD ACROSS THE FRAME, with their aligned descriptor rows.
// keypoints/descriptors are cleared first; descriptors has one row per keypoint.
//
// Strength alone is not enough, because a human anchors these by hand. The
// strongest responses cluster -- several fire on one corner, a few pixels apart
// -- and that is bad twice over: the user cannot tell the dots apart on the map,
// and crowded points barely constrain a pose however accurately they are placed.
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

// Does `place` already own a stored appearance within `maxDistance` (L2) of
// `descriptor`? The single scan behind both of the build's descriptor
// questions, which differ only in where they set the bar: close enough to join
// the point, or close enough to BE a row a previous pass already stored.
bool hasAppearanceWithin(const FeatureDb &db, const glm::vec3 &place,
                         const cv::Mat &descriptor, double maxDistance);

// Does `descriptor` look like the point already anchored at `place`? The same
// question at the pipeline's quality bar (see kMaxDescriptorDistance), and the
// gate on collecting an appearance from a second view: geometry says the point
// should be at that pixel, this says the view shows it rather than a ridge in
// front of it.
bool resemblesAnchoredPoint(const FeatureDb &db, const glm::vec3 &place,
                            const cv::Mat &descriptor);

// The run-phase match step alone: each database anchor this frame appears to
// contain, paired with the pixel it was found at (as a [0,1] fraction).
//
// Separate from the solve because this is where the mode's least visible
// failure lives. The match runs database -> frame, which makes two things true
// by construction: there is at most one correspondence per anchor, and no
// anchor is claimed to be in two places at once. Asked the other way round --
// once per keypoint -- a 20-anchor database can answer "yes" 140 times, and
// PnP then builds its consensus out of contradictions. See the .cpp.
std::vector<Correspondence> matchFeaturesToDb(const FeatureDb &db, const FramePixels &frame);

// The consensus floor's legal range. Below five a pose has no independent
// witness -- RANSAC fits each candidate on a 4-point sample that always votes
// for itself -- and above 25 no single frame can be expected to reach it.
constexpr size_t kMinConsensus = 5;
constexpr size_t kMaxConsensus = 25;

// Run-phase: match the frame against the database and solve the surviving
// 2D-3D pairs with RANSAC PnP. nullopt when there are too few confident
// matches or no consensus pose.
//
// minInliers is how many matches must agree on the pose, clamped to
// [kMinConsensus, kMaxConsensus]; nullopt takes a quarter of what this frame
// matched -- the same demand at every database size, see the .cpp.
std::optional<Waypoint> estimatePoseFromFeatures(const FeatureDb &db,
                                                 const FramePixels &frame,
                                                 float fov, int viewportWidth,
                                                 int viewportHeight,
                                                 std::optional<size_t> minInliers = std::nullopt);
