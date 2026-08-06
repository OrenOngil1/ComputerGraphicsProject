#include "FeatureMatching.h"

#include <algorithm>
#include <cmath>
#include <iostream>
#include <numeric>

#include <opencv2/features2d.hpp>
#include <opencv2/imgproc.hpp>

#include "Pnp.h"   // computeCameraPoseRansac

// The detector works on intensity; collapse the captured RGB to grayscale. The
// wrap constructor shares frame's bytes and cvtColor writes a fresh Mat, so the
// frame is never modified -- the const_cast is only because cv::Mat's wrap
// constructor has no const overload.
static cv::Mat toGray(const FramePixels &frame)
{
    cv::Mat rgb(frame.height, frame.width, CV_8UC3,
                const_cast<unsigned char *>(frame.rgb.data()));
    cv::Mat gray;
    cv::cvtColor(rgb, gray, cv::COLOR_RGB2GRAY);
    return gray;
}

// The one detection entry for every phase: matching compares pre-phase and
// run-phase descriptors, so routing every detection through here makes their
// being computed identically structural rather than a convention.
//
// SIFT, not ORB, by measurement. This terrain has no texture -- every feature
// is a shading gradient -- and ORB's binary brightness comparisons stop
// separating "the same place again" from "a lookalike ridge" on that imagery,
// at any threshold. (Numbers: docs/pose-estimation-modes.md, "Run-phase (B)")
void detectAllFeatures(const FramePixels &frame,
                       std::vector<cv::KeyPoint> &keypoints, cv::Mat &descriptors)
{
    static const cv::Ptr<cv::SIFT> sift = cv::SIFT::create(1000);
    sift->detectAndCompute(toGray(frame), cv::noArray(), keypoints, descriptors);
}

// Unrelated SIFT pairs sit around 400-500 apart (L2); genuine re-sightings
// under a moderate viewpoint change land well under 300. The pipeline's only
// absolute quality bar -- the cross-check is relative -- and the same bar
// resemblesAnchoredPoint uses, so what the database stores and what it accepts
// at match time are one standard. Every capture prints the accepted distances;
// read them against this number before turning it.
static constexpr float kMaxDescriptorDistance = 250.0f;

void detectSpreadFeatures(const FramePixels &frame, int maxCount,
                          std::vector<cv::KeyPoint> &keypoints, cv::Mat &descriptors)
{
    std::vector<cv::KeyPoint> allKps;
    cv::Mat allDesc;
    detectAllFeatures(frame, allKps, allDesc);

    keypoints.clear();
    descriptors.release();
    if (allKps.empty() || maxCount <= 0)
        return;

    // Rank keypoint INDICES by SIFT response (contrast of the scale-space
    // extremum) -- iota fills 0..N-1, the sort puts the strongest first. Those
    // are the most repeatable, and the ones a user would single out anyway.
    std::vector<size_t> order(allKps.size());
    std::iota(order.begin(), order.end(), size_t(0));
    std::sort(order.begin(), order.end(), [&](size_t a, size_t b) {
        return allKps[a].response > allKps[b].response;
    });

    // A rim keypoint is a poor anchor: half its surroundings are off-screen, so
    // its 3D spot is hard to recognise on the map, and it drops out of view
    // under the smallest camera move.
    const float margin = 16.0f;

    // What maxCount points would have if they tiled the frame evenly, pulled in
    // a little so a merely well-spread set still fills the quota. A keypoint is
    // then taken only when it is that far from everything already taken.
    float radius = 0.8f * std::sqrt((float)frame.width * (float)frame.height /
                                    (float)maxCount);

    // A cramped or feature-poor frame may not have maxCount points that far
    // apart. Relax and retry rather than return a short set: a tighter spread
    // still beats the cluster the ranking alone would give.
    std::vector<size_t> chosen;
    for (int attempt = 0; attempt < 3 && (int)chosen.size() < maxCount; attempt++, radius *= 0.5f) {
        chosen.clear();
        for (size_t i : order) {
            const cv::Point2f &pt = allKps[i].pt;
            if (pt.x < margin || pt.y < margin ||
                pt.x > (float)frame.width - margin || pt.y > (float)frame.height - margin)
                continue;

            const bool crowded = std::any_of(chosen.begin(), chosen.end(),
                                             [&](size_t j) {
                                                 return cv::norm(allKps[j].pt - pt) < radius;
                                             });
            if (crowded)
                continue;

            chosen.push_back(i);
            if ((int)chosen.size() == maxCount)
                break;
        }
    }

    keypoints.reserve(chosen.size());
    for (size_t i : chosen) {
        keypoints.push_back(allKps[i]);
        descriptors.push_back(allDesc.row((int)i));   // copies the row
    }
}

bool hasAppearanceWithin(const FeatureDb &db, const glm::vec3 &place,
                         const cv::Mat &descriptor, double maxDistance)
{
    for (size_t i = 0; i < db.anchors.size(); i++) {
        if (db.anchors[i] != place)
            continue;
        if (cv::norm(db.descriptors.row((int)i), descriptor, cv::NORM_L2) <= maxDistance)
            return true;
    }
    return false;
}

bool resemblesAnchoredPoint(const FeatureDb &db, const glm::vec3 &place,
                            const cv::Mat &descriptor)
{
    return hasAppearanceWithin(db, place, descriptor, kMaxDescriptorDistance);
}

std::vector<Correspondence> matchFeaturesToDb(const FeatureDb &db, const FramePixels &frame)
{
    std::vector<Correspondence> correspondences;
    if (db.empty())
        return correspondences;

    std::vector<cv::KeyPoint> keypoints;
    cv::Mat descriptors;
    detectAllFeatures(frame, keypoints, descriptors);
    if (descriptors.empty()) {
        std::cout << "FEATURES: no keypoints in the current view" << std::endl;
        return correspondences;
    }

    // DATABASE -> FRAME, cross-checked; the header explains why that direction.
    //
    // Cross-checking rather than Lowe's ratio test: the ratio test only passes
    // where a feature is locally unique, and on shading-driven terrain every
    // ridge shoulder resembles every other, so it discarded most of the anchors
    // genuinely in frame. Mutual agreement leaves outlier rejection to RANSAC.
    cv::BFMatcher matcher(cv::NORM_L2, /*crossCheck=*/true);
    std::vector<cv::DMatch> matches;
    matcher.match(db.descriptors, descriptors, matches);

    // One correspondence per PLACE -- the closest match wins.
    std::vector<float> bestDistance;   // parallel to correspondences
    for (const cv::DMatch &match : matches) {
        // A cross-checked match is still only a nearest neighbour.
        if (match.distance > kMaxDescriptorDistance)
            continue;

        const glm::vec3 &place = db.anchors[match.queryIdx];
        const cv::Point2f &pt = keypoints[match.trainIdx].pt;
        const Correspondence pair{ place,   // keypoint pixel -> the [0,1] fraction
                                   glm::vec2(pt.x / frame.width, pt.y / frame.height) };

        // This place's slot, if it has one: `anchors` repeats a point once per
        // appearance, so several rows can answer for the same place.
        size_t existing = correspondences.size();   // one past the end = no slot yet
        for (size_t i = 0; i < correspondences.size(); i++) {
            if (correspondences[i].worldPos == place) {
                existing = i;
                break;
            }
        }

        // Keeping both would put one place at two pixels -- a contradiction
        // PnP would quietly average.
        if (existing == correspondences.size()) {
            correspondences.push_back(pair);
            bestDistance.push_back(match.distance);
        } else if (match.distance < bestDistance[existing]) {
            correspondences[existing] = pair;
            bestDistance[existing]    = match.distance;
        }
    }

    // The calibration readout for the cap above: a healthy capture reads tight,
    // a median hugging the cap means the list is best-of-noise.
    if (correspondences.empty()) {
        std::cout << "FEATURES: 0 anchors matched (from " << keypoints.size()
                  << " keypoints in the frame)" << std::endl;
    } else {
        std::vector<float> sorted = bestDistance;
        std::sort(sorted.begin(), sorted.end());
        std::cout << "FEATURES: " << correspondences.size() << " anchors matched (from "
                  << keypoints.size() << " keypoints; accepted distances "
                  << (int)sorted.front() << "-" << (int)sorted.back()
                  << ", median " << (int)sorted[sorted.size() / 2] << ")" << std::endl;
    }
    return correspondences;
}

std::optional<Waypoint> estimatePoseFromFeatures(const FeatureDb &db,
                                                 const FramePixels &frame,
                                                 float fov, int viewportWidth,
                                                 int viewportHeight,
                                                 std::optional<size_t> minInliers)
{
    const std::vector<Correspondence> correspondences = matchFeaturesToDb(db, frame);

    // The default is a quarter of what this frame MATCHED, not of the database
    // -- a frame only sees the anchors of the views near it, so any fraction of
    // the total becomes unmeetable once the database outgrows one frame. What
    // the floor gives up in caution is carried by the reprojection gate and by
    // the caller's plausibility check on the estimate.
    const size_t consensus = std::clamp(minInliers.value_or(correspondences.size() / 4),
                                        kMinConsensus, kMaxConsensus);
    if (correspondences.size() < consensus) {
        std::cout << "FEATURES: not enough to solve -- " << consensus
                  << " agreeing matches are needed; move toward the anchored views"
                  << std::endl;
        return std::nullopt;
    }

    return computeCameraPoseRansac(correspondences, fov,
                                   viewportWidth, viewportHeight, (int)consensus);
}
