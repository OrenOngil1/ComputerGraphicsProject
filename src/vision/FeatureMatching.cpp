#include "FeatureMatching.h"

#include <algorithm>
#include <cmath>
#include <iostream>
#include <numeric>

#include <opencv2/features2d.hpp>
#include <opencv2/imgproc.hpp>

#include "Pnp.h"   // computeCameraPoseRansac

// The detector works on intensity; collapse the captured RGB to grayscale. The wrap
// constructor shares frame's bytes (no copy); cvtColor writes a fresh gray
// Mat, so the frame is never modified (the const_cast is only because cv::Mat's
// wrap constructor has no const overload).
static cv::Mat toGray(const FramePixels &frame)
{
    cv::Mat rgb(frame.height, frame.width, CV_8UC3,
                const_cast<unsigned char *>(frame.rgb.data()));
    cv::Mat gray;
    cv::cvtColor(rgb, gray, cv::COLOR_RGB2GRAY);
    return gray;
}

// The one detection entry for every phase: matching compares pre-phase and
// run-phase descriptors, so they must be computed identically -- routing every
// detection through here makes that structural, not a convention.
//
// SIFT, not ORB, and the choice is measured, not taste. This terrain has no
// texture -- every feature is a shading gradient -- and ORB's descriptor, 256
// binary brightness comparisons, collapses on exactly that imagery: the
// distance between "the same place seen again" and "a different ridge that
// looks alike" stops separating. Two hand-built databases measured it. With
// the acceptance cap at 96 bits the matcher claimed 22-28 of 30 places in
// every frame; tightened to 64 it still claimed 25-34 of 48 with as few as
// ONE place actually on screen. No threshold works when the bands overlap.
// SIFT's 128 gradient-orientation histograms are the standard remedy for
// smooth gradient imagery, and it is on the course brief's tool list.
void detectAllFeatures(const FramePixels &frame,
                       std::vector<cv::KeyPoint> &keypoints, cv::Mat &descriptors)
{
    static const cv::Ptr<cv::SIFT> sift = cv::SIFT::create(1000);
    sift->detectAndCompute(toGray(frame), cv::noArray(), keypoints, descriptors);
}

// SIFT descriptors are 128 floats, normalised by OpenCV so unrelated pairs sit
// around 400-500 apart (L2) and genuine re-sightings under a moderate
// viewpoint change land well under 300. The cap is the pipeline's only
// absolute quality bar (the cross-check is relative), and it also gates which
// detections may join a place as a new appearance (resemblesAnchoredPoint),
// so what the database stores and what it accepts at match time are one
// standard. Every capture prints the accepted distances -- read them against
// this number before turning it.
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

    // Rank by ORB response (corner strength): the strongest keypoints are the
    // most repeatable, and the ones a user would naturally single out.
    std::vector<size_t> order(allKps.size());
    std::iota(order.begin(), order.end(), size_t(0));
    std::sort(order.begin(), order.end(), [&](size_t a, size_t b) {
        return allKps[a].response > allKps[b].response;
    });

    // A keypoint on the frame's rim is a poor anchor: half its surroundings are
    // off-screen, so its 3D spot is hard to recognise on the map, and it drops
    // out of view under the smallest camera move.
    const float margin = 16.0f;

    // The spacing maxCount points would have if they tiled the frame evenly,
    // pulled in a little so a set that is merely well-spread still fills the
    // quota. Then walk the ranking and take a keypoint only when it is that far
    // from everything already taken -- strongest first, so the spread is paid
    // for with the weaker of any two crowded keypoints.
    float radius = 0.8f * std::sqrt((float)frame.width * (float)frame.height /
                                    (float)maxCount);

    // A cramped or feature-poor frame may not have maxCount points that far
    // apart. Rather than return a short set, relax and retry: a smaller spread
    // is still better than the cluster the ranking alone would give.
    std::vector<size_t> chosen;
    for (int attempt = 0; attempt < 3 && (int)chosen.size() < maxCount; attempt++, radius *= 0.5f) {
        chosen.clear();
        for (size_t i : order) {
            const cv::Point2f &pt = allKps[i].pt;
            if (pt.x < margin || pt.y < margin ||
                pt.x > (float)frame.width - margin || pt.y > (float)frame.height - margin)
                continue;

            bool clear = true;
            for (size_t j : chosen)
                clear = clear && cv::norm(allKps[j].pt - pt) >= radius;
            if (!clear)
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

bool resemblesAnchoredPoint(const FeatureDb &db, const glm::vec3 &place,
                            const cv::Mat &descriptor)
{
    for (size_t i = 0; i < db.anchors.size(); i++) {
        if (db.anchors[i] != place)
            continue;
        if (cv::norm(db.descriptors.row((int)i), descriptor, cv::NORM_L2)
                <= kMaxDescriptorDistance)
            return true;
    }
    return false;
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

    // DATABASE -> FRAME, cross-checked. For each stored appearance: where is it
    // in this view, and does that keypoint agree that this is its best match?
    //
    // The direction matters. Asked the other way round -- once per keypoint --
    // a 20-row database can answer "yes" 140 times, claiming one anchor is in
    // dozens of places at once and handing PnP a set of contradictions to build
    // a consensus from.
    //
    // Cross-checking replaces Lowe's ratio test, which is the wrong tool for
    // this scene. The ratio test asks whether the best match beats the SECOND
    // best in the same image, so it only passes where a feature is locally
    // unique. This terrain's appearance is shading, not texture: every ridge
    // shoulder looks like every other ridge shoulder, second-best is nearly as
    // good almost everywhere, and the test throws out true matches wholesale --
    // measured on a hand-built database it kept 1 to 4 of the 10 to 15 anchors
    // that were genuinely in frame. Mutual agreement asks something the scene
    // can actually answer, keeps the one-correspondence-per-anchor property the
    // ratio test was brought in for (and adds one-per-keypoint), and leaves the
    // outlier rejection to RANSAC, where it belongs.
    cv::BFMatcher matcher(cv::NORM_L2, /*crossCheck=*/true);
    std::vector<cv::DMatch> matches;
    matcher.match(db.descriptors, descriptors, matches);

    // One correspondence per PLACE. `anchors` repeats a 3D point once per
    // appearance stored for it, so two rows of the same place can win two
    // different keypoints -- a contradiction PnP would quietly average. Keep
    // whichever appearance matched more closely.
    std::vector<float> bestDistance;   // parallel to correspondences
    for (const cv::DMatch &match : matches) {
        if (match.distance > kMaxDescriptorDistance)
            continue;

        const glm::vec3 &place = db.anchors[match.queryIdx];
        const cv::Point2f &pt = keypoints[match.trainIdx].pt;
        const Correspondence pair{ place,   // keypoint pixel -> the [0,1] fraction
                                   glm::vec2(pt.x / frame.width, pt.y / frame.height) };

        size_t existing = correspondences.size();
        for (size_t i = 0; i < correspondences.size(); i++)
            if (correspondences[i].worldPos == place) {
                existing = i;
                break;
            }

        if (existing == correspondences.size()) {
            correspondences.push_back(pair);
            bestDistance.push_back(match.distance);
        } else if (match.distance < bestDistance[existing]) {
            correspondences[existing] = pair;
            bestDistance[existing]    = match.distance;
        }
    }

    // The accepted distances are the calibration readout for the cap above: a
    // healthy capture reads tight (well under the cap); a median hugging the
    // cap means the list is best-of-noise again.
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
                                                 int viewportHeight)
{
    const std::vector<Correspondence> correspondences = matchFeaturesToDb(db, frame);

    // Consensus floor: a quarter of what this frame claims to see must agree on
    // one pose. A fixed number cannot fit both ends -- 6 is a high bar for a
    // 20-anchor hand-build and a trivial one for a machine-built database of
    // thousands -- but scaling it to the DATABASE would be worse: a frame only
    // ever sees the anchors of the views near it, so any fraction of the total
    // becomes unmeetable as soon as the database covers more views than one
    // frame can contain. A fraction of the matches is the same demand at every
    // size and is never impossible by construction.
    //
    // A quarter rather than a half because cross-checked matches include the
    // out-of-frame anchors that found a plausible-looking home anyway; demanding
    // that half of everything agree penalises a view for matches it was never
    // going to be able to use.
    //
    // The floor keeps it clear of PnP's algebraic minimum (four points always
    // agree with the pose fitted to them, so four inliers say nothing) and high
    // enough that six spread points genuinely pin a pose; the ceiling stops a
    // large database from demanding more agreement than a pose needs.
    const int minInliers = std::clamp((int)correspondences.size() / 4, 6, 25);
    if ((int)correspondences.size() < minInliers) {
        std::cout << "FEATURES: not enough to solve -- " << minInliers
                  << " agreeing matches are needed; move toward the anchored views"
                  << std::endl;
        return std::nullopt;
    }

    return computeCameraPoseRansac(correspondences, fov,
                                   viewportWidth, viewportHeight, minInliers);
}
