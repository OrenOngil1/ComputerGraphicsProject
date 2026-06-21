#include "FeatureMatching.h"

#include <algorithm>
#include <cmath>
#include <iostream>
#include <numeric>

#include <opencv2/features2d.hpp>
#include <opencv2/imgproc.hpp>

#include "Pnp.h"   // computeCameraPoseRansac

// ORB works on intensity; collapse the captured RGB to grayscale. The wrap
// constructor shares frame's bytes (no copy) -- cvtColor writes a fresh gray
// Mat, so the frame itself is never modified (the const_cast is only because
// cv::Mat's wrap constructor has no const overload).
static cv::Mat toGray(const FramePixels &frame)
{
    cv::Mat rgb(frame.height, frame.width, CV_8UC3,
                const_cast<unsigned char *>(frame.rgb.data()));
    cv::Mat gray;
    cv::cvtColor(rgb, gray, cv::COLOR_RGB2GRAY);
    return gray;
}

// The one detection entry for both phases: matching compares pre-phase and
// run-phase descriptors, so they must be computed identically -- routing
// every detection through here makes that structural, not a convention. The
// configuration never changes, so one detector instance serves all calls;
// 1000 features per frame is plenty for a half-window viewport.
static void detectFeatures(const FramePixels &frame,
                           std::vector<cv::KeyPoint> &keypoints, cv::Mat &descriptors)
{
    static const cv::Ptr<cv::ORB> orb = cv::ORB::create(1000);
    orb->detectAndCompute(toGray(frame), cv::noArray(), keypoints, descriptors);
}

void detectTopFeatures(const FramePixels &frame, int maxCount,
                       std::vector<cv::KeyPoint> &keypoints, cv::Mat &descriptors)
{
    std::vector<cv::KeyPoint> allKps;
    cv::Mat allDesc;
    detectFeatures(frame, allKps, allDesc);

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

    const size_t keep = std::min((size_t)maxCount, order.size());
    keypoints.reserve(keep);
    for (size_t i = 0; i < keep; i++) {
        keypoints.push_back(allKps[order[i]]);
        descriptors.push_back(allDesc.row((int)order[i]));   // copies the row
    }
}

std::optional<Waypoint> estimatePoseFromFeatures(const FeatureDb &db,
                                                 const FramePixels &frame,
                                                 float fov, int viewportWidth,
                                                 int viewportHeight)
{
    std::vector<cv::KeyPoint> keypoints;
    cv::Mat descriptors;
    detectFeatures(frame, keypoints, descriptors);
    if (descriptors.empty()) {
        std::cout << "FEATURES: no keypoints in the current view" << std::endl;
        return std::nullopt;
    }

    // Two nearest database descriptors per frame descriptor (Hamming distance,
    // ORB's metric), then Lowe's ratio test: keep a match only when the best
    // is clearly better than the runner-up. Ambiguous descriptors -- repetitive
    // terrain texture looks alike everywhere -- would otherwise flood PnP with
    // confident-looking wrong pairs.
    cv::BFMatcher matcher(cv::NORM_HAMMING);
    std::vector<std::vector<cv::DMatch>> candidates;
    matcher.knnMatch(descriptors, db.descriptors, candidates, 2);

    std::vector<Correspondence> correspondences;
    for (const std::vector<cv::DMatch> &pair : candidates) {
        if (pair.size() < 2 || pair[0].distance >= 0.75f * pair[1].distance)
            continue;
        // imagePos is normalized (see Correspondence): the keypoint is a pixel in this
        // frame, so divide by the frame size to store it as a [0,1] fraction.
        const cv::Point2f &pt = keypoints[pair[0].queryIdx].pt;
        correspondences.push_back({ db.anchors[pair[0].trainIdx],
                                    glm::vec2(pt.x / frame.width, pt.y / frame.height) });
    }

    std::cout << "FEATURES: " << correspondences.size()
              << " confident matches from " << keypoints.size()
              << " keypoints" << std::endl;

    // The database is now hand-built and small (~featureCount per view), so the
    // whole DB is only a few dozen descriptors and a run-phase match yields at
    // most that many inliers -- the old 25 floor (tuned for the automatic DB of
    // hundreds) would refuse every frame. Keep a small consensus floor above the
    // algebraic PnP minimum of 4 so a near-empty match is still rejected.
    const int kMinInliers = 6;
    return computeCameraPoseRansac(correspondences, fov,
                                   viewportWidth, viewportHeight, kMinInliers);
}
