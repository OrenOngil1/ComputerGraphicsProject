#include "FeatureMatching.h"

#include <cmath>
#include <iostream>

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

void harvestViewFeatures(FeatureDb &db, const FramePixels &frame,
                         const std::vector<int> &vertexIds, const Mesh &mesh)
{
    std::vector<cv::KeyPoint> keypoints;
    cv::Mat descriptors;
    detectFeatures(frame, keypoints, descriptors);

    for (size_t i = 0; i < keypoints.size(); i++) {
        // The keypoint's subpixel position rounds to the pixel whose vertex id
        // anchors it. Keypoints over the background (id -1) have no 3D point
        // and drop out -- e.g. silhouette responses against the sky.
        const int x = (int)std::lround(keypoints[i].pt.x);
        const int y = (int)std::lround(keypoints[i].pt.y);
        if (x < 0 || x >= frame.width || y < 0 || y >= frame.height)
            continue;
        const int id = vertexIds[(size_t)y * frame.width + x];
        if (id < 0)
            continue;

        db.descriptors.push_back(descriptors.row((int)i));
        db.anchors.push_back(mesh.worldPos(id));
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

    std::vector<PickedPoint> correspondences;
    for (const std::vector<cv::DMatch> &pair : candidates) {
        if (pair.size() < 2 || pair[0].distance >= 0.75f * pair[1].distance)
            continue;
        const cv::Point2f &pt = keypoints[pair[0].queryIdx].pt;
        correspondences.push_back({ db.anchors[pair[0].trainIdx],
                                    glm::vec2(pt.x, pt.y) });
    }

    std::cout << "FEATURES: " << correspondences.size()
              << " confident matches from " << keypoints.size()
              << " keypoints" << std::endl;

    return computeCameraPoseRansac(correspondences, fov,
                                   viewportWidth, viewportHeight);
}
