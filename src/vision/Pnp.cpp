#include "Pnp.h"

#include <iostream>

#include <opencv2/calib3d.hpp>

#include "../core/Utils.h"

// Split a correspondence list into the parallel point arrays OpenCV wants.
// 3D points are already in centered world space; image points are stored
// normalized and denormalized here against the viewport PnP solves for, so
// they match the intrinsics K built from the same width/height.
static void toCvPoints(const std::vector<Correspondence> &points, int width, int height,
                       std::vector<cv::Point3f> &objectPoints,
                       std::vector<cv::Point2f> &imagePoints)
{
    objectPoints.reserve(points.size());
    imagePoints.reserve(points.size());
    for (const Correspondence &p : points) {
        objectPoints.push_back(glmToCvPoint3f(p.worldPos));
        imagePoints.push_back(glmToCvPoint2f(p.imagePixels(width, height)));
    }
}

// solvePnP returns the world->camera transform (R, t); invert it to recover
// the camera's world pose: eye = -R^T t, and a look-at target one unit along
// the camera's forward axis (+Z in OpenCV's camera frame) -> R^T (0,0,1).
static Waypoint extrinsicsToPose(const cv::Mat &rvec, const cv::Mat &tvec)
{
    cv::Mat R;
    cv::Rodrigues(rvec, R);
    cv::Mat eye     = -R.t() * tvec;
    cv::Mat forward =  R.t() * (cv::Mat_<double>(3, 1) << 0, 0, 1);
    cv::Mat target  = eye + forward;

    return Waypoint{ cvToGlmVec3(eye), cvToGlmVec3(target) };
}

std::optional<Waypoint> computeCameraPose(const std::vector<Correspondence> &pickedPoints,
                                          float fov, int viewportWidth, int viewportHeight)
{
    if (pickedPoints.size() < 4) {
        std::cerr << "PnP needs at least 4 picked points (have "
                  << pickedPoints.size() << ")" << std::endl;
        return std::nullopt;
    }

    std::vector<cv::Point3f> objectPoints;
    std::vector<cv::Point2f> imagePoints;
    toCvPoints(pickedPoints, viewportWidth, viewportHeight, objectPoints, imagePoints);

    cv::Mat_<double> K = getCameraIntrinsicMatrix(fov, viewportWidth, viewportHeight);

    // The explicit `false` is useExtrinsicGuess. Without it, SOLVEPNP_SQPNP
    // (an int enum) binds to that bool parameter instead of `flags`, and the
    // default ITERATIVE solver asserts on the empty rvec/tvec it then expects
    // to hold an initial guess.
    cv::Mat rvec, tvec;
    bool ok = cv::solvePnP(objectPoints, imagePoints, K, cv::Mat(),
                           rvec, tvec, false, cv::SOLVEPNP_SQPNP);
    if (!ok) {
        std::cerr << "cv::solvePnP failed" << std::endl;
        return std::nullopt;
    }

    return extrinsicsToPose(rvec, tvec);
}

std::optional<Waypoint> computeCameraPoseRansac(const std::vector<Correspondence> &points,
                                                float fov, int viewportWidth, int viewportHeight,
                                                int minInliers)
{
    if (points.size() < 4) {
        std::cerr << "PnP (RANSAC) needs at least 4 correspondences (have "
                  << points.size() << ")" << std::endl;
        return std::nullopt;
    }

    std::vector<cv::Point3f> objectPoints;
    std::vector<cv::Point2f> imagePoints;
    toCvPoints(points, viewportWidth, viewportHeight, objectPoints, imagePoints);

    cv::Mat_<double> K = getCameraIntrinsicMatrix(fov, viewportWidth, viewportHeight);

    // 100 iterations, 8px reprojection tolerance (the OpenCV defaults, named
    // only because the inlier list parameter forces spelling them out).
    cv::Mat rvec, tvec;
    std::vector<int> inliers;
    bool ok = cv::solvePnPRansac(objectPoints, imagePoints, K, cv::Mat(),
                                 rvec, tvec, false,
                                 100, 8.0f, 0.99, inliers);
    if (!ok || (int)inliers.size() < minInliers) {
        std::cerr << "cv::solvePnPRansac found no trustworthy pose ("
                  << inliers.size() << " inliers, need " << minInliers << ")"
                  << std::endl;
        return std::nullopt;
    }

    std::cout << "PnP (RANSAC): " << inliers.size() << " of " << points.size()
              << " correspondences are inliers" << std::endl;
    return extrinsicsToPose(rvec, tvec);
}
