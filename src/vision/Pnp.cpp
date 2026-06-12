#include "Pnp.h"

#include <iostream>

#include <opencv2/calib3d.hpp>

#include "../core/Utils.h"

// Split a correspondence list into the parallel point arrays OpenCV wants.
// 3D object points are already in centered world space (every producer stores
// them centered), so no offset is applied; image points are pixel positions.
static void toCvPoints(const std::vector<PickedPoint> &points,
                       std::vector<cv::Point3f> &objectPoints,
                       std::vector<cv::Point2f> &imagePoints)
{
    objectPoints.reserve(points.size());
    imagePoints.reserve(points.size());
    for (const PickedPoint &p : points) {
        objectPoints.push_back(glmToCvPoint3f(p.worldPos));
        imagePoints.push_back(glmToCvPoint2f(p.imagePos));
    }
}

// Shared tail of both solvers. solvePnP returns the world->camera transform
// (R, t); invert it to recover the camera's world pose: eye = -R^T t, and a
// look-at target one unit along the camera's forward axis (+Z in OpenCV's
// camera frame) -> R^T * (0,0,1) in world.
static Waypoint extrinsicsToPose(const cv::Mat &rvec, const cv::Mat &tvec)
{
    cv::Mat R;
    cv::Rodrigues(rvec, R);
    cv::Mat eye     = -R.t() * tvec;
    cv::Mat forward =  R.t() * (cv::Mat_<double>(3, 1) << 0, 0, 1);
    cv::Mat target  = eye + forward;

    Waypoint pose;
    pose.position = glm::vec3(eye.at<double>(0),    eye.at<double>(1),    eye.at<double>(2));
    pose.target   = glm::vec3(target.at<double>(0), target.at<double>(1), target.at<double>(2));
    return pose;
}

std::optional<Waypoint> computeCameraPose(const std::vector<PickedPoint> &pickedPoints,
                                          float fov, int viewportWidth, int viewportHeight)
{
    if (pickedPoints.size() < 4) {
        std::cerr << "PnP needs at least 4 picked points (have "
                  << pickedPoints.size() << ")" << std::endl;
        return std::nullopt;
    }

    std::vector<cv::Point3f> objectPoints;
    std::vector<cv::Point2f> imagePoints;
    toCvPoints(pickedPoints, objectPoints, imagePoints);

    cv::Mat_<double> K = getCameraIntrinsicMatrix(fov, viewportWidth, viewportHeight);

    cv::Mat rvec, tvec;
    bool ok = cv::solvePnP(objectPoints, imagePoints, K, cv::Mat(),
                           rvec, tvec, false, cv::SOLVEPNP_SQPNP);
    if (!ok) {
        std::cerr << "cv::solvePnP failed" << std::endl;
        return std::nullopt;
    }

    return extrinsicsToPose(rvec, tvec);
}

std::optional<Waypoint> computeCameraPoseRansac(const std::vector<PickedPoint> &points,
                                                float fov, int viewportWidth, int viewportHeight)
{
    if (points.size() < 4) {
        std::cerr << "PnP (RANSAC) needs at least 4 correspondences (have "
                  << points.size() << ")" << std::endl;
        return std::nullopt;
    }

    std::vector<cv::Point3f> objectPoints;
    std::vector<cv::Point2f> imagePoints;
    toCvPoints(points, objectPoints, imagePoints);

    cv::Mat_<double> K = getCameraIntrinsicMatrix(fov, viewportWidth, viewportHeight);

    // 100 iterations and an 8px reprojection tolerance (the OpenCV defaults,
    // spelled out because the inlier list parameter forces naming them): a
    // correspondence is an inlier when the candidate pose reprojects its 3D
    // point within 8px of its matched pixel.
    cv::Mat rvec, tvec;
    std::vector<int> inliers;
    bool ok = cv::solvePnPRansac(objectPoints, imagePoints, K, cv::Mat(),
                                 rvec, tvec, false,
                                 100, 8.0f, 0.99, inliers);
    if (!ok || inliers.size() < 4) {
        std::cerr << "cv::solvePnPRansac found no consensus pose ("
                  << inliers.size() << " inliers)" << std::endl;
        return std::nullopt;
    }

    std::cout << "PnP (RANSAC): " << inliers.size() << " of " << points.size()
              << " correspondences are inliers" << std::endl;
    return extrinsicsToPose(rvec, tvec);
}
