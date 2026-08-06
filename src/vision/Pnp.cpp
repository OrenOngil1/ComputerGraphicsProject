#include "Pnp.h"

#include <cmath>
#include <iostream>

#include <opencv2/calib3d.hpp>

// glm <-> OpenCV point conversions.
static cv::Point3f glmToCvPoint3f(const glm::vec3 &v)
{
    return cv::Point3f(v.x, v.y, v.z);
}

static cv::Point2f glmToCvPoint2f(const glm::vec2 &v)
{
    return cv::Point2f(v.x, v.y);
}

static glm::vec3 cvToGlmVec3(const cv::Mat &p)
{
    return glm::vec3(p.at<double>(0), p.at<double>(1), p.at<double>(2));
}

// The 3x3 pinhole intrinsic matrix K for a VERTICAL fov (degrees) on a
// width x height viewport, principal point centered. Square pixels: fx == fy,
// the aspect carried entirely by width/height and cx/cy -- exactly as
// glm::perspective renders. Folding width/height into fx as well makes
// solvePnP assume a wider horizontal FOV than GL drew and pulls every
// estimated camera 20-30% too close.
static cv::Mat_<double> getCameraIntrinsicMatrix(double fov, double width, double height)
{
    double fovyRadians = glm::radians(fov);

    double fy = height / (2.0 * std::tan(fovyRadians / 2.0));
    double fx = fy;

    double cx = width / 2.0;
    double cy = height / 2.0;

    return (cv::Mat_<double>(3, 3) <<
        fx, 0,  cx,
        0,  fy, cy,
        0,  0,  1
    );
}

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
                                                int minInliers, float reprojErrorPx)
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

    // The agreement gate scales with the frame unless the caller pinned it:
    // pixels of tolerance only mean anything relative to how many pixels the
    // frame has (see kHandPlacedReprojFraction).
    if (reprojErrorPx <= 0.0f)
        reprojErrorPx = kHandPlacedReprojFraction * (float)viewportHeight;

    // Budget for hand-anchored databases, where a good share of the matches
    // are false: 5000 draws still land a clean 4-point sample with >90%
    // certainty at true-match fractions down to ~15%. Milliseconds either way.
    constexpr int    kIterations = 5000;
    constexpr double kConfidence = 0.99;

    // AP3P, explicitly: P3P-family solvers draw 4-point minimal samples (not
    // EPnP's 5 -- w^4 vs w^5 odds of a clean draw) and fit them exactly, where
    // EPnP on 5 near-coplanar points (a DEM from altitude) wobbles enough that
    // no consensus ever forms. Full rationale:
    // docs/pose-estimation-modes.md, "Run-phase (B)".
    cv::Mat rvec, tvec;
    std::vector<int> inliers;
    bool ok = cv::solvePnPRansac(objectPoints, imagePoints, K, cv::Mat(),
                                 rvec, tvec, false,
                                 kIterations, reprojErrorPx, kConfidence, inliers,
                                 cv::SOLVEPNP_AP3P);
    if (!ok || (int)inliers.size() < minInliers) {
        std::cerr << "PnP (RANSAC): no trustworthy pose -- " << inliers.size()
                  << " of " << points.size() << " correspondences agree, need "
                  << minInliers << std::endl;
        return std::nullopt;
    }

    std::cout << "PnP (RANSAC): " << inliers.size() << " of " << points.size()
              << " correspondences are inliers" << std::endl;
    return extrinsicsToPose(rvec, tvec);
}
