#include "Pnp.h"

#include <iostream>

#include <opencv2/calib3d.hpp>

#include "../core/Utils.h"

std::optional<Waypoint> computeCameraPose(const std::vector<PickedPoint> &pickedPoints,
                                          float fov, int viewportWidth, int viewportHeight)
{
    if (pickedPoints.size() < 4) {
        std::cerr << "PnP needs at least 4 picked points (have "
                  << pickedPoints.size() << ")" << std::endl;
        return std::nullopt;
    }

    // 3D object points are already in centered world space (PickState stores them
    // centered), so no offset is applied here; image points are the clicked pixels.
    // The named converters carry the glm->cv translation, so this is a plain loop.
    std::vector<cv::Point3f> objectPoints;
    std::vector<cv::Point2f> imagePoints;
    objectPoints.reserve(pickedPoints.size());
    imagePoints.reserve(pickedPoints.size());
    for (const PickedPoint &p : pickedPoints) {
        objectPoints.push_back(glmToCvPoint3f(p.worldPos));
        imagePoints.push_back(glmToCvPoint2f(p.imagePos));
    }

    cv::Mat_<double> K = getCameraIntrinsicMatrix(fov, viewportWidth, viewportHeight);

    cv::Mat rvec, tvec;
    bool ok = cv::solvePnP(objectPoints, imagePoints, K, cv::Mat(),
                           rvec, tvec, false, cv::SOLVEPNP_SQPNP);
    if (!ok) {
        std::cerr << "cv::solvePnP failed" << std::endl;
        return std::nullopt;
    }

    // solvePnP returns the world->camera transform (R, t). Invert it to recover the
    // camera's world pose: eye = -R^T t, and a look-at target one unit along the
    // camera's forward axis (+Z in OpenCV's camera frame) -> R^T * (0,0,1) in world.
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
