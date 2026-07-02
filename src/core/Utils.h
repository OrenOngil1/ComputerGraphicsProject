#pragma once

#include <random>
#include <cmath>

#include <glm/glm.hpp>
#include <opencv2/core/types.hpp>   // cv::Point2f / cv::Point3f
#include <opencv2/core/mat.hpp>     // cv::Mat_

// Small, header-only helpers shared across the codebase.

// A uniformly random index in [0, size). The generator is static so it is
// seeded once. Caller must ensure size > 0.
inline size_t randomIndex(size_t size)
{
    static std::mt19937 gen(std::random_device{}());
    return std::uniform_int_distribution<size_t>(0, size - 1)(gen);
}

// glm <-> OpenCV point conversions.
inline cv::Point3f glmToCvPoint3f(const glm::vec3 &v)
{
    return cv::Point3f(v.x, v.y, v.z);
}

inline cv::Point2f glmToCvPoint2f(const glm::vec2 &v)
{
    return cv::Point2f(v.x, v.y);
}

inline glm::vec3 cvToGlmVec3(const cv::Mat &p)
{
    return glm::vec3(p.at<double>(0), p.at<double>(1), p.at<double>(2));
}

// The 3x3 pinhole intrinsic matrix K for a VERTICAL fov (degrees) rendered
// onto a width x height viewport, principal point at the center.
//
// Square pixels: fx == fy. The aspect is carried entirely by width/height (and
// cx, cy) -- it must NOT also be folded into the focal length. glm::perspective
// takes a vertical FOV and renders square pixels, deriving the horizontal FOV
// from the aspect; the matching pinhole therefore shares one focal length.
// (Folding width/height into fx makes solvePnP assume a wider horizontal FOV
// than GL drew and pulls every estimated camera 20-30% too close.)
inline cv::Mat_<double> getCameraIntrinsicMatrix(double fov, double width, double height)
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
