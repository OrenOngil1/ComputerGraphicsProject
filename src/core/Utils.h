#pragma once

#include <random>
#include <cmath>

#include <glm/glm.hpp>
#include <opencv2/core/types.hpp>   // cv::Point2f / cv::Point3f
#include <opencv2/core/mat.hpp>     // cv::Mat_

// Small, header-only helpers shared across the codebase.

// A uniformly random index in [0, size). `gen` is static so it is seeded once
// (a fresh std::random_device per call would be wasteful and less random).
// Caller must ensure size > 0.
inline size_t randomIndex(size_t size)
{
    static std::mt19937 gen(std::random_device{}());
    return std::uniform_int_distribution<size_t>(0, size - 1)(gen);
}

// glm -> OpenCV point conversions.
inline cv::Point3f glmToCvPoint3f(const glm::vec3 &v)
{
    return cv::Point3f(v.x, v.y, v.z);
}

inline cv::Point2f glmToCvPoint2f(const glm::vec2 &v)
{
    return cv::Point2f(v.x, v.y);
}

// OpenCV -> glm point conversions.
inline glm::vec3 cvToGlmVec3(const cv::Mat &p)
{
    return glm::vec3(p.at<double>(0), p.at<double>(1), p.at<double>(2));
}

// Build the 3x3 pinhole intrinsic matrix K from a vertical field of view and the
// image size. fy is the focal length in pixels mapping the vertical FOV onto
// `height` pixels; (cx,cy) is the principal point at the image center.
inline cv::Mat_<double> getCameraIntrinsicMatrix(double fov, double width, double height)
{
    double fovyRadians = glm::radians(fov);

    double fy = height / (2.0 * std::tan(fovyRadians / 2.0));

    // Square pixels: fx == fy. The image aspect is carried entirely by width
    // and height differing (and by cx, cy) -- it must NOT also be folded into
    // the focal length. glm::perspective takes a VERTICAL fov and renders
    // square pixels, deriving the horizontal FOV from the aspect; the matching
    // pinhole therefore shares one focal length. (The old fx = fy * width/height
    // shrank fx on the non-square player viewport, so solvePnP assumed a wider
    // horizontal FOV than GL drew and pulled every estimated camera ~20-30%
    // too close.)
    double fx = fy;

    double cx = width / 2.0;
    double cy = height / 2.0;

    return (cv::Mat_<double>(3, 3) <<
        fx, 0,  cx,
        0,  fy, cy,
        0,  0,  1
    );
}
