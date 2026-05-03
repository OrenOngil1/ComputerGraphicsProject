#pragma once

#include <random>
#include <vector>
#include <opencv2/core/mat.hpp>

// Utility function to generate a random index
inline size_t randomIndex(size_t size)
{
    static std::mt19937 gen(std::random_device{}());
    return std::uniform_int_distribution<size_t>(0, size - 1)(gen);
}

inline cv::Mat_<double> getCameraIntrinsicMatrix(double fov, double width, double height)
{
    double fovyRadians = glm::radians(fov);

    double fy = height / (2.0 * tan(fovyRadians / 2.0));
    double fx = fy * (width / height);

    double cx = width / 2.0;
    double cy = height / 2.0;

    return (cv::Mat_<double>(3, 3) <<
        fx, 0,  cx,
        0,  fy, cy,
        0,  0,  1
    );
}

// JS-like map function for C++
template<typename T, typename Callable>
inline auto map(const std::vector<T> &input, Callable func) -> std::vector<decltype(func(input[0]))>
{
    std::vector<decltype(func(input[0]))> output;
    output.reserve(input.size());
    for (const auto &item : input) {
        output.push_back(func(item));
    }
    return output;
}