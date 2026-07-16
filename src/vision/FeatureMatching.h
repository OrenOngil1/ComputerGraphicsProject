#pragma once

#include <optional>
#include <vector>

#include <glm/glm.hpp>
#include <opencv2/core.hpp>

#include "../core/Scene.h"    // FramePixels
#include "../core/Camera.h"   // Waypoint

// The feature-matching mode's pre-phase product: ORB descriptors, each
// anchored to a 3D terrain point the user hand-placed on the map. anchors[i]
// is the world-space point behind descriptor row i.
struct FeatureDb {
    cv::Mat descriptors;              // CV_8U, one 32-byte ORB descriptor per row
    std::vector<glm::vec3> anchors;   // centered world space, like Correspondence::worldPos

    bool empty() const { return anchors.empty(); }
};

// Pre-phase suggestion step: detect ORB on the frame and return the strongest
// maxCount keypoints (by corner response) with their aligned descriptor rows.
// keypoints/descriptors are cleared first; descriptors has one row per keypoint.
void detectTopFeatures(const FramePixels &frame, int maxCount,
                       std::vector<cv::KeyPoint> &keypoints, cv::Mat &descriptors);

// Run-phase: detect ORB features in the frame, match against the database
// (brute-force Hamming + Lowe ratio test), and solve the surviving 2D-3D pairs
// with RANSAC PnP. nullopt when there are too few confident matches or no
// consensus pose.
std::optional<Waypoint> estimatePoseFromFeatures(const FeatureDb &db,
                                                 const FramePixels &frame,
                                                 float fov, int viewportWidth,
                                                 int viewportHeight);
