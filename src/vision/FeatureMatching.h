#pragma once

#include <optional>
#include <vector>

#include <glm/glm.hpp>
#include <opencv2/core.hpp>

#include "../core/Scene.h"    // FramePixels, Mesh
#include "../core/Camera.h"   // Waypoint

// The feature-matching mode's pre-phase product: ORB descriptors, each anchored
// to a 3D terrain point the USER hand-placed on the map (Mode D is manual).
// Descriptors from all contributing views are stacked into one matrix;
// anchors[i] is the world-space point behind descriptor row i. Plain data --
// the mode is the only writer.
struct FeatureDb {
    cv::Mat descriptors;              // CV_8U, one 32-byte ORB descriptor per row
    std::vector<glm::vec3> anchors;   // centered world space, like Correspondence::worldPos

    bool empty() const { return anchors.empty(); }
};

// Pre-phase suggestion step: detect ORB on the frame and return the strongest
// maxCount keypoints (by corner response) with their aligned descriptor rows --
// the salient points the user is asked to place on the map, one at a time. The
// rendered view's black background carries no texture, so the strongest
// responses naturally land on the terrain. keypoints/descriptors are cleared
// first; descriptors has one row per returned keypoint.
void detectTopFeatures(const FramePixels &frame, int maxCount,
                       std::vector<cv::KeyPoint> &keypoints, cv::Mat &descriptors);

// Run-phase: detect ORB features in the current frame, match them against the
// database (brute-force Hamming + Lowe ratio test), and solve the surviving
// 2D-3D pairs for the camera pose with the RANSAC PnP solver. nullopt when
// there are too few confident matches or no consensus pose.
std::optional<Waypoint> estimatePoseFromFeatures(const FeatureDb &db,
                                                 const FramePixels &frame,
                                                 float fov, int viewportWidth,
                                                 int viewportHeight);
