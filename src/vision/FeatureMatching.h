#pragma once

#include <optional>
#include <vector>

#include <glm/glm.hpp>
#include <opencv2/core.hpp>

#include "../core/Scene.h"    // FramePixels, Mesh
#include "../core/Camera.h"   // Waypoint

// The feature-matching mode's pre-phase product: ORB descriptors harvested
// from rendered views, each anchored to the 3D terrain point it was detected
// on. Descriptors from all contributing views are stacked into one matrix;
// anchors[i] is the world-space point behind descriptor row i. Plain data --
// the functions below are the only writers.
struct FeatureDb {
    cv::Mat descriptors;              // CV_8U, one 32-byte ORB descriptor per row
    std::vector<glm::vec3> anchors;   // centered world space, like PickedPoint::worldPos

    bool empty() const { return anchors.empty(); }
};

// Pre-phase, one view's worth: detect ORB keypoints on the captured frame,
// anchor each to 3D through the per-pixel vertex-id map (keypoints over
// background pixels drop out), and append the survivors to the database.
// frame and vertexIds must come from the same View (Renderer's
// captureSceneFrame / captureVertexIdFrame) so pixel (x, y) means the same
// surface point in both.
void harvestViewFeatures(FeatureDb &db, const FramePixels &frame,
                         const std::vector<int> &vertexIds, const Mesh &mesh);

// Run-phase: detect ORB features in the current frame, match them against the
// database (brute-force Hamming + Lowe ratio test), and solve the surviving
// 2D-3D pairs for the camera pose with the RANSAC PnP solver. nullopt when
// there are too few confident matches or no consensus pose.
std::optional<Waypoint> estimatePoseFromFeatures(const FeatureDb &db,
                                                 const FramePixels &frame,
                                                 float fov, int viewportWidth,
                                                 int viewportHeight);
