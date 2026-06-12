#pragma once

#include <optional>
#include <vector>

#include "../core/Scene.h"    // PickedPoint
#include "../core/Camera.h"   // Waypoint

// Solve the Perspective-n-Point problem: given >=4 2D-3D correspondences (clicked
// terrain points + their pixel positions), the camera's vertical FOV, and the
// viewport size, estimate the camera pose that projects those 3D points onto those
// pixels. Returns the pose as a Waypoint (eye + look-at target), or std::nullopt
// if there are too few points or the solver fails.
std::optional<Waypoint> computeCameraPose(const std::vector<PickedPoint> &pickedPoints,
                                          float fov, int viewportWidth, int viewportHeight);

// RANSAC flavor, for machine-generated correspondences (feature matching):
// descriptor matching always lets some wrong pairs through, and one bad
// correspondence can wreck a least-squares solve. RANSAC fits candidate poses
// on small random subsets and keeps the one most correspondences agree with,
// so outliers end up outvoted instead of averaged in. nullopt when no
// consensus pose exists; logs the inlier count when one does.
std::optional<Waypoint> computeCameraPoseRansac(const std::vector<PickedPoint> &points,
                                                float fov, int viewportWidth, int viewportHeight);
