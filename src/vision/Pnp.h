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
