#pragma once

#include <optional>
#include <vector>

#include "../core/Scene.h"    // Correspondence
#include "../core/Camera.h"   // Waypoint

// Solve PnP: from >= 4 correspondences, the camera's vertical FOV (degrees),
// and the viewport size, estimate the camera pose that projects the 3D points
// onto their 2D observations. Returns the pose as a Waypoint (eye + look-at
// target), or nullopt if there are too few points or the solver fails.
std::optional<Waypoint> computeCameraPose(const std::vector<Correspondence> &correspondences,
                                          float fov, int viewportWidth, int viewportHeight);

// RANSAC flavor, for machine-generated correspondences: fits candidate poses
// on random subsets and keeps the one most correspondences agree with, so a
// wrong descriptor match is outvoted instead of averaged in.
//
// minInliers is the smallest consensus the caller will trust (>= the algebraic
// minimum of 4): raising it turns a poor-overlap frame into a refusal instead
// of a confident-looking garbage pose. reprojErrorPx is the agreement radius;
// <= 0 selects the fraction below of the frame height -- sized to what a
// true-but-strained pair carries and no wider, or lookalike ridges assemble a
// rival consensus. (Measurements and sizing rationale:
// docs/pose-estimation-modes.md, "Run-phase (B)".)
constexpr float kHandPlacedReprojFraction = 0.03f;   // of the frame height

std::optional<Waypoint> computeCameraPoseRansac(const std::vector<Correspondence> &points,
                                                float fov, int viewportWidth, int viewportHeight,
                                                int minInliers = 4,
                                                float reprojErrorPx = 0.0f);
