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

// RANSAC flavor, for machine-generated correspondences: fits candidate poses on
// random subsets and keeps the one most agree with, so a wrong descriptor match
// is outvoted instead of averaged in. minInliers is the smallest consensus the
// caller will trust (>= the algebraic minimum of 4); reprojErrorPx is the
// agreement radius, and <= 0 selects the fraction below.

// Sized to what a true-but-strained pair carries and no wider, or lookalike
// ridges assemble a rival consensus: at 4.5%, five of eight solves became
// coalitions 60-240 units off. Slack cannot stand in for appearance coverage.
// (docs/pose-estimation-modes.md, "Run-phase (B)")
constexpr float kHandPlacedReprojFraction = 0.03f;   // of the frame height

// inlierIndices, when given, receives the winning consensus (indices into
// `points`) of a successful solve; untouched on refusal.
std::optional<Waypoint> computeCameraPoseRansac(const std::vector<Correspondence> &points,
                                                float fov, int viewportWidth, int viewportHeight,
                                                int minInliers = 4,
                                                float reprojErrorPx = 0.0f,
                                                std::vector<int> *inlierIndices = nullptr);
