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

// RANSAC flavor, for machine-generated correspondences: descriptor matching
// lets some wrong pairs through, and one bad pair can wreck a least-squares
// solve. RANSAC fits candidate poses on random subsets and keeps the one most
// correspondences agree with, so outliers are outvoted instead of averaged in.
//
// minInliers is the smallest consensus the caller will trust. A handful can
// satisfy RANSAC yet leave the pose badly under-constrained; raising the bar
// above the algebraic minimum of 4 lets a caller refuse a poor-overlap frame
// instead of logging a confident-looking garbage pose.
//
// reprojErrorPx is how far an observation may sit from where the candidate pose
// would put it and still count as agreeing. It must cover the real error of a
// TRUE pair and little more: every pixel past that is room in which lookalike
// ridges -- spatially coherent, not random scatter -- assemble a rival consensus.
//
// The default (reprojErrorPx <= 0) is this FRACTION of the frame height, since
// a fixed pixel count is a different angular tolerance at every capture
// resolution. Its size is what a true-but-strained pair carries: SIFT drift
// under a viewpoint change, plus a misjudged depth reprojecting from another
// angle. Tighter passes only near-exact matches and refuses the
// strained-but-true band; looser lets lookalikes outvote the truth.
// (Measurements: docs/pose-estimation-modes.md, "Run-phase (B)")
constexpr float kHandPlacedReprojFraction = 0.03f;   // of the frame height

std::optional<Waypoint> computeCameraPoseRansac(const std::vector<Correspondence> &points,
                                                float fov, int viewportWidth, int viewportHeight,
                                                int minInliers = 4,
                                                float reprojErrorPx = 0.0f);
