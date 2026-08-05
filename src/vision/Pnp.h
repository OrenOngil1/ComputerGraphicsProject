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
// minInliers is the smallest consensus the caller will trust. A handful of
// inliers can satisfy RANSAC yet leave the pose badly under-constrained;
// raising the bar above the algebraic minimum of 4 lets a caller refuse a
// poor-overlap frame outright instead of logging a confident-looking garbage
// pose.
//
// reprojErrorPx is how far an observation may sit from where the candidate pose
// would put it and still count as agreeing. It must cover the error in the
// correspondences themselves, not just pixel noise: a 3D point placed by hand
// on the map lands tens of world units from the feature it belongs to, which is
// tens of pixels of reprojection error. Gate below that and the correct-but-
// imprecise pairs are voted out as outliers, leaving the consensus to be built
// from the wrong ones -- a confident pose hundreds of units off.
constexpr float kHandPlacedReprojErrorPx = 40.0f;

std::optional<Waypoint> computeCameraPoseRansac(const std::vector<Correspondence> &points,
                                                float fov, int viewportWidth, int viewportHeight,
                                                int minInliers = 4,
                                                float reprojErrorPx = kHandPlacedReprojErrorPx);
