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
// would put it and still count as agreeing. It must cover the real error of a
// TRUE pair and little more: every pixel past that is room in which false
// matches -- on self-similar terrain they are lookalike ridges, spatially
// coherent, not random scatter -- can assemble a rival consensus. Hand-placed
// anchors used to need tens of pixels here, because a raw map click landed
// tens of world units from the feature it belonged to; snapping the click onto
// the suggestion's exact viewing ray removed that, leaving keypoint jitter of
// a few pixels plus the parallax of a misjudged depth ALONG the ray seen from
// another angle. 16 px covers those. 40 px was measured to let six-strong
// coalitions of lookalikes outvote the truth -- confident poses hundreds of
// units off.
constexpr float kHandPlacedReprojErrorPx = 16.0f;

std::optional<Waypoint> computeCameraPoseRansac(const std::vector<Correspondence> &points,
                                                float fov, int viewportWidth, int viewportHeight,
                                                int minInliers = 4,
                                                float reprojErrorPx = kHandPlacedReprojErrorPx);
