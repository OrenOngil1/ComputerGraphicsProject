#pragma once

#include <glm/glm.hpp>

// The shared visual language of the overlays: every mode draws the true
// trajectory and the PnP estimate in the same colors, so they read identically
// across modes.
namespace overlay {

// The true (recorded / captured) trajectory.
inline const glm::vec3 truePathColor(0.0f, 0.0f, 1.0f);   // blue

// Everything that visualizes a PnP estimate: ghost terrain, estimated-pose
// markers, computed fly-through path.
inline const glm::vec3 estimateColor(1.0f, 0.5f, 0.0f);   // orange (unique to estimates)
inline const float     estimateGhostAlpha = 0.6f;         // ghost blend transparency
inline const float     estimateGhostTint  = 0.6f;         // how far the ghost tints toward orange
inline const float     estimateMarkerSize = 5.0f;         // estimated-camera dot (GL_POINTS px)

// PICK's mid-pair marker (2D clicked, 3D still pending). White is reserved for
// it -- the completed-point palette deliberately omits white -- so a pending
// pick is never confusable with a completed one.
inline const glm::vec3 pendingPickColor(1.0f, 1.0f, 1.0f);

// One size for all three pick markers (3D point, 2D observation, pending), so
// a correspondence reads as the same dot in both views. GL_POINTS px.
inline const float pickMarkerSize = 10.0f;

} // namespace overlay
