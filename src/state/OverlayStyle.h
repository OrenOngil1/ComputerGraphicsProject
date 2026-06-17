#pragma once

#include <glm/glm.hpp>

// The shared visual language of the overlays, in one place so the modes that
// speak it can't drift apart: PICK and the pose-comparison modes all show a
// PnP estimate (ghost terrain, estimate markers, computed path) in the same
// signature shade, and every mode draws the true/recorded trajectory in the
// same path color. `inline` lets the definitions sit in a shared header
// without an ODR/multiple-definition link error.
namespace overlay {

// The true (recorded / captured) trajectory -- RECORD's flight path and the
// pose-comparison modes' true fly-through both use it.
inline const glm::vec3 truePathColor(0.0f, 0.0f, 1.0f);   // blue

// Everything that visualizes a PnP estimate: the translucent "ghost" terrain,
// the estimated-pose markers, the computed fly-through path.
inline const glm::vec3 estimateColor(1.0f, 0.5f, 0.0f);   // orange (unique to estimates)
inline const float     estimateGhostAlpha = 0.6f;         // ghost blend transparency
inline const float     estimateGhostTint  = 0.6f;         // how far the ghost tints toward orange
inline const float     estimateMarkerSize = 5.0f;         // estimated-camera dot (GL_POINTS px)

// PICK mode: the 2D half of a correspondence that has been clicked in the player
// view but is still waiting for its 3D match in the global view. White is reserved
// for this -- the completed-point palette (pickedPointColor) deliberately omits it --
// so the in-progress pick reads as distinct from every completed one.
inline const glm::vec3 pendingPickColor(1.0f, 1.0f, 1.0f);   // white (reserved)

// PICK mode marker size (GL_POINTS diameter, pixels). One size for all three pick
// markers -- the 3D point in the global view, the 2D observation in the player view,
// and the pending 2D pick -- so a correspondence reads as the same dot in both halves
// and a pending pick matches a completed one in size. The pending pick is told apart
// by its reserved color (pendingPickColor) alone, not by being larger.
inline const float pickMarkerSize = 10.0f;

} // namespace overlay
