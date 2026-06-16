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

} // namespace overlay
