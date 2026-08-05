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

// FEATURE MATCH's build phase: the active SIFT suggestion in the player view --
// red and oversized, the one dot awaiting its 3D anchor.
inline const glm::vec3 suggestionColor(1.0f, 0.0f, 0.0f);
inline const float     suggestionMarkerSize = 22.0f;   // GL_POINTS px

// FEATURE MATCH's hand-built database, drawn on the map as places. Violet is
// the mode's own color for "the database knows this spot": unused by any
// trajectory, and deliberately not green -- green already means "the camera is
// standing on this waypoint" in drawWaypoints, and it disappears into a green
// elevation ramp.
//
// Bright and large when the player's current view can actually use the anchor,
// dim and small when it cannot (out of frame, or behind a ridge). Together with
// the view cone that is the answer to "is this a good place to press B" --
// which is otherwise only knowable by pressing B.
inline const glm::vec3 anchorColor(0.75f, 0.35f, 1.0f);
inline const float     anchorMarkerSize = 12.0f;
inline const glm::vec3 anchorDimColor(0.34f, 0.18f, 0.48f);
inline const float     anchorDimMarkerSize = 5.0f;

// Where the database came from: the recorded flight path and the views the
// anchors were placed from, drawn while the run phase is being measured. Grey
// because it is provenance, not a measurement -- blue is reserved for truth
// captured now, and without the split the two would be identical dots.
inline const glm::vec3 provenanceColor(0.5f, 0.5f, 0.5f);
inline const float     provenanceMarkerSize = 5.0f;

// ── View aids on the global map (the V key) ──────────────────
// The player camera's view cone: which wedge of the map the other pane is
// showing. Pale cyan -- unused by any trajectory or marker, so the aids never
// read as data.
inline const glm::vec3 viewConeColor(0.35f, 0.85f, 1.0f);
inline const float     viewConeWidth = 2.0f;   // GL line px

// A sight line is drawn in the color of the marker it belongs to (PICK's white
// pending dot, FEATURE MATCH's red suggestion), so the line on the map and the
// dot in the player view are visibly the same observation.
inline const float sightLineWidth = 3.0f;

// Sight lines of observations already matched, as a self-check: a 3D marker
// that does not sit on its own line was mis-picked. Dim and thin -- context,
// not the thing being aimed at.
inline const glm::vec3 sightLineDimColor(0.55f, 0.55f, 0.55f);
inline const float     sightLineDimWidth = 1.0f;

} // namespace overlay
