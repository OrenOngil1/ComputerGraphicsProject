#pragma once

#include <vector>
#include <optional>

#include "State.h"
#include "../core/Scene.h"    // Correspondence
#include "../core/Camera.h"   // Waypoint

// The small application modes, grouped in one header; a mode that grows
// substantial moves to its own file (as the pose-comparison modes did).

// Mode A, free navigation: continuous FPS flight of the player camera
// (WASD move, arrows look, Q/E altitude). No discrete keys, no overlay.
class NavigationState : public State {
public:
    void tick(Simulation &sim, GLFWwindow *window, float dt) override;
};

// Records the player's flight: same continuous movement, each new position
// captured as a path point, 'B' storing a camera waypoint, 'O' replacing the
// whole recording with the calibrated build ring. The global view overlays the
// path and waypoints.
class RecordState : public State {
public:
    void onEnter(Simulation &sim) override;   // start a fresh recording
    void handleKey(Simulation &sim, Renderer &renderer, int key, int mods) override;  // 'B' / 'O'
    void tick(Simulation &sim, GLFWwindow *window, float dt) override;
    void renderGlobalOverlay(const Simulation &sim, Renderer &renderer,
                             const glm::mat4 &mvp) const override;
private:
    // Set by 'O'. The ring is a laid-out geometry, not a flight: once it is
    // down, freehand recording stops, because a path point appended after it
    // draws a tail from the ring to wherever the camera drifted, and an extra
    // waypoint breaks the 22.5-degree spacing the ring exists to guarantee.
    // R starts a fresh recording and clears it.
    bool m_ringLaid = false;
};

// Steps the player camera through the recorded waypoints with UP/DOWN. The
// global view shows the path + waypoints, the one under the camera highlighted.
class PlaybackState : public State {
public:
    void onEnter(Simulation &sim) override;   // snap to the first waypoint
    void handleKey(Simulation &sim, Renderer &renderer, int key, int mods) override;
    void renderGlobalOverlay(const Simulation &sim, Renderer &renderer,
                             const glm::mat4 &mvp) const override;
private:
    size_t m_index = 0;   // the selected waypoint
};

// Mode B, pose estimation by color picking (docs/pose-estimation-modes.md,
// "Mode B"). The player camera is seeded at a random recorded waypoint; each
// correspondence is two clicks -- a 2D point in the player view, then its 3D
// match color-picked on the global map -- and 'C' solves PnP. Both views then
// compare estimate against truth (picked points + estimated camera on the
// map, ghost terrain in the player view). Finding the 3D match by eye is the
// hard part, so the map carries aids: V cycles them (full / cone only /
// off), X cancels a half-finished pair, U undoes the last one.
class PickState : public State {
public:
    void onEnter(Simulation &sim) override;                      // seed pose, reset picks
    // 'C' -> solve, 'X' -> cancel the pending 2D pick, 'U' -> undo the last pair
    void handleKey(Simulation &sim, Renderer &renderer, int key, int mods) override;
    void handleMouseButton(Simulation &sim, Renderer &renderer, GLFWwindow *window,
                           int button, int action) override;        // left-click -> pick
    void renderGlobalOverlay(const Simulation &sim, Renderer &renderer,
                             const glm::mat4 &mvp) const override;
    void renderPlayerOverlay(const Simulation &sim, Renderer &renderer,
                             const glm::mat4 &mvp) const override;
private:
    // One 2D pick, stored resize-proof. A pixel (or viewport fraction) only
    // means something at the aspect ratio it was captured at -- glm::perspective
    // derives the horizontal FOV from the aspect, so a resize re-scales the
    // horizontal half of any stored screen coordinate. The camera-space ray
    // (x/z, y/z) is the aspect-invariant form; it is converted back to the live
    // viewport only where pixels are needed (markers, PnP).
    struct Observation {
        glm::vec3 worldPos;   // centered world space, like Correspondence::worldPos
        glm::vec2 imageRay;   // (x/z, y/z), camera space, image convention (x right, y down)
    };

    // The two halves of the two-click pick flow, and the solve behind 'C':
    // phase A stores the player-view click as a pending camera-space ray,
    // phase B color-picks its 3D match off the map and completes the pair.
    void recordImagePick(Simulation &sim, Renderer &renderer, const glm::dvec2 &cursor);
    void recordWorldPick(Simulation &sim, Renderer &renderer, const glm::dvec2 &cursor);
    void solveFromPicks(const Simulation &sim);

    // The two views draw an observation's two different halves:
    // global view ("map"): the 3D worldPos through the scene mvp.
    void drawWorldMarkers(Renderer &renderer, const glm::mat4 &mvp) const;
    // player view: the 2D ray reprojected into the CURRENT viewport -- the
    // marker tracks resizes yet never leaks worldPos's true projection.
    void drawImageMarkers(Renderer &renderer, float fov, const Viewport &viewport) const;
    // global view: each observation's sight line, the locus its 3D point lies
    // on. Direction only -- the depth along it is what the user supplies.
    void drawSightAids(const Simulation &sim, Renderer &renderer, const glm::mat4 &mvp) const;

    std::vector<Observation> m_pickedPoints;
    std::optional<Waypoint>  m_computedCamera;   // PnP result; empty until 'C' succeeds

    // Mid-pair marker for the two-click flow: set to the 2D click's ray while
    // awaiting its 3D match, empty otherwise.
    std::optional<glm::vec2> m_pendingImageRay;
};
