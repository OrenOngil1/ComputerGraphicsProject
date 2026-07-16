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
// captured as a path point, 'B' storing a camera waypoint. The global view
// overlays the path and waypoints.
class RecordState : public State {
public:
    void onEnter(Simulation &sim) override;   // start a fresh recording
    void handleKey(Simulation &sim, Renderer &renderer, int key, int mods) override;  // 'B'
    void tick(Simulation &sim, GLFWwindow *window, float dt) override;
    void renderGlobalOverlay(const Simulation &sim, Renderer &renderer,
                             const glm::mat4 &mvp) const override;
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

// Mode B, pose estimation by color picking. The player camera is seeded at a
// random recorded waypoint (the "unknown" pose). Each correspondence is built
// in two clicks -- a 2D point in the player view, then its 3D match color-
// picked in the global view (the "map") -- and 'C' solves PnP. The global view
// shows the picked points + estimated camera; the player view overlays the
// estimate's translucent ghost terrain for comparison against the true view.
class PickState : public State {
public:
    void onEnter(Simulation &sim) override;                      // seed pose, reset picks
    void handleKey(Simulation &sim, Renderer &renderer, int key, int mods) override; // 'C' -> solve
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

    // The two views draw an observation's two different halves:
    // global view ("map"): the 3D worldPos through the scene mvp.
    void drawWorldMarkers(Renderer &renderer, const glm::mat4 &mvp) const;
    // player view: the 2D ray reprojected into the CURRENT viewport -- the
    // marker tracks resizes yet never leaks worldPos's true projection.
    void drawImageMarkers(Renderer &renderer, float fov, const Viewport &viewport) const;

    std::vector<Observation> m_pickedPoints;
    std::optional<Waypoint>  m_computedCamera;   // PnP result; empty until 'C' succeeds

    // Mid-pair marker for the two-click flow: set to the 2D click's ray while
    // awaiting its 3D match, empty otherwise.
    std::optional<glm::vec2> m_pendingImageRay;
};
