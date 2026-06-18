#pragma once

#include <vector>
#include <optional>

#include "State.h"
#include "../core/Scene.h"    // Correspondence
#include "../core/Camera.h"   // Waypoint

// The concrete application modes, grouped deliberately: each is tiny, they form
// one cohesive set (the mode space), they change together, and only the
// transition logic (Callbacks.cpp) constructs them. The base interface lives in
// State.h so dependents (Renderer) rely on the abstraction, not these concretes.
// Split a mode into its own file once it grows substantial -- the pose-
// comparison modes did exactly that (PoseComparisonState.h, TrackersState.h).

// Free navigation (the old Mode::NONE): continuous FPS flight of the player camera
// (WASD to move, arrows to look, Q/E for altitude). No discrete keys, no overlay.
class NavigationState : public State {
public:
    void tick(Simulation &sim, GLFWwindow *window, float dt) override;
};

// Records the player's flight: the same continuous movement, with each new position
// captured as a path point, and 'B' storing a camera waypoint. The global view overlays
// the path and waypoints.
class RecordState : public State {
public:
    void onEnter(Simulation &sim) override;   // start a fresh recording
    void handleKey(Simulation &sim, Renderer &renderer, int key, int mods) override;  // 'B'
    void tick(Simulation &sim, GLFWwindow *window, float dt) override;
    void renderGlobalOverlay(const Simulation &sim, Renderer &renderer,
                             const glm::mat4 &mvp) const override;
};

// Steps the player camera through the recorded waypoints with UP / DOWN. The
// global view shows the path + waypoints, the one under the camera highlighted.
class PlaybackState : public State {
public:
    void onEnter(Simulation &sim) override;   // snap to the first waypoint
    void handleKey(Simulation &sim, Renderer &renderer, int key, int mods) override;
    void renderGlobalOverlay(const Simulation &sim, Renderer &renderer,
                             const glm::mat4 &mvp) const override;
private:
    // The selected waypoint. Mode-local: born at 0 when PLAYBACK is entered, gone
    // when it is left -- no need for a field on Simulation.
    size_t m_index = 0;
};

// Mode 2: pose estimation by color picking. The player camera is seeded at a random
// recorded waypoint (the "unknown" pose). Each 2D-3D correspondence is built in two
// clicks -- a 2D point in the player (camera) view, then its matching 3D point color-
// picked in the global view (the "map") -- then 'C' solves PnP. The global view shows
// the picked points + estimated camera; the player view overlays a translucent ghost
// of the terrain from the estimated pose, for visual comparison against the true view.
class PickState : public State {
public:
    void onEnter(Simulation &sim) override;                      // seed pose, reset picks
    void handleKey(Simulation &sim, Renderer &renderer, int key, int mods) override; // 'C' -> solve PnP
    void handleMouseButton(Simulation &sim, Renderer &renderer, GLFWwindow *window,
                           int button, int action) override;        // left-click -> pick
    void renderGlobalOverlay(const Simulation &sim, Renderer &renderer,
                             const glm::mat4 &mvp) const override;
    void renderPlayerOverlay(const Simulation &sim, Renderer &renderer,
                             const glm::mat4 &mvp) const override;
private:
    // PICK stores each observation's 2D half as a camera-space viewing ray, not a
    // screen coordinate. A pixel -- or a [0,1] viewport fraction -- only means
    // something against the aspect ratio it was captured at: glm::perspective fixes the
    // vertical FOV and derives the horizontal one from the aspect, so resizing the
    // window silently re-scales the horizontal half of any stored screen coordinate and
    // the observation goes stale. The ray ((x/z, y/z) in camera space) is the
    // aspect-invariant form; it is converted to the live viewport only at the two
    // boundaries that need pixels -- drawn as a marker, and handed to PnP.
    struct Observation {
        glm::vec3 worldPos;   // centered world space, like Correspondence::worldPos
        glm::vec2 imageRay;   // (x/z, y/z), camera space, OpenCV image convention (x right, y down)
    };

    // The two views show an observation's two different halves, so they draw different
    // things -- deliberately NOT a shared helper:
    //  - global view (the "map"): the 3D worldPos, in world space through the scene mvp.
    //  - player view (the camera): the 2D ray reprojected into the CURRENT viewport --
    //    so the marker tracks resizes and stays on the feature, yet never moves to the
    //    reprojection of worldPos (that would leak the true projection).
    void drawWorldMarkers(Renderer &renderer, const glm::mat4 &mvp) const;
    void drawImageMarkers(Renderer &renderer, float fov, const Viewport &viewport) const;

    std::vector<Observation> m_pickedPoints;
    std::optional<Waypoint>  m_computedCamera;   // PnP result; empty until 'C' succeeds

    // The two-phase pick: an observation is a 2D click in the player view paired with a
    // 3D color-pick in the global view (the "map"). Set (to the viewing ray of the 2D
    // click) when the 2D half has been clicked and we are awaiting its 3D match; empty
    // otherwise. Carrying the ray *in* the optional makes "mid-pair but no saved 2D"
    // unrepresentable.
    std::optional<glm::vec2> m_pendingImageRay;
};
