#pragma once

#include <vector>
#include <optional>

#include "State.h"
#include "../core/Scene.h"    // PickedPoint
#include "../core/Camera.h"   // Waypoint

// The concrete application modes, grouped deliberately: each is tiny, they form
// one cohesive set (the mode space), they change together, and only the
// transition logic (Callbacks.cpp) constructs them. The base interface lives in
// State.h so dependents (Renderer) rely on the abstraction, not these concretes.
// Split a mode into its own file once it grows substantial -- PickState and
// TrackersState likely will, once Modes 2/3 land.

// Free navigation (the old Mode::NONE): continuous FPS flight of the player camera
// (WASD to move, arrows to look, Shift+>/< for altitude). No discrete keys, no overlay.
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
    void handleKey(Simulation &sim, int key, int mods) override;             // 'B'
    void tick(Simulation &sim, GLFWwindow *window, float dt) override;
    void renderGlobalOverlay(const Simulation &sim, Renderer &renderer,
                             const glm::mat4 &mvp) const override;
};

// Steps the player camera through the recorded waypoints with UP / DOWN. The
// global view shows the path + waypoints, the one under the camera highlighted.
class PlaybackState : public State {
public:
    void onEnter(Simulation &sim) override;   // snap to the first waypoint
    void handleKey(Simulation &sim, int key, int mods) override;
    void renderGlobalOverlay(const Simulation &sim, Renderer &renderer,
                             const glm::mat4 &mvp) const override;
private:
    // The selected waypoint. Mode-local: born at 0 when PLAYBACK is entered, gone
    // when it is left -- no need for a field on Simulation.
    size_t m_index = 0;
};

// Mode 2: pose estimation by color picking. The player camera is seeded at a random
// recorded waypoint (the "unknown" pose); the user left-clicks terrain points to build
// 2D-3D correspondences, then presses 'C' to solve PnP. The global view shows the
// picked points + estimated camera; the player view overlays a translucent ghost of
// the terrain from the estimated pose, for visual comparison against the true view.
class PickState : public State {
public:
    void onEnter(Simulation &sim) override;                      // seed pose, reset picks
    void handleKey(Simulation &sim, int key, int mods) override; // 'C' -> solve PnP
    void handleMouseButton(Simulation &sim, Renderer &renderer, GLFWwindow *window,
                           int button, int action) override;        // left-click -> pick
    void renderGlobalOverlay(const Simulation &sim, Renderer &renderer,
                             const glm::mat4 &mvp) const override;
    void renderPlayerOverlay(const Simulation &sim, Renderer &renderer,
                             const glm::mat4 &mvp) const override;
private:
    // Draw the picked correspondences as palette-colored markers (shared by both views).
    void drawPickedPoints(Renderer &renderer, const glm::mat4 &mvp) const;

    std::vector<PickedPoint> m_pickedPoints;
    std::optional<Waypoint>  m_computedCamera;   // PnP result; empty until 'C' succeeds
};
