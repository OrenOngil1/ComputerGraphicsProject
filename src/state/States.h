#pragma once

#include "State.h"

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
    void tick(AppState &appState, GLFWwindow *window, float dt) override;
};

// Records the player's flight: the same continuous movement, with each new position
// captured as a path point, and 'B' storing a camera waypoint. The global view overlays
// the path and waypoints.
class RecordState : public State {
public:
    void onEnter(AppState &appState) override;   // start a fresh recording
    void handleKey(AppState &appState, int key, int mods) override;             // 'B'
    void tick(AppState &appState, GLFWwindow *window, float dt) override;
    void renderGlobalOverlay(const AppState &appState, Renderer &renderer,
                             const glm::mat4 &mvp) const override;
};

// Steps the player camera through the recorded waypoints with UP / DOWN. The
// global view shows the path + waypoints, the one under the camera highlighted.
class PlaybackState : public State {
public:
    void onEnter(AppState &appState) override;   // snap to the first waypoint
    void handleKey(AppState &appState, int key, int mods) override;
    void renderGlobalOverlay(const AppState &appState, Renderer &renderer,
                             const glm::mat4 &mvp) const override;
private:
    // The selected waypoint. Mode-local: born at 0 when PLAYBACK is entered, gone
    // when it is left -- no need for a field on AppState.
    size_t m_index = 0;
};

// Mode 2 stub: for now only its entry action (clear the path, matching oren).
// The real implementation adds mouse picking (handleMouseButton), the ghost
// overlay (renderPlayerOverlay), an offscreen pick pass on the Renderer, and
// seeds the camera from a recorded pose. See docs/state-pattern-refactor.md.
class PickState : public State {
public:
    void onEnter(AppState &appState) override;   // clear the path for a clean view
    void handleKey(AppState &, int, int) override {}
};
