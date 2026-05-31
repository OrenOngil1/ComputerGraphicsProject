#pragma once

#include "State.h"

// The concrete application modes, grouped deliberately: each is tiny, they form
// one cohesive set (the mode space), they change together, and only the
// transition logic (Callbacks.cpp) constructs them. The base interface lives in
// State.h so dependents (Renderer) rely on the abstraction, not these concretes.
// Split a mode into its own file once it grows substantial -- PickState and
// TrackersState likely will, once Modes 2/3 land.

// Free navigation (the old Mode::NONE): arrow keys move the player camera,
// '<' / '>' change altitude. No overlay.
class NavigationState : public State {
public:
    void handleKey(AppState &appState, int key, int mods) override;
};

// Records the player's flight: movement is captured as path points, and 'B'
// stores a camera waypoint. The global view overlays the path and waypoints.
class RecordState : public State {
public:
    void onEnter(AppState &appState) override;   // start a fresh recording
    void handleKey(AppState &appState, int key, int mods) override;
    void renderGlobalOverlay(const AppState &appState, Shader &shader,
                             const glm::mat4 &mvp) const override;
};

// Steps the player camera through the recorded waypoints with UP / DOWN. The
// global view shows the path + waypoints, the one under the camera highlighted.
class PlaybackState : public State {
public:
    void onEnter(AppState &appState) override;   // snap to the first record
    void handleKey(AppState &appState, int key, int mods) override;
    void renderGlobalOverlay(const AppState &appState, Shader &shader,
                             const glm::mat4 &mvp) const override;
private:
    // The selected record. Mode-local: born at 0 when PLAYBACK is entered, gone
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
