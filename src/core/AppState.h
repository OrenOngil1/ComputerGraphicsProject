#pragma once

#include <memory>

#include "Scene.h"
#include "Camera.h"

// The active mode is a polymorphic State (see src/state/). AppState only stores a
// pointer to it, so a forward declaration suffices here; State's methods take this
// AppState by reference and are defined in .cpp files that see it complete.
class State;

struct AppState {
    std::unique_ptr<State> currentState;   // the active mode (replaces `Mode mode`)
    float terrainSize = 0.0f;
    Mesh mesh;
    View globalView;   // left half:  global camera + its viewport
    View playerView;   // right half: player camera + its viewport
    std::vector<glm::vec3> pathPoints;
    std::vector<Waypoint> waypoints;

    // Out-of-line (AppState.cpp): a unique_ptr to the forward-declared State can
    // only be destroyed where State is a complete type.
    ~AppState();
};