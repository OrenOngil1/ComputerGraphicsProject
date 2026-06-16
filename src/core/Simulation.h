#pragma once

#include <memory>

#include "Scene.h"
#include "Camera.h"
#include "Lighting.h"

// The active mode is a polymorphic State (see src/state/). Simulation only stores a
// pointer to it, so a forward declaration suffices here; State's methods take this
// Simulation by reference and are defined in .cpp files that see it complete.
class State;

struct Simulation {
    std::unique_ptr<State> currentState;   // the active mode (replaces `Mode mode`)
    float terrainSize = 0.0f;
    Mesh mesh;
    View globalView;   // left half:  global camera + its viewport
    View playerView;   // right half: player camera + its viewport
    std::vector<glm::vec3> pathPoints;
    std::vector<Waypoint> waypoints;

    // The scene's sun: always one of kLightPresets, stored as the index so a
    // separate light copy can't drift from it. Deliberately NOT reset per
    // terrain (unlike the recording state above): a user-chosen lighting
    // setup survives menu round-trips, which Mode 4's pre/run lighting
    // experiment relies on.
    size_t lightPreset = 0;

    const DirectionalLight &light() const { return kLightPresets[lightPreset].light; }

    // Advance to the next preset (the L key) and return its display name, so
    // every future writer of the light goes through the same one-liner.
    const char *cycleLightPreset()
    {
        lightPreset = (lightPreset + 1) % kLightPresetCount;
        return kLightPresets[lightPreset].name;
    }

    // Session exit: Escape sets this to leave the current terrain and return to the
    // menu. Quitting the program is the other signal -- glfwWindowShouldClose, raised
    // by Ctrl+Q or the OS close button. The session loop checks both; run() tells the
    // "back to menu" flag apart from a quit.
    bool returnToMenu = false;

    // Out-of-line (Simulation.cpp): a unique_ptr to the forward-declared State can
    // only be destroyed where State is a complete type.
    ~Simulation();
};