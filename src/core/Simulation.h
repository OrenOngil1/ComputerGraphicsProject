#pragma once

#include <memory>
#include <string>

#include "Scene.h"
#include "Camera.h"
#include "Lighting.h"

// The active mode is a polymorphic State (see src/state/); a forward
// declaration suffices for the unique_ptr member.
class State;

// The per-session application state: everything the modes, callbacks, and
// renderer share. Owned by Application, reset per terrain in loadTerrain.
struct Simulation {
    std::unique_ptr<State> currentState;   // the active mode
    float terrainSize = 0.0f;              // max(cols, rows) of the mesh; scales speeds/sizes
    Mesh mesh;                             // the loaded terrain (uncentered)

    // The DEM this session was loaded from, as the menu gave it. Kept because
    // anything written to disk is only meaningful for one terrain: a saved
    // feature database names it so reloading under a different DEM is refused
    // rather than silently anchoring to the wrong ground.
    std::string terrainFile;

    View globalView;                       // left half:  overview camera + viewport
    View playerView;                       // right half: player camera + viewport
    std::vector<glm::vec3> pathPoints;     // the recorded flight path (RECORD)
    std::vector<Waypoint> waypoints;       // recorded camera poses (RECORD's 'B')

    // The scene's sun, stored as an index into kLightPresets so a separate
    // light copy can't drift from the preset. Deliberately NOT reset per
    // terrain: a user-chosen lighting setup survives menu round-trips, which
    // Mode 4's pre/run lighting experiment relies on.
    size_t lightPreset = 0;

    const DirectionalLight &light() const { return kLightPresets[lightPreset].light; }

    // Advance to the next preset (the L key); returns its display name.
    const char *cycleLightPreset()
    {
        lightPreset = (lightPreset + 1) % kLightPresetCount;
        return kLightPresets[lightPreset].name;
    }

    // The global map's view aids (the V key): the player camera's view cone,
    // and the sight lines of observations awaiting a 3D match. Session-level
    // like the light rather than per-mode -- "where is the player camera
    // looking" is the same question in every mode, and the manual modes are
    // unusable without it. On by default; off gives a clean demo view.
    bool showViewAids = true;

    // Set by Escape: leave the current terrain, back to the menu. Quitting the
    // program is a separate signal (glfwWindowShouldClose, via Ctrl+Q or the
    // OS close button); the session loop checks both.
    bool returnToMenu = false;

    // Out-of-line (Simulation.cpp): the unique_ptr<State> can only be
    // destroyed where State is a complete type.
    ~Simulation();
};
