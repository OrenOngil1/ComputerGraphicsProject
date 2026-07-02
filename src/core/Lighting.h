#pragma once

#include <glm/glm.hpp>

// A single directional light (the sun) -- everything the terrain's Lambert
// shading needs. Scene data on Simulation; the L key swaps presets at runtime.
struct DirectionalLight {
    glm::vec3 direction;   // world space, pointing FROM the light INTO the scene
    glm::vec3 color;       // diffuse light color
    float     ambient;     // base illumination floor [0..1], so shadows aren't pitch black
};

// A named lighting setup, for the L key to cycle through.
struct LightPreset {
    const char      *name;   // printed to the console when selected
    DirectionalLight light;
};

// Deliberately few and visually far apart: the presets exist so the scene's
// appearance can change MARKEDLY (Mode 4 measures how feature matching
// survives a lighting change), not for fine-tuning. [0] is the startup default.
inline const LightPreset kLightPresets[] = {
    { "late-morning sun", { glm::normalize(glm::vec3(-0.4f, -1.0f, -0.3f)),
                            glm::vec3(1.0f), 0.35f } },
    { "noon sun",         { glm::normalize(glm::vec3(0.05f, -1.0f, 0.05f)),
                            glm::vec3(1.0f), 0.45f } },
    { "low warm sun",     { glm::normalize(glm::vec3(-1.0f, -0.25f, 0.2f)),
                            glm::vec3(1.0f, 0.75f, 0.5f), 0.20f } },
    { "overcast",         { glm::vec3(0.0f, -1.0f, 0.0f),
                            glm::vec3(0.55f, 0.55f, 0.6f), 0.60f } },
};
inline const size_t kLightPresetCount = sizeof(kLightPresets) / sizeof(kLightPresets[0]);
