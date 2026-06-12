#pragma once

#include <glm/glm.hpp>

// A single directional light (the sun): everything the terrain's Lambert
// shading needs. Scene data like the cameras -- it lives on Simulation, modes
// and the renderer read it, and a runtime control may swap it (Mode 4's
// experiment changes the lighting between its Pre and Run phases).
struct DirectionalLight {
    glm::vec3 direction;   // world space, pointing FROM the light INTO the scene
    glm::vec3 color;       // diffuse light color
    float     ambient;     // base illumination floor [0..1], so shadows aren't pitch black
};
