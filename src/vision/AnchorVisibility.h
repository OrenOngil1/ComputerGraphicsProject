#pragma once

#include <glm/glm.hpp>

#include "../core/Camera.h"   // Camera, Viewport
#include "../core/Scene.h"    // Mesh

// Can a camera drawing through `viewport` actually use this anchor? Frustum is
// not enough -- SIFT matches what was DRAWN, and a feature behind a ridge was
// not -- so this marches the terrain too. Takes the mesh directly rather than a
// Simulation: three callers ask, in the run phase, the build, and the solver.
bool anchorVisibleFrom(const Mesh &mesh, float terrainSize, const Camera &camera,
                       const Viewport &viewport, const glm::vec3 &anchor);
