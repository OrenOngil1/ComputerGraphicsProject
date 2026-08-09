#pragma once

#include <vector>

#include "../core/Camera.h"   // Camera, Viewport, Waypoint
#include "../core/Scene.h"    // Mesh, Correspondence

// Could a camera at this pose have taken the frame? A lookalike coalition can
// clear every solver gate and still land impossibly far out, under the ground,
// aimed at empty sky, or where ridges hide its own evidence -- one check each,
// and passing all four is what lets the consensus floor sit so low. Judges the
// ESTIMATE alone; each refusal prints which check fired.
bool poseIsPlausible(const Mesh &mesh, float terrainSize, const Camera &prototype,
                     const Waypoint &estimate, const Viewport &vp,
                     const std::vector<Correspondence> &inliers);
