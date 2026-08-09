#pragma once

#include <vector>

#include <glm/glm.hpp>

#include "FeatureMatching.h"   // FeatureDb, FrameCapture
#include "../core/Camera.h"    // Camera, Viewport, Waypoint
#include "../core/Scene.h"     // Mesh

// Give every hand-placed point the descriptors of its appearances in the OTHER
// recorded views: no descriptor is viewpoint-invariant, so one appearance per
// point is what makes free flight fail where a recorded waypoint works.
// Invents no 3D -- an appearance counts only if a keypoint sits near the
// anchor's projection AND still resembles it, so a misjudged depth gains
// nothing (docs/pose-estimation-modes.md, "Mode D"). `prototype` supplies the
// lens, copied per view. Prints its own totals and per-gate losses.
void collectAppearances(FeatureDb &db, const Mesh &mesh, float terrainSize,
                        const std::vector<Waypoint> &views, const Camera &prototype,
                        const Viewport &vp, const FrameCapture &capture);
