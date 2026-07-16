#pragma once

#include <optional>
#include <string>

#include "../core/Scene.h"

// Read a DEM image into a Mesh; nullopt if the image can't be loaded.
std::optional<Mesh> readTerrain(const std::string &filename);

// Per-vertex normals from the height grid by central differences: for a
// surface y = h(x, z) on a unit grid, the (unnormalized) upward normal is
// (h[x-1] - h[x+1], 2, h[z-1] - h[z+1]) -- the cross product of the two grid
// tangents. Border indices clamp, degrading to one-sided differences there.
// Called by readTerrain; declared here so the headless tests can verify it.
void computeNormals(Mesh &mesh);
