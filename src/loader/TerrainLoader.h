#pragma once

#include <optional>
#include <string>

#include "../core/Scene.h"

// Reads a DEM image into a Mesh. Returns std::nullopt if the image can't be
// loaded -- the caller must unwrap before using the terrain, so a failed load
// can't be silently consumed.
std::optional<Mesh> readTerrain(const std::string &filename);

// Per-vertex normals from the height grid by central differences: for a
// surface y = h(x, z) sampled on a unit grid, the (unnormalized) upward
// normal at a vertex is (h[x-1] - h[x+1], 2, h[z-1] - h[z+1]) -- the cross
// product of the two grid tangents. Index clamping at the borders degrades
// the difference to one-sided there. Called by readTerrain; declared here so
// the headless tests can verify it on synthetic grids.
void computeNormals(Mesh &mesh);