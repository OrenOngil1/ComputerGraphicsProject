#pragma once

#include <optional>
#include <string>

#include "../core/Scene.h"

// Reads a DEM image into a Mesh. Returns std::nullopt if the image can't be
// loaded -- the caller must unwrap before using the terrain, so a failed load
// can't be silently consumed.
std::optional<Mesh> readTerrain(const std::string &filename);