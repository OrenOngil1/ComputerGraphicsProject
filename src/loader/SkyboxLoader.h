#pragma once

#include <array>
#include <optional>
#include <string>
#include <vector>

// The six decoded faces of a cubemap, ready for GL upload. Face order is
// +X, -X, +Y, -Y, +Z, -Z -- index i maps to GL_TEXTURE_CUBE_MAP_POSITIVE_X + i.
// Pixels are 8-bit BGR (OpenCV's decode order), rows tightly packed top-down;
// the uploader passes GL_BGR so no channel swap is ever needed.
struct CubemapFaces {
    int size = 0;   // face edge length, pixels (faces are square, all equal)
    std::array<std::vector<unsigned char>, 6> pixels;
};

// Read a skybox's six faces from <dir>/posx.png ... <dir>/negz.png; nullopt
// (with a console warning) if any face is missing, non-square, or a different
// size than the others.
std::optional<CubemapFaces> loadSkybox(const std::string &dir);
