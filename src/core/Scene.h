#pragma once

#include <vector>
#include <glm/glm.hpp>

struct Vertex {
    glm::vec3 position;
    glm::vec3 color;
};

struct PickedPoint {
    glm::vec3 worldPos;
    glm::vec2 imagePos;
};

// A TRACKERS-mode landmark (Mode 3): a sphere resting on the terrain surface,
// identified by a unique flat color. The color doubles as the correspondence
// key -- the blob detector finds it in the rendered frame, and the center is
// the matching 3D point. center is in centered world space (the space the
// cameras live in), like PickedPoint::worldPos.
struct Tracker {
    glm::vec3 center;
    float radius;
    glm::vec3 color;
};

// Grid dimensions are VERTEX COUNTS (cols along x, rows along z), not world
// sizes -- vertices[z * cols + x] is the vertex at grid cell (x, z).
struct Mesh {
    int cols, rows;
    std::vector<Vertex> vertices;
};